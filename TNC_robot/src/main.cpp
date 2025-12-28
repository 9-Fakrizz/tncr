#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/imu.h>
#include <Wire.h>
#include "SparkFun_BNO08x_Arduino_Library.h"

// --- Hardware Pins ---
const int DIR1_PIN = 21; const int DIR2_PIN = 20;
const int PWM1_PIN = 23; const int PWM2_PIN = 22;
#define BNO08X_RST 40
BNO08x myIMU;

// --- micro-ROS Objects ---
rcl_subscription_t subscriber;
rcl_publisher_t publisher;
geometry_msgs__msg__Twist msg_cmd;
sensor_msgs__msg__Imu msg_imu;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){ 
  while(1){ 
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(500); 
  } 
}

void set_motors(float lin_x, float ang_z) {
    float left = lin_x - ang_z;
    float right = lin_x + ang_z;
    
    auto drive = [](int d_pin, int p_pin, float val) {
        digitalWrite(d_pin, val >= 0 ? HIGH : LOW);
        analogWrite(p_pin, (int)constrain(abs(val) * 255, 0, 255));
    };
    
    drive(DIR1_PIN, PWM1_PIN, left);
    drive(DIR2_PIN, PWM2_PIN, right);
}

void cmd_vel_callback(const void * msgin) {
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
    set_motors(msg->linear.x, msg->angular.z);
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
    (void) last_call_time;
    if (timer != NULL) {
        if (myIMU.getSensorEvent() && myIMU.getSensorEventID() == SENSOR_REPORTID_ROTATION_VECTOR) {
            msg_imu.orientation.x = myIMU.getQuatI();
            msg_imu.orientation.y = myIMU.getQuatJ();
            msg_imu.orientation.z = myIMU.getQuatK();
            msg_imu.orientation.w = myIMU.getQuatReal();
            msg_imu.header.frame_id.data = (char*)"imu_link";
            
            RCSOFTCHECK(rcl_publish(&publisher, &msg_imu, NULL));
        }
    }
}

void setup() {
    set_microros_transports();
    pinMode(LED_BUILTIN, OUTPUT);

    // IMU Hardware Init
    pinMode(BNO08X_RST, OUTPUT);
    digitalWrite(BNO08X_RST, LOW); delay(100); digitalWrite(BNO08X_RST, HIGH); delay(500);
    Wire.begin();
    if(!myIMU.begin(0x4B, Wire, -1, BNO08X_RST)) error_loop();
    myIMU.enableRotationVector();

    // Motor Pins
    pinMode(DIR1_PIN, OUTPUT); pinMode(DIR2_PIN, OUTPUT);
    pinMode(PWM1_PIN, OUTPUT); pinMode(PWM2_PIN, OUTPUT);

    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "teensy_bot", "", &support));

    RCCHECK(rclc_subscription_init_default(&subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));
    RCCHECK(rclc_publisher_init_default(&publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu_data"));

    // ใช้ timer_init_default ธรรมดาหาก default2 มีปัญหา
    RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(50), timer_callback));

    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg_cmd, &cmd_vel_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));
}

void loop() {
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}