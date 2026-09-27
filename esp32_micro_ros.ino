/*******************************************************************************
 * Firmware: micro_ros_bluerov2_node.ino
 * Author: Sirine SIOUD (ENICarthage - Mechatronics)
 * Project: Subsea Pipeline Inspection ROV
 * 
 * Description:
 *   Low-level 6-thruster motor actuation firmware for ESP32 using micro-ROS.
 *   
 * Functional Features:
 *   - Subscribes to 6 ROS 2 Float32 command topics (4 vectored at 45° + 2 vertical).
 *   - Converts normalized velocity inputs [-1.0, 1.0] to 8-bit PWM duty cycles (0–255).
 *   - Interfaces with 3x dual L298N H-Bridges via ESP32 LEDC hardware timers (1 kHz).
 *   - Configures deterministic multi-subscriber callback execution via rclc_executor.
 ******************************************************************************/
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>

// ── L298N Pin Definitions ──────────────────────────────────
// Each L298N has: IN1, IN2 (direction) + ENA (PWM speed) for motor A
//                IN3, IN4 (direction) + ENB (PWM speed) for motor B
//
// L298N #1 → Thruster Front Left (A) + Thruster Front Right (B)
#define FL_IN1  25
#define FL_IN2  26
#define FL_ENA  27   // PWM

#define FR_IN3  14
#define FR_IN4  12
#define FR_ENB  13   // PWM

// L298N #2 → Thruster Back Left (A) + Thruster Back Right (B)
#define BL_IN1  19
#define BL_IN2  18
#define BL_ENA  5    // PWM

#define BR_IN3  17
#define BR_IN4  16
#define BR_ENB  4    // PWM

// L298N #3 → Thruster Vertical Left (A) + Thruster Vertical Right (B)
#define VL_IN1  32
#define VL_IN2  33
#define VL_ENA  15   // PWM

#define VR_IN3  22
#define VR_IN4  23
#define VR_ENB  21   // PWM

// ── LEDC PWM config (ESP32) ────────────────────────────────
// ESP32 Arduino uses ledcSetup / ledcAttachPin / ledcWrite
#define PWM_FREQ    1000   // 1 kHz
#define PWM_RES     8      // 8-bit → 0–255

// Channel assignments (0–15 available)
#define CH_FL  0
#define CH_FR  1
#define CH_BL  2
#define CH_BR  3
#define CH_VL  4
#define CH_VR  5

// ── micro-ROS ──────────────────────────────────────────────
rcl_subscription_t sub_thruster_front_left;
rcl_subscription_t sub_thruster_front_right;
rcl_subscription_t sub_thruster_back_left;
rcl_subscription_t sub_thruster_back_right;
rcl_subscription_t sub_thruster_vertical_left;
rcl_subscription_t sub_thruster_vertical_right;

std_msgs__msg__Float32 msg_thruster_front_left;
std_msgs__msg__Float32 msg_thruster_front_right;
std_msgs__msg__Float32 msg_thruster_back_left;
std_msgs__msg__Float32 msg_thruster_back_right;
std_msgs__msg__Float32 msg_thruster_vertical_left;
std_msgs__msg__Float32 msg_thruster_vertical_right;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

#define LED_PIN 2

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// ── Error loop ─────────────────────────────────────────────
void error_loop(){
  while(1){
    // digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(500);
  }
}

// ── Motor Driver Helper ────────────────────────────────────
// value: -1.0 to +1.0
//   positive → forward  (IN1=HIGH, IN2=LOW)
//   negative → backward (IN1=LOW,  IN2=HIGH)
//   zero     → brake    (IN1=LOW,  IN2=LOW)
// in1, in2: direction pins
// pwmChannel: LEDC channel attached to the ENA/ENB pin
void drive_motor(float value, int in1, int in2, int enaPin) {
 // Clamp to [-1.0, 1.0]
  if (value >  1.0f) value =  1.0f;
  if (value < -1.0f) value = -1.0f;

  int speed = (int)(fabs(value) * 255.0f);  // 0–255

  if (value > 0.0f) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (value < 0.0f) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    // Brake 
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }

  ledcWrite(enaPin, speed);
}

// ── Callbacks ──────────────────────────────────────────────
void callback_thruster_front_left(const void * msgin){
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
  drive_motor(msg->data, FL_IN1, FL_IN2, FL_ENA);  // pin, not CH_FL
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
}

void callback_thruster_front_right(const void * msgin){
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
  drive_motor(msg->data, FR_IN3, FR_IN4, FR_ENB);
}

void callback_thruster_back_left(const void * msgin){
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
  drive_motor(msg->data, BL_IN1, BL_IN2, BL_ENA);
}

void callback_thruster_back_right(const void * msgin){
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
  drive_motor(msg->data, BR_IN3, BR_IN4, BR_ENB);
}

void callback_thruster_vertical_left(const void * msgin){
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
  drive_motor(msg->data, VL_IN1, VL_IN2, VL_ENA);
}

void callback_thruster_vertical_right(const void * msgin){
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
  drive_motor(msg->data, VR_IN3, VR_IN4, VR_ENB);
}

// ── Setup ──────────────────────────────────────────────────
void setup() {
  set_microros_transports();

  // LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  // Direction pins
  int dirPins[] = {
    FL_IN1, FL_IN2,
    FR_IN3, FR_IN4,
    BL_IN1, BL_IN2,
    BR_IN3, BR_IN4,
    VL_IN1, VL_IN2,
    VR_IN3, VR_IN4
  };
  for (int i = 0; i < 12; i++) {
    pinMode(dirPins[i], OUTPUT);
    digitalWrite(dirPins[i], LOW);
  }

  // PWM channels (LEDC — ESP32 Arduino)
  ledcAttach(FL_ENA, PWM_FREQ, PWM_RES);
  ledcAttach(FR_ENB, PWM_FREQ, PWM_RES);
  ledcAttach(BL_ENA, PWM_FREQ, PWM_RES);
  ledcAttach(BR_ENB, PWM_FREQ, PWM_RES);
  ledcAttach(VL_ENA, PWM_FREQ, PWM_RES);
  ledcAttach(VR_ENB, PWM_FREQ, PWM_RES);

  delay(2000);

  // micro-ROS init
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "micro_ros_bluerov2_node", "", &support));

  // Subscribers
  RCCHECK(rclc_subscription_init_default(&sub_thruster_front_left, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "bluerov2/thruster_front_left_cmd"));
  RCCHECK(rclc_subscription_init_default(&sub_thruster_front_right, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "bluerov2/thruster_front_right_cmd"));
  RCCHECK(rclc_subscription_init_default(&sub_thruster_back_left, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "bluerov2/thruster_back_left_cmd"));
  RCCHECK(rclc_subscription_init_default(&sub_thruster_back_right, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "bluerov2/thruster_back_right_cmd"));
  RCCHECK(rclc_subscription_init_default(&sub_thruster_vertical_left, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "bluerov2/thruster_vertical_left_cmd"));
  RCCHECK(rclc_subscription_init_default(&sub_thruster_vertical_right, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "bluerov2/thruster_vertical_right_cmd"));

  // Executor — 6 handles
  RCCHECK(rclc_executor_init(&executor, &support.context, 6, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_thruster_front_left,    &msg_thruster_front_left,    &callback_thruster_front_left,    ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_thruster_front_right,   &msg_thruster_front_right,   &callback_thruster_front_right,   ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_thruster_back_left,     &msg_thruster_back_left,     &callback_thruster_back_left,     ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_thruster_back_right,    &msg_thruster_back_right,    &callback_thruster_back_right,    ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_thruster_vertical_left,  &msg_thruster_vertical_left,  &callback_thruster_vertical_left,  ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_thruster_vertical_right, &msg_thruster_vertical_right, &callback_thruster_vertical_right, ON_NEW_DATA));

  digitalWrite(LED_PIN, LOW);
}

// ── Loop ───────────────────────────────────────────────────
void loop() {
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}