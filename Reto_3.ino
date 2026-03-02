// ==========================
// ====== LIBRARIES =========
// ==========================
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>
#include <math.h>

// ==========================
// ====== ROS OBJECTS =======
// ==========================
rcl_node_t node;
rcl_subscription_t subscriber;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;

std_msgs__msg__Float32 msg;

// ==========================
// ====== PIN CONFIG =========
// ==========================
#define IN1_PIN     18
#define IN2_PIN     15
#define EN_PIN      4     // PWM

#define LED_PIN     2

#define PWM_CHANNEL 0
#define PWM_FREQ    980
#define PWM_RES     8     // 0-255

#define INPUT_MIN  -1.0
#define INPUT_MAX   1.0

// ==========================
// ====== MACROS ============
// ==========================
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// ==========================
// ====== ERROR LOOP ========
// ==========================
void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

// ==========================
// ====== CALLBACK =========
// ==========================
void subscription_callback(const void * msgin)
{
  const std_msgs__msg__Float32 * received_msg =
    (const std_msgs__msg__Float32 *)msgin;

  float input = constrain(received_msg->data, INPUT_MIN, INPUT_MAX);

  // -------------------------
  // Dirección
  // -------------------------
  if(input > 0){
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);
  }
  else if(input < 0){
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, HIGH);
  }
  else{
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, LOW);
  }

  // -------------------------
  // Magnitud → PWM
  // -------------------------
  float duty = fabs(input);
  uint32_t pwm_value = (uint32_t)(duty * ((1 << PWM_RES) - 1));

  ledcWrite(PWM_CHANNEL, pwm_value);
}

// ==========================
// ====== SETUP =============
// ==========================
void setup(){

  set_microros_transports();

  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  // Configuración PWM
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RES);
  ledcAttachPin(EN_PIN, PWM_CHANNEL);

  delay(2000);

  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  RCCHECK(rclc_node_init_default(
    &node,
    "motor",
    "",
    &support));

  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/cmd_pwm"));

  RCCHECK(rclc_executor_init(
    &executor,
    &support.context,
    1,
    &allocator));

  RCCHECK(rclc_executor_add_subscription(
    &executor,
    &subscriber,
    &msg,
    &subscription_callback,
    ON_NEW_DATA));
}

// ==========================
// ====== LOOP ==============
// ==========================
void loop(){
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
  delay(10);
}
