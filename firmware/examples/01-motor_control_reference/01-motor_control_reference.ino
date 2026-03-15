/*
 * ==========================================================================
 * File         : motor_control.ino
 * Description  : Create a Micro-ROS node that receives a setpoint on the 
 *                /set_point topic and controls the velocity of a DC motor in 
 *                closed-loop using encoders to match the given setpoint. 
 *                Additionally, publish the motor's inputs and outputs.
 *                       
 * Author       : Ricard
 * Date         : 09/03/2025
 * Version      : motor_control 
 * ==========================================================================
 * Additional Notes:
 * - Using a control signal [-255, 255]
 * - 
 * ==========================================================================
 */
// Include micro ros and rcl libraries
#include <micro_ros_arduino.h> // Micro-ROS library for ESP32

#include <stdio.h> // Standard I/O for debugging
#include <rcl/rcl.h> // ROS 2 client library (Core)
#include <rcl/error_handling.h> // ROS 2 error handling utilities
#include <rclc/rclc.h> // Micro-ROS client library
#include <rclc/executor.h> // Executor to handle callbacks

#include <std_msgs/msg/float32.h>  // include Float32 msg type
#include <rcutils/logging_macros.h>

// Instantiate Subscriber and msg
rcl_subscription_t setpoint_subscriber; // /set_point subscriber 
std_msgs__msg__Float32 setpoint_signal; // Holds the motor vel setpoint value with sign

rcl_publisher_t motor_output_publisher; // /motor_output publisher
std_msgs__msg__Float32 motor_output; // Holds the motor vel output value with sign

rcl_publisher_t motor_input_publisher; // /motor_output publisher
std_msgs__msg__Float32 motor_input; // Holds the motor vel output value with sign

// Instantiate micro ros executor and support classes
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t publish_timer; 
rcl_timer_t velcontrol_timer; // used for velocity control                   

// Define pins
#define LED_PIN 2 // error indicating LED
#define PWM_PIN_IN1 18     // Driver In 1
#define PWM_PIN_IN2 19     // Driver In 2
#define ENC_A 16           // Encoder cahnnel A
#define ENC_B 17           // Encoder channel B
// Define both PWM configurations
#define PWM_FRQ 5000   // PWM frequency (Hz)
#define PWM_RES 8      // PWM resolution (bits)
#define PWM_CHNL0 0     // PWM channel for in 1
#define PWM_CHNL1 1     // PWM channel for in 2
// Value limit for msg
#define MSG_MIN_VAL 0  // Minimum regulating signal value
#define MSG_MAX_VAL 1  // Maximum regulating signal value
// Error handling macros
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Executes a statement (X) every N milliseconds
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)\

// Micro Ros connection state machine 
enum states {
  WAITING_AGENT,        // Waiting for ROS 2 agent connection
  AGENT_AVAILABLE,      // Agent detected
  AGENT_CONNECTED,      // Successfully connected
  AGENT_DISCONNECTED    // Connection lost
} state;

bool create_entities();
void destroy_entities();

// Local variables
float pi = 3.1416;

// Controller
float kp = 12.58;
float ki = 187.88;
float kd = 0.0;

float T = 0.05; // sample time seconds
float K1,K2,K3;
float setpoint = 0.0; // velocity reference
double vel; // actual velocity
float e[3] = {0,0,0}; // e[0] actual error, e[1] last error, e[2] error before last
float u[2] = {0,0}; // u[0] actual output, u[1] last output
// encoder
volatile int pos = 0;
volatile int pos_ant = 0;

// Encoder interrupt
void IRAM_ATTR int_callback(){
  if (digitalRead(ENC_B)==0){
    pos += 1; 
  } else{
    pos -= 1;
  }
}

void velcontrol_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    
    if (timer != NULL) {
        // ---------------- Estimate Velocity ----------------
        static int last_pos = 0;
        static unsigned long last_time = 0;

        int pos_now = pos;
        unsigned long now = micros();
        
        double dt = (now - last_time) / 1e6; // Convert to seconds
        if (dt > 0) {
            vel = (pos_now - last_pos) / dt; // Counts per second
            vel = (vel / (12.0 * 34.0)) * (2.0 * pi); // Radians per second
            motor_output.data = vel;
            //Serial.print("motor_output (rad/s): "); Serial.println(vel);
        }
        
        last_pos = pos_now;
        last_time = now;
        
        // ---------------- Calculate error -----------------
        e[0] = setpoint - vel;
        u[0] = K1*e[0]+K2*e[1]+K3*e[2]+u[1]; // discrete-time PID difference equation
        // ---------------- Limit control signal ------------
        if(u[0]>255) u[0] = 255;
        if(u[0]<-255) u[0] = -255;
        motor_input.data = u[0];
        // ---------------- Send control signal to motors ---
        if(u[0]>=0){
          ledcWrite(PWM_CHNL0, u[0]); //
          ledcWrite(PWM_CHNL1, 0); // set to 0
        } else{
          ledcWrite(PWM_CHNL1, -u[0]); //
          ledcWrite(PWM_CHNL0, 0); // set to 0
        }
        // ---------------- Shift values --------------------
        e[2]=e[1];
        e[1]=e[0];
        u[1]=u[0];
    }
}

void publisher_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    //motor_output.data = 
    RCSOFTCHECK(rcl_publish(&motor_output_publisher, &motor_output, NULL))
    RCSOFTCHECK(rcl_publish(&motor_input_publisher, &motor_input, NULL))
  }
}

// Error callback 
void error_loop(){
  while(1){
    // Blink LED every 100 ms
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

// Subscription callback
void subscription_callback(const void * msgin)
{  
  // Type casting to Float32 and pointer assignment to setpoint_signal
  const std_msgs__msg__Float32 * setpoint_signal = (const std_msgs__msg__Float32 *)msgin;
  setpoint = setpoint_signal->data;

}

void setup() {
  set_microros_transports(); // micro ros configuration
  Serial.begin(115200);
  
  // Configure PWM
  // Set pins as output
  pinMode(PWM_PIN_IN1, OUTPUT); 
  pinMode(PWM_PIN_IN2, OUTPUT);
  // Configure pwm channel
  ledcSetup(PWM_CHNL0, PWM_FRQ, PWM_RES); 
  ledcSetup(PWM_CHNL1, PWM_FRQ, PWM_RES);
  // Assign pins
  ledcAttachPin(PWM_PIN_IN1, PWM_CHNL0);
  ledcAttachPin(PWM_PIN_IN2, PWM_CHNL1);

  // Configure encoder pins
  pinMode(ENC_A, INPUT);
  pinMode(ENC_B, INPUT);
  // Attach interrupt callback to rising edge of channel A
  attachInterrupt(ENC_A, int_callback, RISING);

  K1 = kp + T*ki + kd/T;
  K2 = -kp - 2.0*kd/T;
  K3 = kd/T; 

  // Iinitial state
  state = WAITING_AGENT;
}

void loop() {
  switch (state) {

    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;

    case AGENT_AVAILABLE:
      state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) {
        destroy_entities();
      };
      break;

    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (state == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
      }
      break;
      

    case AGENT_DISCONNECTED:
      destroy_entities();
      state = WAITING_AGENT;
      break;
      
    default:
      break;
  }
}

bool create_entities(){
  // Initialize Micro-ROS
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "motor", "RCG", &support));

  // Create motor output Publisher
  RCCHECK(rclc_publisher_init_best_effort(
    &motor_output_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "motor_output"));

  // Create motor input Publisher
  RCCHECK(rclc_publisher_init_best_effort(
    &motor_input_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "motor_input"));

  // Create setpoint Subscriber
  RCCHECK(rclc_subscription_init_default(
    &setpoint_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "set_point"));

  // Initialize Timers
  // Timer for publishing 
  const unsigned int publisher_timer_timeout = 50;  // miliseconds
  RCCHECK(rclc_timer_init_default(
      &publish_timer,
      &support,
      RCL_MS_TO_NS(publisher_timer_timeout),
      publisher_timer_callback));
  // Timer for velocity control
  const unsigned int velcontrol_timer_timeout = T*1000;
  RCCHECK(rclc_timer_init_default(
      &velcontrol_timer,
      &support,
      RCL_MS_TO_NS(velcontrol_timer_timeout),
      velcontrol_timer_callback));

  // Initialize Executor
  // create zero initialised executor (no configured) to avoid memory problems
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &setpoint_subscriber, &setpoint_signal, &subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &publish_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &velcontrol_timer));

  return true;
}

void destroy_entities()
{
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_subscription_fini(&setpoint_subscriber, &node);
  rcl_publisher_fini(&motor_output_publisher, &node);
  rcl_publisher_fini(&motor_input_publisher, &node);
  rcl_timer_fini(&publish_timer);
  rcl_timer_fini(&velcontrol_timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}