
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/bool.h>

#define IR_SENSOR_PIN 4  // Use the digital pin D4

rcl_publisher_t publisher;
std_msgs__msg__Bool msg;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;
rclc_executor_t executor;

unsigned long timer_timeout = 100; // 100 ms timer for sending messages

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    // Read the IR sensor
    bool object_detected = digitalRead(IR_SENSOR_PIN) == LOW; // Assuming LOW when an object is detected

    // Publish the sensor state
    msg.data = object_detected;
    rcl_publish(&publisher, &msg, NULL);
  }
}


const int IN1 = 33;
const int IN2 = 25;
const int ENA = 26;
const int IN3 = 27;
const int IN4 = 14;
const int ENB = 12;
const int IROP=4;

const int speed= 255;

void setup() {
  // put your setup code here, to run once:

  pinMode(IR_SENSOR_PIN, INPUT);
  set_microros_transports();

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);

  rclc_node_init_default(&node, "esp32_node", "", &support);

  rclc_publisher_init_default(&publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "ir_sensor_topic");

  rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_timeout), timer_callback);

  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_timer(&executor, &timer);
pinMode(IROP, INPUT);
pinMode (IN1, OUTPUT);
 pinMode (IN2, OUTPUT);
 pinMode (ENA, OUTPUT);

 pinMode (IN3, OUTPUT);
 pinMode (IN4, OUTPUT);
 pinMode (ENB, OUTPUT);

 stopMotors();
}

void loop() {
  // put your main code here, to run repeatedly:
// Handle Micro-ROS communications
  
int sensorStatus = digitalRead(IROP);
if (sensorStatus == 0){
  moveForward();
  Serial.println("Moving forward");
}
else{
  stopMotors();
  Serial.println("Stopped Moving");

}
rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
  delay(100);


}

void moveForward()
{
 digitalWrite(IN1, HIGH);
 digitalWrite(IN2,LOW);
 analogWrite(ENA, speed);

 digitalWrite(IN3, HIGH);
 digitalWrite(IN4,LOW);
 analogWrite(ENB, speed);
}

void stopMotors()
{
  digitalWrite(IN1, LOW);
 digitalWrite(IN2,LOW);
 analogWrite(ENA, 0);

 digitalWrite(IN3, LOW);
 digitalWrite(IN4,LOW);
 analogWrite(ENB, 0);
}

