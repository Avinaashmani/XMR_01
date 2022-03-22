#include <ros.h>
#include <geometry_msgs/Twist.h>
int motor_a1 = 7;
int motor_a2 = 6;
int motor_b1 = 5;
int motor_b2 = 4;

int speed_a = 8, speed_b = 9;

float angular_mapped_negative = 0.0, linear_mapped_negative = 0.0, angular_mapped_positive = 0.0, linear_mapped_positive = 0.0, linear_mapped = 0.0, angular_mapped=0.0;

int encoder_a1 = 3;
int encoder_a2 = 4;

int encoder_b1 = 19;
int encoder_b2 = 18;

float linear = 0, angular = 0;
float wheel_radius = 0.0650, wheel_seperation = 0.0286;
float wheel_left = 0.0, wheel_right = 0.0;

ros::NodeHandle node;

void motor_callback (const geometry_msgs::Twist & twist)
{
  linear = twist.linear.x;
  angular = twist.angular.z;

  wheel_left = (linear/wheel_radius) + ((angular*wheel_seperation)/(2.0*wheel_radius));
  wheel_right = (linear/wheel_radius) - ((angular*wheel_seperation)/(2.0*wheel_radius)); 

}
ros::Subscriber <geometry_msgs::Twist> motor_sub ("/cmd_vel", motor_callback);


void setup() 
{
  Serial.begin (57600);
  pinMode (motor_a1, OUTPUT);
  pinMode (motor_a2, OUTPUT);
  pinMode (motor_b1, OUTPUT);
  pinMode (motor_b2, OUTPUT);
  
  node.initNode();
  node.subscribe (motor_sub);
  
}

void loop() {
  node.spinOnce();
  analogWrite (speed_a, wheel_left);
  analogWrite (speed_b, wheel_right);

  
  digitalWrite (motor_a1, HIGH);
  digitalWrite (motor_a2, LOW);
  digitalWrite (motor_b1, HIGH);
  digitalWrite (motor_b2, LOW);

}
