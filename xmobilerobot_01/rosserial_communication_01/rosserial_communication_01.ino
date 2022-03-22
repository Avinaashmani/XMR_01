#include <ros.h>
#include <geometry_msgs/Twist.h>
int motor_a1 = 7;
int motor_a2 = 6;
int motor_b1 = 5;
int motor_b2 = 4;

int speed_a = 8, speed_b = 9;

float mapped_angular = 0.0, mapped_linear = 0.0, linear_mapped = 0.0;

int encoder_a1 = 3;
int encoder_a2 = 4;

int encoder_b1 = 19;
int encoder_b2 = 18;

int led_1 = 13, led_2 = 17;
float linear = 0, angular = 0;

ros::NodeHandle node;

void motor_callback (const geometry_msgs::Twist & twist)
{
  linear = twist.linear.x;
  angular = twist.angular.z;
//
//  linear_mapped = map (linear, -0.26, 0.26, -255, 255);
//  analogWrite (speed_a, -1*(linear_mapped));
//  analogWrite (speed_b, -1*(linear_mapped));

    analogWrite (speed_a, 255);
    analogWrite (speed_b, 255);

  if (linear > 0)
  {
//    analogWrite (speed_a, 255);
//    analogWrite (speed_b, 255);
    digitalWrite (motor_a1, HIGH);
    digitalWrite (motor_a2, LOW);
    digitalWrite (motor_b1, HIGH );
    digitalWrite (motor_b2, LOW);
  }
  else if (linear < 0 )
  { 
    
//    analogWrite (speed_a, 255);
//    analogWrite (speed_b, 255);
    digitalWrite (motor_a1, LOW);
    digitalWrite (motor_a2, HIGH);
    digitalWrite (motor_b1, LOW);
    digitalWrite (motor_b2, HIGH);
  }
    else if (angular < 0)
  {
//    analogWrite (speed_a, 255);
//    analogWrite (speed_b, 255);
    digitalWrite (motor_a1, HIGH);
    digitalWrite (motor_a2, LOW);
    digitalWrite (motor_b1, LOW);
    digitalWrite (motor_b2, HIGH);
  }
  else if (angular > 0)
  {
//    analogWrite (speed_a, 255);
//    analogWrite (speed_b, 255);   
    digitalWrite (motor_a1, LOW);
    digitalWrite (motor_a2, HIGH);
    digitalWrite (motor_b1, HIGH);
    digitalWrite (motor_b2, LOW);
  }

  else
  {
    digitalWrite (motor_a1, LOW);
    digitalWrite (motor_a2, LOW);
    digitalWrite (motor_b1, LOW);
    digitalWrite (motor_b2, LOW);
  }
}

ros::Subscriber <geometry_msgs::Twist> motor_sub ("/cmd_vel", motor_callback);

void led_blink ()
{
  digitalWrite (led_2, HIGH);
  digitalWrite (led_1, LOW);
  delay (250);
  digitalWrite (led_2, LOW);
  digitalWrite (led_1, HIGH);
  delay (250);
}

void setup() 
{
  Serial.begin (57600);
  pinMode (motor_a1, OUTPUT);
  pinMode (motor_a2, OUTPUT);
  pinMode (motor_b1, OUTPUT);
  pinMode (motor_b2, OUTPUT);
  pinMode (speed_a, OUTPUT);
  pinMode (speed_b, OUTPUT);
  pinMode (led_1, OUTPUT);
  pinMode (led_2, OUTPUT);
  
  node.initNode();
  node.subscribe (motor_sub);
  
}

void loop() {
  node.spinOnce();
  led_blink();

}
