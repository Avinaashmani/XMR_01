#include <ros.h>
#include <ros/time.h>

#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>

// pin 9 is ENB for Right Motor
// pin 8 in ENB for Left Motor

int motor_a1 = 7, motor_a2 = 6, motor_b1 = 5, motor_b2 = 4;
int speed_a = 8, speed_b = 9;

int case_flag ;


float linear = 0.0, angular = 0.0;
double wheel_sep = 0.0285, wheel_radius = 0.0325;

double wheel_left_speed = 0, wheel_right_speed = 0;

void motor_init();
void wheel_left (int pwm_2);
void wheel_right (int pwm_1);
void arc (int pwm_r, int pwm_l);

ros::NodeHandle node;

std_msgs::Float32 right_speed;
std_msgs::Float32 left_speed;

ros::Publisher speed_left ("/wheel_speed/right", &right_speed);
ros::Publisher speed_right("wheel_speed/left", &left_speed);

void motor_callback (geometry_msgs::Twist &twist)
{
  linear = twist.linear.x;
  angular = twist.angular.z;
  
  wheel_left_speed = (linear/wheel_sep) + ((angular/wheel_sep) / (2.0 * wheel_radius));
  wheel_right_speed =(linear/wheel_sep) - ((angular/wheel_sep) / (2.0 * wheel_radius));
  
  if (linear == 0 && angular != 0 )
  {
    case_flag = 1;
  }
  else if (linear !=0 && angular ==0)
  {
    case_flag = 2;
  }
  else if (linear !=0 && angular != 0)
  {
    case_flag = 0;
  } 
}

ros::Subscriber <geometry_msgs::Twist> motor_sub ("/cmd_vel", motor_callback);

void setup() 
{
  Serial.begin (57600);

  motor_init();
  node.initNode ();
  node.subscribe (motor_sub);
  node.advertise (speed_left);
  node.advertise (speed_right);
}

void loop() 
{
  
  if (case_flag ==1)
  {
    wheel_left (wheel_left_speed * 10);
    wheel_right (wheel_right_speed * 10);
  }
  else if (case_flag == 2)
  {
    wheel_left (wheel_left_speed * 10);
    wheel_right (wheel_right_speed * 10);    
  }
  else if (case_flag == 0)
  {
    arc (wheel_right_speed * 10, wheel_left_speed * 10);
  }

  right_speed.data = wheel_right_speed;
  left_speed.data = wheel_left_speed;

  speed_left.publish(&left_speed);
  speed_right.publish(&right_speed);
  node.spinOnce();
}

void motor_init()
{
  pinMode (motor_a1, OUTPUT);
  pinMode (motor_a2, OUTPUT);
  pinMode (motor_b1, OUTPUT);
  pinMode (motor_b2, OUTPUT);

  pinMode (speed_a, OUTPUT);
  pinMode (speed_b, OUTPUT);  
}

void wheel_right (int pwm_1)
{
  if (pwm_1 > 0)
  {
    analogWrite (speed_b, pwm_1);
    digitalWrite (motor_a1, HIGH);
    digitalWrite (motor_a2, LOW); 
  }

  else if (pwm_1 < 0)
  {
    analogWrite (speed_b, abs(pwm_1));
    digitalWrite (motor_a1, LOW);
    digitalWrite (motor_a2, HIGH);
  }

  
  else if (pwm_1 == 0)
  {
    analogWrite (speed_b, pwm_1);
    digitalWrite (motor_a1, LOW);
    digitalWrite (motor_a2, LOW);
  }
  
}

void wheel_left (int pwm_2)
{
  if (pwm_2 > 0)
  {
    analogWrite (speed_a, pwm_2);
    digitalWrite (motor_b1, HIGH);
    digitalWrite (motor_b2, LOW);
  }
  else if (pwm_2 < 0)
  {
    analogWrite (speed_a, abs(pwm_2));
    digitalWrite (motor_b1, LOW);
    digitalWrite (motor_b2, HIGH);
  }

  else if (pwm_2 == 0 )
  {
    analogWrite (speed_a, pwm_2);
    digitalWrite (motor_b1, LOW);
    digitalWrite (motor_b2, LOW);
  }
  
 }

void arc (int pwm_r, int pwm_l)
{
  if (linear>0 && angular <0)
  {
    analogWrite (speed_a, abs(pwm_r));
    analogWrite (speed_b, abs(pwm_l));
    digitalWrite (motor_a1, HIGH); digitalWrite (motor_a2, LOW);
    digitalWrite (motor_b1, HIGH); digitalWrite (motor_b2, LOW);
  }

  else if (linear>0 && angular>0)
  {
    analogWrite (speed_a, abs(pwm_r));
    analogWrite (speed_b, abs(pwm_l));
    digitalWrite (motor_a1, HIGH); digitalWrite (motor_a2, LOW);
    digitalWrite (motor_b1, HIGH); digitalWrite (motor_b2, LOW);    
  }

  else if (linear <0 && angular>0)
  {
    analogWrite (speed_a, abs(pwm_r));
    analogWrite (speed_b, abs(pwm_l));
    digitalWrite (motor_a1, LOW); digitalWrite (motor_a2, HIGH);
    digitalWrite (motor_b1, LOW); digitalWrite (motor_b2, HIGH);
  }
  else if (linear<0 && angular<0)
  {
    analogWrite (speed_a, abs(pwm_r));
    analogWrite (speed_b, abs(pwm_l));
    digitalWrite (motor_a1, LOW); digitalWrite (motor_a2, HIGH);
    digitalWrite (motor_b1, LOW); digitalWrite (motor_b2, HIGH);
  }
  else 
  {
    analogWrite (speed_a, abs (pwm_r));
    analogWrite (speed_b, abs (pwm_l));
    digitalWrite (motor_a1, LOW); digitalWrite (motor_a2, LOW);
    digitalWrite (motor_b1, LOW); digitalWrite (motor_b2, LOW);
   
  }
}
