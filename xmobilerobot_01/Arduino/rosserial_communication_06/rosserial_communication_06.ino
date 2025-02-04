#include <ros.h>
#include <ros/time.h>

#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>

// pin 9 is ENB for Right Motor
// pin 8 in ENB for Left Motor

int motor_a1 = 7, motor_a2 = 6, motor_b1 = 5, motor_b2 = 4;
int speed_a = 8, speed_b = 9;

volatile int position_a = 0;
volatile int position_b = 0;

float cpr_rev = 44.0;

int case_flag ;

int enc_a1 = 3;
int enc_a2 = 2;

int enc_b1 = 18;
int enc_b2 = 19;

float linear = 0.0, angular = 0.0;
double wheel_sep = 0.0285, wheel_radius = 0.0325;

double wheel_left_speed = 0, wheel_right_speed = 0;

unsigned long old_time = 0;
unsigned long new_time = 0;
unsigned long delta_t = 0;

void motor_init();
void wheel_left (int pwm_2);
void wheel_right (int pwm_1);
void arc (int pwm_r, int pwm_l);
void calculate_velocities();

float linear_x_speed = 0.0, angular_z_speed = 0.0;
float linear_speed_left = 0.0, linear_speed_right = 0.0;
volatile float left_vel = 0.0, right_vel = 0.0;
ros::NodeHandle node;

std_msgs::Float32 right_speed;
std_msgs::Float32 left_speed;

std_msgs::Int16 enc_a;
std_msgs::Int16 enc_b;

ros::Publisher speed_left ("/wheel_speed/right", &right_speed);
ros::Publisher speed_right("wheel_speed/left", &left_speed);

ros::Publisher encoder_a ("/encoder_data/right", &enc_a);
ros::Publisher encoder_b ("/encoder_data/left", &enc_b);

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
  pinMode (enc_a1, INPUT);
  pinMode (enc_a2, INPUT);
  pinMode (enc_b1, INPUT);
  pinMode (enc_b2, INPUT);

  old_time = millis();
  
  motor_init();
  node.initNode ();
  node.subscribe (motor_sub);
  node.advertise (speed_left);
  node.advertise (speed_right);

  node.advertise (encoder_a);
  node.advertise (encoder_b);
  
  attachInterrupt (digitalPinToInterrupt (enc_a1), read_encoder_a, RISING);
  attachInterrupt (digitalPinToInterrupt (enc_b1), read_encoder_b, RISING);
}

void loop() 
{
  // new_time = millis();
  if (case_flag ==1)
  {
    wheel_left (wheel_left_speed * 10);
    wheel_right (wheel_right_speed * 10);
    new_time = millis();
  }
  else if (case_flag == 2)
  {
    new_time = millis();
    wheel_left (wheel_left_speed * 10);
    wheel_right (wheel_right_speed * 10);    
  }
  else if (case_flag == 0)
  {
    new_time = millis();
    arc (wheel_right_speed * 10, wheel_left_speed * 10);
  }
  
  linear_x_speed = (left_vel + right_vel /2);
  angular_z_speed = (left_vel - right_vel / wheel_sep);

  linear_speed_right = ((2.0 * linear_x_speed) - (wheel_sep * angular_z_speed)) ;
  linear_speed_left = ((2.0 * linear_x_speed) + (wheel_sep * angular_z_speed)) ;
    
  right_speed.data = linear_speed_right / (new_time - old_time);
  left_speed.data = linear_speed_left / (new_time - old_time);

  enc_a.data = position_a/100;
  enc_b.data = position_b/100; 
 
  speed_left.publish(&left_speed);
  speed_right.publish(&right_speed);

  encoder_a.publish (&enc_a);
  encoder_b.publish (&enc_b);
  
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
    right_vel = position_a / cpr_rev;
    left_vel = position_b / cpr_rev;
  }

  else if (pwm_1 < 0)
  {
    analogWrite (speed_b, abs(pwm_1));
    digitalWrite (motor_a1, LOW);
    digitalWrite (motor_a2, HIGH);
    right_vel = position_a / 44;
    left_vel = position_b / cpr_rev;
  }

  
  else if (pwm_1 == 0)
  {
    analogWrite (speed_b, pwm_1);
    digitalWrite (motor_a1, LOW);
    digitalWrite (motor_a2, LOW);
    right_vel = 0.0;
    left_vel = 0.0;
    position_a = 0;
    position_b = 0;
  }
  
}

void wheel_left (int pwm_2)
{
  if (pwm_2 > 0)
  {
    analogWrite (speed_a, pwm_2);
    digitalWrite (motor_b1, HIGH);
    digitalWrite (motor_b2, LOW);
    right_vel = position_a / 44;
    left_vel = position_b / 44;
  }
  else if (pwm_2 < 0)
  {
    analogWrite (speed_a, abs(pwm_2));
    digitalWrite (motor_b1, LOW);
    digitalWrite (motor_b2, HIGH);
    right_vel = position_a / 44;
    left_vel = position_b / 44;
  }

  else if (pwm_2 == 0 )
  {
    analogWrite (speed_a, pwm_2);
    digitalWrite (motor_b1, LOW);
    digitalWrite (motor_b2, LOW);
   right_vel = 0.0;
   left_vel = 0.0;
   position_a = 0;
   position_b = 0;
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
    right_vel = position_a / cpr_rev;
    left_vel = position_b / cpr_rev;
  }

  else if (linear>0 && angular>0)
  {
    analogWrite (speed_a, abs(pwm_r));
    analogWrite (speed_b, abs(pwm_l));
    digitalWrite (motor_a1, HIGH); digitalWrite (motor_a2, LOW);
    digitalWrite (motor_b1, HIGH); digitalWrite (motor_b2, LOW);    
    right_vel = position_a / cpr_rev;
    left_vel = position_b / cpr_rev;
  }

  else if (linear <0 && angular>0)
  {
    analogWrite (speed_a, abs(pwm_r));
    analogWrite (speed_b, abs(pwm_l));
    digitalWrite (motor_a1, LOW); digitalWrite (motor_a2, HIGH);
    digitalWrite (motor_b1, LOW); digitalWrite (motor_b2, HIGH);
    right_vel = position_a / cpr_rev;
    left_vel = position_b / cpr_rev;
  }
  else if (linear<0 && angular<0)
  {
    right_vel = position_a / cpr_rev;
    left_vel = position_b / cpr_rev;
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
    right_vel = 0.0;
    left_vel = 0.0;
    position_a = 0;
    position_b = 0;
   
  }
}

void read_encoder_a ()
{
  int i = digitalRead(enc_a2);
  
  if (i > 0)
  { 
    position_a--;
  }
  else
  {
    position_a++;
  }
}

void read_encoder_b()
{
  int j = digitalRead (enc_b2); 
  
  if (j >0)
  {
    position_b--;
    
  }
  else
  {
    position_b++;
    
  }
}
