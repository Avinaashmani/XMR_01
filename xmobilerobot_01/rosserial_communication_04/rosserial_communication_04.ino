#include <ros.h>
#include <ros/time.h>

#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <geometry_msgs/Quaternion.h>
// pin 9 is ENB for Right Motor
// pin 8 in ENB for Left Motor

int motor_a1 = 7, motor_a2 = 6, motor_b1 = 5, motor_b2 = 4;
int speed_a = 8, speed_b = 9;

int case_flag ;

ros::NodeHandle node;
std_msgs::Float64 speed_left;
std_msgs::Float64 speed_right;
nav_msgs::Odometry odom;
tf::TransformBroadcaster odom_broadcaster;

ros::Publisher odom_pub ("/odom", &odom); 

float linear = 0.0, angular = 0.0;
double wheel_sep = 0.0285, wheel_radius = 0.0325;

double wheel_left_speed = 0, wheel_right_speed = 0;

void motor_init();
void wheel_left (int pwm_2);
void wheel_right (int pwm_1);
void arc (int pwm_r, int pwm_l);


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

//ros::Time last_time;



double th = 0.0, x = 0.0, y = 0.0;
ros::Subscriber <geometry_msgs::Twist> motor_sub ("/cmd_vel", motor_callback);
double last_time = 0.0;
double current_time = 0.0;
void setup() 
{
  Serial.begin (57600);
  current_time = (node.now()).toSec();
  last_time = (node.now()).toSec();
  motor_init();
  odom_broadcaster.init(node);
  node.initNode ();
  node.subscribe (motor_sub);
  node.advertise (odom_pub);
}

void loop() 
{
current_time = (node.now()).toSec();
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

double vx = ((wheel_right_speed  + wheel_left_speed ) / 2)*10;
double vy = 0.0;
double vth = ((wheel_right_speed - wheel_left_speed)/ wheel_sep)*10; 

double  dt = (current_time - last_time);
double  delta_x = (vx * cos(th))/100000;
double  delta_y = (vx * sin(th))/100000;
double  delta_th = (vth )/1000000;

x += delta_x;
y += delta_y;
th+= delta_th;


geometry_msgs::TransformStamped odom_trans;
odom_trans.header.stamp = node.now();
odom_trans.header.frame_id = "odom";
odom_trans.child_frame_id = "base_link";

odom_trans.transform.translation.x = x;
odom_trans.transform.translation.y = y;
odom_trans.transform.translation.z = 0.0;
odom_trans.transform.rotation = tf::createQuaternionFromYaw(th);

// send the transform
odom_broadcaster.sendTransform(odom_trans);

// Odometry message

odom.header.stamp = node.now();
odom.header.frame_id = "odom";

// set the position
odom.pose.pose.position.x = x;
odom.pose.pose.position.y = y;
odom.pose.pose.position.z = 0.0;
odom.pose.pose.orientation = tf::createQuaternionFromYaw(th);

// set the velocity
odom.child_frame_id = "base_link";
odom.twist.twist.linear.x = vx;
odom.twist.twist.linear.y = vy;
odom.twist.twist.angular.z = vth;
//odom_broadcaster.sendTransform(odom_trans);
odom_pub.publish(&odom);
node.spinOnce();
last_time = current_time;
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

//void set_pose_transalation()
//{
//double  dt = (current_time - last_time);
//double  delta_x = (vx * cos(th)) * dt;
//double  delta_y = (vx * sin(th)) * dt;
//double  delta_th = vth * dt;
//
//x += delta_x;
//y += delta_y;
//th+= delta_th;
//
//
//geometry_msgs::TransformStamped odom_trans;
//odom_trans.header.stamp = node.now();
//odom_trans.header.frame_id = "odom";
//odom_trans.child_frame_id = "base_link";
//
//odom_trans.transform.translation.x = x;
//odom_trans.transform.translation.y = y;
//odom_trans.transform.translation.z = 0.0;
//odom_trans.transform.rotation = tf::createQuaternionFromYaw(th);
//
//// send the transform
//odom_broadcaster.sendTransform(odom_trans);
//
//// Odometry message
//
//odom.header.stamp = node.now();
//odom.header.frame_id = "odom";
//
//// set the position
//odom.pose.pose.position.x = x;
//odom.pose.pose.position.y = y;
//odom.pose.pose.position.z = 0.0;
//odom.pose.pose.orientation = tf::createQuaternionFromYaw(th);
//
//// set the velocity
//odom.child_frame_id = "base_link";
//odom.twist.twist.linear.x = vx;
//odom.twist.twist.linear.y = vy;
//odom.twist.twist.angular.z = vth;
//}
