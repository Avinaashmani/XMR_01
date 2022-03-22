int motor_a1 = 7, motor_a2 = 6, motor_b1 = 5, motor_b2 = 4;
int speed_a = 8, speed_b = 9;

void setup() 
{
   Serial.begin (57600); 
   pinMode (motor_a1, OUTPUT);
   pinMode (motor_a2, OUTPUT);
   pinMode (motor_b1, OUTPUT);
   pinMode (motor_b2, OUTPUT);
   pinMode (speed_a, OUTPUT);
   pinMode (speed_b, OUTPUT);
}

void loop() 
{
  float  linear_x = 50;
  float linear = map(linear_x, 10, 100, 255, 0);
  Serial.println (map(linear_x, 0, 0.26, 0, 255));
  analogWrite (speed_a, 100);
  analogWrite (speed_b, 255);
  digitalWrite (motor_a1, LOW);
  digitalWrite (motor_a2, HIGH);
  digitalWrite (motor_b1, LOW);
  digitalWrite (motor_b2, HIGH);
}
