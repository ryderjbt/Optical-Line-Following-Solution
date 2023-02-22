#include <Wire.h>

#define I2C_SLAVE_ADDR 0x04
// 4 in hexadecimal
#define servoPin 6

#define IRSa 12 
#define IRSb 14  
#define IRSc 27 
#define IRSd 26 
#define IRSe 25 
#define IRSf 33  
// Defines pins for each sensor
int leftMotor_forward;
int leftMotor_backwards = -115;
int rightMotor_forward;
int rightMotor_backwards = -115;
int servoAngle_straight = 60;
int servoAngle;
// Initialises integer variables for motor speeds and steering angle
float weightedAverage;
// Initalises weighted average variable essential to PID
float X0 = 9.0, X1 = 29.0, X2 = 39.0, X3 = 55.0, X4 = 70.0, X5 = 85.0;
// Initialises distances from a reference point (far left of sensor array) for each sensor
float derivative = 0, integral = 0;
// Initalises differntial and integral variables used in PID
float error = 0, lastError = 0;
// Initalises error (EEEBot's distance from centre) and
//lastError which is tracked to predict movement using derivative
float k = 0.4;
// Initalises k, a value adjusted depending on the situation
//to produce accurate adjustments to motor speeds and servo angle
long lastTime = 0;
// Initalises lastTime, keeps track of the previous time measured to find
//change in time between PID function being called, used to predict movement using derivative
long difference;
// Initalises difference, stores time passed since previous PID call
//used to find integral which increases if the EEEBot has not been able to turn

void setup()
{
  pinMode(IRSa, INPUT);
  pinMode(IRSb, INPUT);
  pinMode(IRSc, INPUT);
  pinMode(IRSd, INPUT);
  pinMode(IRSe, INPUT);
  pinMode(IRSf, INPUT);
// Sets the pins conected to the sensor to input mode to recieve the signals from the sensors
  Serial.begin(9600);
  Wire.begin();
// join i2c bus (address optional for the master) - on the Arduino NANO
//the default I2C pins are A4 (SDA), A5 (SCL)
}

void loop()
{ // Line following solution main loop, uses PID to control movement absed on line,
  //if the EEEBot loses the line it will reverse until the line has been found again
  float S0, S1, S2, S3, S4, S5;
  // Initalises variables to hold sensor values
  float u;
  // Initalises variable to hold adjustment value from PID
  S0 = analogRead(IRSa);
  S1 = analogRead(IRSb);
  S2 = analogRead(IRSc);
  S3 = analogRead(IRSd);
  S4 = analogRead(IRSe);
  S5 = analogRead(IRSf);
  // Reads the value of each sensor
  weightedAverage = (((S0 * X0) + (S1 * X1) + (S2 * X2) + (S3 * X3) + (S4 * X4) + (S5 * X5)) / (S0 + S1 + S2 + S3 + S4 + S5));
  // Calculates weighted average by dividng the sum of the products of sensors and their distance from the far left point of the array
  u = PID(weightedAverage);
  // Calls PID function and stores value returned (adjustment value) as u
  servoAngle = servoAngle_straight + u;
  // Adjusts the angle of the servo depending on u to turn towards the line
  leftMotor_forward  = 115 + (k * u);
  // Adjusts left motor speed using the product of u and calibrated value k
  //to accurately alter speed
  rightMotor_forward  = 115 - (k * u);
  // Adjusts right motor speed using the product of u and calibrated value k
  //done via subtraction as when turning right or left the motors most speed up and slow down inversely
  if ((S0 + S1 + S2 + S3 + S4 + S5) == (24570))
  {
    // If loop checks total value of sensors to determine if they have lost the line
    //if line is lost (sensor total == 24750) the EEEBot reverses until the line is found
    backwards();
    delay(300);
  }
  else;
  {
    // Else the line can be seen the EEEBot drives forwards at speeds and angles determined by PID
    forwards();
    delay(100);
  }
}

void forwards()
{ // Function transmits motor & steering values to slave arduino to make EEEBot go forward
  //speed of motors and servo angle determined using PID
  Wire.beginTransmission(I2C_SLAVE_ADDR);
  Wire.write((byte)((leftMotor_forward & 0x0000FF00) >> 8));
  Wire.write((byte)(leftMotor_forward & 0x000000FF));
  Wire.write((byte)((rightMotor_forward & 0x0000FF00) >> 8));
  Wire.write((byte)(rightMotor_forward & 0x000000FF));
  Wire.write((byte)((servoAngle & 0x0000FF00) >> 8));
  Wire.write((byte)(servoAngle & 0x000000FF));
  Wire.endTransmission();
}

void backwards() 
{// Fucntion transmits motor & steering values to slave arduino to make EEEBot go straight backward
  Wire.beginTransmission(I2C_SLAVE_ADDR);
  Wire.write((byte)((leftMotor_backwards & 0x0000FF00) >> 8));
  Wire.write((byte)(leftMotor_backwards & 0x000000FF));
  Wire.write((byte)((rightMotor_backwards & 0x0000FF00) >> 8));
  Wire.write((byte)(rightMotor_backwards & 0x000000FF));
  Wire.write((byte)((servoAngle_straight & 0x0000FF00) >> 8));
  Wire.write((byte)(servoAngle_straight & 0x000000FF));
  Wire.endTransmission();
}

float PID(float weightedAverage)
{
  // Function used to implement PID control scheme by calculating the value
  //the EEEBot must be adjusted by and returning it to the main loop
  float currentTime = millis();
  // Initalises local variable to keep track of time when the function was called
  float Kp = 25 , Ki = 0.01, Kd = 3;
  // Initalises gain factors used to tune expected results from values 
  float  u;
  // Initlaises local variable for adjustment value found
  difference = (currentTime - lastTime);
  // Finds the difference in time between the current and previous call of the function
  error = (47.83333333 - weightedAverage);
  // Calculates the error by subtracting the weighted average of where the line lies in
  //relation to the EEEBot from the centre's distance from the array's leftmost side
  integral = (integral + (difference * error));
  // Calculates the integral using the value of previous integrals added
  //to the product of the time difference and current error
  //integral value increases in influence as PID is called without
  //significant change to the error i.e. stuck
  derivative = ((error - lastError) / difference);
  // Calculates derivative by dividing the difference between the current
  //and previous error by the difference between current and previous time
  lastError = error;
  //Sets lastError to the current error for the next time the function is called
  lastTime = currentTime;
  //Sets lastTime to the current time for the next time the function is called
  u = ((Kp * error) + (Ki * integral) + (Kd * derivative));
  // Calculates u by summing the products of the calculated values for
  //error, integral and derivative (PID) by their respective gain factors
  return (u);
  //Returns u (adjustment value)
}