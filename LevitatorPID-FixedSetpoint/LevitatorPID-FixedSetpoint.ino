/*
 * @file LevitadorPID-FixedSetpoint
 * 
 * This is used to control the position of a ball in an air levitator with a tuned PID.
 * Setpoint and measured values calculated in centimeters using digital time-of-flight sensor VL53L0X
 * Setpoint can be changed through the serial com interface. Initial setpoint fixed to 30cm and afterward it is modified writing the number in cm through COM port
 *
 * @author [DDM]
 * @version  V1.0
 * @date  2019-10-20
 * @https://github.com/davidoises/AirLevitatorPIDControl
 */
#include "Arduino.h"
#include "Wire.h"
#include "DFRobot_VL53L0X.h"

/* Distance/setpoint and sensor related variables */
DFRobotVL53L0X sensor;
double distance = 0;
int set_point = 30;
bool first = 1;

/* PWM setup for requency, HW channel and resolution */
const int freq = 100;
const int ledChannel = 0;
const int resolution = 8;

/* PID related variables */
double error = 0;
double prev_integral = 0;
double prev_error = 0;
double pwm_control = 0;
unsigned long prev_time = 0;

void setup() {

  // UART baudrate and setup
  Serial.begin(115200);

  // Sensor init including I2C bus setup
  Wire.begin();
  sensor.begin(0x50);
  sensor.setMode(Continuous,Low);
  sensor.start();
  
  //PWM setup
  ledcSetup(ledChannel, freq, resolution);
  ledcAttachPin(4, ledChannel);
  ledcWrite(ledChannel, 0);

  // Delay so that the fan runs for a bit before starting the controller
  delay(4000);
  prev_time = millis();
}

void loop()
{

  // Serial check to receive new setpoint
  if(Serial.available())
  {
    int temp = Serial.parseInt();
    if(temp != 0)
    {
      set_point = temp;
    }
  }
  
  //Get the distance
  double temp = sensor.getDistance()/10;

  // Filter garbage values (I2C throughs FF in all bytes) by setting the previous distance insted of the garbage reading
  if(temp > 6000)
    temp = distance;
  // Avoid filter only for the first reading
  else if(first)
  {
    distance = temp;
    first = 0;
  }

  // Filter: if new distance is less than possible maximum (95) and the difference between current and previous measurement is small ( <20) move forward
  if(temp < 95 && abs(distance-temp)<20)
  {
    distance = temp;  

    // Sample time calculation
    unsigned long current_time = millis();
    double delta_time = ((double)current_time - (double)prev_time)/1000;

    // PID constants
    double kp = 170;
    double ki = 15;
    double kd = 17;

    // PID calculation
    error = (double)distance - set_point;
    double integral = (((double)delta_time)/2)*(error + prev_error)+ prev_integral;
    double diff = (error - prev_error)/((double)delta_time);
    pwm_control = kp*error + ki*integral + kd*diff;

    // Variables setup for next PID contorl law calculation
    prev_time = current_time;
    prev_error = error;
    prev_integral = integral;

    // Actual output signal from PID  control law
    int pwm = constrain(pwm_control, 0, 1023);
    pwm = map(pwm, 0, 1023, 0, 255);
    ledcWrite(ledChannel, pwm);
  
    // Actual values printing (including actual measure of the sample time)
    Serial.print(distance);
    Serial.print(" ");
    Serial.print(set_point);
    Serial.print(" ");
    Serial.println(delta_time);
    
    // Sample time; Mainly defined due to sensor capabilities
    delay(50);
  }
  else
  {
    delay(10);
  }
}
