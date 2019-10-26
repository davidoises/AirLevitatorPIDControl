/*
 * @file LevitadorPID-SineSetpoint
 * 
 * This is used to control the position of a ball in an air levitator with a tuned PID.
 * Setpoint and measured values calculated in centimeters using digital time-of-flight sensor VL53L0X
 * Setpoint is fixed to the function: 30*(sin(x/6)+1) + 10; where x increases over time every 300ms
 * Initially setpoint is fixed on 39cm and after receiving a 1 through serial interfaces it changes to the sine function
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
double mapped_distance = 0;
double distance = 0;
int set_point = 39;
bool first = 1;
unsigned long initial_time = 0;
double x = -10;
bool ascending = 1;

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
  delay(1000);
  prev_time = millis();
}

void loop()
{

  // Serial check to receive just a "1". "1" starts the sine wave setpoint
  if(Serial.available())
  {
    int temp = Serial.parseInt();
    if(temp == 1)
      initial_time = 1;
  }
  
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

    // Actual values printing
    Serial.print(distance);
    Serial.print(" ");
    Serial.println(set_point);

    // Sample time; Mainly defined due to sensor capabilities
    delay(50);

    // Sine wave generation. Independant variable increases every 300ms
    if((millis() - initial_time) > 300 && initial_time != 0)
    {
      initial_time = millis();
      set_point = 30*(sin(x/6)+1) + 10; 
      if(ascending)
      {
        x++;
        if(x>10)
        {
          ascending = 0;
        }
      }
      if(!ascending)
      {
        x--;
        if(x<-10)
        {
          ascending = 1;
        }
      }
    }
  }
  else
  {
    delay(10);
  }
}
