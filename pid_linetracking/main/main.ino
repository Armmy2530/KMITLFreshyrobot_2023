#include <motor_esp32.h>
#include <ESP32Servo.h>
#include <analogWrite.h>
#include <PS4Controller.h>
#include <Ramp.h>

enum
{
  HOME,
  STARTING,
  RUN
} state = HOME;
enum
{
  CENTER,
  RIGHT,
  LEFT
} lastline_state = CENTER;

// Motor L
#define m2a 12
#define m2b 14
#define pwm2 13
// Motor R
#define m1a 27
#define m1b 26
#define pwm1 25

// Motor parameters
#define power_percentage 100.0

Motor L_motor(m1a, m1b, pwm1);
Motor R_motor(m2a, m2b, pwm2);

class Interpolation
{
public:
  rampInt myRamp;
  int interpolationFlag = 0;
  int savedValue;

  int go(int input, int duration)
  {

    if (input != savedValue)
    { // check for new data
      interpolationFlag = 0;
    }
    savedValue = input; // bookmark the old value

    if (interpolationFlag == 0)
    {                                                  // only do it once until the flag is reset
      myRamp.go(input, duration, LINEAR, ONCEFORWARD); // start interpolation (value to go to, duration)
      interpolationFlag = 1;
    }

    int output = myRamp.update();
    return output;
  }

  int update()
  {
    return (int)myRamp.update();
  }
};

Interpolation interp_motor_L;
Interpolation interp_motor_R;

// sensor_pin
#define L2_pin 33
#define L1_pin 32
#define C_pin 35
#define R1_pin 34
#define R2_pin 39

// Use for compare the analog value to digital value (B = analog > ref , W = analog <= ref)
int ref_sensor[5] = {3000, 3000, 3000, 3000, 3000};

// Sensor variable
boolean sensor_bool[5] = {0, 0, 0, 0, 0};
int sensor_val[5] = {1, 1, 1, 1, 1};

// Motor&PID parameter
int baseSpeed = 160;
int maxSpeed = 180;
float pid1_parameter[3] = {16, 15, 0};       // Pi 0.0000001
float pid_forward_parameter[3] = {10, 5, 0}; // {8.5, 10, 0.001}

bool debug = true;

struct Remote
{
  int L_X;
  int L_Y;
  int R_X;
  int R_Y;
  boolean L1;
  boolean R1;
  int L2;
  int R2;
  boolean L3;
  boolean R3;
  boolean Option;
  boolean Share;
  boolean Up;
  boolean Down;
  boolean Left;
  boolean Right;
  boolean Square;
  boolean Cross;
  boolean Circle;
  boolean Triangle;
};

Remote controller_data;

void setup()
{
  pinMode(L2_pin, INPUT);
  pinMode(L1_pin, INPUT);
  pinMode(C_pin, INPUT);
  pinMode(R1_pin, INPUT);
  pinMode(R2_pin, INPUT);
  Serial.begin(115200);
  int pev_ms = millis();
  int current_ms = millis();

  delay(500);

  // while (true)
  // {
  //   /* code */
  //   readSensor();
  //   trackline_pid_nooutline(pid_forward_parameter, 145);
  //   delay(10);
  // }

  // ~~~~~~~~~~ start the mission ~~~~~~~~~~~~~

  // start to node 1 and turn right to zig-zag road
  trackline_Cross(pid1_parameter, baseSpeed, 1, 50, 80);
  tr_sensor(100);

  // ____________________ Mk1 ____________________
  // _____ Start the zig zag road mission _____
  // node 1 -> 2 -> 3
  // trackline_L(pid1_parameter, baseSpeed, 1, 50, 80);
  // tl_sensor(100);

  // trackline_outline(pid1_parameter, baseSpeed, 50, 80);
  // tl_sensor(100);

  trackline_duration(pid1_parameter, baseSpeed, 1000, 0, 0,false);
  // delay(500);
  trackline_duration_nooutline(pid_forward_parameter, baseSpeed, 1000, 0, 0,false);
  // delay(500);
  trackline_duration(pid1_parameter, baseSpeed, 1050, 0, 0,false);
  // delay(500);
  // heading_center(70,0);
  // node 3 -> 4 and turn left to middle road
  trackline_L(pid1_parameter, 110, 1, 50, 80);
  // __________ END __________

  // at node 4  turn left
  tl_sensor(100);

  // _____ Start the middle road (blank road) mission _____
  // cross the cross line from node 4 -> 1
  trackline_Cross(pid1_parameter, baseSpeed, 1, baseSpeed, 200);
  trackline_Cross(pid1_parameter, baseSpeed, 1, baseSpeed, 200);
  trackline_Cross(pid1_parameter, 110, 1, 0, 0);
  // __________ END __________

  // at node 1  turn right -> node 5
  tr_sensor(100);

  // _____ Start the upper road (obstacle) mission _____
  // from node 1 -> 5 and turn right
  // trackline_R(pid1_parameter, 120, 1, 50, 100);
  trackline_outline(pid1_parameter, baseSpeed, 0, 0);
  tr_sensor(100);
  // node 5 -> 6 : go through the obstacle

  // Mk1
  // trackline_R(pid1_parameter, 180, 3, 155, 80);

  // Mk2
  trackline_duration(pid1_parameter, 180, 150, 0, 0,false);
  trackline_duration(pid1_parameter, 230, 900, 0, 0,false);

  // __________ END __________

  // _____ Start the traffic circle mission _____
  // node 6 to start point of traffic circle and turn right
  trackline_Cross(pid1_parameter, baseSpeed, 1, 50, 80);
  tr_sensor(100);

  // // MK 2
  // fd(150);delay(150);stop(true);
  // sl(90);delay(300);stop(true);
  // fd(150);delay(850);stop(true);

  //  MK 1
  // track the line untill cannot find any line
  while (sensor_bool[0] || sensor_bool[1] || sensor_bool[2] || sensor_bool[3] || sensor_bool[4])
  {
    trackline_pid(pid1_parameter, 80);
    // trackline_pid(pid1_parameter, 110);
  }
  // walk with specific speed untill find the line
  stop(false);
  // m(46, 85);
  m(90,185);
  readSensor();
  while (!(sensor_bool[0] || sensor_bool[1] || sensor_bool[2] || sensor_bool[3] || sensor_bool[4]))
  {
    readSensor();
  }
  // trackline until meet the end of traffic cycle
  while (!sensor_bool[4])
  {
    trackline_pid(pid1_parameter, 80);
    // trackline_pid(pid1_parameter, 110);
  }
  // __________ END __________
  delay(50);
  // turn right and go to finish point (node 7)
  tr_sensor(70);
  while (sensor_bool[0] || sensor_bool[1] || sensor_bool[2] || sensor_bool[3] || sensor_bool[4])
  {
    // trackline_pid(pid1_parameter, 80);
    trackline_pid(pid1_parameter, 125);
  }

  // Finish the mission
  stop(true);
}
void loop()
{
  readSensor();
  // trackline_pid(pid_forward_parameter, 145);
}
