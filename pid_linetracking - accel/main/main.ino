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

// sensor_pin
#define L2_pin 33
#define L1_pin 32
#define C_pin 35
#define R1_pin 34
#define R2_pin 39

// Use for compare the analog value to digital value (B = analog > ref , W = analog <= ref)

// At compettion
int ref_sensor[5] = {3000, 3000, 3000, 3000, 3000};

// At pratice room
// int ref_sensor[5] = {2000, 2000, 2000, 2000, 2000};

// Sensor variable
boolean sensor_bool[5] = {0, 0, 0, 0, 0};
int sensor_val[5] = {1, 1, 1, 1, 1};

// Motor&PID parameter
float accel = 800;
int baseSpeed = 120;
int maxSpeed = 180;
float pid1_parameter[3] = {15, 20, 0};          // Pi 0.0000001
float pid_forward_parameter[3] = {13.5, 10, 0}; // {8.5, 10, 0.001}

bool debug = false;

// time funtion
unsigned long current_ms = millis(), pev_ms = millis();

class Accelation
{
public:
  float current = 0.0, target = 0.0;
  int current_ms = millis(), pev_ms = millis();
  float accel = 0.0; // +- pwm per second
  void setAccel(float src_accel)
  {
    accel = src_accel;
  }
  void go(float src_target)
  {
    target = src_target;
  }
  void update()
  {
    current_ms = millis();
    // reset pev_ms if it not using for 100ms
    current_ms - pev_ms >= 100 ? pev_ms = current_ms : pev_ms;
    if (current < target)
    {
      // increase = (diff_time / ms) * accel(pwm/s)
      float increase = (float(current_ms - pev_ms) / 1000) * accel;
      // check value that we going to increse is not over then target
      current = (current + increase) > target ? target : current + increase;
    }
    else if (current > target)
    {
      // increase = (diff_time / ms) * accel(pwm/s)
      float decrease = (float(current_ms - pev_ms) / 1000) * accel;
      // check value that we going to increse is not over then target
      current = (current - decrease) < target ? target : current - decrease;
    }
    pev_ms = millis();
  }

  int output()
  {
    return (int)current;
  }
};

Accelation L_speed;
Accelation R_speed;

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
  // PS4.begin("14:85:7f:50:7a:61");

  delay(5000);

  // calibrate_sensor(3,60);

  // trackline_Cross(pid1_parameter, baseSpeed, 1, 50, 80);
  // tl_sensor(150);
  // trackline_duration(pid1_parameter, baseSpeed, 1000, 50, 0);
  // tr_sensor(150);
  // trackline_R(pid1_parameter, baseSpeed, 3, 50, 50);

  // trackline_Cross(pid1_parameter, baseSpeed, 1, 50, 80);
  // trackline_R(pid1_parameter, baseSpeed, 3, 50, 50);
  // tr_sensor(150);
  // trackline_boots(pid_forward_parameter,pid1_parameter, 180, 700, 120);
  //   tr_sensor(150);
  //   trackline_R(pid1_parameter, baseSpeed, 3, 50, 50);
  //   tr_sensor(150);
  //   trackline_Cross(pid1_parameter, baseSpeed, 1, 50, 80);
}
void loop()
{
  // runtime funtion
  // m_accel_runtime();

  readSensor();
  trackline_pid(pid1_parameter, 135);

  // trackline_Cross(pid_forward_parameter, 135, 1, 0, 0);
  // // delay(500);
  // tl_sensor(30);
  // // delay(500);
  // tl_sensor(30);
}
