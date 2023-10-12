#include <ESP32Servo.h>
#include <analogWrite.h>
#include <motor_esp32.h>
#include <PS4Controller.h>

// Motor L 
#define m2a  12
#define m2b  14
#define pwm2 13 
// Motor R 
#define m1a  27
#define m1b  26
#define pwm1 25
// Motor parameters
#define max_speed_percentage 70

Motor L_motor(m1a,m1b,pwm1);
Motor R_motor(m2a,m2b,pwm2);

int Target_servo = 30;
int servo_pin = 5; 

float current_L = 0, pev_L = 0, target_L = 0;
float current_R = 0, pev_R = 0, target_R = 0;
float freerange = 25;
// drive mode 2 setting
int min_percentage = 25;
int max_percentage = 125;

Servo myservo;

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
    Serial.begin(115200);
    Serial.println("Hello world");
    PS4.begin("14:85:7f:50:7a:61");
    myservo.attach(servo_pin,500,2600);
    myservo.write(30);
}

void loop()
{
    getRemote();
    if (isinFreerange(controller_data.L_X, freerange) && isinFreerange(controller_data.L_Y, freerange))
    {
        target_L = 0;
        target_R = 0;
        current_L = target_L;
        current_R = target_R;
        pev_L = current_L;
        pev_R = current_R;
    }
    else if (isinFreerange(controller_data.L_Y, freerange))
    {
        target_L = float(controller_data.L_X) / 128 * 255;
        target_R = 0 - float(controller_data.L_X) / 128 * 255;
        current_L = target_L;
        current_R = target_R;
        pev_L = current_L;
        pev_R = current_R;
    }
    else if (isinFreerange(controller_data.L_X, freerange))
    {
        target_L = float(controller_data.L_Y) / 128 * 255;
        target_R = float(controller_data.L_Y) / 128 * 255;
        current_L = target_L;
        current_R = target_R;
        pev_L = current_L;
        pev_R = current_R;
    }
    else if (controller_data.L_X > 0)
    {
        target_R = (255 / 100) * (float(controller_data.L_Y) / 128) * float(map(controller_data.L_X, 0, 128, max_percentage, min_percentage));
        target_L = 255 * (float(controller_data.L_Y) / 128);
        current_L = target_L;
        current_R = target_R;
        pev_L = current_L;
        pev_R = current_R;
    }
    else if (controller_data.L_X < 0)
    {
        target_L = (255 / 100) * (float(controller_data.L_Y) / 128) * float(map(controller_data.L_X, -128, 0, min_percentage, max_percentage));
        target_R = (float(controller_data.L_Y) / 128) * 255;
        current_L = target_L;
        current_R = target_R;
        pev_L = current_L;
        pev_R = current_R;
    }
    
    m(current_L, current_R);

    if(controller_data.Cross){
      Serial.println("go 10");
      myservo.write(10);
      delay(500);
      myservo.write(30);
    }
    else if(controller_data.Triangle){
      Serial.println("go 120");
      myservo.write(120);
      delay(1000);
      myservo.write(30);
    }
}

void m(int l, int r)
{
    l = l * (float(max_speed_percentage)/100);
    r = r * (float(max_speed_percentage)/100);
    
    L_motor.m(l);
    R_motor.m(r);

    Serial.print("currentL: ");
    Serial.print(l);
    Serial.print("currentR: ");
    Serial.println(r);
}

bool isinFreerange(int value, float freerange)
{
    int percent = round(125 * (freerange / 100));
    int min_per = 0 - percent;
    int max_per = 0 + percent;
    // Serial.print("value:");
    // Serial.print(value);
    // Serial.print("freerange:");
    // Serial.print(freerange);
    // Serial.print("percent:");
    // Serial.print(percent);
    // Serial.print("min:");
    // Serial.print(min_per);
    // Serial.print("max:");
    // Serial.print(max_per);
    // Serial.print("true:");
    // Serial.println(value >= min_per && value <= max_per);
    if(value >= min_per && value <= max_per)return true; else return false;
}

void getRemote(){
  controller_data.L_X = PS4.LStickX();
  controller_data.L_Y = PS4.LStickY();
  controller_data.R_X = PS4.RStickX();
  controller_data.R_Y = PS4.RStickY();
  controller_data.Right = PS4.Right();
  controller_data.Down = PS4.Down();
  controller_data.Up = PS4.Up();
  controller_data.Left = PS4.Left();
  controller_data.Square = PS4.Square();
  controller_data.Cross = PS4.Cross();
  controller_data.Circle = PS4.Circle();
  controller_data.Triangle = PS4.Triangle();
  controller_data.L1 = PS4.L1();
  controller_data.R1 = PS4.R1();
  controller_data.Share = PS4.Share();
  controller_data.Option = PS4.Options();
  controller_data.L3 = PS4.L3();
  controller_data.R3 = PS4.R3();
  controller_data.L2 = PS4.L2Value();
  controller_data.R2 = PS4.R2Value();
}
