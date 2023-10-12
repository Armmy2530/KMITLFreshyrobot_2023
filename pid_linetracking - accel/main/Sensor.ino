// Calibration parameter
int dif_ref = 300;

void getRemote()
{
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

void readSensor()
{
  sensor_val[0] = analogRead(L2_pin);
  sensor_val[1] = analogRead(L1_pin);
  sensor_val[2] = analogRead(C_pin);
  sensor_val[3] = analogRead(R1_pin);
  sensor_val[4] = analogRead(R2_pin);
  for (int n = 0; n <= 4; n++)
  {
    sensor_bool[n] = B(sensor_val[n], ref_sensor[n]);
  }

  if (debug == true)
  {
    for (int n = 0; n <= 4; n++)
    {
      Serial.print(String(sensor_val[n]) + " ");
    }
    Serial.println();
  }
}

void calibrate_sensor(int n_check, int cal_speed)
{
  readSensor();
  int white_val[5] = {0, 0, 0, 0, 0};
  int black_val[5] = {0, 0, 0, 0, 0};
  int avg_val[5] = {0, 0, 0, 0, 0};
  int confirm_black = 0;
  Serial.println("Start Calibrat sensor ..........");

  // number of loop for find avg white and black color from sensor
  /////////////////////////////////////////////////////////////////////////////////
  for (int i = 0; i < n_check; i++)
  {
    // Start from all sensor in white area (no black line on sensor) and save in array
    /////////////////////////////////////////////////////////////////////////////////
    for (int n = 0; n <= 4; n++)
    {
      readSensor();
      white_val[n] = sensor_val[n];
    }
    /////////////////////////////////////////////////////////////////////////////////

    // Moving forward until all sensor are on black area (sensor see all black line)
    /////////////////////////////////////////////////////////////////////////////////
    fd(cal_speed);
    while (true)
    {
      readSensor();
      for (int i = 0; i <= 4; i++)
      {
        readSensor();
        if (abs(sensor_val[i] - white_val[i]) >= dif_ref)
        {
          confirm_black++;
        }
        Serial.print(String(sensor_val[i]) + " ");
      }
      Serial.println();
      if (confirm_black == 5)
      {
        for (int n = 0; n <= 4; n++)
        {
          black_val[n] = sensor_val[n];
        }
        delay(50);
        m(0, 0);
        break;
      }
      else
        confirm_black = 0;
    }
    /////////////////////////////////////////////////////////////////////////////////

    // Average max and min value to get the middle value (ref value) for the sensor
    /////////////////////////////////////////////////////////////////////////////////
    for (int n = 0; n <= 4; n++)
    {
      avg_val[n] = (white_val[n] + black_val[n]) / 2;
    }
    Serial.print("loop: ");
    Serial.print(i);
    for (int n = 0; n <= 4; n++)
    {
      Serial.print(" ");
      Serial.print(avg_val[n]);
    }
    Serial.println();
    /////////////////////////////////////////////////////////////////////////////////

    // Moving backward untill all sensor on white area (all sensor not see the black line)
    /////////////////////////////////////////////////////////////////////////////////
    bd(cal_speed);
    while ((W(sensor_val[0], avg_val[0]) && W(sensor_val[1], avg_val[1]) && W(sensor_val[2], avg_val[2]) && W(sensor_val[3], avg_val[3]) && W(sensor_val[4], avg_val[4])))
    {
      readSensor();
    }
    delay(500);
    m(0, 0);
    ////////////////////////////////////////////////////////////////////////////////
    delay(1000);
  }
  // !!!(end of for loop)!!! /////////////////////////////////////////////////////////

  // Finish all loop, get all value and avg them all to get final value
  /////////////////////////////////////////////////////////////////////////////////
  Serial.print("Final Ref: ");
  Serial.print(" {");
  for (int n = 0; n <= 4; n++)
  {
    ref_sensor[n] = avg_val[n];
    Serial.print(String(ref_sensor[n]) + ", ");
  }
  Serial.println(" }");
  /////////////////////////////////////////////////////////////////////////////////
}

bool W(int n, int ref)
{
  if (n > ref)
  {
    return true;
  }
  else
  {
    return false;
  }
}
bool B(int n, int ref)
{
  if (n <= ref)
  {
    return true;
  }
  else
  {
    return false;
  }
}
