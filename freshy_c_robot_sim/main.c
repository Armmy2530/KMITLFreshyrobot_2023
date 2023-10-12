int lastline_state = 0; // 0 center  ||  1 left || 2 right
int OUT_LINE = 80000;
int error = 0;
double pre_error = 0.0, sum_error = 0.0;
double kp = 30.0, kd = 3.0, ki = 0.01;

// Use for compare the analog value to digital value (B = analog > ref , W = analog <= ref)
int[] ref_sensor = new int[5]{500, 500, 500, 500, 500};

// Sensor variable
bool[] sensor_bool = new bool[5]{false, false, false, false, false};
int[] sensor_val = new int[5]{1, 1, 1, 1, 1};

// Motor&PID parameter
int baseSpeed = 120;
int maxSpeed = 180;

// sensor_pin
const int L2_pin = 0;
const int L1_pin = 1;
const int C_pin = 2;
const int R1_pin = 3;
const int R2_pin = 4;

void readSensor()
{
    sensor_val[0] = analog(L2_pin);
    sensor_val[1] = analog(L1_pin);
    sensor_val[2] = analog(C_pin);
    sensor_val[3] = analog(R1_pin);
    sensor_val[4] = analog(R2_pin);
    for (int n = 0; n <= 4; n++)
    {
        sensor_bool[n] = (sensor_val[n] <= ref_sensor[n]) ? true : false;
    }
    // lcd("%d %d %d %d %d",sensor_bool[0],sensor_bool[1],sensor_bool[2],sensor_bool[3],sensor_bool[4]);
}

int Cal_Error()
{
    if (sensor_bool[0] || sensor_bool[1] || sensor_bool[2] || sensor_bool[3] || sensor_bool[4])
    {
        // LEFT
        error = (sensor_bool[0]) ? -4 : error;
        error = (sensor_bool[1] && sensor_bool[0]) ? -3 : error;
        error = (sensor_bool[1]) ? -2 : error;
        error = (sensor_bool[2] && sensor_bool[1]) ? -1 : error;

        // CENTER
        error = (sensor_bool[2]) ? 0 : error;

        // RIGHT
        error = (sensor_bool[2] && sensor_bool[3]) ? 1 : error;
        error = (sensor_bool[3]) ? 2 : error;
        error = (sensor_bool[3] && sensor_bool[4]) ? 3 : error;
        error = (sensor_bool[4]) ? 4 : error;

        lastline_state = (sensor_bool[4] || sensor_bool[3]) ? 2 : lastline_state;
        lastline_state = (sensor_bool[0] || sensor_bool[1]) ? 1 : lastline_state;

        return error;
    }
    else
    {
        return OUT_LINE;
    }
}
void trackline_pid(int base_speed)
{
    // local variable for motor speed
    int leftSpeed = 0, rightSpeed = 0;

    // Read Sensor from analogRead
    readSensor();
    int error_actual = Cal_Error();

    // If not find any line spin to lastest line
    if (error_actual == OUT_LINE)
    {
        switch (lastline_state)
        {
        case 0:
            leftSpeed = baseSpeed;
            rightSpeed = baseSpeed;
            break;
        case 1:
            leftSpeed = -baseSpeed;
            rightSpeed = baseSpeed;
            break;
        case 2:
            leftSpeed = baseSpeed;
            rightSpeed = -baseSpeed;
            break;
        }
    }
    else // if not, then trackline using pid algorithem
    {
        int motorSpeed = (int)(kp * error + kd * (error - pre_error) + ki * (sum_error));
        leftSpeed = base_speed + motorSpeed;
        rightSpeed = base_speed - motorSpeed;
        pre_error = error;
        sum_error += error;
    }
    // lcd("error=%d",error);
    m(leftSpeed, rightSpeed);
}

void trackline_R(int base_speed, int line, int finalmove_speed, int ms_sleep)
{
    int count = 0;
    while (count < line)
    {
        trackline_pid(base_speed);
        if (sensor_bool[4] && sensor_bool[3] && sensor_bool[2])
        {
            readSensor();
            while (sensor_bool[4])
            {
                readSensor();
                fd(finalmove_speed);
            }
            count++;
        }
    }
    sleep(ms_sleep);
    ao();
}

void trackline_L(int base_speed, int line, int finalmove_speed, int ms_sleep)
{
    int count = 0;
    while (count < line)
    {
        trackline_pid(base_speed);
        if (sensor_bool[1] && sensor_bool[0] && sensor_bool[2])
        {
            readSensor();
            while (sensor_bool[0])
            {
                readSensor();
                fd(finalmove_speed);
                sleep(50);
            }
            count++;
        }
    }
    sleep(ms_sleep);
    ao();
}

void trackline_duration(int base_speed, int duration, int finalmove_speed, int ms_sleep)
{
    int start_ms = clock() , current_ms = clock();
    while (current_ms - start_ms <= duration)
    {
        trackline_pid(base_speed);
    }
    sleep(ms_sleep);
    ao();
}

void tr_sensor(int speed)
{
    sr(speed);
    sleep(200);
    readSensor();
    while (!sensor_bool[2])
    {
        sr(speed);
        readSensor();
    }
    ao();
}

void tl_sensor(int speed)
{
    sl(speed);
    sleep(200);
    readSensor();
    while (!sensor_bool[2])
    {
        sl(speed);
        readSensor();
    }
    ao();
}

void m(int l, int r)
{
    l = (int)((float)l / 255 * 100);
    r = (int)((float)r / 255 * 100);

    motor(1, l);
    motor(2, r);
}

void setup()
{
    sw1_press();
    readSensor();
    // trackline_R(int base_speed, int line, int finalmove_speed, int ms_sleep)
    // trackline_L(80, 1, 50, 200);
    // tr_sensor(100);
    trackline_L(80, 1, 50, 200);
    tl_sensor(100);
    trackline_L(80, 1, 50, 200);
}
void loop()
{
    // readSensor();
    // trackline_pid(150);
}