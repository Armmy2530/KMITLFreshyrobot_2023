#define OUT_LINE 8000 // error code for not found line on any sensor
float error = 0, pre_error = 0, sum_error;

bool W(int n)
{
    return n == 0;
}
bool B(int n)
{
    return n == 1;
}

signed int Cal_Error()
{
    if (sensor_bool[0] || sensor_bool[1] || sensor_bool[2] || sensor_bool[3] || sensor_bool[4])
    {
        // LEFT
        error = (sensor_bool[0]) ? -4 : error;
        error = (sensor_bool[1]) ? -2 : error;
        error = (sensor_bool[1] && sensor_bool[0]) ? -3 : error;
        error = (sensor_bool[2] && sensor_bool[1]) ? -1 : error;

        // CENTER
        error = (sensor_bool[2]) ? 0 : error;

        // RIGHT

        error = (sensor_bool[3]) ? 2 : error;
        error = (sensor_bool[4]) ? 4 : error;
        error = (sensor_bool[3] && sensor_bool[4]) ? 3 : error;
        error = (sensor_bool[2] && sensor_bool[3]) ? 1 : error;

        lastline_state = (sensor_bool[4] || sensor_bool[3]) ? RIGHT : lastline_state;
        lastline_state = (sensor_bool[0] || sensor_bool[1]) ? LEFT : lastline_state;

        return error;
    }
    else
    {
        return OUT_LINE;
    }
}
void trackline_pid(float pid_parameter[3], int base_speed)
{
    // local variable for motor speed
    int leftSpeed, rightSpeed;

    // Read Sensor from analogRead
    readSensor();
    signed int error_actual = Cal_Error();

    Serial.print(millis());
    Serial.print(",");
    Serial.print(error_actual);
    Serial.print(",");
    Serial.print(pid_parameter[0] * error);
    Serial.print(",");
    Serial.print(pid_parameter[1] * (error - pre_error));
    Serial.print(",");
    Serial.print(pid_parameter[2] * sum_error);
    Serial.print(",");
    Serial.print(sum_error);
    Serial.print(",");
    Serial.print(pre_error);

    // If not find any line spin to lastest line
    if (error_actual == OUT_LINE)
    {
        switch (lastline_state)
        {
        case CENTER:
            leftSpeed = 150;
            rightSpeed = 150;
            break;
        case LEFT:
            leftSpeed = -150;
            rightSpeed = 150;
            break;
        case RIGHT:
            leftSpeed = 150;
            rightSpeed = -150;
            break;
        }
    }
    else // if not, then trackline using pid algorithem
    {
        int motorSpeed = (int)(pid_parameter[0] * error + pid_parameter[1] * (error - pre_error) + pid_parameter[2] * (sum_error));
        leftSpeed = base_speed + motorSpeed;
        rightSpeed = base_speed - motorSpeed;
        pre_error = error;
        sum_error += error;
    }

    Serial.print(",");
    Serial.print(leftSpeed);
    Serial.print(",");
    Serial.print(rightSpeed);

    Serial.println();

    m(leftSpeed, rightSpeed);

    delayMicroseconds(10);
}

void trackline_pid_nooutline(float pid_parameter[3], int base_speed)
{
    // local variable for motor speed
    int leftSpeed, rightSpeed;

    // Read Sensor from analogRead
    readSensor();
    signed int error_actual = Cal_Error();

    Serial.print(millis());
    Serial.print(",");
    Serial.print(error_actual);
    Serial.print(",");
    Serial.print(pid_parameter[0] * error);
    Serial.print(",");
    Serial.print(pid_parameter[1] * (error - pre_error));
    Serial.print(",");
    Serial.print(pid_parameter[2] * sum_error);
    Serial.print(",");
    Serial.print(sum_error);
    Serial.print(",");
    Serial.print(pre_error);

    int motorSpeed = (int)(pid_parameter[0] * error + pid_parameter[1] * (error - pre_error) + pid_parameter[2] * (sum_error));
    leftSpeed = base_speed + motorSpeed;
    rightSpeed = base_speed - motorSpeed;
    pre_error = error;
    sum_error += error;

    Serial.print(",");
    Serial.print(leftSpeed);
    Serial.print(",");
    Serial.print(rightSpeed);

    Serial.println();

    m(leftSpeed, rightSpeed);

    delayMicroseconds(10);
}

void trackline_R(float pid[3], int base_speed, int line, int finalmove_speed, int ms_delay)
{
    int count = 0;
    while (count < line)
    {
        trackline_pid(pid, base_speed);

        if (sensor_bool[4] && sensor_bool[2])
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
    delay(ms_delay);
    stop(true);
}

void trackline_L(float pid[3], int base_speed, int line, int finalmove_speed, int ms_delay)
{
    int count = 0;
    while (count < line)
    {
        trackline_pid(pid, base_speed);
        if (sensor_bool[0] && sensor_bool[2])
        {
            while (sensor_bool[0])
            {
                fd(finalmove_speed);
                readSensor();
            }
            count++;
        }
    }
    delay(ms_delay);
    stop(true);
}

void trackline_Cross(float pid[3], int base_speed, int line, int finalmove_speed, int ms_delay)
{
    int count = 0;
    while (count < line)
    {
        trackline_pid(pid, base_speed);
        if (sensor_bool[4] && sensor_bool[0])
        {
            Serial.println("found cross" + String(count));
            while (sensor_bool[4] || sensor_bool[0])
            {
                fd(finalmove_speed);
                readSensor();
            }
            count++;
        }
    }
    delay(ms_delay);
    stop(true);
}

void trackline_outline(float pid[3], int base_speed, int finalmove_speed, int ms_delay)
{
    int count = 0;
    int start_ms = -99, current_ms = millis();
    readSensor();
    while (true)
    {
        current_ms = millis();
        trackline_pid(pid, base_speed);
        if (!(sensor_bool[0] || sensor_bool[1] || sensor_bool[2] || sensor_bool[3] || sensor_bool[4]))
        {
            start_ms == -99 ? start_ms = millis() : start_ms;
            if (current_ms - start_ms >= 35)
            {
                break;
            }
        }
        else
        {
            start_ms = -99;
        }
    }
    delay(ms_delay);
    stop(true);
}

void trackline_duration(float pid[3], int base_speed, int duration, int finalmove_speed, int ms_delay, bool stop_hard)
{
    int start_ms = millis(), current_ms = millis();
    while (current_ms - start_ms <= duration)
    {
        trackline_pid(pid, base_speed);
        current_ms = millis();
    }
    delay(ms_delay);
    stop_hard ? stop(true) : stop(false);
}

void trackline_duration_nooutline(float pid[3], int base_speed, int duration, int finalmove_speed, int ms_delay, bool stop_hard)
{
    int start_ms = millis(), current_ms = millis();
    while (current_ms - start_ms <= duration)
    {
        trackline_pid_nooutline(pid, base_speed);
        current_ms = millis();
    }
    delay(ms_delay);
    stop_hard ? stop(true) : stop(false);
}

void tr_sensor(int speed)
{
    // sr(speed);
    // delay(200);
    readSensor();
    while (sensor_bool[4])
    {
        sr(speed);
        readSensor();
    }
    readSensor();
    while (!sensor_bool[4])
    {
        sr(speed);
        readSensor();
    }
    stop(true);
}

void tr_sensor_custom(int speed,int target)
{
    // sr(speed);
    // delay(200);
    readSensor();
    while (sensor_bool[target])
    {
        sr(speed);
        readSensor();
    }
    readSensor();
    while (!sensor_bool[target])
    {
        sr(speed);
        readSensor();
    }
    stop(true);
}

void tl_sensor(int speed)
{
    // sl(speed);
    // delay(200);
    readSensor();
    while (sensor_bool[0])
    {
        sl(speed);
        readSensor();
    }
    readSensor();
    while (!sensor_bool[0])
    {
        sl(speed);
        readSensor();
    }
    stop(true);
}

void heading_center(int speed, int target_postion)
{
    readSensor();
    signed int error = Cal_Error();
    while (true)
    {
        readSensor();
        signed int error = Cal_Error();
        if (error == OUT_LINE)
        {
            int leftSpeed, rightSpeed;
            switch (lastline_state)
            {
            case CENTER:
                leftSpeed = 150;
                rightSpeed = 150;
                break;
            case LEFT:
                leftSpeed = -150;
                rightSpeed = 150;
                break;
            case RIGHT:
                leftSpeed = 150;
                rightSpeed = -150;
                break;
            }
            m(leftSpeed, rightSpeed);
        }
        else if (error > target_postion)
        {
            tr(speed);
        }
        else if (error < target_postion)
        {
            tl(speed);
        }
        else
        {
            break;
        }
        delay(10);
    }
    stop(true);
}