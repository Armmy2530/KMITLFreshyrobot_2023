void stop(boolean hard)
{
    if (hard)
    {
        m(-999, -999);
    }
    else
    {
        m(0, 0);
    }
}
void tl(int speed) { m(0, speed); }
void tr(int speed) { m(speed, 0); }
void sl(int speed) { m(-speed, speed); }
void sr(int speed) { m(speed, -speed); }
void fd(int speed) { m(speed, speed); }
void bd(int speed) { m(-speed, -speed); }

void m(int l, int r)
{
    l = (float(power_percentage) / 100) * l;
    r = (float(power_percentage) / 100) * r;

    L_motor.m(l);
    R_motor.m(r);
}
