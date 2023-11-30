#include <positionalnew.h>

#define ppwm1 13
#define pdir1 14
#define enc1 26
#define enc2 27
#define ppwm2 4
#define pdir2 15
#define enc3 32
#define enc4 33
#define lsDown 23
#define lsclose 22

UniversalEncoder m2enc(enc1, enc2), m1enc(enc3, enc4);
Motor m2(ppwm1, pdir1), m1(ppwm2, pdir2);
positionalnew pid1(&m1), pid2(&m2);

int start = 0, setpointH = 0, setpointV = 0, pwm1 = 0, pwm2 = 0;
bool m1stop = false, verticalflag = false, m2stop = true, horizontalflag = false;
//int lvertical5 = 5500,
//    lvertical4 = 10250,
//    lvertical3 = 14890,
//    lvertical2 = 19680,
//    lvertical1 = 22335,
//
//    lhorizontal5 = 1400,
//    lhorizontal4 = 2561,
//    lhorizontal3 = 3433,
//    lhorizontal2 = 4838,
//    lhorizontal1 = 5850;

 int lvertical5 = 1200,
    lvertical4 = 1200,
    lvertical3 = 1200,
    lvertical2 = 1200,
    lvertical1 = 1200,

    lhorizontal5 = 300,
    lhorizontal4 = 1000,
    lhorizontal3 = 1400,
    lhorizontal2 = 1800,
    lhorizontal1 = 2200;

void initreset()
{
  m1.setPWM(0);
  m2.setPWM(0);
  pid1.setPulse(m1.getReadings());
  pid2.setPulse(m2.getReadings());
  setpointH = 0;
  setpointV = 0;
  m1stop = false, m2stop = true, verticalflag = false, horizontalflag = false;
}

void setup()
{
  Serial.begin(115200);

  pinMode(lsDown, INPUT_PULLUP);
  pinMode(lsclose, INPUT_PULLUP);

  m1.setEncoder(&m1enc);
  m2.setEncoder(&m2enc);

  pid1.setThreshold(400);
  pid1.setOutputLimits(-100, 100);
  pid1.setAggTunings(1.09, 0, 0);
  pid1.setSoftTunings(0.2, 0, 0);

  pid2.setThreshold(0);
  pid2.setOutputLimits(-100, 100);
  pid2.setAggTunings(1.09, 0, 0);
  pid2.setSoftTunings(1, 0, 0);
}
void loop()
{
  if (Serial.available())
  {
    start = Serial.readStringUntil(',').toInt();
    pwm1 = Serial.readStringUntil(',').toInt();
    pwm2 = Serial.readStringUntil('\n').toInt();
  }

  Serial.println(String(m1.getReadings()) + ", " + String(m2.getReadings()) + ", " + String(digitalRead(lsDown)) + ", " + String(digitalRead(lsclose)) + ", " + String(pwm1) + ", " + String(pwm2)); // + ", " + String(pid1.Input) + ", " + String(pid1.Output) + ", " + String(pid1.Setpoint) + ", ");

  if (start == 0)
  {
    if (horizontalflag)
    {
//      Serial.println(String(pid1.Input) + ", " + String(pid1.Output) + ", " + String(pid1.Setpoint) + ", ");
      pid1.compute();
    }

    if (verticalflag)
    {
      Serial.println(String(pid2.Input) + ", " + String(pid2.Output) + ", " + String(pid2.Setpoint) + ", ");
      pid2.compute();
    }
  }

  if (start == 1)
  {
    m1stop = false;
    if (digitalRead(lsDown) == HIGH && !m1stop)
    {
      m1.setPWM(-pwm1);
    }
    else if (digitalRead(lsDown) == LOW && !m1stop)
    {
      m1stop = true;
      m1.setPWM(0);
      m1.reset();
      start = -1;
    }
  }

  if (digitalRead(lsclose) == LOW)
  {
    m2.setPWM(pwm2);
    m2stop = false;
  }
  else if (m2stop == false)
  {
    m2.setPWM(0);
    m2stop = true;
  }

  if (start == 2)
  {
    if (abs(lhorizontal5 - m2.getReadings()) <= 400)
    {
      pid1.setPulse(lvertical5 + 50);
      setpointV = lvertical5;
    }
    else if (abs(lhorizontal4 - m2.getReadings()) <= 400)
    {
      pid1.setPulse(lvertical4 + 50);
      setpointV = lvertical4;
    }
    else if (abs(lhorizontal3 - m2.getReadings()) <= 400)
    {
      pid1.setPulse(lvertical3 + 50);
      setpointV = lvertical3;
    }
    else if (abs(lhorizontal2 - m2.getReadings()) <= 400)
    {
      pid1.setPulse(lvertical2 + 50);
      setpointV = lvertical2;
    }
    else if (abs(lhorizontal1 - m2.getReadings()) <= 400)
    {
      pid1.setPulse(lvertical1 + 50);
      setpointV = lvertical1;
    }

    if (setpointV != 0)
    {
      horizontalflag = true;
      verticalflag = false;
      start = 0;
    }
  }

  if (start == 3)
  {
    if (abs(setpointV - m1.getReadings()) <= 200 && verticalflag == false)
    {
      setpointH = (m2.getReadings() - 300);
      pid2.setPulse(setpointH);
      verticalflag = true;
      horizontalflag = false;
      start = 0;
    }
  }

  if (start == 4)
  {
    initreset();
  }
}
