#include <positionalnew.h>

#define grabpwm1 13
#define grabdir 14
#define grabenc1 26
#define grabenc2 27

#define stackpwm1 4
#define stackdir 15
#define stackenc1 32
#define stackenc2 33

#define lsDown 23
#define lsclose 22

UniversalEncoder grabenc(grabenc1, grabenc2), stackenc(stackenc1, stackenc2);
Motor grabmtr(grabpwm1, grabdir), stackmtr(stackpwm1, stackdir);
positionalnew grabpid(&grabmtr), stackpid(&stackmtr);

int start = 0, setpointH = 0, setpointV = 0, grabpwm = 0, stackpwm = 0;
bool stackstop = false, grabflag = false, grabstop = true, stackflag = false, reset1 = false;

int lstack5 = 5500,
    lstack4 = 10250,
    lstack3 = 14890,
    lstack2 = 19680,
    lstack1 = 22335,

    lgrab5 = 1400, // 1400
    lgrab4 = 2561, // 1161
    lgrab3 = 3433, // 872
    lgrab2 = 4838, // 1405
    lgrab1 = 5850; // 1012

void output()
{
  Serial.println(String(grabmtr.getReadings()) + ", " + String(stackmtr.getReadings()) + ", " + String(digitalRead(lsDown)) + ", " + String(digitalRead(lsclose)) + ", " + String(grabpwm) + ", " + String(stackpwm));
}

void setup()
{
  Serial.begin(115200);

  pinMode(lsDown, INPUT_PULLUP);
  pinMode(lsclose, INPUT_PULLUP);

  grabmtr.setEncoder(&grabenc);
  stackmtr.setEncoder(&stackenc);

  grabpid.setThreshold(400);
  grabpid.setOutputLimits(-100, 100);
  grabpid.setAggTunings(1.09, 0, 0);
  grabpid.setSoftTunings(0.2, 0, 0);

  stackpid.setThreshold(0);
  stackpid.setOutputLimits(-100, 100);
  stackpid.setAggTunings(1.09, 0, 0);
  stackpid.setSoftTunings(1, 0, 0);
}

void loop()
{
  if (Serial.available())
  {
    start = Serial.readStringUntil(',').toInt();
    grabpwm = Serial.readStringUntil(',').toInt();
    stackpwm = Serial.readStringUntil('\n').toInt();
    grabpid.setOutputLimits(-grabpwm, grabpwm);
    stackpid.setOutputLimits(-stackpwm, stackpwm);
  }
  output();
  if (start == 0)
  {
    if (stackflag)
    {
      //      Serial.println(String(stackpid.Input) + ", " + String(stackpid.Output) + ", " + String(stackpid.Setpoint) + ", ");
      stackpid.compute();
    }

    if (grabflag)
    {
      //      Serial.println(String(grabpid.Input) + ", " + String(grabpid.Output) + ", " + String(grabpid.Setpoint) + ", ");
      grabpid.compute();
      reset1 = true;
    }
  }

  if (start == 1)
  {
    reset1 = false;
    if (digitalRead(lsDown) == HIGH && !stackstop)
    {
      //      Serial.println("grabmtr start");
      stackmtr.setPWM(-stackpwm);
    }
    else if (digitalRead(lsDown) == LOW && !stackstop)
    {
      Serial.print("grabmtr stop");
      stackmtr.setPWM(stackpwm);
      stackstop = true;
    }
    else if (digitalRead(lsDown) == HIGH && stackstop)
    {
      stackmtr.setPWM(0);
      stackmtr.reset();
      start = 0;
      stackstop = !stackstop;
    }
  }

  if (digitalRead(lsclose) == LOW) //&& grabstop) // lsclose is a key of controller
  {
    grabmtr.setPWM(grabpwm);
    grabstop = !grabstop;
  }
  else if (grabstop == false)
  {
    grabmtr.setPWM(0);
    grabstop = true;
  }

  if (start == 2)
  {
    if (abs(lgrab5 - grabmtr.getReadings()) <= 400)
    {
      stackpid.setPulse(lstack5 + 50);
      setpointV = lstack5;
    }
    else if (abs(lgrab4 - grabmtr.getReadings()) <= 400)
    {
      stackpid.setPulse(lstack4 + 50);
      setpointV = lstack4;
    }
    else if (abs(lgrab3 - grabmtr.getReadings()) <= 400)
    {
      stackpid.setPulse(lstack3 + 50);
      setpointV = lstack3;
    }
    else if (abs(lgrab2 - grabmtr.getReadings()) <= 400)
    {
      stackpid.setPulse(lstack2 + 50);
      setpointV = lstack2;
    }
    else if (abs(lgrab1 - grabmtr.getReadings()) <= 400)
    {
      stackpid.setPulse(lstack1 + 50);
      setpointV = lstack1;
    }

    if (setpointV != 0)
    {
      stackflag = true;
      start = 0;
    }
  }

  if (start == 3)
  {
    if (abs(setpointV - stackmtr.getReadings()) <= 200 && grabflag == false)
    {
      setpointH = (grabmtr.getReadings() - 300);
      grabpid.setPulse(setpointH);
      grabflag = true;
      stackflag = false;
      start = 0;
    }

    else
    {
      Serial.println("Failed");
    }
  }

  if (reset1 == true)
  {
    grabpid.compute();
    stackmtr.setPWM(0);
    setpointH = 0;
    setpointV = 0;
    start = -1;
    stackstop = false, grabstop = true, grabflag = false, stackflag = false;
    // reset = false;
  }
}
