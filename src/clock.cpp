#include <Arduino.h>
#include <AccelStepper.h>
#include <ezTime.h>
#include <WiFi.h>

#define HOUR_SENSOR_PIN 2
#define MINUTE_SENSOR_PIN 4

const unsigned int StepsPerRevOfMainRotor = 64;
//sources differ for the gear ratio
const double GearRatio = 64; //amazon says this
//const double GearRatio = 63.68395; //this *may* be more accurate?
const double StepsPerRevolution = StepsPerRevOfMainRotor * GearRatio;

const int MaxRPM = 15;
const float MaxSpeed = StepsPerRevolution * MaxRPM / 60.0f;

AccelStepper stepper(AccelStepper::FULL4WIRE, 32, 26, 25, 27);

//14 teeth on stepper motor, 12 teeth on minute hand
const double ClockGearRatio = 14 / 12.0;

void ZeroHands()
{
  //find our zero point
  stepper.setSpeed(MaxSpeed);

  long hourPosition = -1;
  long minutePosition = -1;
  int numMinutePositions = 0;
  long minutePositions[12];
  bool wasMinuteTriggered = false;
  //go round and round until we hit both sensors
  while (hourPosition < 0 && minutePosition < 0)
  {
    //round and round she goes, where she stops...well that's what we want to find out
    if (!stepper.runSpeed())
    {
      yield();
      continue;
    }

    if (hourPosition < 0 && digitalRead(HOUR_SENSOR_PIN) == LOW)
    {
      hourPosition = stepper.currentPosition();
      Serial.println("Hit hour sensor = " + String(hourPosition));
    }
    if (minutePosition < 0 && digitalRead(MINUTE_SENSOR_PIN) == LOW)
    {
      minutePosition = stepper.currentPosition();
      Serial.println("Hit minute sensor = " + String(minutePosition));
    }
    if (digitalRead(MINUTE_SENSOR_PIN) == LOW)
    {
      if (!wasMinuteTriggered)
      {
        wasMinuteTriggered = true;
        if (numMinutePositions < 12)
        {
          Serial.println("Tracked minute sensor = " + String(stepper.currentPosition()));
          minutePositions[numMinutePositions] = stepper.currentPosition();
          numMinutePositions++;
        }
        //wait a bit to indicate a hit
        delay(1000);
      }
    }
    else
      wasMinuteTriggered = false;
    //indicate when we hit sensors
    if (digitalRead(MINUTE_SENSOR_PIN) == LOW || digitalRead(HOUR_SENSOR_PIN) == LOW)
      digitalWrite(BUILTIN_LED, LOW);
    else
      digitalWrite(BUILTIN_LED, HIGH);
  }
  Serial.println("Found sensors:");
  Serial.println("  Hour = " + String(hourPosition));
  Serial.println("  Minute = " + String(minutePosition));

  Serial.println("  or...");
  for (int i = 0; i < numMinutePositions; i++)
  {
    Serial.println("    " + String(minutePositions[i]));
  }
}

void SetTime()
{
  ZeroHands();
}

void setup() {
  // put your setup code here, to run once:
  pinMode(HOUR_SENSOR_PIN, INPUT_PULLUP);
  pinMode(MINUTE_SENSOR_PIN, INPUT_PULLUP);

  //TODO: start wifi
  WiFi.begin("cjweiland", "areallygoodkey");

  waitForSync();

  Timezone myTZ;

	// Provide official timezone names
	// https://en.wikipedia.org/wiki/List_of_tz_database_time_zones
	myTZ.setLocation("America/New_York");

  SetTime();
}

void loop()
{
  // put your main code here, to run repeatedly:
  events(); //for ezTime
  stepper.run(); //for AccelStepper
}