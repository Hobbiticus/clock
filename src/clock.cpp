#include <Arduino.h>
#include <AccelStepper.h>
#include <ezTime.h>
#include <WiFi.h>

#define HOUR_SENSOR_PIN 21
#define MINUTE_SENSOR_PIN 18
#define STEPPER_ENABLE_PIN 13

//#define USE_FET

const unsigned int StepsPerRevOfMainRotor = 32;
//sources differ for the gear ratio
const double GearRatio = 64; //amazon says this
//const double GearRatio = 63.68395; //this *may* be more accurate?
const double StepsPerRevolution = StepsPerRevOfMainRotor * GearRatio;

const int MaxRPM = 16; //absolute max is probably about 17?
const float MaxSpeed = StepsPerRevolution * MaxRPM / 60.0f;

AccelStepper stepper(AccelStepper::FULL4WIRE, 27, 25, 26, 33);
Timezone myTZ;

//14 teeth on stepper motor, 12 teeth on minute hand
const double ClockGearRatio = 12 / 14.0;

const double StepsPerHour = ClockGearRatio * StepsPerRevolution;
const double StepsPerMinute = StepsPerHour / 60;
const double StepsPerSecond = StepsPerMinute / 60;
const double StepsPer12Hours = StepsPerHour * 12;

void HomeHands()
{
  Serial.println("Homing hands...");
  stepper.setSpeed(MaxSpeed);

  //if we see both sensors are set already, move back a bit
  if (digitalRead(MINUTE_SENSOR_PIN) == LOW && digitalRead(HOUR_SENSOR_PIN) == LOW)
  {
    stepper.move(-10 * StepsPerMinute);
    while (stepper.run())
    {}
  }

  //find our zero point
  //go round and round until we hit both sensors
#ifdef USE_FET
  digitalWrite(STEPPER_ENABLE_PIN, LOW);
#else
  stepper.enableOutputs();
#endif
  while (true)
  {
    digitalWrite(BUILTIN_LED, digitalRead(MINUTE_SENSOR_PIN) == LOW || digitalRead(HOUR_SENSOR_PIN) == LOW ? HIGH : LOW);
    //round and round she goes, where she stops...well that's what we want to find out
    if (!stepper.runSpeed())
    {
      yield();
    }

    if (digitalRead(HOUR_SENSOR_PIN) == LOW && digitalRead(MINUTE_SENSOR_PIN) == LOW)
      break;
  }

  //move just a little farther to (hopefully) exactly 12:00
  //stepper.move(StepsPerMinute * 0.5);
  //while (stepper.run())
  //{}
  delay(100);
#ifdef USE_FET
  digitalWrite(STEPPER_ENABLE_PIN, HIGH);
#else
  stepper.disableOutputs();
#endif

  //to make the math a little easier...
  if (myTZ.hourFormat12() > 6)
    //so we go backwards to the correct time
    stepper.setCurrentPosition(StepsPer12Hours);
  else
    //so we got forwards to the correct time
    stepper.setCurrentPosition(0);
  Serial.println("Hands are homed!");
  delay(1000);
}

void MoveToCurrentTime()
{
  //need actual time/current time and mapping to/from current time and target stepper position
  uint8_t hour = myTZ.hourFormat12();
  uint8_t minute = myTZ.minute();
  uint8_t second = myTZ.second();
  //uint16_t ms = myTZ.ms(); //maybe?
  Serial.println("Time is now: " + String(hour) + ":" + String(minute) + ":" + String(second));

  long seconds = hour * 60 * 60 + minute * 60 + second;
  long pos = seconds * StepsPerSecond;
  //don't do anything if there is no place to go
  if (pos == stepper.currentPosition())
    return;

  if (pos < StepsPerHour && stepper.currentPosition() > 11 * StepsPerHour)
  {
    //crossed 12:00 - need to fix the math so we don't rewind
    stepper.setCurrentPosition(stepper.currentPosition() - StepsPer12Hours);
  }

#ifdef USE_FET
  digitalWrite(STEPPER_ENABLE_PIN, LOW);
#else
  stepper.enableOutputs();
  stepper.moveTo(stepper.currentPosition());
  stepper.run();
  //delay(100);
#endif
  delay(100);
  if (pos < stepper.currentPosition())
  {
    //move back a little further
    stepper.moveTo(pos - StepsPerMinute * 5);
    while (stepper.run())
    {
      digitalWrite(BUILTIN_LED, digitalRead(MINUTE_SENSOR_PIN) == LOW || digitalRead(HOUR_SENSOR_PIN) == LOW ? HIGH : LOW);
    }
  }
  Serial.println("Seconds passed the hour = " + String(seconds));
  Serial.println("Moving to step position " + String(pos));

  stepper.moveTo(pos);
  while (stepper.run())
  {
    digitalWrite(BUILTIN_LED, digitalRead(MINUTE_SENSOR_PIN) == LOW || digitalRead(HOUR_SENSOR_PIN) == LOW ? HIGH : LOW);
  }

  digitalWrite(BUILTIN_LED, digitalRead(MINUTE_SENSOR_PIN) == LOW || digitalRead(HOUR_SENSOR_PIN) == LOW ? HIGH : LOW);
  delay(100);
#ifdef USE_FET
  digitalWrite(STEPPER_ENABLE_PIN, HIGH);
#else
  delay(100);
  stepper.disableOutputs();
#endif
}

void setup()
{
  Serial.begin(115200);
  Serial.println("Setting up...");

  pinMode(HOUR_SENSOR_PIN, INPUT_PULLUP);
  pinMode(MINUTE_SENSOR_PIN, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  stepper.setMaxSpeed(MaxSpeed);
  stepper.setAcceleration(200);
#ifdef USE_FET
  pinMode(STEPPER_ENABLE_PIN, OUTPUT);
#else
  //stepper.setEnablePin(STEPPER_ENABLE_PIN);
#endif

  //start wifi
  WiFi.mode(WIFI_STA);
  WiFi.begin("cjweiland", "areallygoodkey");
  while (WiFi.status() != WL_CONNECTED)
    delay(100);

  waitForSync();

	// Provide official timezone names
	// https://en.wikipedia.org/wiki/List_of_tz_database_time_zones
	myTZ.setLocation("America/New_York");

  HomeHands();
  MoveToCurrentTime();
}

void loop()
{
  events(); //for ezTime

  MoveToCurrentTime();

  delay(100);
}
