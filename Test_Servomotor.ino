#include <Servo.h>


const uint8_t SERVO_PIN   = 10;     
const uint8_t h_PIN       = 9;     
const uint8_t test_PIN       = 8;     
const float   MOVE_SPEED_DPS  = 70.0f;  
const unsigned long CHANGE_INTERVAL = 2000;
int pos=0;   


Servo          myServo;
Servo myservo_h; 
int            currentAngle   = 0;
int            targetAngle    = 0;
unsigned long  lastStepTime   = 0;
unsigned long  lastChangeTime = 0;
uint8_t        stage          = 1;   


void updateServo(Servo &servo,
                 int   &currentAng,
                 int   &targetAng,
                 unsigned long &lastStep,
                 float speedDPS)
{
  if (currentAng == targetAng) return;
  unsigned long stepInterval = (unsigned long)(1000.0f / max(speedDPS, 0.1f));
  unsigned long now = millis();
  if (now - lastStep < stepInterval) return;

  int dir = (targetAng > currentAng) ? 1 : -1;
  currentAng += dir;
  servo.write(currentAng);
  lastStep = now;
}

void setup() {
  Serial.begin(115200);
  myServo.attach(SERVO_PIN);
  myServo.attach(h_PIN);
  myServo.attach(test_PIN);

  myServo.write(0);
  currentAngle   = 0;
  targetAngle    = 0;
  lastStepTime   = millis();
  lastChangeTime = millis();
}

void loop() {
  unsigned long now = millis();

  updateServo(myServo, currentAngle, targetAngle, lastStepTime, MOVE_SPEED_DPS);

  if (currentAngle == targetAngle && now - lastChangeTime >= CHANGE_INTERVAL) {
    lastChangeTime = now;

      myservo_h.write(pos);

      if (stage == 1) {
     //0 -> 90
      targetAngle = 90;
      stage = 2;
      Serial.println(F("Stage 1 → moving to 90°")); }
      else if (stage == 2) {
      //90 -> 0
      targetAngle = 0;
      stage = 3;
      Serial.println(F("Stage 2 → moving to 0°"));
  }

}
