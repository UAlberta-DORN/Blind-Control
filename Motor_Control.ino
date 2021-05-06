int x = 0;
bool aSent = false;
bool bSent = false;


#include <AccelStepper.h>
#define dirPin1 15
#define stepPin1 2
#define sleepPin1 13
#define dirPin2 27
#define stepPin2 14
#define sleepPin2 12
#define motorInterfaceType 1

// Define a stepper and the pins it will use
AccelStepper stepper1 = AccelStepper(motorInterfaceType, stepPin1, dirPin1);
AccelStepper stepper2 = AccelStepper(motorInterfaceType, stepPin2, dirPin2);



void setup() {
 Serial.begin(115200);
 Serial.setTimeout(1);
 stepper1.setMaxSpeed(300);
 stepper1.setAcceleration(1000);
 stepper2.setMaxSpeed(500);
 stepper2.setAcceleration(200);
 pinMode(sleepPin1, OUTPUT);
 digitalWrite(sleepPin1, HIGH);
 pinMode(sleepPin2, OUTPUT);
 digitalWrite(sleepPin2, LOW);
 delay(100);
}

void loop() {
  stepper1.run();
  stepper2.run();
  if (Serial.available() > 0) {
    delay(100);
    char c = Serial.read();
    x = Serial.readString().toInt();
    if (c == 'A'){
      stepper1.moveTo(x);
      //Serial.println('a');
      aSent = false;
    }else if(c == 'B'){
      stepper2.moveTo(x);
      //Serial.println('b');
      bSent = false;
    }
    Serial.println(x);
  }
  if (stepper1.distanceToGo() == 0){
    stepper1.setCurrentPosition(0);
    if (!aSent){
      Serial.println('P');
      aSent = true;
    }
  }
  if (stepper2.distanceToGo() == 0){
    stepper2.setCurrentPosition(0);
    digitalWrite(sleepPin2, LOW);
    if (!bSent){
      Serial.println('Q');
      bSent = true;
    }
  }else{
    digitalWrite(sleepPin2, HIGH);
  }
}
