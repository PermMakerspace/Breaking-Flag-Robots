/*
  Button
 
 Turns on and off a light emitting diode(LED) connected to digital  
 pin 13, when pressing a pushbutton attached to pin 2. 
 
 
 The circuit:
 * LED attached from pin 13 to ground 
 * pushbutton attached to pin 2 from +5V
 * 10K resistor attached to pin 2 from ground
 
 * Note: on most Arduinos there is already an LED on the board
 attached to pin 13.
 
 
 created 2005
 by DojoDave <http://www.0j0.org>
 modified 30 Aug 2011
 by Tom Igoe
 
 This example code is in the public domain.
 
 http://www.arduino.cc/en/Tutorial/Button
 */

// constants won't change. They're used here to 
// set pin numbers:
const int buttonPinF = 2;
const int buttonPinB = 3;

int pinStb = 12;

int motorAPin1 = 8;
int motorAPin2 = 7;
int motorAPinPWM = 9;

int motorBPin1 = 5;
int motorBPin2 = 4;
int motorBPinPWM = 6;

//int potencPin = A0;

int joyXpin = A0;
int joyYpin = A1;

// variables will change:
int buttonState = 0;         // variable for reading the pushbutton status
int buttonPressed = LOW;
int buttonUnpressed = HIGH;

int deadZone = 15;

void setup() {

  // initialize the pushbutton pin as an input:
  pinMode(buttonPinF, INPUT);
  pinMode(buttonPinB, INPUT);
  
  pinMode(motorAPin1, OUTPUT);  
  pinMode(motorAPin2, OUTPUT);
  pinMode(motorAPinPWM, OUTPUT);    

  pinMode(motorBPin1, OUTPUT);  
  pinMode(motorBPin2, OUTPUT);
  pinMode(motorBPinPWM, OUTPUT);    

Serial.begin(9600);

//pinMode(potencPin, INPUT);

pinMode(joyXpin, INPUT);
pinMode(joyYpin, INPUT);
}

void loop(){
  // read the state of the pushbutton value:
  //buttonState = digitalRead(buttonPin);

  int potencValue = 1024;//analogRead(potencPin);

  int outputValue = map(potencValue, 0, 1023, 150, 255);  
  //analogWrite(motorPin, outputValue);
  
  int joyX = analogRead(joyXpin) - 512;
  int joyY = analogRead(joyYpin) - 512;
  
  Serial.print("x ");
  Serial.print(joyX);
  Serial.print(" y ");
  Serial.print(joyY);
  Serial.println(" ");

  int mLeft = 0;//joyY; // A
  int mRight = 0;//joyY; // B
  /*
  if (joyX > 10) {
    if (joyY > 10) {
        mRight = mRight - joyX;
        if (mRight < 0) mRight = 0;
    } 
    //if (joyY < -10) {
    //   mRight = mRight + joyX; 
    //}
  }
  if (joyX < -10) {
    if (joyY > 10) {
        mLeft = mLeft + joyX;
        if (mLeft < 0) mLeft = 0;
    } 
    //if (joyY < -10) {
    //   mLeft = mLeft - joyX; 
    //} 
  }
  */

  if (joyY > deadZone) { // forward
    //int diff = joyY - abs(joyX); // разница между направлением и поворотом. если поворот больше. то один мотор стопится вообще. если нет, из одного из моторов вычитается модуль х
    // но можно проще. из не поворачивающего мотора из y вычитать х и ограничивать нулем
  
    if (abs(joyX) < deadZone) {
      mLeft = joyY;
      mRight = joyY;
    } else if (joyX > 0) {
      mLeft = joyY;
      mRight = joyY  - abs(joyX);
      if (mRight < 0) {
        mRight = 0;
      }
    } else if (joyX < 0) {
      mRight = joyY;
      mLeft = joyY - abs(joyX);
      if (mLeft < 0) {
        mLeft = 0;
      }      
    }  
  }
  if (joyY < -deadZone) { // backward
    if (joyX < 100) {
      mRight = -255;
    }
    if (joyX > -100) {
      mLeft = -255;
    }    
  }
  if (abs(joyX) < deadZone && abs(joyY) < deadZone) {
    mLeft = 0;
    mRight = 0;
  }
  
  // razvorot po knopkam
  if (digitalRead(buttonPinB) == buttonPressed) {
    mLeft = 255;
    mRight = -255;
  }
  if (digitalRead(buttonPinF) == buttonPressed) {
    mLeft = -255;
    mRight = 255;
  }  
  
  Serial.print("l ");
  Serial.print(mLeft);
  Serial.print(" r ");
  Serial.print(mRight);
  Serial.println(" ");
  
  //mLeft = map(mLeft, -512, 512, -255, 255); 
  //mRight = map(mRight, -512, 512, -255, 255);
  
  if (abs(mLeft) < deadZone) {
    mLeft = 0;
  }
  if (abs(mRight) < deadZone) {
    mRight = 0;
  }
  
  Serial.print("L ");
  Serial.print(mLeft);
  Serial.print(" R ");
  Serial.print(mRight);
  Serial.println(" ");
  
  Serial.println(" ");
  
  setMotor(1, mLeft);
  setMotor(0, mRight);
    
  /*
  if (digitalRead(buttonPinB) == buttonPressed) {     
    setMotor(0, outputValue);      
    setMotor(1, outputValue);      
  } 
  else if (digitalRead(buttonPinF) == buttonPressed) {     
    setMotor(0, -outputValue);      
    setMotor(1, -outputValue);      
  } 
  else {
    setMotor(0, 0);
    setMotor(1, 0);    
  }
  */
  //Serial.println(potencValue);// + " " + outputValue);
  
  delay(50);
}

void setMotor(int motor, int power) {
  int pin1 = motorAPin1;
  int pin2 = motorAPin2;
  int pinPwm = motorAPinPWM;
  
  if (motor == 1) {
    pin1 = motorBPin1;
    pin2 = motorBPin2;
    pinPwm = motorBPinPWM;    
  }
  
  if (power > 255)
    power = 255;
  if (power < -255)
    power = -255;  
  
  if (power < 0) { // negative = run "backwards"

      digitalWrite(pinStb, HIGH);

      digitalWrite(pin1, LOW);
      digitalWrite(pin2, HIGH);

      analogWrite(pinPwm, abs(power));

  } else { // positive = run "forwards"

      digitalWrite(pinStb, HIGH);

      digitalWrite(pin1, HIGH);
      digitalWrite(pin2, LOW);

      analogWrite(pinPwm, abs(power));

  }
}
