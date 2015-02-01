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

int motorAPin1 = 7;
int motorAPin2 = 8;
int motorAPinPWM = 9;

int motorBPin1 = 4;
int motorBPin2 = 5;
int motorBPinPWM = 6;

//int potencPin = A0;

int joyXpin = A0;
int joyYpin = A1;

// variables will change:
int buttonState = 0;         // variable for reading the pushbutton status
int buttonPressed = LOW;
int buttonUnpressed = HIGH;

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
  
  Serial.println(joyX);
  Serial.println(joyY);
  Serial.println(" ");

  int mLeft = joyY; // A
  int mRight = JoyY; // B
  
  if (joyX > 0) {
    mRight = mRight - joyX;
    
  }
  if (joyX < 0) {
    mLeft = mLeft - joyX; 
  }
  
 //     digitalWrite(pin1, HIGH);
  //    digitalWrite(pin2, LOW);

  //    analogWrite(pinPwm, abs(power));
  
  
  
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
  
  //Serial.println(potencValue);// + " " + outputValue);
  
  delay(100);
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
