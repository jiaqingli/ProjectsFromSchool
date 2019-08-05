#include <Wire.h>
#include <Adafruit_MotorShield.h>

// Create the motor shield object
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

// stepper motor with 200 steps
// motor port #2 (M3 and M4)
int steps = 200;
const int shakeTimetot = 9500;
const int rollTimetot = 6700; 

Adafruit_StepperMotor *myMotor = AFMS.getStepper(steps, 1);

// physical parameters
double rollerLength = 1.66*3.14; //inches
double gearRatio = 1.0;
double k = steps/(rollerLength*gearRatio);
double labelLength[9] = {0.73, 1.96, 0.97, 0.78, 1.77, 2.15, 0.95, 1.59, 12.6}; //inch
int stepNum[9] = {(int)(k*.7), (int)(k*1.9), (int)(k*0.9), (int)(k*0.7), (int)(k*1.7), (int)(k*2.1), (int)(k*0.9), (int)(k*1.5), (int)(k*12.6)} ;

//button as switch
const int buttonPin = 2;


void setup() {  
  Serial.begin(9600);
  
  //for(int i = 0; i < 8; i++){Serial.print(stepNum[i]);}
  
  AFMS.begin();  // create with the default frequency 1.6KHz
  
  myMotor->setSpeed(10);  // 10 rpm   

  int buttonState = LOW; 
  pinMode(buttonPin, INPUT);
  while(digitalRead(buttonPin) == LOW){Serial.println(digitalRead(buttonPin));}
  
  //go for 9  loops
  for(int i = 0; i < 9; i++){
    int n = stepNum[i];
    int rn = n/2;
    
    //shake 8 times
    if(i<8) {
      unsigned long shakeStart = millis();
      for(int j = 0; j < 6; j++){
        // back and forth
        myMotor->step(rn + 2, BACKWARD, DOUBLE); 
        myMotor->step(rn, FORWARD, DOUBLE); 
      }
      unsigned long shakeEnd = millis();
      unsigned long intervalShake = shakeTimetot - (shakeEnd - shakeStart);
      if(intervalShake > 0){
        delay(intervalShake);
      }

      //slide
      unsigned long slideStart = millis();
      myMotor->step(n, BACKWARD, DOUBLE);
      unsigned long slideEnd = millis();
      unsigned long intervalSlide = rollTimetot - (slideEnd - slideStart);
      if(intervalSlide > 0){
          delay(intervalSlide);  
      }
      
    } else{
      myMotor -> step(n, BACKWARD, SINGLE);
    }
  }

  /*
  for (int k = 0; k < 8; k++){
    int n = stepNum[k];
    int rn = n/2;
    for(int j = 0; j < 6; j++){
        // back and forth
        myMotor->step(rn + 1, BACKWARD, SINGLE); 
        myMotor->step(rn, FORWARD, SINGLE); 
     }
    myMotor->step(n, BACKWARD, DOUBLE);
  }
  */
}

void loop() {

}
