/*

MCG3307 · Control Systems - Project
Adapted from encoder_position.ino (author: Prof. Davide Spinello) and DC_motor_controller.ino (author: Prof. T.K. Uchida)

*/

#include <util/atomic.h> // for the ATOMIC_BLOCK macro

// DEFINING ELECTRICAL CONNECTIONS //

/*

Encoder:  

If the encoder is spinning clockwise output A is triggered first
If the encoder is spinning counterclockwise output B is triggered first

*/

#define encoderPinA 2
#define encoderPinB 3

/*

H-Bridge:

Enable Pin (Green Lead): H-Brdige 1 -> Digital 10
- When HIGH (5V), motor spins, when LOW motor is turned off
- We use this pin to PWM the motor -> controls the speed
- We write to this pin using analogWrite() with a val. between 0 and 255 to PWM the motor 

Control Pin 1 (Orange Lead): H-Bridge 2 -> Digital 4
Control Pin 2 (Orange Lead): H-Bridge 7 -> Digital 5
- Control pins are used to control the direction the motor spins
- When LOW and HIGH -> 1 direction, HIGH and LOW -> other direction, both HIGH or LOW, motor stops spinning
- We write to these pins using digitalWrite() with HIGH or LOW values

*/

#define controlPin1 4
#define controlPin2 5
#define enablePin 10

// GLOBAL VARIABLES //

//For finding and storing encoder angle and platform displacement
const double encoderPinionCircum = 150.796447372; 
double count, encoderAngle, displacement = 0; 
boolean A, B;
byte state, statepos;

//For computing raw control signal
const double K_P = 8.0*8.0; // changed to 8.0*2 for Task 4 
const double K_I = 0.1*8.0; // changed to 0.1*2 for Task 4 
const double K_D = 0.2*8.0; // changed to 0.2*2 for Task 4  
unsigned long currTime, prevTime = 0;
double desiredDisp = 0; //holds value of reference signal at a given time
double dispError, prevError = 0; //current and previous error of platform displacement
double dispErrorDeriv, dispErrorSum = 0; //derivative and ~integral of error signal
double controlSignal = 0; 

//For transforming (bounding) the raw control signal
const double acceptableError = 0.1; //acceptable steady state error
const double minSpeedSignal = 150.0; //the min value we can write to the H-bridge enable pin that moves the platform
const double maxSpeedSignal = 255.0; 
int dirControlSignal = 0; 
double speedControlSignal = 0;

// SETUP //

void setup() {
  
  Serial.begin(9600);
  
  //Encoder pin modes
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);
  
  //Motor pin modes
  pinMode(controlPin1, OUTPUT);
  pinMode(controlPin2, OUTPUT);
  pinMode(enablePin, OUTPUT);

  //Attaching interrupt which triggers when A or B pin values rise/fall -> calls Achange() and Bchange() functions to update count, and later displacement
  attachInterrupt(0, Achange, CHANGE);
  attachInterrupt(1, Bchange, CHANGE);
    
}

// LOOP //

void loop() {
  
  //Test battery: 
  //setMotor(-1, 255, enablePin, controlPin1, controlPin2);
  
  //Get current displacement of platform (using atomic block so this doesn't interfere with interrupts)
  double disp = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    disp = displacement;
  }

  //Update time variables
  prevTime = currTime; 
  currTime = millis(); 

  //Get reference signal (i.e., desired displacement of platform at current time)
  desiredDisp = reference();

  //Compute error signal and its derrivative and integral
  prevError = dispError; 
  dispError = desiredDisp - disp; 
  dispErrorDeriv = (dispError-prevError) / ((currTime-prevTime) * 0.001); //*0.001 to convert from milliseconds to seconds
  dispErrorSum += dispError * ((currTime-prevTime) * 0.001);
  
  //Compute raw control signal
  controlSignal = K_P*dispError + K_I*dispErrorSum + K_D*dispErrorDeriv; 

  //Send transformed control signal to motor (and turn off motor after short period)
  transformControlSignal(); 
  setMotor(dirControlSignal, speedControlSignal, enablePin, controlPin1, controlPin2);
  delay(20);
  setMotor(0, 0, enablePin, controlPin1, controlPin2);

  //Log results
  // Serial.print("Current displacement: ");
  // Serial.println(disp);
  // Serial.print("Desired displacement: "); 
  // Serial.println(desiredDisp);
  // Serial.print("Current error: "); 
  // Serial.println(dispError);
  // Serial.print("Raw control signal: "); 
  // Serial.println(controlSignal); 
  // Serial.print("Direction control: ");
  // Serial.println(dirControlSignal);
  // Serial.print("Speed control: ");
  // Serial.println(speedControlSignal);
  // Serial.println("****");
  log(); 
  
  delay(5);
  
}

// READING ENCODER //

//Updates encoder angle and platform displacement global variables
void updateDisplacement () {
  encoderAngle = count/500.0*360.0/4.0;
  displacement = (encoderAngle/360)*encoderPinionCircum;
}

// Detecting encoder count (A)
void Achange() {
    A = digitalRead(encoderPinA);
    B = digitalRead(encoderPinB);
    
    if ((A==HIGH) && (B==HIGH)) state = 1;
    if ((A==HIGH) && (B==LOW))  state = 2;
    if ((A==LOW) && (B==LOW))   state = 3;
    if ((A==LOW) && (B==HIGH))  state = 4;
    
    switch (state)
    {
        case 1: {
            if (statepos == 2) count++;
            if (statepos == 4) count--;
            break;
        } case 2: {
            if (statepos == 1) count--;
            if (statepos == 3) count++;
            break;
        } case 3: {
            if (statepos == 2) count--;
            if (statepos == 4) count++;
            break;
        } default: {
            if (statepos == 1) count++;
            if (statepos == 3) count--;
        }
    }
    statepos = state;
    updateDisplacement(); 
}

// Detecting encoder count (B)
void Bchange() {
    A = digitalRead(encoderPinA);
    B = digitalRead(encoderPinB);
    
    if ((A==HIGH) && (B==HIGH)) state = 1;
    if ((A==HIGH) && (B==LOW))  state = 2;
    if ((A==LOW) && (B==LOW))   state = 3;
    if ((A==LOW) && (B==HIGH))  state = 4;
    
    switch (state)
    {
        case 1: {
            if (statepos == 2) count++;
            if (statepos == 4) count--;
            break;
        } case 2: {
            if (statepos == 1) count--;
            if (statepos == 3) count++;
            break;
        } case 3: {
            if (statepos == 2) count--;
            if (statepos == 4) count++;
            break;
        } default: {
            if (statepos == 1) count++;
            if (statepos == 3) count--;
        }
    }
    statepos = state;
    updateDisplacement(); 
}

// GENERATE REFERENCE SIGNAL //

double reference(){
  double ms_per_s = 1000.0; 
  if(currTime > (8*ms_per_s)) return 0;
  if(currTime > (5.71*ms_per_s)) return 40;
  if(currTime > (3.43*ms_per_s)) return 120; 
  if(currTime > (2.29*ms_per_s)) return 80;
  if(currTime > (1.14*ms_per_s)) return 40;
  return 0; 
  //return 50; 
}

// TRANSFORM RAW CONTROL SIGNAL //

void transformControlSignal(){
  //If current error is less than acceptable (steady-state) error -> stop the motor
  if (fabs(dispError) <= acceptableError){
    dirControlSignal = 0;
    speedControlSignal = 0;
  }
  else{
    double absControlSignal = fabs(controlSignal);
    dirControlSignal = (controlSignal > 0)? -1:1; 
    if(absControlSignal <= minSpeedSignal) speedControlSignal = minSpeedSignal; 
    if(absControlSignal >= maxSpeedSignal) speedControlSignal = maxSpeedSignal; 
    if(absControlSignal > minSpeedSignal && absControlSignal < maxSpeedSignal) speedControlSignal = absControlSignal; 
  }
}

// SETTING MOTOR //

//Used to set the direction and speed of the motor
//Given value for the direction ∈ [-1,1], speed ∈ [0, 255] and corresponding pins
void setMotor(int dir, int speed, int enPin, int contPin1, int contPin2){
  
  //Write speed (PWM)
  analogWrite(enPin, speed);
  
  //Write direction
  if(dir == 1){
    digitalWrite(contPin1, HIGH);
    digitalWrite(contPin2, LOW);
  }
  else if(dir == -1){
    digitalWrite(contPin1, LOW);
    digitalWrite(contPin2, HIGH);
  }
  else{
    digitalWrite(contPin1, LOW);
    digitalWrite(contPin2, LOW);
  }

}

// DATA COLLECTION //

void log(){
  Serial.print(currTime/1000.0); //ms -> s
  Serial.print(", ");
  Serial.println(displacement);
  Serial.print("\r");
}
