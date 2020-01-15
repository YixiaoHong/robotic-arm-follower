//---------------------------------------------------//
//---------------------Library-----------------------//
//---------------------------------------------------//
#include <Servo.h>
#include <Wire.h>
#include <SoftwareSerial.h>



//---------------------------------------------------//
//-------------------Variables-----------------------//
//---------------------------------------------------//

//---------------------Gyros---------------------//
long accelX, accelY, accelZ;
float gForceX, gForceY, gForceZ;

long gyroXCalli = 0, gyroYCalli = 0, gyroZCalli = 0;
long gyroXPresent = 0, gyroYPresent = 0, gyroZPresent = 0;
long gyroXPast = 0, gyroYPast = 0, gyroZPast = 0;
float rotX, rotY, rotZ;

float angelX = 0, angelY = 0, angelZ = 0;

int angleZ = 0;
int angleX = 0;
int angleY = 0;

long timePast = 0;
long timePresent = 0;

//---------------------Servos---------------------//
Servo baseServo;  // create servo object to control a servo
int baseServoSingal = 8;

Servo rootServo;  
int rootServoSingal = 6;

Servo midServo;  
int midServoSingal = 2;

//---------------------Base Servo Variables---------------------//
int servoOut;

//Base
int encoderPosCount_base = 0;
int correctEncoderPosCount_base = 0;
int aVal_base;
boolean bCW_base;

//Root
int encoderPosCount_root = 0;
int correctEncoderPosCount_root = 0;
int aVal_root;
boolean bCW_root;

//Mid
int encoderPosCount_mid = 0;
int correctEncoderPosCount_mid = 0;
int aVal_mid;
boolean bCW_mid;

//---------------------Encoder Variables---------------------//
//Base
int pinA = 6;  // Connected to CLK on KY-040
int pinB = 7;  // Connected to DT on KY-040
int pinALast;

//Root
int pinC = 5;  // Connected to CLK on KY-040
int pinD = 4;  // Connected to DT on KY-040
int pinCLast;

//Mid
int pinE = 2;  // Connected to CLK on KY-040
int pinF = 3;  // Connected to DT on KY-040
int pinELast;


//---------------------TIMER VARIABLES---------------------//
long t_now = 0;
long t_last_print = 0;
int T_sample = 20;                                        // sample time in milliseconds (ms)
int T_print = 1000;                                       // sample print to monitor time in milliseconds (ms)


//---------------------------------------------------//
//---------------------Set Up------------------------//
//---------------------------------------------------//

void setup() {
  //Base Servo
  baseServo.attach(11);  // attaches the servo on pin 9 to the servo object
  rootServo.attach(10);  // attaches the servo on pin 9 to the servo object
  midServo.attach(9);  // attaches the servo on pin 9 to the servo object

  //Initial Servo Position
  baseServo.write(map(8, 0, 20, 0, 180));
  rootServo.write(map(6, 0, 20, 0, 180));
  midServo.write(map(2, 0, 20, 0, 180));

  //Base Encoder
  pinMode (pinA,INPUT);
  pinMode (pinB,INPUT);
  pinALast = digitalRead(pinA);

  //Root Encoder
  pinMode (pinC,INPUT);
  pinMode (pinD,INPUT);
  pinCLast = digitalRead(pinC); 

  //Mid Encoder
  pinMode (pinE,INPUT);
  pinMode (pinF,INPUT);
  pinELast = digitalRead(pinE);    
  
  //Serial
  Serial.begin (9600);

  //GyroScope Set Up
  Wire.begin();
  setUpMPU();
  Serial.println("Start Cali");
  callibrateGyroValues();
  timePresent = millis();
  Serial.println("Finish Cali");

  

}

//---------------------------------------------------//
//-----------------------Main------------------------//
//---------------------------------------------------//


void loop() {

  //Read Gyroscope
  Read_Gyro();
  
  // Read the encoder 
  base_encoder_read();
  root_encoder_read();
  mid_encoder_read();
  
  // Write control signal to servo motor
  servoSignalOut();

  // Print sensed angle and setpoint angle
  print_results();


  
}

//---------------------------------------------------//
//---------------------Functions---------------------//
//---------------------------------------------------//


//---------------------Encoder Correction Function---------------------//
int correctEncoder(int input){
    int output = input % 40;
    if (output<0){
      output = output + 40;
    }
    return output;
 }


//---------------------Encoder Half Control Function---------------------//
void base_halfcorrection(int input){
    if (input>=0 && input <=19){
      baseServoSingal = input;
    }
    else {
      baseServoSingal = 39 - input;
    }
}

void root_halfcorrection(int input){
    if (input>=0 && input <=19){
      rootServoSingal = input;
    }
    else {
      rootServoSingal = 39 - input;
    }
}

void mid_halfcorrection(int input){
    if (input>=0 && input <=19){
      midServoSingal = input;
    }
    else {
      midServoSingal = 39 - input;
    }
}


//---------------------Base Encoder Read Function---------------------//
void base_encoder_read() {
  aVal_base = digitalRead(pinA);
  if (aVal_base != pinALast){ // Means the knob is rotating
  // if the knob is rotating, we need to determine direction
  // We do that by reading pin B.
  if (digitalRead(pinB) != aVal_base) {  // Means pin A Changed first - We're Rotating Clockwise
    encoderPosCount_base ++;
    bCW_base = true;
  } else {// Otherwise B changed first and we're moving CCW
    bCW_base = false;
    encoderPosCount_base--;
  }
  correctEncoderPosCount_base = correctEncoder(encoderPosCount_base);   
  } 
  pinALast = aVal_base;
}


//---------------------Root Encoder Read Function---------------------//
void root_encoder_read() {
  aVal_root = digitalRead(pinC);
  if (aVal_root != pinCLast){ // Means the knob is rotating
  // if the knob is rotating, we need to determine direction
  // We do that by reading pin B.
  if (digitalRead(pinD) != aVal_root) {  // Means pin A Changed first - We're Rotating Clockwise
    encoderPosCount_root ++;
    bCW_root = true;
  } else {// Otherwise B changed first and we're moving CCW
    bCW_root = false;
    encoderPosCount_root--;
  }
  correctEncoderPosCount_root = correctEncoder(encoderPosCount_root);   
  } 
  pinCLast = aVal_root;
}


//---------------------Mid Encoder Read Function---------------------//
void mid_encoder_read() {
  aVal_mid = digitalRead(pinE);
  if (aVal_mid != pinELast){ // Means the knob is rotating
  // if the knob is rotating, we need to determine direction
  // We do that by reading pin B.
  if (digitalRead(pinF) != aVal_mid) {  // Means pin A Changed first - We're Rotating Clockwise
    encoderPosCount_mid ++;
    bCW_mid = true;
  } else {// Otherwise B changed first and we're moving CCW
    bCW_mid = false;
    encoderPosCount_mid--;
  }
  correctEncoderPosCount_mid = correctEncoder(encoderPosCount_mid);   
  } 
  pinELast = aVal_mid;
}

//---------------------Servo Siugnal Out---------------------//
void servoSignalOut(){
  base_halfcorrection(correctEncoderPosCount_base);
  servoOut = map(baseServoSingal, 0, 20, 0, 180);     // scale it to u se it with the servo (value between 0 and 180)
  baseServo.write(servoOut);                  // sets the servo position according to the scaled value 

  root_halfcorrection(correctEncoderPosCount_root);
  servoOut = map(rootServoSingal, 0, 20, 0, 180);     // scale it to u se it with the servo (value between 0 and 180)
  rootServo.write(servoOut);                  // sets the servo position according to the scaled value 

  mid_halfcorrection(correctEncoderPosCount_mid);
  servoOut = map(midServoSingal, 0, 20, 0, 180);     // scale it to u se it with the servo (value between 0 and 180)
  midServo.write(servoOut);                  // sets the servo position according to the scaled value 
}



//---------------------Print Result Function---------------------//
void print_results() {

  t_now = millis();
  
  if (t_now - t_last_print >= T_print){
    t_last_print = t_now;
    
    // ==================INSERT PRINT ALGORITHM HERE ==================== //
    Serial.println("correctEncoderPosCount_base");
    Serial.println(correctEncoderPosCount_base); 

    Serial.println("correctEncoderPosCount_root");
    Serial.println(correctEncoderPosCount_root); 

    Serial.println("correctEncoderPosCount_mid");
    Serial.println(correctEncoderPosCount_mid); 

    // ==================INSERT PRINT ALGORITHM HERE ==================== //

    
    }
}


//---------------------Gyro Functions---------------------//
void setUpMPU() {
  // power management
  Wire.beginTransmission(0b1101000);          // Start the communication by using address of MPU
  Wire.write(0x6B);                           // Access the power management register
  Wire.write(0b00000000);                     // Set sleep = 0
  Wire.endTransmission();                     // End the communication

  // configure gyro
  Wire.beginTransmission(0b1101000);
  Wire.write(0x1B);                           // Access the gyro configuration register
  Wire.write(0b00000000);
  Wire.endTransmission();

  // configure accelerometer
  Wire.beginTransmission(0b1101000);
  Wire.write(0x1C);                           // Access the accelerometer configuration register
  Wire.write(0b00000000);
  Wire.endTransmission();  
}

void callibrateGyroValues() {
    for (int i=0; i<5000; i++) {
      getGyroValues();
      gyroXCalli = gyroXCalli + gyroXPresent;
      gyroYCalli = gyroYCalli + gyroYPresent;
      gyroZCalli = gyroZCalli + gyroZPresent;
    }
    gyroXCalli = gyroXCalli/5000;
    gyroYCalli = gyroYCalli/5000;
    gyroZCalli = gyroZCalli/5000;
}

void readAndProcessAccelData() {
  Wire.beginTransmission(0b1101000); 
  Wire.write(0x3B); 
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); 
  while(Wire.available() < 6);
  accelX = Wire.read()<<8|Wire.read(); 
  accelY = Wire.read()<<8|Wire.read(); 
  accelZ = Wire.read()<<8|Wire.read(); 
  processAccelData();
}

void processAccelData() {
  gForceX = accelX/16384.0;
  gForceY = accelY/16384.0; 
  gForceZ = accelZ/16384.0;
}

void readAndProcessGyroData() {
  gyroXPast = gyroXPresent;                                   // Assign Present gyro reaging to past gyro reading
  gyroYPast = gyroYPresent;                                   // Assign Present gyro reaging to past gyro reading
  gyroZPast = gyroZPresent;                                   // Assign Present gyro reaging to past gyro reading
  timePast = timePresent;                                     // Assign Present time to past time
  timePresent = millis();                                     // get the current time in milli seconds, it is the present time
  
  getGyroValues();                                            // get gyro readings
  getAngularVelocity();                                       // get angular velocity
  calculateAngle();                                           // calculate the angle  
}

void getGyroValues() {
  Wire.beginTransmission(0b1101000);                          // Start the communication by using address of MPU 
  Wire.write(0x43);                                           // Access the starting register of gyro readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6);                              // Request for 6 bytes from gyro registers (43 - 48)
  while(Wire.available() < 6);                                // Wait untill all 6 bytes are available
  gyroXPresent = Wire.read()<<8|Wire.read();                  // Store first two bytes into gyroXPresent
  gyroYPresent = Wire.read()<<8|Wire.read();                  // Store next two bytes into gyroYPresent
  gyroZPresent = Wire.read()<<8|Wire.read();                  //Store last two bytes into gyroZPresent
}

void getAngularVelocity() {
  rotX = gyroXPresent / 131.0;                                
  rotY = gyroYPresent / 131.0; 
  rotZ = gyroZPresent / 131.0;
}

void calculateAngle() {  
  // same equation can be written as 
  // angelZ = angelZ + ((timePresentZ - timePastZ)*(gyroZPresent + gyroZPast - 2*gyroZCalli)) / (2*1000*131);
  // 1/(1000*2*131) = 0.00000382
  // 1000 --> convert milli seconds into seconds
  // 2 --> comes when calculation area of trapezium
  // substacted the callibated result two times because there are two gyro readings
  angelX = angelX + ((timePresent - timePast)*(gyroXPresent + gyroXPast - 2*gyroXCalli)) * 0.00000382;
  angelY = angelY + ((timePresent - timePast)*(gyroYPresent + gyroYPast - 2*gyroYCalli)) * 0.00000382;
  angelZ = angelZ + ((timePresent - timePast)*(gyroZPresent + gyroZPast - 2*gyroZCalli)) * 0.00000382;
}

void Read_Gyro(){
  readAndProcessAccelData();
  readAndProcessGyroData();
  angleZ = angelZ;
  angleZ = angleZ % 360;
  if (angleZ > 0 ){
    angleZ = 360 - angleZ;
  }
  else {
    angleZ = - angleZ;
  }

  angleX = angelX;
  angleX = angleX % 360;
  if (angleX > 0 ){
    angleX = 360 - angleX;
  }
  else {
    angleX = - angleX;
  }

  angleY = angelY;
  angleY = angleY % 360;
  if (angleY > 0 ){
    angleY = 360 - angleY;
  }
  else {
    angleY = - angleY;
  }

  //Serial.println(angleZ);
  //Serial.println(angleX);
  //Serial.println(angleY);
  
  }
