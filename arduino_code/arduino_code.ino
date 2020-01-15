//---------------------------------------------------//
//---------------------Library-----------------------//
//---------------------------------------------------//
#include <Servo.h>
#include <Wire.h>
#include <SoftwareSerial.h>

#define gyroInterrupt 18;

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

int gyroServoBase = 0;
int gyroServoRoot = 0;

//---------------------Servos---------------------//
Servo baseServo;  // create servo object to control a servo
int baseServoSingal = 0;

Servo rootServo;  
int rootServoSingal = 0;

Servo midServo;  
int midServoSingal = 0;

Servo midServo_gyro;  
int midServoSingal_gyro = 0;

Servo gripperServo;
int gripperMoterSingnal = 90;

int servoOut = 0;
//---------------------Servo Variables---------------------//
int baseServoOut;
int rootServoOut;
int midServoOut;
int gripperServoOut;

//Gyro_Mid
int encoderPosCount_mid_gyro = 0;
int correctEncoderPosCount_mid_gyro = 0;
int aVal_mid_gyro;
boolean bCW_mid_gyro;

//---------------------Encoder Variables---------------------//
#define PinCLK 2
#define PinDT 3

static long virtualPosition=0;    // without STATIC it does not count correctly!!!

bool CLK = false;
bool DT = false;
bool allCLK[5] = {1, 0, 0, 1, 1};
bool allDT[5] = {1, 1, 0, 0, 1};
int count = 0;
int direc = 0;  //0 not known, 1 cw, 2 ccw
int countPlus, minusCount;

//---------------------Button VARIABLES---------------------//
int buttonPin = 19;
int buttonState = 0; //open
int last_button = 0;
int flag = 0;  //open state


//---------------------------------------------------//
//---------------------Set Up------------------------//
//---------------------------------------------------//

void setup() {
  //Servo
  baseServo.attach(11);  // attaches the servo on pin 11 to the servo object
  rootServo.attach(10);  // attaches the servo on pin 10 to the servo object
  midServo.attach(9);  // attaches the servo on pin 9 to the servo object
  gripperServo.attach(6);

  //button
  pinMode(buttonPin,INPUT);


  //Initial Servo Position
  baseServo.write(76);
  rootServo.write(54);
  midServo.write(18);
  gripperServo.write(map(10, 0, 20, 0, 180)); 
  
  //Serial
  Serial.begin (9600);

  //GyroScope Set Up
  Wire.begin();
  setUpMPU();
  Serial.println("Start Cali");
  callibrateGyroValues();
  
  timePresent = millis();
  Serial.println("Finish Cali");

  //Encoder Interrupt Setting
  pinMode(CLK,INPUT);
  pinMode(DT,INPUT);  
  attachInterrupt (0,mid_encoder_read,CHANGE);   // interrupt 0 is always connected to pin 2 on Arduino UNO
  attachInterrupt (1,mid_encoder_read,CHANGE);

  //GyroScope Interrupt Setting
  attachInterrupt (2,gyro_ISR,CHANGE);  //interrupt 2 is pin18

  //Button Interrupt Setting
  attachInterrupt (3,button_ISR,CHANGE);

}

//---------------------------------------------------//
//-----------------------Main------------------------//
//---------------------------------------------------//

void loop() {

  cli();
 
  gyro_servoSignalOut();  
  
  sei();
  
  delay (100);
}

//---------------------------------------------------//
//-----------------ISR Functions---------------------//
//---------------------------------------------------//



void gyro_ISR (){

  cli();
  Read_Gyro();   //Read Gyroscope
  sei();

}

void encoder_ISR (){

  cli();
  mid_encoder_read();  //Read Encoder
  sei();
  
}

void button_ISR(){

  cli();
  
  buttonState = digitalRead(buttonPin);
  
  if (buttonState < last_button){
      flag = 1 - flag;
    }
  
  if (flag == 0 ){
    gripperOpen();
  }
  
  if (flag == 1){
    gripperClose();
  }
  
  last_button = buttonState;
  
  sei();

}

//---------------------------------------------------//
//---------------------Functions---------------------//
//---------------------------------------------------//


//---------------------Encoder Correction Function---------------------//
//Corrent final reading when the encoder reading exceed 0-360 degree
int correctEncoder(int input){
    int output = input % 40;
    if (output<0){
      output = output + 40;
    }
    return output;
 }


//---------------------Encoder Half Control Function---------------------//
//Make the encoder output mirror to the 0-180 reading since the servo motor can only rotate 0-180 degree
int gyro_mid_halfcorrection_encoder(int input, int error){
    input = input*9;
    int output;

    if (input>=0 && input <= 180 - error){
      output = input + error;
    }
    else if (360- error<=input && input<360){
      output = input - (360-error);
    }
    else {
      output = 360-error - input;
    }

    return output;

}


//---------------------Gyro Mid Encoder Read Function---------------------//
void mid_encoder_read() {
cli(); 
  DT=digitalRead(PinDT);
  CLK=digitalRead(PinCLK);
  countPlus=count+1;
  minusCount=3-count;
  
  if (count==0){
    if(DT==allDT[countPlus]&&CLK==allCLK[countPlus]){
      direc = 1;
     // Serial.print(allDT[countPlus]);
    //  Serial.println(allCLK[countPlus]);
      count++;
    }
    else if(DT==allDT[minusCount]&&CLK==allCLK[minusCount]){
      direc = 2;
    //  Serial.print(allDT[minusCount]);
    //  Serial.println(allCLK[minusCount]);
      count++;
    }
  }
  if(DT==allDT[countPlus]&&CLK==allCLK[countPlus]&&direc==1){   //cw
  //  Serial.print(allDT[countPlus]);
  //  Serial.println(allCLK[countPlus]);
    if(count==3) {
            count=0;
            direc=0;
            virtualPosition++;
       //    Serial.print("---------------------   ");
            Serial.println(virtualPosition);
    }
    else if(count<3) count++;
  }
  else if(DT==allDT[minusCount]&&CLK==allCLK[minusCount]&&direc==2){    //ccw
   // Serial.print(allDT[minusCount]);
   // Serial.println(allCLK[minusCount]);
    if(count==3) {
            count=0;
            direc=0;
            virtualPosition--;
      //      Serial.print("---------------------   ");
            Serial.println(virtualPosition);
    }
    else if(count<3) count++;
  }
  sei();
}


void gripperOpen(){
  //mid_halfcorrection_encoder(correctEncoderPosCount_mid);
  servoOut = map(10, 0, 20, 0, 180);     // scale it to u se it with the servo (value between 0 and 180)
  gripperServo.write(servoOut);                  // sets the servo position according to the scaled value 

}

void gripperClose(){
  //mid_halfcorrection_encoder(correctEncoderPosCount_mid);
  servoOut = map(0, 0, 20, 0, 180);     // scale it to u se it with the servo (value between 0 and 180)
  gripperServo.write(servoOut);                  // sets the servo position according to the scaled value 

}


//---------------------Servo Reading Correction Function---------------------//
int base_correction_gyro(int input, int error){
    int output;

    if (input>=0 && input <=error){
      output = -input + error;
    }
    else if (error+180<=input && input<360){
      output = 360 + error - input;
    }
    else {
      output = input - error;
    }

    return output;

}

int root_correction_gyro(int input, int error){
    int output;

    if (input>=0 && input <=180-error){
      output = input + error;
    }
    else if (360-error<=input && input<360){
      output = error + input -360 ;
    }
    else {
      output = 360-error-input;
    }

    return output;

}




//---------------------Gyro Servo Siugnal Out---------------------//
void gyro_servoSignalOut_mid(){
  midServoSingal_gyro = gyro_mid_halfcorrection_encoder(correctEncoderPosCount_mid_gyro,27);
  servoOut = map(midServoSingal_gyro, 0, 180, 0, 180);     // scale it to u se it with the servo (value between 0 and 180)
  midServo.write(servoOut);                  // sets the servo position according to the scaled value 
}


void gyro_servoSignalOut(){
  int gyroBaseError = 76;
  gyroServoBase = base_correction_gyro(angleZ,gyroBaseError);
  baseServoOut = map(gyroServoBase, 0, 180, 0, 180);     // scale it to u se it with the servo (value between 0 and 180)
  baseServo.write(baseServoOut);                  // sets the servo position according to the scaled value 

  int gyroRootError = 70;
  gyroServoRoot = root_correction_gyro(angleX,gyroRootError);
  rootServoOut = map(gyroServoRoot, 0, 180, 0, 180);     // scale it to u se it with the servo (value between 0 and 180)
  rootServo.write(rootServoOut);                  // sets the servo position according to the scaled value 

  gyro_servoSignalOut_mid();
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
