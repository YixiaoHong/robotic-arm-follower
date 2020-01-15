
volatile boolean TurnDetected;
volatile boolean up;

const int PinCLK=2;                   // Used for generating interrupts using CLK signal
const int PinDT=3;                    // Used for reading DT signal

unsigned long oldTime = millis();
unsigned long currentTime;

static long virtualPosition=0;    // without STATIC it does not count correctly!!!

void isr ()  {    // Interrupt service routine is executed when a HIGH to LOW transition is detected on CLK
 cli();
 if (digitalRead(PinCLK))
   up = digitalRead(PinDT);
 else
   up = !digitalRead(PinDT);
 TurnDetected = true;
 currentTime = millis();
 Serial.println("--------------isr");
 sei();
}


void setup ()  {
 pinMode(PinCLK,INPUT);
 pinMode(PinDT,INPUT);  
 attachInterrupt (0,isr,FALLING);   // interrupt 0 is always connected to pin 2 on Arduino UNO
 Serial.begin (9600);
 Serial.println("Start");
}

void loop ()  {
 if (TurnDetected)  {        // do this only if rotation was detected
   cli();
   if (currentTime-oldTime > 60){
      Serial.print(oldTime);
      Serial.print("\t");
      Serial.print(currentTime);
      Serial.print("\t");
      Serial.print(currentTime-oldTime);
      Serial.print("\n");
     if (up)
       virtualPosition++;
     else
       virtualPosition--;
   Serial.println (virtualPosition);
   oldTime=currentTime;
   }
   TurnDetected = false;          // do NOT repeat IF loop until new rotation detected
 sei();
 }
}
