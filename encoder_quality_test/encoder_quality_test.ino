#define CLK 2
#define DATA 3
#define BUTTON A5
#define YLED A2

volatile boolean TurnDetected;
volatile boolean up;
unsigned long oldTime = millis();
unsigned long currentTime;

static long virtualPosition=0;

void isr ()  {    // Interrupt service routine is executed when a HIGH to LOW transition is detected on CLK
 cli();
 if (digitalRead(CLK))
   up = digitalRead(DATA);
 else
   up = !digitalRead(DATA);
 TurnDetected = true;
 currentTime = millis();
 Serial.println("--------------isr");
 sei();
}

void setup() {
  pinMode(CLK, INPUT);
  pinMode(CLK, INPUT_PULLUP);
  pinMode(DATA, INPUT);
  pinMode(DATA, INPUT_PULLUP);
  pinMode(BUTTON, INPUT);
  pinMode(BUTTON, INPUT_PULLUP);
  pinMode(YLED,OUTPUT);

  attachInterrupt (0,isr,FALLING);

  Serial.begin (115200);
  Serial.println("KY-040 Quality test:");
}

static uint8_t prevNextCode = 0;

void loop() {
uint32_t pwas=0;

   if( read_rotary() ) {

      Serial.print(prevNextCode&0xf,HEX);Serial.print(" ");

      if ( (prevNextCode&0x0f)==0x0b) Serial.println("eleven ");
      if ( (prevNextCode&0x0f)==0x07) Serial.println("seven ");
   }

   if (digitalRead(BUTTON)==0) {

      delay(10);
      if (digitalRead(BUTTON)==0) {
          Serial.println("Next Detent");
          while(digitalRead(BUTTON)==0);
      }
   }

   if (TurnDetected)  {        // do this only if rotation was detected
   cli();
   if (currentTime-oldTime > 60){
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

// A vald CW or CCW move returns 1, invalid returns 0.
int8_t read_rotary() {
  static int8_t rot_enc_table[] = {0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0};

  prevNextCode <<= 2;
  if (digitalRead(DATA)) prevNextCode |= 0x02;
  if (digitalRead(CLK)) prevNextCode |= 0x01;
  prevNextCode &= 0x0f;

  return ( rot_enc_table[( prevNextCode & 0x0f )]);
}
