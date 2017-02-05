// Test MD03a / Pololu motor with encoder
// speed control (PI), V & I display
// Credits:
//   Dallaby   http://letsmakerobots.com/node/19558#comment-49685
//   Bill Porter  http://www.billporter.info/?p=286
//   bobbyorr (nice connection diagram) http://forum.pololu.com/viewtopic.php?f=15&t=1923


#define InA1            10                      // INA motor pin
#define InB1            11                      // INB motor pin 
#define PWM1            9                       // PWM motor pin

// Encoder variables
enum PinAssignments {
  encoderPinA = 2,
  encoderPinB = 3
};
boolean A_set = false;
boolean B_set = false;

#define LOOPTIME        100                     // PID loop time

unsigned long lastMilli = 0;                    // loop timing 
unsigned long lastMilliPrint = 0;               // loop timing
int speed_req = 10;                            // speed (Set Point)
int speed_act = 0;                              // speed (actual value)
int PWM_val = 0;                                // (25% = 100; 50% = 150; 75% = 225; 100% = 255)
volatile long count = 0;                        // rev counter
float Kp =   .4;                                // PID proportional control Gain
float Kd =    1;                                // PID Derivitave control gain


void setup() {                       
 Serial.begin(115600);
 pinMode(InA1, OUTPUT);
 pinMode(InB1, OUTPUT);
 pinMode(PWM1, OUTPUT);
 digitalWrite(encoderPinA, HIGH);
 digitalWrite(encoderPinB, HIGH);
 attachInterrupt(digitalPinToInterrupt(encoderPinA), doEncoderA, CHANGE);
 attachInterrupt(digitalPinToInterrupt(encoderPinB), doEncoderB, CHANGE);

 analogWrite(PWM1, PWM_val);
 digitalWrite(InA1, LOW);
 digitalWrite(InB1, HIGH);
}

void loop() {
 getParam();                                                                 // check keyboard
 if((millis()-lastMilli) >= LOOPTIME)   {                                    // enter tmed loop
   lastMilli = millis();
   getMotorData();                                                           // calculate speed, volts and Amps
   PWM_val= updatePid(PWM_val, speed_req, speed_act);                        // compute PWM value
   analogWrite(PWM1, PWM_val);                                               // send PWM to motor
 }
 printMotorInfo();                                                           // display data
}

void getMotorData()  {                                                    
 static long countAnt = 0;                                              // last count
 speed_act = ((count - countAnt)*(60*(1000/LOOPTIME)))/(360);          // 360 counts per output shaft rev
 countAnt = count;                  
}

int updatePid(int command, int targetValue, int currentValue)   {             // compute PWM value
float pidTerm = 0;                                                            // PID correction
int error=0;                                  
static int last_error=0;          
 if (speed_req < 0.0){
    digitalWrite(InA1, HIGH);
    digitalWrite(InB1, LOW);                  
 }else {
    digitalWrite(InA1, LOW);
    digitalWrite(InB1, HIGH);    
 }
 error = abs(targetValue) - abs(currentValue); 
 pidTerm = (Kp * error) + (Kd * (error - last_error));                            
 last_error = error;
 return constrain(command + int(pidTerm), 0, 255);
}

void printMotorInfo()  {                                                      // display data
 if((millis()-lastMilliPrint) >= 500)   {                     
   lastMilliPrint = millis();
   Serial.print("SP:");             Serial.println(speed_req);  
   Serial.print("  RPM:");          Serial.println(speed_act);
   Serial.print("  PWM:");          Serial.println(PWM_val);
   Serial.println("");              
 }
}

// Interrupt on A changing state
void doEncoderA(){
  // Test transition
  A_set = digitalRead(encoderPinA) == HIGH;
  // and adjust counter + if A leads B
  count += (A_set != B_set) ? +1 : -1;
}

// Interrupt on B changing state
void doEncoderB(){
  // Test transition
  B_set = digitalRead(encoderPinB) == HIGH;
  // and adjust counter + if B follows A
  count += (A_set == B_set) ? +1 : -1;
}

void getParam()  {
 char param, cmd;
 if(!Serial.available())    return;
 delay(10);                  
 param = Serial.read();                              // get parameter byte
 if(!Serial.available())    return;
 cmd = Serial.read();                                // get command byte
 Serial.flush();
 switch (param) {
   case 'v':                                         // adjust speed
     if(cmd=='+')  {
       speed_req += 5;
       if(speed_req>40)   speed_req=40;
     }
     if(cmd=='-')    {
       speed_req -= 5;
       if(speed_req<-40)   speed_req=-40;
     }
     break;
   case 's':                                        // adjust direction
     if(cmd=='+'){
       digitalWrite(InA1, LOW);
       digitalWrite(InB1, HIGH);
     }
     if(cmd=='-')   {
       digitalWrite(InA1, HIGH);
       digitalWrite(InB1, LOW);
     }
     break;
   case 'o':                                        // user should type "oo"
     digitalWrite(InA1, LOW);
     digitalWrite(InB1, LOW);
     speed_req = 0;
     break;
   default: 
     Serial.println("???");
   }
}

