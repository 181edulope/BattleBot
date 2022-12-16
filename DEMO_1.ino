
#include <Servo.h>


// button setup
int start_button = 50; // sets start button to pin 30
int start_time = 0; // sets start time
int button_flag = 0; // flag for start button

long fight_time = 110000; // this is 120 seconds in milliseconds, max time of fight


int noSpeed=0;


// IR sensor 3
int IR3 = 51;




// IR SENSOR ( BACK)
int potPin = A0;
int LTR = A1;
int redLED = 2; // interrupt trigger
volatile bool led = false;

// IR sensor ( Front)

int potPin2 = A2;
int LTR2 = A3;
int redLED2 = 18; // interrupt trigger
volatile bool led2 = false;

//ULTRASONIC SENSOR
int blueLED = 28;
int blueISR = 3;   // interrupt trigger
volatile bool led3 = false;
int trigPin = 11;
int echoPin = 12;

// motor driver and Actuator  functions 
//initialize motor pinouts
int frontLeftMotorSpeed=10; //13//speed is white PWM
int frontLeftMotorDir=25; //orange CCW/CW

int frontRightMotorSpeed=9; //12
int frontRightMotorDir=23;

int backLeftMotorSpeed =8; //11
int backLeftMotorDir = 27;

//backright motor doesnt work
//int backRightMotorSpeed =10;
//int backRightMotorDir =29;

int halfSpeed = 200;
int maxSpeed = map (100,0,1023,0,100);
//delayTime
int delayTime = 700;

// actuator setup

int IN3 = 31;
int enable3 = 33;

// Servo motor setup 
Servo myservo;
int servoPin = 4;  // will change to different pin





//int delayTime = 500;

void setup() {
  pinMode(start_button, INPUT) ;
  pinMode(frontRightMotorSpeed, OUTPUT);
  pinMode(frontRightMotorDir, OUTPUT);

  pinMode(frontLeftMotorSpeed, OUTPUT);
  pinMode(frontLeftMotorDir, OUTPUT);

  pinMode(backLeftMotorSpeed, OUTPUT);
  pinMode(backLeftMotorDir, OUTPUT);

  //pinMode(backRightMotorSpeed, OUTPUT);
  //pinMode(backRightMotorDir, OUTPUT);
  Serial.begin(9600);
  
  // IR sensor Interrupt setup 
  pinMode(redLED,INPUT_PULLUP);
  pinMode(redLED2,INPUT_PULLUP);
  //attachInterrupt(digitalPinToInterrupt(redLED),bounds, FALLING);
  attachInterrupt(digitalPinToInterrupt(redLED2),bounds2, FALLING);
  pinMode(IR3,INPUT);
  
  // Ultrasonic Sensor Interrupt setup
  pinMode(blueLED,OUTPUT);
  pinMode(blueISR,INPUT_PULLUP);
  pinMode(trigPin,OUTPUT);
  pinMode(echoPin,INPUT);
  attachInterrupt(digitalPinToInterrupt(blueISR),sonic1,RISING);
  
  // motor driver setup 
  // driver 1
  
  
  // actuator setup 
  
  pinMode(IN3,OUTPUT);
  pinMode(enable3,OUTPUT);
  //Stop2();
  // servo initialization
  myservo.attach(servoPin);
  Serial.println("-------- SYSTEM STARTING --------");
}

void loop() {
 Release();
  Stop();
 if (digitalRead(start_button) == 1){ // check to see if button has been pressed
    start_time = millis(); // starts the system   
    button_flag = 1;
    Serial.println("\n\nBUTTON PRESSED: STARTING BATTLE MODE\n\n");
  }

 
  while(button_flag == 1){
    
    if (millis() > start_time + fight_time){
      break;
    }
  
  
    lookLeft();
  //delay(100);
  lookRight();
  //delay(100);
  fullSpeedForward();
  //delay(800);
 // Stop();
  
  
  
  
  // ISR for IR sensor Routine  back sensor
  if (led){
    //led = false;     moved this one after rotateRight()
    Serial.println("INTERRUPT!!  front");
     rotateRight();
     
    delay(900);
    led = false;
  }
  
  // ISR for IR sensor Routine front sensor
  if (led2){
    //rotateLeft();
    rotateRight();
    delay(900);
    Serial.println("INTERRUPT!!  BACKWARD!!! ");
    led2 = false;
  }
    
  
  
  
  // ISR for UltraSonic Sensor Routine/actuator ISR
  
  if(led3){
    led3= false; 
  
    //  attack trigger extend
    Extend();
    delay(3000);
    Stop2();
    fullSpeedForward();
    delay(2000);
    Release(); // release
  }
  
  // Condition to trigger the attack based off distance to target
  if (distance1() <= 12){
    digitalWrite(blueLED,HIGH);
    Serial.println("Target detected");
  }
  else{
    digitalWrite(blueLED,LOW);
    Serial.println("No Target detected");
  }
  }
  button_flag = 0;
  start_time = 0;
  Serial.println("Waiting for button press");
  delay(50);
}




void bounds(){   //  ISR for IR SENSOR  
  led = true;
}

void bounds2(){ //  ISR for IR SENSOR
  led2 = true;
}
 
void sonic1(){  //ISR for ULtraSonic Sensor
  led3 = true;
}



void Extend(){  //Actuator extend
  digitalWrite(IN3,LOW);
  digitalWrite(enable3,HIGH);
  Serial.println("SYSTEM EXTENDING LEVER ARM!");
}


void Release(){  // Actuator release
  digitalWrite(IN3,HIGH);
  digitalWrite(enable3,HIGH);
  Serial.println("SYSTEM RELEASING LEVER ARM!");
}



void Stop2(){   // motor stop
  digitalWrite(enable3,LOW);
  digitalWrite(IN3,LOW);
  Serial.println("SYSTEM STOPPING!");
}

long distance1(){
digitalWrite(trigPin,LOW);
delayMicroseconds(2);
digitalWrite(trigPin,HIGH);
delayMicroseconds(10);
digitalWrite(trigPin,LOW);
long pulseTime = pulseIn(echoPin,HIGH);

long distance = (pulseTime / 2) * 0.0344;
Serial.println(distance);
//delay(300);
return distance;
}


int lookLeft()
{
    myservo.write(130); 
    delay(100);
    int distance = distance1();
    delay(100);
    
    myservo.write(50);
    delay(500); 
    return distance;
    
    
}

int lookRight()
{
  delay(100);
    myservo.write(0); 
    delay(300);
    int distance = distance1();
    delay(100);
    myservo.write(50);
     delay(100);
    return distance;
}






/*void stopServo(int distance, currentLocation){
  if(distance < 20){
    
  }
}
*/


void IR_Sensor1(){
  long pot =  analogRead(potPin);
  long potRead = map(pot,0.0,1023.0,0.0,100.0);
//  Serial.println(digitalRead(redLED));
//  Serial.print("potentiometer read: ");
//  Serial.print(potRead);
//  Serial.println(" ");
 // delay(400);
  long LTR_Read = map(analogRead(LTR),0.0,1023.0,0.0,100.0);
 // Serial.println(" LTR Read:");
 // Serial.println(LTR_Read);
  //delay(400);

long pot2 =  analogRead(potPin2);
  long potRead2 = map(pot2,0.0,1023.0,0.0,100.0);
  Serial.println(digitalRead(redLED2));
  Serial.print("potentiometer read2: ");
  Serial.print(potRead2);
  Serial.println(" ");
  //delay(400);
  long LTR_Read2 = map(analogRead(LTR2),0.0,1023.0,0.0,100.0);
  Serial.println(" LTR Read2:");
  Serial.println(LTR_Read2);
  //delay(400);
  
  if ( LTR_Read <= 50.0){
    Serial.println(" you are in Play !!!");
  }
}


void speedMap(int value){
int dutyVal = map(value, 0, 1023, 0,255);
digitalWrite(frontLeftMotorDir, HIGH);
analogWrite(frontLeftMotorSpeed, dutyVal);
Serial.println(dutyVal);
}


void halfSpeedForward(){ //dont use half speed
analogWrite(frontRightMotorSpeed, halfSpeed);
digitalWrite(frontRightMotorDir, HIGH);

analogWrite(frontLeftMotorSpeed, halfSpeed);
digitalWrite(frontLeftMotorDir, LOW);

analogWrite(backLeftMotorSpeed, halfSpeed);
digitalWrite(backLeftMotorDir, LOW);

//backright motor does not work anymore
//analogWrite(backRightMotorSpeed, halfSpeed);
//digitalWrite(backRightMotorDir, LOW);
}

void fullSpeedForward(){
analogWrite(frontRightMotorSpeed, maxSpeed);
digitalWrite(frontRightMotorDir, HIGH);

analogWrite(frontLeftMotorSpeed, maxSpeed);
digitalWrite(frontLeftMotorDir, LOW);

analogWrite(backLeftMotorSpeed, maxSpeed);
digitalWrite(backLeftMotorDir, LOW);

//backright motor does not work anymore
//analogWrite(backRightMotorSpeed, maxSpeed);
//digitalWrite(backRightMotorDir, LOW);
}

void rotateRight(){
    analogWrite(frontRightMotorSpeed, maxSpeed);
    digitalWrite(frontRightMotorDir, LOW);

    analogWrite(frontLeftMotorSpeed, maxSpeed);
    digitalWrite(frontLeftMotorDir, LOW);

    analogWrite(backLeftMotorSpeed, maxSpeed);
    digitalWrite(backLeftMotorDir, LOW);
}
void rotateLeft(){
    analogWrite(frontRightMotorSpeed, maxSpeed);
    digitalWrite(frontRightMotorDir, HIGH);

    analogWrite(frontLeftMotorSpeed, maxSpeed);
    digitalWrite(frontLeftMotorDir, HIGH);

    analogWrite(backLeftMotorSpeed, maxSpeed);
    digitalWrite(backLeftMotorDir, HIGH);
}


void Stop(){
  analogWrite(frontRightMotorSpeed, 1023);
digitalWrite(frontRightMotorDir, LOW);

analogWrite(frontLeftMotorSpeed, 1023);
digitalWrite(frontLeftMotorDir, LOW);

analogWrite(backLeftMotorSpeed, 1023);
digitalWrite(backLeftMotorDir, LOW);

//analogWrite(backRightMotorSpeed, noSpeed);
//digitalWrite(backRightMotorDir, LOW);
}

/*
void forward(){
  // front left motor
  analogWrite(enable, 255); //153 is half speed, 255 is full
  analogWrite(enable2, 255);
  digitalWrite(IN1, HIGH);
  digitalWrite(INN2, LOW);
  digitalWrite(input3, LOW); // Clockwise: LOW & CCW: HIGH
  digitalWrite(input4, LOW);

// back left motor 
  analogWrite(Enable, 255); //153 is half speed, 255 is full
  analogWrite(Enable2, 255);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(inPut3, LOW); // Clockwise: LOW & CCW: HIGH
  digitalWrite(inPut4, HIGH);

// front right  motor 
  analogWrite(Enable12, 255); //153 is half speed, 255 is full
  analogWrite(Enable34, 255);
  digitalWrite(iN1, HIGH);
  digitalWrite(iN2, LOW);
  digitalWrite(InPut3, HIGH); // Clockwise: LOW & CCW: HIGH
  digitalWrite(InPut4, LOW);  

//back right  motor 
  analogWrite(ENABLE, 255); //153 is half speed, 255 is full
  analogWrite(ENABLE2, 255);
  digitalWrite(INNN1, HIGH);
  digitalWrite(INNN2, LOW);
  digitalWrite(inPUT3, LOW); // Clockwise: LOW & CCW: HIGH
  digitalWrite(inPUT4, LOW); 

}  
*/
