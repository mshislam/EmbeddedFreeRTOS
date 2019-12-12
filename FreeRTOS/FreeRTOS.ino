#include <Arduino_FreeRTOS.h>
#include <LiquidCrystal.h>
#include <TimeLib.h>
#define TIME_HEADER  "T"   // Header tag for serial time sync message
#define TIME_REQUEST  7    // ASCII bell character requests a time sync message 
#include "SoftwareSerial.h"
SoftwareSerial mySerial(11, 12);
# define Start_Byte 0x7E
# define Version_Byte 0xFF
# define Command_Length 0x06
# define End_Byte 0xEF
# define Acknowledge 0x00 //Returns info with command 0x41 [0x01: info, 0x00: no info]
# define ACTIVATED LOW
#include <semphr.h>
///////////////////////////////////////////////Module 1////////////////////////////////////////////////////////////////////


const char gear[4]={'P','D','N','R'};
int gearIndex=0;
#define enA 2
#define in1 32
#define in2 33
#define enB 8
#define in3 34
#define in4 35

const int trigPin = 9;
const int echoPin = 10;
const int buzzer = 42;

long duration;
int distance;



int motorSpeedA = 0;
int motorSpeedB = 0;



//////////////////////////////////////////////////////////light sensor////////////////////////////////////////////////////////
int LDR = 0;     //analog pin to which LDR is connected, here we set it to 0 so it means A0
int LDRValue = 0;      //that’s a variable to store LDR values
int light_sensitivity = 300;    //This is the approx value of light surrounding your LDR
int extraS=0;
int extraM=0;
int extraH=0;

////////////////////////////////////////////////////////////M2///////////////////////////////////////////////////////////////////////
// initialize the library by associating any needed LCD interface pin
// with the arduino pin number it is connected to
const int rs = 3, en = 40, d4 = 7, d5 = 6, d6 = 5, d7 = 4;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);



const char *monthName[12] = {
  "Jan", "Feb", "Mar", "Apr", "May", "Jun",
  "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
};

tmElements_t tm;

const int sensor=A5; // Assigning analog pin A5 to variable 'sensor'
float tempc; //variable to store temperature in degree Celsius

float tempf; //variable to store temperature in Fahreinheit

float vout; //temporary variable to hold sensor reading


 bool parse=false;
 
void M1_Braking(void *pvParameters);
void M2_LCD(void *pvParameters);
///////////////////////////////////////////M3///////////////////////////////////////
SemaphoreHandle_t Sem;

int buttonNext = 50;
int buttonPause = 49;
int buttonPrevious = 48;
boolean isPlaying = false;

void setup() {
  ///////m1////////////////
   pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  //Ultrasonic sensor
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
  pinMode(buzzer, OUTPUT);

  //////////\m1//////////////////////////////

  
  //temp sensor code
  pinMode(sensor,INPUT); // Configuring sensor pin as input
  //dateTime LCD code
  //bool parse=false;
  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  pinMode(13, OUTPUT);     //we mostly use 13 because there is already a built in yellow LED in arduino which shows output when 13 pin is enabled
  Serial.begin(9600);

  /////////////////////////m3/////////////////////////
  pinMode(buttonPause, INPUT);
  digitalWrite(buttonPause,HIGH);
  pinMode(buttonNext, INPUT);
  digitalWrite(buttonNext,HIGH);
  pinMode(buttonPrevious, INPUT);
  digitalWrite(buttonPrevious,HIGH);
  Sem=xSemaphoreCreateBinary();
  mySerial.begin (9600);
  /////////////////////////////////////////////////////
  
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB, on LEONARDO, MICRO, YUN, and other 32u4 based boards.
  }
    xTaskCreate(
      M2_LCD
      ,  "LCD_module"
      ,  1000  // Stack size
      ,  NULL
      ,  1  // Priority
     ,  NULL );

    xTaskCreate(
    M1_Braking
    ,  "Braking_module"
    ,  1000  // Stack size
    ,  NULL
    ,  2 // Priority
    ,  NULL );
     xTaskCreate(
      M3_MP3
      ,  "LCD_module"
      ,  1000  // Stack size
      ,  NULL
      ,  1  // Priority
     ,  NULL );
  playFirst();
  isPlaying = true;
 if (getDate(__DATE__) && getTime(__TIME__)) {
    parse = true;
    // and configure the RTC with this info
  }
  

}

void loop() {
  // put your main code here, to run repeatedly:

}
void M3_MP3(void *pvParameters){
   TickType_t xLastWakeTime;
  const TickType_t xDelays = pdMS_TO_TICKS(100);
   xLastWakeTime = xTaskGetTickCount();
   while(1){
    
   
 if (digitalRead(buttonPause) == ACTIVATED)
  {
          Serial.println("button pushed");

    if(isPlaying)
    {
      pause();
      isPlaying = false;
      Serial.println("not taken");
      xSemaphoreGive(Sem);
      Serial.println("Semaphore taken");
      
    }else
    { 
      xSemaphoreTake(Sem,portMAX_DELAY);
      isPlaying = true;
      play();
      
    }
  }else{
  }
 if (digitalRead(buttonNext) == ACTIVATED)
  {
    if(isPlaying)
    { xSemaphoreGive(Sem);
      playNext();
      xSemaphoreTake(Sem,portMAX_DELAY);
    }
  }
   if (digitalRead(buttonPrevious) == ACTIVATED)
  {
    Serial.print("button 3 pressed");
    if(isPlaying)
    {
      xSemaphoreGive(Sem);
      playPrevious();
      xSemaphoreTake(Sem,portMAX_DELAY);

    }
  }
 vTaskDelayUntil(&xLastWakeTime,xDelays); 
   }
}
void digitalClockDisplay(){
  // digital clock display of the time
  lcd.print("Time "  );
  lcd.print(hour());
  Serial.print(hour());
  printDigits(minute());
  printDigits(second());
  lcd.setCursor(0, 1);
  lcd.print("Date ");
  lcd.print(day());
  lcd.print("/");
  lcd.print(month());
  lcd.print("/");
  lcd.print(year()); 
   
  Serial.print(" ");
  Serial.print(day());
  Serial.print(" ");
  Serial.print(month());
  Serial.print(" ");
  Serial.print(year()); 
  Serial.println(); 
}

void printDigits(int digits){
  // utility function for digital clock display: prints preceding colon and leading 0
  lcd.print(":");
  if(digits < 10)
    lcd.print('0');
  lcd.print(digits);
}


void processSyncMessage() {
  unsigned long pctime;
  const unsigned long DEFAULT_TIME = 1357041600; // Jan 1 2013

  //if(Serial.find(TIME_HEADER)) {
     pctime = Serial.parseInt();
     if( pctime >= DEFAULT_TIME) { // check the integer is a valid time (greater than Jan 1 2013)
       setTime(pctime); // Sync Arduino clock to the time received on the serial port
     }
  //}
}

time_t requestSync()
{
  Serial.write(TIME_REQUEST);  
  return 0; // the time will be sent later in response to serial mesg
}

bool getTime(const char *str)
{
  int Hour, Min, Sec;

  if (sscanf(str, "%d:%d:%d", &Hour, &Min, &Sec) != 3) return false;
  tm.Hour = Hour;
  tm.Minute = Min;
  tm.Second = Sec;
  return true;
}

bool getDate(const char *str)
{
  char Month[12];
  int Day, Year;
  uint8_t monthIndex;

  if (sscanf(str, "%s %d %d", Month, &Day, &Year) != 3) return false;
  for (monthIndex = 0; monthIndex < 12; monthIndex++) {
    if (strcmp(Month, monthName[monthIndex]) == 0) break;
  }
  if (monthIndex >= 12) return false;
  tm.Day = Day;
  tm.Month = monthIndex + 1;
  tm.Year = CalendarYrToTm(Year);
  return true;
}

void M1_Braking(void *pvParameters){
   TickType_t xLastWakeTime;
  const TickType_t xDelays = pdMS_TO_TICKS(1000);
   xLastWakeTime = xTaskGetTickCount();
   while(1){
    digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

// Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);


// Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
// Calculating the distance
  distance= duration*0.034/2;
  if (distance <= 80){
  digitalWrite(buzzer, HIGH);
  }
  else{
  digitalWrite(buzzer, LOW);
  }
  // Prints the distance on the Serial Monitor
  Serial.println("               this is Braking");
//  Serial.print("Distance: ");
//  Serial.println(distance);

  
  int xAxis = analogRead(A1); // Read Joysticks X-axis
  int yAxis = analogRead(A2); // Read Joysticks Y-axis
  // Y-axis used for forward and backward control
//  Serial.print(gear[gearIndex]);
  if (yAxis < 470) {
    if(gearIndex>0){
      gearIndex-=1;
    }
    else{
      gearIndex=3;
    }
    // Set Motor A backward
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    // Set Motor B backward
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    // Convert the declining Y-axis readings for going backward from 470 to 0 into 0 to 255 value for the PWM signal for increasing the motor speed
    motorSpeedA = map(yAxis, 470, 0, 0, 100);
    motorSpeedB = map(yAxis, 470, 0, 0, 100);
  }
  else if (yAxis > 550) {
    // Set Motor A forward
    gearIndex+=1;
    gearIndex%=4;
    if(distance>80){
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    // Set Motor B forward
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    // Convert the increasing Y-axis readings for going forward from 550 to 1023 into 0 to 255 value for the PWM signal for increasing the motor speed
    motorSpeedA = map(yAxis, 550, 1023, 0, 100);
    motorSpeedB = map(yAxis, 550, 1023, 0, 100);
    }else{
      
    motorSpeedA = 0;
    motorSpeedB = 0;
      }
  }
  // If joystick stays in middle the motors are not moving
  else {
    motorSpeedA = 0;
    motorSpeedB = 0;
  }
  // X-axis used for left and right control
  if (xAxis < 470) {
    // Convert the declining X-axis readings from 470 to 0 into increasing 0 to 255 value
    int xMapped = map(xAxis, 470, 0, 0, 255);
    // Move to left - decrease left motor speed, increase right motor speed
    motorSpeedA = motorSpeedA - xMapped;
    motorSpeedB = motorSpeedB + xMapped;
    // Confine the range from 0 to 255
    if (motorSpeedA < 0) {
      motorSpeedA = 0;
    }
    if (motorSpeedB > 255) {
      motorSpeedB = 255;
    }
  }
  if (xAxis > 550) {
    // Convert the increasing X-axis readings from 550 to 1023 into 0 to 255 value
    int xMapped = map(xAxis, 550, 1023, 0, 255);
    // Move right - decrease right motor speed, increase left motor speed
    motorSpeedA = motorSpeedA + xMapped;
    motorSpeedB = motorSpeedB - xMapped;
    // Confine the range from 0 to 255
    if (motorSpeedA > 255) {
      motorSpeedA = 255;
    }
    if (motorSpeedB < 0) {
      motorSpeedB = 0;
    }
  }
  // Prevent buzzing at low speeds (Adjust according to your motors. My motors couldn't start moving if PWM value was below value of 70)
  if (motorSpeedA < 70) {
    motorSpeedA = 0;
  }
  if (motorSpeedB < 70) {
    motorSpeedB = 0;
  }
  analogWrite(enA, motorSpeedA); // Send PWM signal to motor A
  analogWrite(enB, motorSpeedB); // Send PWM signal to motor B
   
    vTaskDelayUntil(&xLastWakeTime,xDelays);
   }
    
}

void M2_LCD(void *pvParameters){
  TickType_t xLastWakeTime;
  const TickType_t xDelays = pdMS_TO_TICKS(1000);
   xLastWakeTime = xTaskGetTickCount();
   while(1){
    LDRValue = analogRead(LDR);      //reads the ldr’s value through LDR 
    Serial.println(LDRValue);//prints the LDR values to serial monitor
//    Serial.println("                    this is LCD");
    delay(50);        //This is the speed by which LDR sends value to arduino
 
    if (LDRValue < light_sensitivity) 
      {
        digitalWrite(24, HIGH);
      }
    else
      {
        digitalWrite(24, LOW);
      }
   
//  if (Serial.available()) {
    processSyncMessage();
//  }
//  if (timeStatus()!= timeNotSet) {
//    digitalClockDisplay();  
//  }
//  if (timeStatus() == timeSet) {
//    digitalWrite(13, HIGH); // LED on if synced
//  } else {
//    digitalWrite(13, LOW);  // LED off if needs refresh
//  }
//DateTime Loop
  //getDate(__DATE__) ;
  //getTime(__TIME__);
  //tm.Second+=extraS;
//Serial.print(tm.Second+extraS);
//  Serial.println(__TIME__);
//  Serial.println(__DATE__);
  //lcd.print(__TIME__);
  if(tm.Second+extraS==60){
    extraS-=60;
    extraM+=1;
  }
  if(tm.Minute+extraM==60){
    extraM-=60;
    extraH+=1;
  }

  
  lcd.print(tm.Hour+extraH);
  lcd.print(":");
  lcd.print(tm.Minute+extraM);
  lcd.print(":");
  if(tm.Second+extraS<10){
    lcd.print("0");
  }
  lcd.print(tm.Second+extraS);

    // temp loop
  vout=analogRead(sensor); //Reading the value from sensor
  vout=(vout*500)/1023;
  tempc=vout; // Storing value in Degree Celsius
  tempf=(vout*1.8)+32; // Converting to Fahrenheit
//  Serial.print("in DegreeC=");
//  Serial.print("\t");
//  Serial.print(tempc);
//  Serial.print(" ");
//  Serial.print("in Fahrenheit=");
//  Serial.print("\t");
//  Serial.print(tempf);
//  Serial.println();
  lcd.print(" ");
  lcd.print(tempc);
  lcd.print("C");

  //Date loop
  lcd.setCursor(0, 1);
  lcd.print(__DATE__);

  //gear loop
  lcd.print(" ");
  lcd.print(gear[gearIndex]);
  extraS+=1;
//  Serial.println(extraS);
  delay(1000);
  // set the cursor to column 0, line 1
  // (note: line 1 is the second row, since counting begins with 0):
  lcd.setCursor(0, 0);
   vTaskDelayUntil(&xLastWakeTime,xDelays);
   }
}
void playFirst()
{
  execute_CMD(0x3F, 0, 0);
  delay(500);
  setVolume(30);
  delay(500);
  execute_CMD(0x11,0,1); 
  delay(500);
}
void pause()
{
  execute_CMD(0x0E,0,0);
  delay(500);
}
void play()
{
  execute_CMD(0x0D,0,1); 
  delay(500);
}
void playNext()
{ 
  execute_CMD(0x01,0,1);
  delay(500);
}
void playPrevious()
{
  execute_CMD(0x02,0,1);
  delay(500);
}
void setVolume(int volume)
{
  execute_CMD(0x06, 0, volume); // Set the volume (0x00~0x30)
  delay(2000);
}
void execute_CMD(byte CMD, byte Par1, byte Par2)
// Excecute the command and parameters
{
// Calculate the checksum (2 bytes)
word checksum = -(Version_Byte + Command_Length + CMD + Acknowledge + Par1 + Par2);
// Build the command line
byte Command_line[10] = { Start_Byte, Version_Byte, Command_Length, CMD, Acknowledge,
Par1, Par2, highByte(checksum), lowByte(checksum), End_Byte};
//Send the command line to the module
for (byte k=0; k<10; k++)
{
mySerial.write( Command_line[k]);
}
}


 
