#include <Arduino_FreeRTOS.h> 
#include <Servo.h>
#include <LiquidCrystal.h>
#include <SPI.h>
#include <MFRC522.h>


//----------------------------------------- variables ----------------------------------------
bool locked = false;
bool engine = false;
bool seatBelt = false;


//----------------- servo
int  x = A0;
int y = A1;
int servo1Angle=90;
int servo2Angle=90;
int valx=0;
int valy=0;

Servo myservo1; 
Servo myservo2; 

//----------------- LCD
const int rs = 11, en = 10, d4 =9 ,d5 = 8, d6 = 7, d7 = 6;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
char* rain=malloc(sizeof(char)*3);

//-----------------push button seatbelt
#define seatPush 2
int buzzer = 36;

//----------------- rain sensor

int rainPin = A2;

// you can adjust the threshold value
int thresholdValue = 500;

//------------------ RFID
#define RST_PIN         39          // Configurable, see typical pin layout above
#define SS_PIN          53         // Configurable, see typical pin layout above

MFRC522 mfrc522(SS_PIN, RST_PIN);  // Create MFRC522 instance
//------------------- Ultrasonic
const int trigPin = 13;
const int echoPin = 12;
// defines variables
long duration;
int distance;

//----------------------------------- define tasks -------------------------------------------

void Mirrors( void *pvParameters); 
void LCD( void *pvParameters);
void RainSensor( void *pvParameters);
void RFID( void *pvParameters);
void Ultrasonic( void *pvParameters);
void Buzzer( void *pvParameters);


//------------------------------------------ Functions -----------------------------------------
unsigned long getID(){
  if ( ! mfrc522.PICC_ReadCardSerial()) { //Since a PICC placed get Serial and continue
    return -1;
  }
  unsigned long hex_num;
  hex_num =  mfrc522.uid.uidByte[0] << 24;
  hex_num += mfrc522.uid.uidByte[1] << 16;
  hex_num += mfrc522.uid.uidByte[2] <<  8;
  hex_num += mfrc522.uid.uidByte[3];
  mfrc522.PICC_HaltA(); // Stop reading
  return hex_num;
}
//-------------------------------------------------------------------------------------------

void handler_belt ()
{

    Serial.println(digitalRead(seatPush));
}
void setup() {
  Serial.begin(9600);
  //--------------SeatBelt
  pinMode(seatPush, INPUT_PULLUP);
  pinMode(buzzer, OUTPUT);
  digitalWrite(buzzer,HIGH);
  attachInterrupt(digitalPinToInterrupt(seatPush), handler_belt, FALLING);

  // ------------Mirrors setup
  pinMode(x, INPUT);
  pinMode(y, INPUT);
  myservo1.attach(40);  
  myservo2.attach(41); 

  // ------------lcd setup
  lcd.begin(16, 2); 
  //lcd.print("hello, world!");

  // ------------rain sensor setup
  pinMode(rainPin, INPUT);

// ---------------- RFID
  SPI.begin();  
  mfrc522.PCD_Init();    // Init MFRC522
  mfrc522.PCD_DumpVersionToSerial();  // Show details of PCD - MFRC522 Card Reader details

//-------------- Ultrasonic
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
  
// -----------------------------
  xTaskCreate (Mirrors, "Mirrors", 1000, NULL, 1, NULL);
  xTaskCreate (LCD, "LCD", 1500, NULL, 1, NULL);
  xTaskCreate (RainSensor, "RainSensor", 1000, NULL, 1, NULL);
  xTaskCreate (RFID, "RFID", 1000, NULL, 1, NULL);
  xTaskCreate (Ultrasonic, "Ultrasonic", 1000, NULL, 1, NULL);
  xTaskCreate (Buzzer, "Buzzer", 500, NULL, 1, NULL);

}

//------------------------------------------ Tasks -----------------------------------

void Buzzer (void *pvParameters) // buzz when belt. 
{
  TickType_t xLastWakeTime;
  const TickType_t xDelay=pdMS_TO_TICKS(25);
  xLastWakeTime = xTaskGetTickCount();  
  while (1) {
//  /  if(!seatBelt)
   //   digitalWrite(buzzer,HIGH);
//   / else
//      /digitalWrite(buzzer,LOW);

    delay(15);
    vTaskDelayUntil(&xLastWakeTime,xDelay);
  }
}

void Mirrors (void *pvParameters) // Mirrors. 
{
  TickType_t xLastWakeTime;
  const TickType_t xDelay=pdMS_TO_TICKS(25);
  xLastWakeTime = xTaskGetTickCount();  
  while (1) {
    valx = analogRead(x);  
    valy = analogRead(y); 
    
    if(valx<341){
      servo1Angle+=2;   //right
    }
    else if(valx>682){
      servo1Angle-=2;
    }
    if(valy<341){
      servo2Angle+=2;
    }
    else if(valy>682){
      servo2Angle-=2;
    }
    if(servo1Angle<0){
      servo1Angle=0;
    }
    if(servo1Angle>90){
      servo1Angle=90;
    }
    if(servo2Angle<90){
      servo2Angle=90;
    }
    if(servo2Angle>180){
      servo2Angle=180;
    }
    myservo1.write(servo1Angle);
    myservo2.write(servo2Angle);
    delay(15);
    vTaskDelayUntil(&xLastWakeTime,xDelay);
  }
}

void LCD( void *pvParameters) //LCD. 
{
  TickType_t xLastWakeTime;
  const TickType_t xDelay=pdMS_TO_TICKS(500);
  xLastWakeTime=xTaskGetTickCount();
  while (1) {
    lcd.clear();
     // set the cursor to column 0, row 1
    // (note: line 1 is the second row, since counting begins with 0):
    lcd.setCursor(0, 0);
    lcd.print(rain);
    lcd.setCursor(0, 1);
    lcd.print("R");
    lcd.setCursor(1, 1);
    lcd.print(servo1Angle);
    lcd.setCursor(5, 1);
    lcd.print("L");
    lcd.setCursor(6,1);
    lcd.print(servo2Angle);
    lcd.setCursor(14, 1);
    // print the number of seconds since reset:
    lcd.print(millis() / 1000);
    vTaskDelayUntil(&xLastWakeTime,xDelay);
  }
    
}

void RainSensor(void *pvParameters){
  TickType_t xLastWakeTime;
  const TickType_t xDelay= pdMS_TO_TICKS(370);
  xLastWakeTime= xTaskGetTickCount();
  while(1){
    int sensorValue = analogRead(rainPin);
    //Serial.print(sensorValue);
    if(sensorValue < thresholdValue){
      Serial.println(" - It's wet");
      rain="wet";
   
    }
    else {
      //Serial.println(" - It's dry");
      rain="dry";
    }
    
    vTaskDelayUntil(&xLastWakeTime,xDelay);
  }
}

void RFID(void *pvParameters){
  TickType_t xLastWakeTime;
  const TickType_t xDelay= pdMS_TO_TICKS(990);
  xLastWakeTime= xTaskGetTickCount();
  while(1){
    if(mfrc522.PICC_IsNewCardPresent()) {
      unsigned long uid = getID();
      if(uid != -1){
        Serial.print("Card detected, UID: ");
        Serial.println(uid);
      }
    }
    vTaskDelayUntil(&xLastWakeTime,xDelay);
  }
}

void Ultrasonic(void *pvParameters){
  TickType_t xLastWakeTime;
  const TickType_t xDelay= pdMS_TO_TICKS(107);
  xLastWakeTime= xTaskGetTickCount();
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
    // Prints the distance on the Serial Monitor
    if(distance<=10){
      Serial.print("Distance: ");
      Serial.println(distance);
    }
    vTaskDelayUntil(&xLastWakeTime,xDelay);
  }
}
void loop() {
 
}
/*----------------------------------- LCD Info---------------------------------------*/
/*
 *  The circuit:
 * LCD RS pin to digital pin 12
 * LCD Enable pin to digital pin 11
 * LCD D4 pin to digital pin 5
 * LCD D5 pin to digital pin 4
 * LCD D6 pin to digital pin 3
 * LCD D7 pin to digital pin 2
 * LCD R/W pin to ground
 * LCD VSS pin to ground
 * LCD VCC pin to 5V
 * 10K resistor:
 * ends to +5V and ground
 * wiper to LCD VO pin (pin 3) 
 * 
 * 
 * 
 * 
 */
