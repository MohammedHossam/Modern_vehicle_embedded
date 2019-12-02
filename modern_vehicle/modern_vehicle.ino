#include <Arduino_FreeRTOS.h>
#include <Servo.h>
#include <LiquidCrystal.h>
#include <SPI.h>
#include <MFRC522.h>
#include <semphr.h>

//-------------------------------------------semaphors--------------------------------
SemaphoreHandle_t sem_buzzer;
SemaphoreHandle_t sem_engine;
SemaphoreHandle_t sem_engineOut;
SemaphoreHandle_t sem_rf;
//----------------------------------------- variables ----------------------------------------
bool locked = false;
bool engine = false;
bool seatBelt = false;

//------------------ Engine
const int buttonEngine = 48;    //47
const int ledEngine =  41;
int buttonEngineState = 0;        // variable for reading the pushbutton status
#define enA 5
#define in1 24
#define in2 25
#define enB 4   // 3=> 4
#define inB1 27
#define inB2 26


//----------------- servo
int  x = A0;
int y = A1;
int servoRAngle = 90;
int servoLAngle = 90;
int valx = 0;
int valy = 0;

Servo servoRight;
Servo servoLeft;

//----------------- LCD
const int rs = 11, en = 10, d4 = 9 , d5 = 8, d6 = 7, d7 = 6;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
char* rain = malloc(sizeof(char) * 3);
char* beltDisplay = malloc(sizeof(char) * 3);


//-----------------push button seatbelt
int seatPush = 46;          //49
int buttonState = 0;         // variable for reading the pushbutton status
int lastButtonState = 0;
int buzzerState = 0;
int ledPin =  42;   //35
int buzzer = 3;   //36

//--------------- Fuel Level
int sensor_water = A2;  //A3

//----------------- rain sensor

int rainPin = A3;  // A2

// you can adjust the threshold value
int thresholdValue = 500;

//------------------ RFID
#define SS_PIN        53         // Configurable, see typical pin layout above
#define RST_PIN       49          // Configurable, see typical pin layout above
#define ledLock       43    //30

MFRC522 mfrc522(SS_PIN, RST_PIN);  // Create MFRC522 instance
//------------------- Ultrasonic
const int trigPin = 13;
const int echoPin = 12;
const int ultraBuzzer = 39;
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
void handler_belt( void *pvParameters);
void setup2( );


//------------------------------------------ Functions -----------------------------------------
unsigned long getID() {
  if ( ! mfrc522.PICC_ReadCardSerial()) { //Since a PICC placed get //Serial and continue
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

void setup() {
  //Serial.begin(9600);
  // ---------------- RFID
  //while (!//Serial);    // Do nothing if no serial port is opened (added for Arduinos based on ATMEGA32U4)
  SPI.begin();      // Init SPI bus
  mfrc522.PCD_Init();   // Init MFRC522
  delay(4);       // Optional delay. Some board do need more time after init to be ready, see Readme
  //  mfrc522.PCD_DumpVersionTo//Serial();  // Show details of PCD - MFRC522 Card Reader details
  //Serial.println(F("Scan PICC to see UID, SAK, type, and data blocks..."));

  //---------------semphrs
  sem_buzzer = xSemaphoreCreateCounting( 1, 1 );
  sem_engine = xSemaphoreCreateCounting( 1, 0);
  sem_rf = xSemaphoreCreateCounting(1, 0);
  sem_engineOut = xSemaphoreCreateMutex();
  if (sem_engineOut != NULL)
    xSemaphoreGive(sem_engineOut);

  //--------------SeatBelt
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);
  pinMode(seatPush, INPUT);

  pinMode(buzzer, OUTPUT);
  digitalWrite(buzzer, LOW);



  //-------------------- Engine
  pinMode(ledEngine, OUTPUT);
  digitalWrite(ledEngine, LOW);
  pinMode(buttonEngine, INPUT_PULLUP);
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(inB1, OUTPUT);
  pinMode(inB2, OUTPUT);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(inB1, LOW);
  digitalWrite(inB2, HIGH);

  //   ------------Mirrors setup
  pinMode(x, INPUT);
  pinMode(y, INPUT);
  servoRight.attach(47);  //40
  servoLeft.attach(44); //41

  // ------------lcd setup
  lcd.begin(16, 2);


  // ------------rain sensor setup
  pinMode(rainPin, INPUT);

  // ------------rfid lock
  pinMode(ledLock, OUTPUT);
  digitalWrite(ledLock, LOW);

  //  //-------------- Ultrasonic
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
  pinMode(ultraBuzzer, OUTPUT);
  digitalWrite(ultraBuzzer, LOW);


  // -----------------------------
  // xTaskCreate (Buzzer, "Buzzer", 1000, NULL, 1, NULL);
  xTaskCreate (handler_belt, "belt_handler", 500, NULL, 2, NULL);
  xTaskCreate (Engine, "Engine", 500, NULL, 3, NULL);
  xTaskCreate (Mirrors, "Mirrors", 300, NULL, 2, NULL);
  xTaskCreate (LCD, "LCD", 1000, NULL, 2, NULL);
  xTaskCreate (RainSensor, "RainSensor", 400, NULL, 2, NULL);
  xTaskCreate (RFID, "RFID", 500, NULL, 1, NULL);
  xTaskCreate (Ultrasonic, "Ultrasonic", 300, NULL, 3, NULL);

  //---------------------------------




}

//------------------------------------------ Tasks -----------------------------------

void handler_belt(void *pvParameters)
{
  TickType_t xLastWakeTime;
  const TickType_t xDelay = pdMS_TO_TICKS(190);
  xLastWakeTime = xTaskGetTickCount();
  int ledLight = 0;
  int count = 0;
  while (1) {
    xSemaphoreTake(sem_engine, portMAX_DELAY);
    xSemaphoreGive(sem_engine);
    buttonState = digitalRead(seatPush);
    if (buttonState != lastButtonState) {
      if (buttonState == HIGH) {
        ledLight = !ledLight;
        digitalWrite(ledPin, ledLight);
        beltDisplay = ledLight ? "Off" : "ON";
        //  xSemaphoreTake(sem_buzzer, portMAX_DELAY);
        //digitalWrite(buzzer, ledLight);

    if (ledLight == HIGH) {
      xSemaphoreTake(sem_buzzer, portMAX_DELAY);
      digitalWrite(buzzer, HIGH);
      xSemaphoreGive(sem_buzzer);
      Serial.println("dasdas");
    }
    else {
      xSemaphoreTake(sem_buzzer, portMAX_DELAY);
      digitalWrite(buzzer, LOW);
      xSemaphoreGive(sem_buzzer);
    }
      }
    }
    delay(50);
    lastButtonState = buttonState;
    vTaskDelayUntil(&xLastWakeTime, xDelay);
  }
}
//
void Buzzer (void *pvParameters) // buzz when belt.
{
  TickType_t xLastWakeTime;
  const TickType_t xDelay = pdMS_TO_TICKS(150);
  xLastWakeTime = xTaskGetTickCount();
  while (1) {
    digitalWrite(buzzer, LOW);
    //xSemaphoreTake(sem_engine, portMAX_DELAY);
    //xSemaphoreGive(sem_engine);
    //    xSemaphoreTake(sem_buzzer, portMAX_DELAY);
    digitalWrite(buzzer, HIGH);
    delay(100);
    vTaskDelayUntil(&xLastWakeTime, xDelay);

  }
}

void Mirrors (void *pvParameters) // Mirrors.
{
  TickType_t xLastWakeTime;
  const TickType_t xDelay = pdMS_TO_TICKS(150);
  xLastWakeTime = xTaskGetTickCount();
  while (1) {
    xSemaphoreTake(sem_engine, portMAX_DELAY);
    xSemaphoreGive(sem_engine);
    valx = analogRead(x);
    valy = analogRead(y);

    if (valx < 341) {
      servoRAngle += 2; //right
    }
    else if (valx > 682) {
      servoRAngle -= 3;
    }
    if (valy < 341) {
      servoLAngle += 3;
    }
    else if (valy > 682) {
      servoLAngle -= 3;
    }
    if (servoRAngle < 0) {
      servoRAngle = 0;
    }
    if (servoRAngle > 90) {
      servoRAngle = 90;
    }
    if (servoLAngle < 90) {
      servoLAngle = 90;
    }
    if (servoLAngle > 180) {
      servoLAngle = 180;
    }
    servoRight.write(servoRAngle);
    servoLeft.write(servoLAngle);
    delay(15);
    vTaskDelayUntil(&xLastWakeTime, xDelay);
  }
}

void LCD( void *pvParameters) //LCD.
{
  TickType_t xLastWakeTime;
  const TickType_t xDelay = pdMS_TO_TICKS(180);
  xLastWakeTime = xTaskGetTickCount();
  while (1) {
    lcd.clear();
    xSemaphoreTake(sem_engine, portMAX_DELAY);
    xSemaphoreGive(sem_engine);
    // set the cursor to column 0, row 1
    // (note: line 1 is the second row, since counting begins with 0):

    // Second row
    lcd.setCursor(0, 0);
    lcd.print("R");
    lcd.setCursor(1, 0);
    lcd.print(servoRAngle);

    lcd.setCursor(0, 1);
    lcd.print("L");
    lcd.setCursor(1, 1);
    lcd.print(servoLAngle);

    lcd.setCursor(6, 0);
    lcd.print("Belt");
    lcd.setCursor(7, 1);
    lcd.print(beltDisplay);


    lcd.setCursor(12, 1);
    lcd.print(rain);

    int value = analogRead(sensor_water);
    lcd.setCursor(12, 0);
    lcd.print("F");
    lcd.setCursor(13, 0);
    int fuelLevel = value / 45;
    lcd.print(fuelLevel);

    //lcd.setCursor(14, 1);
    //lcd.print(millis() / 1000);
    vTaskDelayUntil(&xLastWakeTime, xDelay);
  }

}

void RainSensor(void *pvParameters) {
  TickType_t xLastWakeTime;
  const TickType_t xDelay = pdMS_TO_TICKS(240);
  xLastWakeTime = xTaskGetTickCount();
  while (1) {
    xSemaphoreTake(sem_engine, portMAX_DELAY);
    xSemaphoreGive(sem_engine);
    int sensorValue = analogRead(rainPin);
    ////Serial.print(sensorValue);
    if (sensorValue < thresholdValue) {
      //Serial.println(" - It's wet");
      rain = "wet";

    }
    else {
      ////Serial.println(" - It's dry");
      rain = "dry";
    }

    vTaskDelayUntil(&xLastWakeTime, xDelay);
  }
}

void RFID(void *pvParameters) {
  TickType_t xLastWakeTime;
  const TickType_t xDelay = pdMS_TO_TICKS(300);
  xLastWakeTime = xTaskGetTickCount();
  byte lock = LOW;     // car is locked
  while (1) {
    xSemaphoreTake(sem_engineOut, portMAX_DELAY);
    digitalWrite(ledLock, lock);
    //Serial.print(lock);
    if (mfrc522.PICC_IsNewCardPresent()) {
      unsigned long uid = getID();
      if (uid == 21043 ) {  //4294934562
        //Serial.print("Card detected, UID: ");
        //Serial.println(uid);
        lock = !lock;
        if (lock == HIGH) {
          xSemaphoreGive(sem_rf);
          //   delay(10);
        }
        else {
          xSemaphoreTake(sem_rf, portMAX_DELAY);
        }
      }
    }
    xSemaphoreGive(sem_engineOut);
    vTaskDelayUntil(&xLastWakeTime, xDelay);
  }
}


void Ultrasonic(void *pvParameters) {
  TickType_t xLastWakeTime;
  const TickType_t xDelay = pdMS_TO_TICKS(50);
  xLastWakeTime = xTaskGetTickCount();
  while (1) {
    xSemaphoreTake(sem_engine, portMAX_DELAY);
    xSemaphoreGive(sem_engine);
    Serial.println("dasdas");
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    // Sets the trigPin on HIGH state for 10 micro seconds
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    // Reads the echoPin, returns the sound wave travel time in microseconds
    duration = pulseIn(echoPin, HIGH);
    // Calculating the distance
    distance = duration * 0.034 / 2;
    // Prints the distance on the //Serial Monitor
    if (distance <= 10) {
      analogWrite(enA, 0);
      analogWrite(enB, 0);

      xSemaphoreTake(sem_buzzer, portMAX_DELAY);
      digitalWrite(ultraBuzzer, HIGH);
      xSemaphoreGive(sem_buzzer);

      Serial.print("Distance: ");
      Serial.println(distance);
    }
    else {
      //      int pwmOutput = 255;
      //      analogWrite(enA, pwmOutput); // Send PWM signal to L298N Enable pin
      //      int pwmout2 = 255;
      //      analogWrite(enB, pwmout2); // Send PWM signal to L298N Enable pin

      xSemaphoreTake(sem_buzzer, portMAX_DELAY);
      digitalWrite(ultraBuzzer, LOW);
      xSemaphoreGive(sem_buzzer);
    }
    vTaskDelayUntil(&xLastWakeTime, xDelay);
  }
}

void Engine (void *pvParameters) // buzz when belt.
{
  TickType_t xLastWakeTime;
  const TickType_t xDelay = pdMS_TO_TICKS(100);
  xLastWakeTime = xTaskGetTickCount();
  int engineStart = 0;
  int lastButtonEngineState = 0;
  int count = 0;
  while (1) {
    xSemaphoreTake(sem_rf, portMAX_DELAY);
    buttonEngineState = digitalRead(buttonEngine);
    if (buttonEngineState != lastButtonEngineState) {
      if (buttonEngineState == HIGH) {
        count++;
        if (count > 1) {
          engineStart = !engineStart;
          if (engineStart) {
            xSemaphoreTake(sem_engineOut, portMAX_DELAY);
            xSemaphoreGive(sem_engine);
            digitalWrite(ledEngine, engineStart);
          }
          else {
            analogWrite(enA, 0);
            analogWrite(enB, 0);
            digitalWrite(buzzer, LOW);
            digitalWrite(ledEngine, engineStart);
            //digitalWrite(ledPin, LOW);
            xSemaphoreTake(sem_engine, portMAX_DELAY);
            xSemaphoreGive(sem_rf);
            xSemaphoreGive(sem_engineOut);
          }
        }
      }

      lastButtonEngineState = buttonEngineState;

    }
    xSemaphoreGive(sem_rf);
    if (engineStart) {
      int pwmOutput = 255;
      analogWrite(enA, pwmOutput); // Send PWM signal to L298N Enable pin5
      int pwmout2 = 255;
      analogWrite(enB, pwmout2); // Send PWM signal to L298N Enable pin
    }
    else {
      analogWrite(enA, 0);
      analogWrite(enB, 0);
    }
    vTaskDelayUntil(&xLastWakeTime, xDelay);
  }
}


void loop() {

}
/*----------------------------------- LCD Info---------------------------------------*/
/*
    The circuit:
   LCD RS pin to digital pin 12
   LCD Enable pin to digital pin 11
   LCD D4 pin to digital pin 5
   LCD D5 pin to digital pin 4
   LCD D6 pin to digital pin 3
   LCD D7 pin to digital pin 2
   LCD R/W pin to ground
   LCD VSS pin to ground
   LCD VCC pin to 5V
   10K resistor:
   ends to +5V and ground
   wiper to LCD VO pin (pin 3)




*/
