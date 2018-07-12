#include <PID_v1.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include<stdlib.h>
#include "DHT.h"

#define DHTPIN 7
#define DHTTYPE DHT11

DHT dht(DHTPIN, DHTTYPE);
LiquidCrystal_I2C lcd(0x27, 20, 4);


const byte interruptPin1 = 2;
const byte interruptPin2 = 3;

int Move = 0;
int a[3];
int analog = A0;
int gop = 0 ;
double Setpoint, Input, Output, Outputf;
double aggKp = 4, aggKi = 0.2, aggKd = 5;
double consKp = 1, consKi = 0.05, consKd = 0.25;

PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);

void setup() {
  // pinMode( PIN_OUTPUT, OUTPUT);
  pinMode(9, OUTPUT);
  digitalWrite(9, HIGH);
  Setpoint = 20;
  myPID.SetMode(AUTOMATIC);
  dht.begin();
  pinMode(interruptPin1, INPUT);
  pinMode(interruptPin2, INPUT);
  attachInterrupt(digitalPinToInterrupt(interruptPin1), re, RISING);
  attachInterrupt(digitalPinToInterrupt(interruptPin2), re, RISING);
  lcd.begin();
  lcd.backlight();
  Serial.begin(9600);
  char ATcscs[] = {'A', 'T', '+', 'C', 'S', 'C', 'S', '=', '"', 'G', 'S', 'M', '"'};
  Serial.println(ATcscs);
  lcd.print("     EVERYTHING     ");
  lcd.print("    INITIALIZED");
  delay(2000);
  lcd.clear();
  delay(100);
  lcd.print("       MADE BY:       ");
  lcd.print("  Amin pishevar     ");
  lcd.print("  Hamed Rahimi");
  delay(2000);
}


void loop() {
  lcd.clear();
  Input = dht.readTemperature();
  int x = dht.readTemperature();
  double gap = abs(Setpoint - Input);
  if (gap < 10)
  { //we're close to setpoint, use conservative tuning parameters
    myPID.SetTunings(consKp, consKi, consKd);
  }
  else
  {
    //we're far from setpoint, use aggressive tuning parameters
    myPID.SetTunings(aggKp, aggKi, aggKd);
  }

  myPID.Compute();
  analogWrite( A1 , Output);
  lcd.print("  temperature is :            ");
  lcd.print(x);
  a[0] = analogRead(analog);
  delay(1000) ;
  a[1] = analogRead(analog);
  a[2] = abs(a[0] - a[1]);
  /*
    if (a[2] <= 10)
    {
     lcd.clear();
     delay(100);
     lcd.print("SYSTEM DETECTED");
     lcd.setCursor(0, 16);
     lcd.print("POWER OFF");
     delay(2000);
     lcd.clear();
     delay(100);
     lcd.print("SIMCARD IS READY");
     Serial.println("AT+CMGF=1");
     delay(1000);
     Serial.println();
     Serial.print("AT+CMGS=\""); // send the SMS number
     Serial.print("+989211871202");
     Serial.println("\"");
     delay(1000);
     Serial.print(" POWER OFF :( "); // SMS body
     delay(500);
     Serial.write(0x1A);
     lcd.clear();
     delay(200);
     lcd.print("SMS SENT");
     delay(5000);
    }

  */
  if (gop == 1)
  {
   
    lcd.clear();
    delay(1000);
    lcd.setCursor(0,2);
    lcd.print("SYSTEM DETECTED");
    lcd.setCursor(1, 7);
    lcd.print("MOTION");
    delay(2000);
    lcd.clear();
    delay(100);
    lcd.print("SIMCARD IS READY");
    delay(2000);
    Serial.println("AT+CMGF=1");
    delay(1000);
    Serial.println();
    Serial.print("AT+CMGS=\""); // send the SMS number
    Serial.print("+989178497984");
    Serial.println("\"");
    delay(1000);
    Serial.print(" MOTION DETECTED :( "); // SMS body
    delay(500);
    Serial.write(0x1A);
    lcd.clear();
    delay(100);
    lcd.print("SMS SENT");
    gop = 0;
    digitalWrite(9, HIGH                                                                                                              );
    Move = 1;
    delay(1000);

  }
  /* if ( Move == 1)
    {
     if (a[2] <= 10 )
     {
       digitalWrite(9, LOW);   // sets the LED on

       lcd.clear();
       delay(200);
       lcd.print("SYSTEM DETECTED");
       lcd.setCursor(0, 16);
       lcd.print("Stealing");
       delay(2000);
       lcd.clear();
       delay(200);
       lcd.print("SIMCARD IS READY");
       delay(2000);
       Serial.println("AT+CMGF=1");
       delay(1000);
       Serial.println();
       Serial.print("AT+CMGS=\""); // send the SMS number
       Serial.print("+989178700187");
       Serial.println("\"");
       delay(1000);
       Serial.print(" DANGER ALARM :( "); // SMS body
       delay(500);
       Serial.write(0x1A);
       lcd.clear();
       delay(200);
       lcd.print("SMS SENT");
       delay(5000);
       Move=0
       digitalWrite(9, HIGH);    // sets the LED off
     }
    }
  */
}

void re()
{
  gop = 1; 
  digitalWrite(9, LOW);
}

