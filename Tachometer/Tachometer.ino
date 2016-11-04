#include <LiquidCrystal.h>
LiquidCrystal lcd(12,11,5,4,3,2);
int readPin = A0;
int irSignal;
int sens = 100;
int prevStat = LOW;
int count = 0; //since last second
int revs = 0;
int recTime = 0;
int recDur = 0;
int prevTime = 0;
float rpm;
int now = 0;
int revDur = 0;
const int numToAv = 750;
int readings[numToAv];
int currentIndex;
int total;
float average;


void setup() {
  lcd.begin(16,2);
  lcd.setCursor(0,0);
  lcd.print("Rpm:        ");
  pinMode(readPin,INPUT);
}

void loop() {
  irSignal = analogRead(readPin);
  count++;
  if(irSignal > sens){
    prevStat = LOW;
  }
  if(irSignal < sens && prevStat == LOW){
    now = millis();
    revDur = now - prevTime;
    
    if(++currentIndex >= numToAv){
      currentIndex = 0;
    }
    total -= readings[currentIndex];
    readings[currentIndex] = revDur;
    total += revDur;
    average = (float) total / numToAv;
    
    rpm = 60000.00 / average;
    
    if(count >= 500){
      lcd.setCursor(0,1);
      lcd.print(rpm);
      count = 0;
    }

    /*
    Serial.print("\n\rAverage = "); Serial.print(average);
    Serial.print("\tTotal = "); Serial.print(total);
    Serial.print("\tRpm = "); Serial.print(rpm);
    */
    
    prevStat = HIGH;
    prevTime = now;
    
  }

/*
 * //rolling average
 * const int numToAv = 1000;
 * int readings[numToAv];
 * int currentIndex;
 * int total;
 * float average;
 * 
 * currentIndex = currentIndex % numToAv;
 * total -= readings[currentIndex];
 * readings[currentIndex] = revDur
 * total += readings[currentIndex];
 * average = total / numToAv;
 * currentIndex++;
 * 
 * 
 * 
 */
  
  /*
  now = millis();
  if(now - prevTime >= 1000){
    rpm = revs * 60;
    revs = 0;
    prevTime = now;
    Serial.print("\n\rRpm: "); Serial.print(rpm);
  }
  
  if(count++ >= 999){
    count = 0;
    recTime = millis();
    recDur = recTime - prevTime;
    prevTime = recTime;
    rpm = 60000 * revs / recDur;
    Serial.print("\n\rRpm: "); Serial.print(rpm);
    revs = 0;
  }
  */
}
