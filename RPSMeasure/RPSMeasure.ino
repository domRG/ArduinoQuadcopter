int irSignal;
int readPin = A0;
int revCount = 0;
int index = 0;
bool prevState;
int sensitivity = 100;
int numToAverage = 100;
int readings[numToAverage];

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(readPin,INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  irSignal = analogRead(readPin);
  if (irSignal > sensitivity)
  {
    prevState = LOW;
  }
  if (irSignal < sensitivity && prevState == LOW)
  {
    revCount++;
    index++;
    index = index % numToAverage;
    readings[index] = revTime;
    prevState = HIGH;
    //Serial.print("\n\rIRSignal:  "); Serial.print(irSignal); Serial.print("\tCount:  "); Serial.print(revCount);
  }
  //Serial.print("\n\rIRSignal:  "); Serial.print(irSignal);
}
