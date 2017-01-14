int irSignal[4];
int readPin[4] = {A0,A1,A2,A3};
int index[4] = {0,0,0,0};
bool prevState[4];
const int sensitivity = 100;
const int numToAverage = 50;
// The micro-second time for each revolution for each sensor over the last 'numToAverage' readings:
// (This is actually micro-seconds divided by 4 to reduce potential for overlow.)
unsigned int readings[4][numToAverage];

// The absolute micro-seconds time (divided by 4) for the start of the current revolution:
unsigned long prevTimeDiv4[4] = {0,0,0,0};

// Running total of all readings.  Is equal to sum(readings[i]):
unsigned long runTotalDiv4[4] = {0,0,0,0};

// Revolutions per second for each sensor:
double rps[4] = {0,0,0,0};

void setup() {
  int i;
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(A0,INPUT);
  pinMode(A1,INPUT);
  pinMode(A2,INPUT);
  pinMode(A3,INPUT);

  for (i = 0; i < numToAverage; i++) {
    readings[0][i] = readings[1][i] = readings[2][i] = readings[3][i] = 0;
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  irSignal[0] = analogRead(A0);
  irSignal[1] = analogRead(A1);
  unsigned long nowDiv4 = micros() >> 2;  // divide by 4 to keep numbers smaller
  irSignal[2] = analogRead(A2);
  irSignal[3] = analogRead(A3);

  updateRPS(0, nowDiv4);
  updateRPS(1, nowDiv4);
  updateRPS(2, nowDiv4);
  updateRPS(3, nowDiv4);

  //Serial.print("\n\rRps 0,1,2,3:"); Serial.print("\t"); Serial.print(runAvg[0]); Serial.print("\t"); Serial.print(irSignal[0]); Serial.print("\t"); Serial.print(rps[2]); Serial.print("\t"); Serial.print(rps[3]);
  Serial.print(rps[0]); Serial.print("\t"); Serial.print(rps[1]); Serial.print("\t"); Serial.print(rps[2]); Serial.print("\t"); Serial.print(rps[3]); Serial.print("\t");
  Serial.println();
}

void updateRPS(int i, unsigned long nowDiv4) {
  // revTimeDiv4 is the time for one revolution (divided by 4) (we're only measuring fast spinning things)
  unsigned int revTimeDiv4;  // range: 0 -- 65 thousand (not very many microseconds)
  // the position in the reading array that we're going to put the new time
  int readingIndex;
  
  if (irSignal[i] > sensitivity)
  {
    prevState[i] = LOW;
  }
  else if (prevState[i] == LOW)
  {
    prevState[i] = HIGH;

    // nowDiv4 and prevTimeDiv4 are both big, but their difference should be small, so fit into an unsigned int:
    revTimeDiv4 = (unsigned int)(nowDiv4 - prevTimeDiv4[i]);
    prevTimeDiv4[i] = nowDiv4;  // update outside 'if' as we are starting another revolution whether or not the last one took too long
    if (revTimeDiv4 > 0) {  // catches basic overflow
      //Serial.print(revTimeDiv4); Serial.print("\t");
      
      readingIndex = index[i];
      index[i] = (readingIndex + 1) % numToAverage;
      //Serial.print(readingIndex); Serial.print("\t");
      
      //Serial.print(runTotalDiv4[i]); Serial.print("\t");  // old running total
      //Serial.print(readings[i][readingIndex]); Serial.print("\t");  // old value we're removing
      runTotalDiv4[i] = (runTotalDiv4[i] + revTimeDiv4) - readings[i][readingIndex];
      readings[i][readingIndex] = revTimeDiv4;
      //Serial.print(runTotalDiv4[i]); Serial.print("\t");
      
      // Undo the divide by 4 so we get real rev-per-second:
      rps[i] = (double)(numToAverage * 1000000 / 4) / runTotalDiv4[i]; // 1000000 microseconds in 1 second, 1000000/x = number of turns that fit in 1 second (averaged over 0.1 second)
      
    }
  }
}

