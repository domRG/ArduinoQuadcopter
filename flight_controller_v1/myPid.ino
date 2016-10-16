/*
float rotError[] = {0,0,0}; //rpy (xyz)
float errorChange[] = {0,0,0};
float xErrorI = 0;
float yErrorI = 0;

void pidRot(){
  
  rotError[0] = mpuReturn[6] - control[0];
  if(rotError[0] < balancePrecision && rotError[0] > -balancePrecision){
    rotError[0] = 0;
  }

  xErrorI += rotError[0];
  //Serial.print(xErrorI); Serial.print("\t");
  speeds[0] -= (int)(rotError[0] * Kp) + (int)(xErrorI * Ki) + (int)(errorChange[0] * Kd);
  speeds[1] += (int)(rotError[0] * Kp) + (int)(xErrorI * Ki) + (int)(errorChange[0] * Kd);
  speeds[2] -= (int)(rotError[0] * Kp) + (int)(xErrorI * Ki) + (int)(errorChange[0] * Kd);
  speeds[3] += (int)(rotError[0] * Kp) + (int)(xErrorI * Ki) + (int)(errorChange[0] * Kd);

  rotError[1] = mpuReturn[7] - control[1];
  if(rotError[1] < balancePrecision && rotError[1] > -balancePrecision){
    rotError[1] = 0;
  }

  yErrorI += rotError[1];
  //Serial.print(yErrorI); Serial.print("\t");
  speeds[0] -= (int)(rotError[1] * Kp) + (int)(yErrorI * Ki) + (int)(errorChange[1] * Kd);
  speeds[1] -= (int)(rotError[1] * Kp) + (int)(yErrorI * Ki) + (int)(errorChange[1] * Kd);
  speeds[2] += (int)(rotError[1] * Kp) + (int)(yErrorI * Ki) + (int)(errorChange[1] * Kd);
  speeds[3] += (int)(rotError[1] * Kp) + (int)(yErrorI * Ki) + (int)(errorChange[1] * Kd);
  
  servoSpeed();
   
}
*/
