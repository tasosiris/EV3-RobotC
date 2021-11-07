#pragma config(Sensor, S1,     ,               sensorEV3_Color)
#pragma config(Sensor, S2,     ,               sensorEV3_Color)
#pragma config(Sensor, S3,     ,               sensorEV3_Color)
#pragma config(Motor,  motorA,          a,             tmotorEV3_Medium, PIDControl, encoder)
#pragma config(Motor,  motorB,          b,             tmotorEV3_Medium, PIDControl, reversed, driveLeft, encoder)
#pragma config(Motor,  motorC,          c,             tmotorEV3_Medium, PIDControl, driveRight, encoder)
#pragma config(Motor,  motorD,          d,             tmotorEV3_Medium, PIDControl, encoder)


#include "hitechnic-colour-v2.h"

//7.85 orio mpatarias  |  ean to kseperasei FORTISH


tSensors lfport;
float offset;
float Tp;
float kp;
float kd;
float error;
float lastError;
float turn;
float j=0;
int comb1;
float degb=0;
float degc=0;
float lastdegb=0;
float lastdegc=0;
float stallstopb;
float stallstopc;
int comb2;
bool detectBlack;
float offs;
int cubenum[4];
int whichcube=0;
bool end=false;
bool failcheck=false;

void lfpdf()
{
  error=SensorValue[lfport]-offset;
  turn=kp*error+kd*(error-lastError);
  motor(b)=Tp+turn;
  motor(c)=Tp-turn;
  lastError=error;
}

void lfpd2()
{
  error=SensorValue[S2]-SensorValue[S3]+offset;
  turn=kp*error+kd*(error-lastError);
  motor(b)=Tp+turn;
  motor(c)=Tp-turn;
  lastError=error;

}

void move(tMotor m1, int degoal, int Tp){
  resetMotorEncoder(m1);
  motor(m1)=Tp;
  while(fabs(getMotorEncoder(m1))<=degoal){}
  motor(m1)=0;
}

void movestr(tMotor m1, tMotor m2, int str, int degoal, int Tp, bool pd, float kp, float kd, float skp, float sTp){
  resetMotorEncoder(m1);
  resetMotorEncoder(m2);
  error=0;
  j=0;
  while(abs(nMotorEncoder(m1))<=degoal-pd*40){
    if(abs(j)<=abs(Tp)) j=j+abs(Tp)/Tp*(0.2);\
    error=abs(getMotorEncoder(m1))-abs(getMotorEncoder(m2));
    turn=-(abs(Tp)/Tp)*abs(kp)*error+(-(abs(Tp)/Tp))*abs(kd)*(error-lastError);
    if(str==50) turn=0;\
    motor(m1)=(j+turn);
    motor(m2)=((1-0.02*str)*(j-turn));
    lastError=error;
  }
  if(pd==1)
    while((abs(getMotorEncoder(m1))+abs(getMotorEncoder(m2)))/(1+abs(1-0.02*str))<=degoal-1)
  {
    error=(abs(getMotorEncoder(m1))+abs(getMotorEncoder(m2)))/(1+abs(1-0.02*str))-degoal;
    motor(m1)=-(abs(Tp)/Tp)*abs(skp)*error+(abs(Tp)/Tp)*abs(sTp);
    motor(m2)=(-str/50+1)*(-(abs(Tp)/Tp)*abs(skp)*error+(abs(Tp)/Tp)*abs(sTp));
  }\

}
void movecol(tMotor m1, tMotor m2, int str, tSensors lfportStop, int degoal, int Tp, bool pd, float kp, float kd){
  resetMotorEncoder(m1);
  resetMotorEncoder(m2);
  error=0;
  j=0;
  while(degoal/abs(degoal)*SensorValue[lfportStop]<=degoal){
    if(abs(j)<=abs(Tp)) j=j+abs(Tp)/Tp*(0.2);\
    error=abs(getMotorEncoder(m1))-abs(getMotorEncoder(m2));
    turn=-(abs(Tp)/Tp)*abs(kp)*error+(-(abs(Tp)/Tp))*abs(kd)*(error-lastError);
    if(pd==0) turn=0;\
    if(str==50) turn=0;\
    motor(m1)=(j+turn);
    motor(m2)=((1-0.02*str)*(j-turn));
    lastError=error;
  }
}

void LFstop(tSensors lfportStop, int degoal)
{
  resetMotorEncoder(c);
  resetMotorEncoder(b);
  while(degoal/abs(degoal)*SensorValue(lfportStop)<=degoal)
  {lfpdf();}
  motor(c)=0;
  motor(b)=0;

}

void stall()
{
  nMotorPIDSpeedCtrl[b]=mtrNoReg;
  nMotorPIDSpeedCtrl[c]=mtrNoReg;
  degb=0;
  degc=0;
  lastdegb=0;
  lastdegc=0;
  resetMotorEncoder(c);
  resetMotorEncoder(b);
  stallstopb=0;
  stallstopc=0;
  motor[b]=-35;
  motor[c]=-35;
  setMotorBrakeMode(b, motorCoast);
  setMotorBrakeMode(c, motorCoast);
  wait1Msec(200);
  clearTimer(1);

  while((stallstopb==0||stallstopc==0)&&time1[1]<2000)
  {wait1Msec(50);

    degb=getMotorEncoder(b);
    if(stallstopb==0&&abs(degb-lastdegb)<5){
      move(b, 15, 30);
      setMotorSpeed(b, 0);
      stallstopb=1;}
    lastdegb=degb;

    degc=getMotorEncoder(c);
    if(stallstopc==0&&abs(degc-lastdegc)<5){
      move(c, 15, 30);
      setMotorSpeed(c, 0);
      stallstopc=1;}
    lastdegc=degc;
  }
  setMotorBrakeMode(b, motorBrake);
  setMotorBrakeMode(c, motorBrake);
  motor[b]=-20;
  motor[c]=-20;
  wait1Msec(250);
  motor[b]=0;
  motor[c]=0;
  nMotorPIDSpeedCtrl[b]=mtrSpeedReg;
  nMotorPIDSpeedCtrl[c]=mtrSpeedReg;
}

void upoi(){
  move(a, 180, 40);
}

void downoiextra(){
  nMotorPIDSpeedCtrl[a]=mtrNoReg;
  motor[a]=-10;
  wait1Msec(600);
  motor[a]=0;
  nMotorPIDSpeedCtrl[a]=mtrSpeedReg;
}

void placeElement(bool FixedOffs)
{
  //mprosta 25 + diplo line follower
  resetMotorEncoder(b);
  resetMotorEncoder(c);
  setMotorSync(b, c, 100, -30);
  while(getMotorEncoder(c)<35)
  {}
  setMotorSpeed(b, 0);
  setMotorSpeed(c, 0);

  clearTimer(1);
  offs=(FixedOffs*2-1)*22+0.4*(SensorValue[S3]-SensorValue[S2]-(FixedOffs*2-1)*22);
  offset=22; Tp=0; kp=-0.3; kd=-12;
  while(time1[1]<500) {lfpd2(); wait1Msec(3);}
  resetMotorEncoder(b);
  resetMotorEncoder(c);
  offset=offs;
  Tp=12; kp=-0.17; kd=-5.5;
  while(getMotorEncoder(c)<115)
  { lfpd2();
    wait1Msec(3);
  }
  setMotorSpeed(b, 0);
  setMotorSpeed(c, 0);

  //to afinei, paei kato gia 95
  resetMotorEncoder(d);
  setMotorSpeed(d, 10);
  while(getMotorEncoder(d)<165){}
  setMotorSpeed(d, 0);

  //paei 243 mprosta
  while(((abs(getMotorEncoder(b))+abs(getMotorEncoder(c)))/2)<205)
  { lfpd2();
    wait1Msec(3);
  }
  setMotorSpeed(b, 0);
  setMotorSpeed(c, 0);

  //teleionei tin kinisi pou to afinei
  setMotorSpeed(d, 10);
  while(nMotorEncoder(d)<275){}
  setMotorSpeed(d, 0);

  //anoigokleinei
  resetMotorEncoder(d);
  setMotorSpeed(d, -30);
  while(nMotorEncoder(d)>-120){}
  setMotorSpeed(d, 0);

  setMotorSpeed(d, 30);
  while(nMotorEncoder(d)<=0){}
  setMotorSpeed(d, 0);
  wait1Msec(50);
}

void placeElementRev(bool fixedOffs)
{//mprosta 25 + diplo line follower
  resetMotorEncoder(b);
  resetMotorEncoder(c);
  setMotorSync(b, c, 100, -30);
  while(getMotorEncoder(c)<35)
  {}
  setMotorSpeed(b, 0);
  setMotorSpeed(c, 0);

  clearTimer(1);
  offs=(FixedOffs*2-1)*22+0.4*(SensorValue[S3]-SensorValue[S2]-(FixedOffs*2-1)*22);
  offset=22; Tp=0; kp=-0.3; kd=-12;
  while(time1(1)<500)
  { lfpd2();
    wait1Msec(3);
  }

  resetMotorEncoder(b);
  resetMotorEncoder(c);
  offset=offs;
  Tp=12; kp=-0.17; kd=-5.5;
  while(((abs(getMotorEncoder(b))+abs(getMotorEncoder(c)))/2)<150)
  { lfpd2();
    wait1Msec(3);
  }
  setMotorSpeed(b, 0);
  setMotorSpeed(c, 0);


  //paei 242 mprosta
  while(((abs(getMotorEncoder(b))+abs(getMotorEncoder(c)))/2)<205)
  { lfpd2();
    wait1Msec(3);
  }
  setMotorSpeed(b, 0);
  setMotorSpeed(c, 0);

  //to afinei, paei kato gia 85
  resetMotorEncoder(d);
  setMotorSpeed(d, 10);
  while(nMotorEncoder(d)<160){}
  setMotorSpeed(d, 0);

  //paei 60 piso
  movestr(c, b, 0, 55, -12, 1, 0.4, 0.1, 0.2, -2);
  motor(c)=0;
  motor(b)=0;

  //teleionei tin kinisi pou to afinei
  setMotorSpeed(d, 10);
  while(nMotorEncoder(d)<270){}
  setMotorSpeed(d, 0);

  //anoigokleinei
  resetMotorEncoder(d);
  setMotorSpeed(d, -30);
  while(nMotorEncoder(d)>-120){}
  setMotorSpeed(d, 0);

  setMotorSpeed(d, 30);
  while(nMotorEncoder(d)<=0){}
  setMotorSpeed(d, 0);
}

//#taskStart##taskStart##taskStart##taskStart##taskStart##taskStart##taskStart##taskStart##taskStart##taskStart##taskStart##taskStart#

task htblack(){
  tHTCS2 colorSensor;
  initSensor(&colorSensor, S4);
  while(true){
    readSensor(&colorSensor);
    if(colorSensor.color==0)
    {detectBlack=true; }
  }
}
task down2(){
  nMotorPIDSpeedCtrl[d]=mtrNoReg;
  motor(d)=-8;
  wait1Msec(300);
  motor(d)=0;
  nMotorPIDSpeedCtrl[d]=mtrSpeedReg;
  move(d, 472, 20);
}

task up(){
  move(d, 277, -20);
}

task upfull(){
  move(d, 470, -20);
}

task firstdownoi(){
  move(a, 250, -80);
  nMotorPIDSpeedCtrl[a]=mtrNoReg;
  motor[a]=-10;
  wait1Msec(600);
  motor[a]=0;
  nMotorPIDSpeedCtrl[a]=mtrSpeedReg;
}

task upoiextra(){
  move(a, 100, 20);
}

task downoi(){
  move(a, 50, -20);
}

task lastupoi(){
  nMotorPIDSpeedCtrl[a]=mtrNoReg;
  motor[a]=50;
  wait1Msec(650);
  motor[a]=0;
  nMotorPIDSpeedCtrl[a]=mtrSpeedReg;
}
task down(){
  move(d, 277, 20);}

task shake(){
  move(d, 50, 100);
  move(d, 50, -60);
}

task sound(){
  playTone(500, 0.01);
}



task identifycubecol(){
  tHTCS2 colorSensor;
  initSensor(&colorSensor, S4);
  int cubecolor=0;
  bool sawcolor[4];
  sawcolor[0]=false;
  sawcolor[1]=false;
  sawcolor[2]=false;
  sawcolor[3]=false;
  cubenum[0]=69;
  cubenum[1]=69;
  cubenum[2]=69;
  cubenum[3]=69;

  while(whichcube<4){
  readSensor(&colorSensor);
    if((colorSensor.color==5||colorSensor.color==6)&&sawcolor[0]==false){
      cubenum[0]=whichcube+1; if(cubenum[0]<=0){cubenum[0]=cubenum[0]+4;}; sawcolor[0]=true;
      whichcube++; startTask(sound);}
    if((colorSensor.color==2||colorSensor.color==3)&&sawcolor[1]==false){
      cubenum[1]=whichcube; if(cubenum[1]<=0){cubenum[1]=cubenum[1]+4;}; sawcolor[1]=true;
      whichcube++; startTask(sound); }
    if((colorSensor.color==8||colorSensor.color==9)&&sawcolor[2]==false){
      cubenum[2]=whichcube-1; if(cubenum[2]<=0){cubenum[2]=cubenum[2]+4;}; sawcolor[2]=true;
      whichcube++; startTask(sound);}
    if(colorSensor.color==4&&sawcolor[3]==false){
      cubenum[3]=whichcube-2; if(cubenum[3]<=0){cubenum[3]=cubenum[3]+4;}; sawcolor[3]=true;
      whichcube++; startTask(sound);}

  }
}


//#tasksEnd##tasksEnd##tasksEnd##tasksEnd##tasksEnd##tasksEnd##tasksEnd##tasksEnd##tasksEnd##tasksEnd##tasksEnd##tasksEnd##tasksEnd#



//#taskMain##taskMain##taskMain##taskMain##taskMain##taskMain##taskMain##taskMain##taskMain##taskMain##taskMain##taskMain##taskMain#
//#taskMain##taskMain##taskMain##taskMain##taskMain##taskMain##taskMain##taskMain##taskMain##taskMain##taskMain##taskMain##taskMain#
//#taskMain##taskMain##taskMain##taskMain##taskMain##taskMain##taskMain##taskMain##taskMain##taskMain##taskMain##taskMain##taskMain#
task basic()

{
  tHTCS2 colorSensor;
  initSensor(&colorSensor, S4);
  movestr(c, b, 28, 258, 60, 0, 0, 0, 0, 0);
  movestr(b, c, 28, 245, 60, 0, 0, 0, 0, 0);

  lfport=S2; offset=43; Tp=80; kp=0.64; kd=58;
  resetMotorEncoder(c);
  while(getMotorEncoder(c)<140){lfpdf();}
  startTask(identifycubecol);
  while(whichcube<4) lfpdf();
  movestr(b, c, 0, 30, 80, 0, 0, 0, 0, 0);
  movestr(c, b, 36, 830, 80, 0, 0, 0, 0, 0);


  displayTextLine(2, "%d", cubenum[0]);
  displayTextLine(3, "%d", cubenum[1]);
  displayTextLine(4, "%d", cubenum[2]);
  displayTextLine(5, "%d", cubenum[3]);

  lfport=S2; offset=32; Tp=25; kp=0.24; kd=22;
  resetMotorEncoder(c);
  while(getMotorEncoder(c)<300)
  {lfpdf();}

  lfport=S2; offset=32; Tp=80; kp=0.82; kd=53;
  while(getMotorEncoder(c)<2600)
  {lfpdf();}
  motor(b)=0;
  motor(c)=0;

  LFstop(S3, -17);
  motor(b)=0;
  motor(c)=0;

  movestr(b, c, 0, 120, 30, 1, -0.4, -0.1, -0.2, 10);
  movestr(c, b, 100, 265, 30, 1, -0.4, -0.1, -0.2, 10);

  lfport=S2; offset=25; Tp=35; kp=0.27; kd=29;
  //slow linefollower
  playTone(999, 10);

  resetMotorEncoder(c);
  while(abs(getMotorEncoder(c))<60)
  {
    lfpdf();
  }
  //#detecting 1 black##detecting 1 black##detecting 1 black##detecting 1 black##detecting 1 black##detecting 1 black##detecting 1 black#
  detectBlack=false;
  startTask(htblack);


  resetMotorEncoder(c);
  lfport=S2; offset=25; Tp=55; kp=0.41; kd=38;
  while(abs(getMotorEncoder(c))<370)
  {
    lfpdf();
  }
  stopTask(htblack);
  if(detectBlack==false){comb1=1; playTone(500, 10); }

  //#detecting 2 black##detecting 2 black##detecting 2 black##detecting 2 black##detecting 2 black##detecting 2 black##detecting 2 black#
  detectBlack=false;
  startTask(htblack);
  while(abs(getMotorEncoder(c))<800)
  {
    lfpdf();
  }
  stopTask(htblack);
  if(detectBlack==false){comb1=2; playTone(500, 10); }

  //#detecting 3 black##detecting 3 black##detecting 3 black##detecting 3 black##detecting 3 black##detecting 3 black##detecting 3 black#
  detectBlack=false;

  startTask(htblack);

  while(abs(getMotorEncoder(c))<1095)
  {
    lfpdf();
  }
  motor[b]=0;
  motor[c]=0;


  stopTask(htblack);
  if(detectBlack==false){comb1=3; playTone(500, 10); }

  startTask(firstdownoi);
  LFstop(S3, -20);

  movestr(b, c, 0, 80, 30, 1, -0.4, -0.1, 0, 10);
  movestr(c, b, 100, 275, 30, 1, -0.4, -0.1, -0.2, 10);
  motor(b)=0;
  motor(c)=0;

  failcheck=true;
  lfport=S1;offset=18;Tp=-24; kp=0.6; kd=68;
  while(SensorValue[S2]>20&&SensorValue[S3]>20)
  {lfpdf();}
  resetMotorEncoder(b);
  while(getMotorEncoder(b)<85)
  {lfpdf();}
  setMotorSpeed(b, 0);
  setMotorSpeed(c, 0);
  failcheck=false;

  move(c, 25, -15);
  upoi();
  startTask(upoiextra);

  //

    if (comb1!=3)
  {
    //take blue 1 normal
    if (cubenum[1]<3)
    {
      startTask(down2);

      move(c, 30, 15);

      motor(b)=20;
      motor(c)=20;
      while(getColorReflected(S3)>30){}
      motor(b)=0;
      motor(c)=0;

      movestr(b, c, 0, 190, 30, 1, -0.4, -0.1, -0.2, 10);
      movestr(c, b, 100, 260, 30, 1, -0.3, -0.1, -0.2, 10);
      movestr(b, c, 0, 110, -30, 1, 0.4, 0.1, 0.2, -10);
      motor(b)=0;
      movestr(c, b, 50, 180, 25, 1, 0, 0, -0.2, 8);
      movestr(c, b, 50, 20, -25, 1, 0, 0, -0.2, -8);
      motor(c)=0;
      movestr(b, c, 0, 25, 20, 1, 0.4, 0.1, 0.2, -10);
      motor(b)=0;
      motor(c)=0;
      startTask(up);
      wait1Msec(400);
      movestr(b, c, 0, 45, -30, 0, 0.4, 0.1, 0.2, -10);
      movestr(b, c, 100, 260, 30, 1, -0.3, -0.1, 0, 30);
      movecol(b, c, 100, S2, -30, 20, 1, -0.3, -0.1);
      motor(b)=0;
      motor(c)=0;

    }
    else
    {
      //take blue 1 reverse
      movestr(b, c, 0, 130, 25, 1, 0.3, 0.1, -0.2, 8);
      movestr(b, c, 100, 120, 30, 1, -0.3, -0.1, 0, 30);
      startTask(down2);
      movestr(b, c, 100, 400, 30, 1, -0.3, -0.1, 0.2, 10);
      //movestr(b, c, 0, -50, 20, 1, 0.20, 0.1, -0.2, 8);
      //movecol(b, c, 0, S4, -13, -20, 1, 0.3, 0.1);
      movestr(b, c, 0, 125, -20, 1, 0.20, 0.1, -0.2, 8);
      movestr(b, c, 100, 180, 15, 1, -0.20, -0.1, 0, 8);
      movestr(c, b, 100, 10, 15, 1, -0.20, -0.1, 0, 8);
      movestr(b, c, 0, 13, 20, 1, 0.20, 0.1, -0.2, 8);
      motor(b)=0;
      motor(c)=0;
      move(d, 277, -20);
      movestr(b, c, 0, 110, -30, 1, -0.3, -0.1, 0.2, -0);
      movestr(b, c, 100, 265, 30, 1, -0.3, -0.1, 0.2, 10);
      movecol(b, c, 100, S2, -30, 20, 1, -0.3, -0.1);
      motor(b)=0;
      motor(c)=0;
    }

  }
  else
  {
    if(cubenum[1]<3)
    {
      //take blue 2 normal
      startTask(down2);

      move(c, 30, 15);

      motor(b)=20;
      motor(c)=20;
      while(getColorReflected(S3)>30){}
      motor(b)=0;
      motor(c)=0;

      movestr(b, c, 0, 320, 30, 1, -0.4, -0.1, -0.2, 10);
      movestr(c, b, 100, 260, 30, 1, -0.3, -0.1, -0.2, 10);
      movestr(b, c, 0, 252, 30, 1, 0.4, 0.1, 0.2, -10);
      motor(b)=0;
      motor(c)=0;
      movestr(c, b, 50, 320, 25, 1, 0, 0, -0.2, 8);
      movestr(c, b, 50, 20, -25, 1, 0, 0, -0.2, -8);
      motor(c)=0;
      startTask(up);
      wait1Msec(300);
      movestr(b, c, 0, 15, -30, 0, 0.4, 0.1, 0.2, -10);
      movestr(c, b, 50, 200, -30, 1, -0.3, -0.1, 0, 30);
      movestr(b, c, 0, 365, -30, 1, -0.3, -0.1, 0, 30);
      movestr(b, c, 100, 180, 30, 1, -0.3, -0.1, 0, 30);
      movecol(b, c, 100, S2, -25, 16, 1, -0.3, -0.1);
      motor(b)=0;
      motor(c)=0;
    }
    else
    {
      //take blue 2 reverse
      startTask(down2);

      move(c, 30, 15);

      motor(b)=20;
      motor(c)=20;
      while(getColorReflected(S3)>30){}
      motor(b)=0;
      motor(c)=0;
      lfport=S3; offset=30; Tp=25; kp=-0.18; kd=-20;
      movestr(b, c, 0, 307, 30, 1, -0.4, -0.1, -0.2, 10);
      movestr(c, b, 100, 260, 30, 1, -0.3, -0.1, -0.2, 10);
      LFstop(S2, -20);
      LFstop(S2, 40);
      movestr(c, b, 50, 280, 25, 1, 0, 0, -0.2, 8);
      movestr(b, c, 50, 176, 12, 1, 0, 0, -0.2, -8);
      motor(b)=0;
      startTask(up);
      wait1Msec(300);
      movestr(b, c, 0, 440, -30, 1, -0.4, -0.1, -0.2, 10);
      movestr(b, c, 100, 260, 30, 1, -0.3, -0.1, 0, 30);
      movecol(b, c, 100, S2, -30, 18, 1, -0.3, -0.1);
      motor(b)=0;
      motor(c)=0;
    }
  }
  lfport=S2; offset=30; Tp=35; kp=0.27; kd=29;

    while(getMotorEncoder(c)<100)
  { lfpdf();
  }

  lfport=S2; offset=30; Tp=50; kp=0.48; kd=37;
  LFstop(S3, -30);
  movestr(b, c, 0, 230, 25, 1, -0.4, -0.1, -0.2, 10);
  movestr(c, b, 100, 250, 25, 1, -0.3, -0.1, -0.2, 8);
  motor(b)=0;
  motor(c)=0;
  /*
  int i;
  for(i=0; i<2; i=i+1){
  while(SensorValue[S2]>20){lfpdf();}
  setMotorSync(b, c, 100, -30);
  while(SensorValue[S2]<40){}}
  setMotorSpeed(b, 0);
  setMotorSpeed(c, 0);

  resetMotorEncoder(c);
  setMotorSync(b, c, 100, -30);
  while(getMotorEncoder(c)<175)
  {}
  setMotorSpeed(b, 0);
  setMotorSpeed(c, 0);
  */



  lfport=S3; offset=30; Tp=50; kp=-0.4; kd=-37;
  while(SensorValue[S2]>20) lfpdf();
  while(SensorValue[S2]<40)lfpdf();
  while(SensorValue[S2]>20)lfpdf();
  while(SensorValue[S2]<40)lfpdf();
  movestr(b, c, 0, 175, 25, 1, 0.4, 0.1, 0.2, 8);
  failcheck=true;
  setMotorSync(b, c, 0, 30);
  while(SensorValue[S1]<40){}
  while(SensorValue[S1]>20){}
  while(SensorValue[S1]<40){}


  lfport=S1; offset=18; Tp=-16; kp=0.4; kd=45;
  resetMotorEncoder(b);
  resetMotorEncoder(c);
  while(getMotorEncoder(c)>-410)
  {lfpdf();}
  setMotorSpeed(b, 0);
  setMotorSpeed(c, 0);

  resetMotorEncoder(c);
  setMotorSpeed(c, -30);
  while(getMotorEncoder(c)>-115){}
  setMotorSpeed(c, 0);

  resetMotorEncoder(b);
  setMotorSpeed(b, 0);
  setMotorSpeed(b, -30);
  while(getMotorEncoder(b)<115){}
  setMotorSpeed(b, 0);

  startTask(downoi);
clearTimer(2);

    lfport=S1; offset=25; Tp=-15; kp=0.15; kd=4.2;
  while((colorSensor.color!=9&&colorSensor.color!=8)&&time1[2]<=1400)
  {lfpdf();
    readSensor(&colorSensor);}

  setMotorSpeed(b, 0);
  setMotorSpeed(c, 0);
  failcheck=false;
  downoiextra();


  //elenxoume tin katefthinsi
  //****************************************************************************************************************
  //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  if (cubenum[1]==2||cubenum[1]==4){
    movestr(b, c, 0, 545, 40, 1, 0.4, 0.1, 0.2, 10);
    movestr(c, b, 100, 260, 35, 1, -0.3, -0.1, -0.2, 10);
    startTask(lastupoi);
    movecol(b, c, 0, S2, -20, 30, 1, -0.4, -0.1);
    movestr(b, c, 0, 185, 25, 1, -0.4, -0.1, -0.2, 6);
    movestr(c, b, 100, 255, 30, 1, -0.3, -0.1, -0.2, 10);
    setMotorSpeed(b, 0);
    setMotorSpeed(c, 0);
    resetMotorEncoder(b);
    resetMotorEncoder(c);
    startTask(shake);
    //if   -->kanoniko
    //else -->anapodo
    if (cubenum[1]==2)
    {
      //place blue 2
      offset=37; Tp=10; kp=-0.4; kd=-15;
      while(getMotorEncoder(c)<20)
      { lfpd2();
      }


      Tp=13; kp=-0.2; kd=-8;
      while(getMotorEncoder(c)<115)
      { lfpd2();
      }
      setMotorSpeed(b, 0);
      setMotorSpeed(c, 0);

      //to afinei, paei kato gia 95
      resetMotorEncoder(d);
      setMotorSpeed(d, 10);
      while(nMotorEncoder(d)<160){}
      setMotorSpeed(d, 0);

      //paei 243 mprosta
      while(colorSensor.color!=12&&colorSensor.color!=17)
      {readSensor(&colorSensor);
        lfpd2();
      }
      playTone(500, 3);
      resetMotorEncoder(c);
      while(getMotorEncoder(c)<60)
      { lfpd2();
      }
      setMotorSpeed(b, 0);
      setMotorSpeed(c, 0);

      //teleionei tin kinisi pou to afinei
      setMotorSpeed(d, 10);
      while(nMotorEncoder(d)<280){}
      setMotorSpeed(d, 0);

      //anoigokleinei
      resetMotorEncoder(d);
      setMotorSpeed(d, -30);
      while(nMotorEncoder(d)>-120){}
      setMotorSpeed(d, 0);

      setMotorSpeed(d, 30);
      while(nMotorEncoder(d)<=0){}
      setMotorSpeed(d, 0);

      movestr(b, c, 0, 35, -30, 0, 0, 0, 0.2, 8);

    }
    //***************************************************************************************************************
    else
    {
      //place blue 4
      offset=35; Tp=10; kp=-0.4; kd=-15;
      while(getMotorEncoder(c)<90)
      { lfpd2();
      }


      Tp=13; kp=-0.2; kd=-9;
      while(colorSensor.color!=12&&colorSensor.color!=17)
      {
        readSensor(&colorSensor);
        lfpd2();
      }
      setMotorSpeed(b, 0);
      setMotorSpeed(c, 0);
      resetMotorEncoder(b);
      resetMotorEncoder(c);

      //paei 242 mprosta
      while(getMotorEncoder(c)<70)
      { lfpd2();
      }
      setMotorSpeed(b, 0);
      setMotorSpeed(c, 0);

      //to afinei, paei kato gia 85
      resetMotorEncoder(d);
      setMotorSpeed(d, 10);
      while(nMotorEncoder(d)<160){}
      setMotorSpeed(d, 0);

      //paei 60 piso
      movestr(c, b, 0, 50, -12, 1, 0.4, 0.1, 0.2, -2);
      motor(c)=0;
      motor(b)=0;

      //teleionei tin kinisi pou to afinei
      setMotorSpeed(d, 10);
      while(nMotorEncoder(d)<280){}
      setMotorSpeed(d, 0);

      //anoigokleinei
      resetMotorEncoder(d);
      setMotorSpeed(d, -30);
      while(nMotorEncoder(d)>-120){}
      setMotorSpeed(d, 0);

      setMotorSpeed(d, 30);
      while(nMotorEncoder(d)<=0){}
      setMotorSpeed(d, 0);

    }
    startTask(upfull);
    movestr(b, c, 0, 262, -30, 1, 0.3, 0.1, 0.2, 8);
    movestr(b, c, 100, 255, 25, 1, 0.3, 0.1, 0.2, 7);
  }
  //****************************************************************************************************************
  //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  else
  {
    movestr(b, c, 0, 545, 40, 1, 0.4, 0.1, 0.2, 10);
    movestr(c, b, 100, 265, 35, 1, -0.3, -0.1, -0.2, 10);
    movecol(b, c, 0, S2, -20, 45, 1, -0.4, -0.1);
    movecol(b, c, 0, S2, 40, 40, 1, -0.4, -0.1);
    movecol(b, c, 0, S2, -20, 35, 1, -0.4, -0.1);
    movestr(b, c, 0, 140, 30, 1, -0.4, -0.1, -0.2, 6);
    movestr(c, b, 100, 260, 30, 1, -0.3, -0.1, -0.2, 10);
    setMotorSpeed(b, 0);
    setMotorSpeed(c, 0);
    resetMotorEncoder(b);
    resetMotorEncoder(c);

    startTask(lastupoi);
    lfport=S2; offset=40; Tp=35; kp=0.3; kd=29;
    LFstop(S3, -30);
    setMotorSpeed(b, 0);
    setMotorSpeed(c, 0);

    movestr(b, c, 0, 205, 30, 1, -0.4, -0.1, -0.2, 10);
    motor(c)=0;
    motor(b)=0;

    movestr(c, b, 100, 262, 40, 1, -0.4, -0.1, -0.2, 10);
    motor(c)=0;
    motor(b)=0;

    //stall
    stall();
    startTask(shake);

    if (cubenum[1]==1)
    {
      //place blue 1

      placeElement(0);

    }
    //***************************************************************************************************************
    else
    {
      //place blue 3
      placeElementRev(0);


    }
    movestr(b, c, 0, 155, -30, 1, -0.4, -0.1, -0.2, 10);
    movestr(c, b, 50, 470, 50, 1, 0, 0, 0.2, 7);
    movecol(c, b, 50, S3, -40, 15, 1, -0.3, -0.1);
    startTask(upfull);
    lfport=S3; offset=30; Tp=35; kp=-0.27; kd=-29;
    LFstop(S2, -20);
    movestr(b, c, 0, 180, 30, 1, 0.3, 0.1, 0.2, 8);
    movestr(c, b, 100, 210, 50, 1, 0.3, 0.1, 0.2, 7);
    failcheck=true;
    movecol(c, b, 100, S1, -20, 12, 1, -0.3, -0.1);
    motor[b]=0;
    motor[c]=0;
    lfport=S1; offset=30; Tp=-25; kp=-0.43; kd=-20;
    LFstop(S2, -20);
    resetMotorEncoder(c);
    while(abs(getMotorEncoder(c))<222) lfpdf();
    motor[b]=0;
    motor[c]=0;

  }
  startTask(firstdownoi);

  //==========================================================================================================================
  //))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))
  //((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((()
  //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

  //slow linefollower
  playTone(999, 10);
  lfport=S1; offset=30; Tp=-15; kp=-0.26; kd=-12;
  resetMotorEncoder(c);
  while(abs(getMotorEncoder(c))<20)
  {
    lfpdf();
  }
  //#detecting 1 black##detecting 1 black##detecting 1 black##detecting 1 black##detecting 1 black##detecting 1 black##detecting 1 black#
  detectBlack=false;
  startTask(htblack);

  resetMotorEncoder(c);
  while(abs(getMotorEncoder(c))<60)
  {
    lfpdf();
  }


  lfport=S1; offset=30; Tp=-30; kp=-0.5; kd=-24;
  resetMotorEncoder(c);

  while(abs(getMotorEncoder(c))<220)
  {
    lfpdf();
  }
  stopTask(htblack);
  if(detectBlack==false){comb2=3; playTone(500, 10); }

  //#detecting 2 black##detecting 2 black##detecting 2 black##detecting 2 black##detecting 2 black##detecting 2 black##detecting 2 black#
  detectBlack=false;
  resetMotorEncoder(c);
  startTask(htblack);
  while(abs(getMotorEncoder(c))<370)
  {
    lfpdf();
  }
  stopTask(htblack);
  if(detectBlack==false){comb2=2; playTone(500, 10); }

  //#detecting 3 black##detecting 3 black##detecting 3 black##detecting 3 black##detecting 3 black##detecting 3 black##detecting 3 black#
  detectBlack=false;
  resetMotorEncoder(c);
  startTask(htblack);

  while(getColorReflected(S2)>20||getColorReflected(S3)>20)
  {
    lfpdf();
  }

  motor[b]=0;
  motor[c]=0;
  LFstop(S3, 40);

  stopTask(htblack);
  if(detectBlack==false){comb2=1; playTone(500, 10); }
  failcheck=false;

  /*for(int i=3; i>0; i--)
  {
  if (SensorValue[ht]>=14){comb2=i; playTone(500, 10);}
  LFstop(S3, -20);
  LFstop(S3, 40);
  if (i>1)
  { resetMotorEncoder(c);
  while(abs(getMotorEncoder(c))<35)
  {lfpdf();}
  setLEDColor(ledRed);

  movestr(b, c, 0, 45, -20, 1, 0.3, 0.1, 0.2, 8);
  setLEDColor(ledGreenFlash);

  resetMotorEncoder(c);
  while(abs(getMotorEncoder(c))<210)
  {lfpdf();}
  }
  }*/
  movestr(b, c, 0, 170, -30, 1, 0.3, 0.1, 0.2, 10);

  movestr(b, c, 100, 255, 25, 1, 0.3, 0.1, 0.2, 8);
  lfport=S2; offset=30; Tp=50; kp=0.4; kd=37;
  LFstop(S3, -20);
  movestr(b, c, 0, 165, 20, 1, 0.3, 0.1, 0.2, 8);
  movestr(b, c, 100, 260, 25, 1, 0.3, 0.1, 0.2, 8);
  movestr(b, c, 0, 60, -20, 1, 0.3, 0.1, 0.2, 8);
  movestr(b, c, 100, 260, 25, 1, 0.3, 0.1, 0.2, 8);
  motor[b]=0;
  motor[c]=0;

  failcheck=true;
  lfport=S1; offset=18; Tp=-24; kp=-0.6; kd=-68;
  while(SensorValue[S2]>20&&SensorValue[S3]>20)
  {lfpdf();}
  resetMotorEncoder(b);
  while(getMotorEncoder(b)<85)
  {lfpdf();}
  setMotorSpeed(b, 0);
  setMotorSpeed(c, 0);

  move(b, 25, -15);
  upoi();
  startTask(upoiextra);
  failcheck=false;
  //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //###############################################################################################################################
if (comb1!=1)
  {
    if (cubenum[2]<3)
    {
      //take red 1 normal
      startTask(down2);

      move(b, 30, 15);

      motor(b)=20;
      motor(c)=20;
      while(getColorReflected(S2)>30){}
      motor(b)=0;
      motor(c)=0;

      movestr(b, c, 0, 190, 30, 1, -0.4, -0.1, -0.2, 10);
      movestr(b, c, 100, 260, 30, 1, -0.3, -0.1, -0.2, 10);
      movestr(b, c, 0, 110, -30, 1, 0.4, 0.1, 0.2, -10);
      motor(c)=0;
      movestr(b, c, 50, 198, 25, 1, 0, 0, -0.2, 8);
      movestr(b, c, 50, 25, -25, 1, 0, 0, -0.2, -8);
      movestr(b, c, 0, 20, 25, 1, 0.4, 0.1, 0.2, -10);
      motor(b)=0;
      move(d, 277, -20);
      movestr(b, c, 0, 25, -30, 0, 0.4, 0.1, 0.2, -10);
      movestr(c, b, 100, 260, 30, 1, -0.3, -0.1, 0, 30);
      movecol(c, b, 100, S3, -30, 20, 1, -0.3, -0.1);
      motor(b)=0;
      motor(c)=0;

    }
    //===============================================================================================================================
    else
    {
      //take red 1 reverse
      movestr(b, c, 0, 110, 25, 1, 0.3, 0.1, -0.2, 8);
      movestr(c, b, 100, 120, 30, 1, -0.3, -0.1, 0, 30);
      startTask(down2);
      movestr(c, b, 100, 400, 30, 1, -0.3, -0.1, 0.2, 10);
      //movestr(b, c, 0, -50, 20, 1, 0.20, 0.1, -0.2, 8);
      //movecol(b, c, 0, S4, -13, -20, 1, 0.3, 0.1);
      movestr(b, c, 0, 175, -20, 1, 0.20, 0.1, -0.2, 8);
      movestr(c, b, 100, 188, 15, 1, -0.20, -0.1, 0, 8);
      //movestr(b, c, 0, 20, 20, 1, 0.20, 0.1, -0.2, 8);
      movestr(b, c, 0, 13, 25, 1, 0.3, 0.1, -0.2, 8);
      motor(b)=0;
      motor(c)=0;
      move(d, 277, -20);
      movestr(b, c, 0, 70, -30, 1, -0.3, -0.1, 0.2, -0);
      movestr(c, b, 100, 250, 30, 1, -0.3, -0.1, 0.2, 10);
      movecol(c, b, 100, S3, -40, 15, 1, 0.2, 0.1);
      motor(b)=0;
      motor(c)=0;
    }

  }
  //===============================================================================================================================
  //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
  else
  {
    if(cubenum[2]<3)
    {
      //take red 2 normal

      move(b, 30, 15);

      motor(b)=20;
      motor(c)=20;
      while(getColorReflected(S2)>30){}
      motor(b)=0;
      motor(c)=0;
      startTask(down2);
      movestr(b, c, 0, 300, 30, 1, -0.4, -0.1, -0.2, 10);
      movestr(b, c, 100, 260, 30, 1, -0.3, -0.1, -0.2, 10);
      movestr(b, c, 0, 240, 30, 1, 0.4, 0.1, 0.2, -10);
      motor(b)=0;
      motor(c)=0;
      movestr(b, c, 50, 308, 20, 1, 0, 0, -0.2, 8);
      movestr(b, c, 50, 40, -25, 1, 0, 0, -0.2, -8);
      motor(b)=0;
      startTask(up);
      wait1Msec(250);
      movestr(b, c, 0, 15, -30, 0, 0.4, 0.1, 0.2, -10);
      movestr(b, c, 50, 200, -30, 1, -0.3, -0.1, 0, 30);
      movestr(b, c, 0, 360, -30, 1, -0.3, -0.1, 0, 30);
      movestr(c, b, 100, 260, 30, 1, -0.3, -0.1, 0, 30);
      movecol(c, b, 100, S3, -30, 18, 1, -0.3, -0.1);
      motor(b)=0;
      motor(c)=0;
    }
    //===============================================================================================================================
    else
    {
      //take red 2 reverse
      startTask(down2);

      move(b, 30, 15);

      motor(b)=20;
      motor(c)=20;
      while(getColorReflected(S2)>30){}
      motor(b)=0;
      motor(c)=0;
      lfport=S2; offset=30; Tp=25; kp=0.18; kd=20;
      movestr(b, c, 0, 300, 30, 1, -0.4, -0.1, -0.2, 10);
      movestr(b, c, 100, 260, 30, 1, -0.3, -0.1, -0.2, 10);
      LFstop(S3, -20);
      LFstop(S3, 40);
      move(b, 275, 20);
      move(c, 176, 15);
      startTask(up);
      wait1Msec(500);
      movestr(b, c, 0, 440, -30, 1, -0.4, -0.1, -0.2, 10);
      movestr(c, b, 100, 260, 30, 1, -0.3, -0.1, 0, 30);
      movecol(c, b, 100, S3, -35, 18, 1, -0.3, -0.1);
      motor(b)=0;
      motor(c)=0;
    }
  }

  //###############################################################################################################################
  //===============================================================================================================================
  //kokino

  lfport=S3; offset=30; Tp=35; kp=-0.27; kd=-29;
  LFstop(S2, -30);

  //dialegei 1 h 3 / 2 h 4
  if (cubenum[2]==1||cubenum[2]==3)//111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111//
  {
    movestr(b, c, 0, 240, 30, 1, 0.3, 0.1, 0.2, 8);
    movestr(b, c, 100, 250, 25, 1, -0.3, -0.1, -0.2, 8);
    motor(b)=0;
    motor(c)=0;
    lfport=S2; offset=30; Tp=35; kp=0.27; kd=29;
    LFstop(S3, -20);
    movestr(b, c, 0, 190, 25, 1, -0.3, -0.1, -0.2, 8);
    motor[b]=0;
    motor[c]=0;
    movestr(c, b, 100, 255, 25, 1, -0.3, -0.1, -0.2, 8);
    motor[b]=0;
    motor[c]=0;
    resetMotorEncoder(c);
    startTask(shake);
    //dialegei 1 h 3

    if (cubenum[2]==1)//222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222//
    {
      //place red 1
      offset=-10; Tp=10; kp=-0.4; kd=-20;
      while(getMotorEncoder(c)<110)
      { lfpd2();}

      Tp=13; kp=-0.2; kd=-8;
      while(getMotorEncoder(c)<210)
      { lfpd2();
      }
      setMotorSpeed(b, 0);
      setMotorSpeed(c, 0);

      //to afinei, paei kato gia 95
      resetMotorEncoder(d);
      setMotorSpeed(d, 10);
      while(nMotorEncoder(d)<160){}
      setMotorSpeed(d, 0);

      //paei 243 mprosta
      while(colorSensor.color!=17)
      {
      readSensor(&colorSensor);
      lfpd2();
      }
      while(getMotorEncoder(c)<295)
      { lfpd2();
      }
      setMotorSpeed(b, 0);
      setMotorSpeed(c, 0);

      //teleionei tin kinisi pou to afinei
      setMotorSpeed(d, 10);
      while(nMotorEncoder(d)<270){}
      setMotorSpeed(d, 0);

      //anoigokleinei
      resetMotorEncoder(d);
      setMotorSpeed(d, -30);
      while(nMotorEncoder(d)>-120){}
      setMotorSpeed(d, 0);

      setMotorSpeed(d, 30);
      while(nMotorEncoder(d)<=0){}
      setMotorSpeed(d, 0);
    }

    //dialegei 1 h 3
    else//222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222//
    {
      //place red 3
      offset=-25; Tp=10; kp=-0.4; kd=-20;
      while(getMotorEncoder(c)<95)
      { lfpd2();
      }

      Tp=13; kp=-0.2; kd=-8;
      while(getMotorEncoder(c)<160)
      { lfpd2();
      }
      setMotorSpeed(b, 0);
      setMotorSpeed(c, 0);


      //paei 242 mprosta
      while(getMotorEncoder(c)<297)
      { lfpd2();
      }
      setMotorSpeed(b, 0);
      setMotorSpeed(c, 0);

      //to afinei, paei kato gia 85
      resetMotorEncoder(d);
      setMotorSpeed(d, 10);
      while(nMotorEncoder(d)<170){}
      setMotorSpeed(d, 0);

      //paei 60 piso
      movestr(c, b, 0, 55, -12, 1, 0.4, 0.1, 0.2, -2);
      motor(c)=0;
      motor(b)=0;

      //teleionei tin kinisi pou to afinei
      setMotorSpeed(d, 10);
      while(nMotorEncoder(d)<300){}
      setMotorSpeed(d, 0);

      //anoigokleinei
      resetMotorEncoder(d);
      setMotorSpeed(d, -30);
      while(nMotorEncoder(d)>-120){}
      setMotorSpeed(d, 0);

      setMotorSpeed(d, 30);
      while(nMotorEncoder(d)<=0){}
      setMotorSpeed(d, 0);
      wait1Msec(500);

    }

    //1PerneiPrasino11PerneiPrasino11PerneiPrasino11PerneiPrasino11PerneiPrasino11PerneiPrasino11PerneiPrasino11PerneiPrasino11PerneiPrasino1
    //1PerneiPrasino11PerneiPrasino11PerneiPrasino11PerneiPrasino11PerneiPrasino11PerneiPrasino11PerneiPrasino11PerneiPrasino11PerneiPrasino1
    //1PerneiPrasino11PerneiPrasino11PerneiPrasino11PerneiPrasino11PerneiPrasino11PerneiPrasino11PerneiPrasino11PerneiPrasino11PerneiPrasino1


  if (comb2!=1)//22222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222//
    {
      if (cubenum[3]<3) //333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333//
      {
        //take green 1 normal no wall
        movestr(b, c, 0, 50, -30, 1, 0.3, 0.1, 0.2, 8);
        movestr(c, b, 100, 510, 25, 1, -0.3, -0.1, -0.2, 8);
        offset=37; Tp=25; kp=-1; kd=-50;
        while(SensorValue[S2]+SensorValue[S3]>30){
        movestr(b, c, 0, 2, 30, 1, 0.3, 0.1, 0.2, 8);}
        movestr(b, c, 0, 65, 30, 1, 0.3, 0.1, 0.2, 8);
        motor[b]=0;
        motor[c]=0;
        startTask(up);
        wait1Msec(200);
        movestr(b, c, 0, 100, -30, 1, 0.3, 0.1, 0.2, 8);
        movestr(b, c, 100, 255, 25, 1, 0.3, 0.1, 0.2, 8);
        motor[c]=0;
        motor[b]=0;
        wait1Msec(200);
        movecol(b, c, 0, S3, -20, 25, 1, -0.3, -0.1);
        motor[c]=0;
        motor[b]=0;

        movestr(b, c, 0, 125, 25, 1, 0.3, 0.1, 0.2, 8);
        motor[c]=0;
        motor[b]=0;


        movecol(b, c, 100, S3, -20, 15, 1, -0.3, -0.1);
        movecol(b, c, 100, S3, 40, 15, 1, -0.3, -0.1);
        motor[c]=0;
        motor[b]=0;
      }

      else//3333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333//
      {
        //take green 1 reverse no wall
        movestr(b, c, 0, 50, -30, 1, 0.3, 0.1, 0.2, 8);
        movestr(c, b, 100, 500, 15, 1, -0.3, -0.1, -0.2, 8);
        motor[c]=0;
        movestr(b, c, 50, 270, 15, 1, 0.3, 0.1, 0.2, 8);

        motor[c]=0;
        motor[b]=0;
        resetMotorEncoder(c);
        motor[b]=25;
        motor[c]=25;
        while(getMotorEncoder(c)<420){};

        motor[c]=0;
        motor[b]=0;
        movestr(c, b, 100, 380, 25, 1, -0.3, -0.1, -0.2, 8);
        motor[c]=0;
        motor[b]=0;
        movestr(b, c, 50, 20, -15, 1, 0.3, 0.1, 0.2, 8);
        motor[c]=0;
        motor[b]=0;
        movestr(b, c, 0, 35, 30, 1, 0.3, 0.1, 0.2, 8);
        motor[c]=0;
        motor[b]=0;
        startTask(up);
        wait1Msec(300);
        movestr(c, b, 0, 50, -30, 1, -0.3, -0.1, -0.2, 8);
        motor[c]=0;
        motor[b]=0;
        movestr(c, b, 100, 210, 25, 1, -0.3, -0.1, -0.2, 8);
        movecol(c, b, 100, S3, -18, 15, 1, -0.3, -0.1);
        motor[c]=0;
        motor[b]=0;


      }
    }

    else//222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222//
    {
      if (cubenum[3]<3)//333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333//
      {
        //take green 2 normal no wall
        movestr(b, c, 0, 65, -30, 1, 0.3, 0.1, 0.2, 8);
        movestr(c, b, 100, 500, 25, 1, -0.3, -0.1, -0.2, 8);
        movestr(c, b, 50, 335, 25, 1, -0.3, -0.1, -0.2, 8);
        movestr(b, c, 50, 200, 25, 1, 0.3, 0.1, 0.2, 8);
        movestr(b, c, 0, 30, 20, 1, 0.3, 0.1, 0.2, 8);
        motor[b]=0;
        motor[c]=0;
        startTask(up);
        wait1Msec(400);
        movestr(b, c, 50, 388, -25, 1, 0.3, 0.1, 0.2, 8);
        movestr(b, c, 0, 285, -40, 1, 0.3, 0.1, 0.2, 8);
        failcheck=true;
        movecol(b, c, 0, S1, -20, -20, 1, 0.2, 8);
        failcheck=false;
        movestr(c, b, 100, 200, 25, 1, -0.3, -0.1, -0.2, 8);
        movecol(c, b, 100, S3, -30, 15, 1, -0.3, -0.1);
        motor[c]=0;
        motor[b]=0;
      }
      else
      {
        //take green 2 reverse no wall
        movestr(b, c, 0, 50, -30, 1, 0.3, 0.1, 0.2, 8);
        movestr(c, b, 100, 500, 25, 1, -0.3, -0.1, -0.2, 8);
        movestr(c, b, 50, 150, 25, 1, -0.3, -0.1, -0.2, 8);
        movestr(b, c, 0, 155, 30, 1, 0.3, 0.1, 0.2, 8);
        movestr(c, b, 100, 122, 15, 1, -0.3, -0.1, -0.2, 8);
        motor[b]=0;
        motor[c]=0;
        movestr(b, c, 0, 25, 30, 1, 0.3, 0.1, 0.2, 8);
        motor[b]=0;
        motor[c]=0;
        startTask(up);
        wait1Msec(600);
        movestr(b, c, 50, 145, -25, 1, 0.3, 0.1, 0.2, 8);
        movestr(b, c, 0, 250, -30, 1, 0.3, 0.1, 0.2, 8);
        failcheck=true;
        movecol(b, c, 0, S1, -20, -20, 1, 0.2, 8);
        failcheck=false;
        movestr(c, b, 100, 200, 25, 1, -0.3, -0.1, -0.2, 8);
        movecol(c, b, 100, S3, -30, 15, 1, -0.3, -0.1);
        motor[c]=0;
        motor[b]=0;
      }

    }
  }

  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //+MegaloElse++MegaloElse++MegaloElse++MegaloElse++MegaloElse++MegaloElse++MegaloElse++MegaloElse++MegaloElse++MegaloElse++MegaloElse+
  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

  else//1111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111//
  {
    lfport=S3; offset=30; Tp=35; kp=-0.27; kd=-29;
    resetMotorEncoder(c);
    while(getMotorEncoder(c)<100) lfpdf();
    LFstop(S2, -30);
    setMotorSpeed(b, 0);
    setMotorSpeed(c, 0);

    movestr(b, c, 0, 183, 30, 1, -0.4, -0.1, -0.2, 10);
    motor(c)=0;
    motor(b)=0;
    movestr(b, c, 100, 268, 40, 1, -0.4, -0.1, -0.2, 10);
    motor(c)=0;
    motor(b)=0;

    //stall
    stall();
    startTask(shake);
    if (cubenum[2]==2)//222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222//
    {
      //place red 2
      placeElement(1);


    }
    else//222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222//
    {
      //place red 4
      placeElementRev(1);


    }
    movestr(b, c, 0, 145, -30, 1, -0.4, -0.1, -0.2, 10);
    movestr(b, c, 50, 470, 50, 1, 0, 0, 0.2, 7);
    movecol(b, c, 50, S2, -40, 15, 1, -0.3, -0.1);
    startTask(up);
    lfport=S2; offset=30; Tp=25; kp=0.18; kd=20;
    resetMotorEncoder(c);
    while(getMotorEncoder(c)<100) lfpdf();
    motor[c]=0;
    motor[b]=0;
    Tp=50; kp=0.4; kd=37;
    LFstop(S3 , -20);

    //2PerneiPrasino22PerneiPrasino22PerneiPrasino22PerneiPrasino22PerneiPrasino22PerneiPrasino22PerneiPrasino22PerneiPrasino22PerneiPrasino2
    //2PerneiPrasino22PerneiPrasino22PerneiPrasino22PerneiPrasino22PerneiPrasino22PerneiPrasino22PerneiPrasino22PerneiPrasino22PerneiPrasino2
    //2PerneiPrasino22PerneiPrasino22PerneiPrasino22PerneiPrasino22PerneiPrasino22PerneiPrasino22PerneiPrasino22PerneiPrasino22PerneiPrasino2
  if (comb2!=1)//22222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222//
    {
      if (cubenum[3]<3)//33333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333//
      {
        //take green 1 normal wall
        movestr(b, c, 0, 185, 25, 1, -0.4, -0.1, -0.2, 7);
        movestr(c, b, 100, 260, 30, 1, -0.3, -0.1, -0.2, 10);

        motor(b)=0;
        motor(c)=0;
          startTask(down);
        wait1Msec(500);
        movestr(b, c, 0, 40, -20, 1, -0.4, -0.1, -0.2, 7);
        motor(b)=0;
        motor(c)=0;
        movestr(b, c, 50, 232, 20, 1, 0, 0, -0.2, 10);
        motor(b)=0;
        motor(c)=0;
        startTask(up);
        wait1Msec(400);
        movestr(b, c, 0, 30, -20, 1, -0.4, -0.1, -0.2, 7);
        movestr(c, b, 100, 230, 30, 1, -0.3, -0.1, -0.2, 8);
        movecol(c, b, 100, S3, -30, 15, 1, -0.3, -0.1);
        motor[c]=0;
        motor[b]=0;
      }

      else//33333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333//
      {
        //take green 1 reverse wall
        movestr(b, c, 0, 185, 25, 1, -0.4, -0.1, -0.2, 7);
        movestr(c, b, 100, 260, 30, 1, -0.3, -0.1, -0.2, 10);

        motor(b)=0;
        motor(c)=0;
        startTask(down);
        wait1Msec(500);
        movestr(b, c, 0, 30, -20, 1, -0.4, -0.1, -0.2, 7);
        motor(b)=0;
        motor(c)=0;
        movestr(b, c, 100, 120, 25, 1, -0.3, -0.1, -0.2, 10);
        movestr(b, c, 0, 50, 20, 1, -0.4, -0.1, -0.2, 7);
        movestr(c, b, 50, 172, 20, 1, -0.3, -0.1, -0.2, 10);
        motor(b)=0;
        motor(c)=0;
        startTask(up);
        wait1Msec(400);
        movestr(b, c, 0, 60, -25, 1, -0.4, -0.1, -0.2, 7);
        movestr(c, b, 100, 200, 25, 1, -0.3, -0.1, -0.2, 8);
        movecol(c, b, 100, S3, -30, 15, 1, -0.3, -0.1);
        motor[c]=0;
        motor[b]=0;
      }
    }
    else
    {
      if (cubenum[3]<3)
      {
        //take green 2 normal wall
        movestr(b, c, 0, 185, 25, 1, -0.4, -0.1, -0.2, 7);
        movestr(c, b, 100, 260, 30, 1, -0.3, -0.1, -0.2, 10);

        motor(b)=0;
        motor(c)=0;
        startTask(down);
        wait1Msec(500);
        movestr(b, c, 0, 315, 20, 1, -0.4, -0.1, -0.2, 7);
        motor(b)=0;
        motor(c)=0;
        movestr(b, c, 50, 248, 20, 1, 0, 0, -0.2, 10);
        motor(b)=0;
        motor(c)=0;
        startTask(up);
        wait1Msec(400);
        movestr(b, c, 50, 237, -20, 1, 0, 0, -0.2, 10);
        movestr(b, c, 0, 345, -20, 1, -0.4, -0.1, -0.2, 7);
        movestr(c, b, 100, 200, 30, 1, -0.3, -0.1, -0.2, 8);
        movecol(c, b, 100, S3, -30, 15, 1, -0.3, -0.1);
        motor[c]=0;
        motor[b]=0;
      }
      else
      {
        //take green 2 reverse wall
        movestr(b, c, 0, 178, 25, 1, -0.4, -0.1, -0.2, 7);
        movestr(c, b, 100, 260, 30, 1, -0.3, -0.1, -0.2, 10);

        motor(b)=0;
        motor(c)=0;
        startTask(down);
        wait1Msec(500);
        lfport=S3; offset=30; Tp=35; kp=-0.27; kd=-29;
        movecol(c, b, 0, S3, -20, 35, 1, -0.3, -0.1);
        movestr(b, c, 0, 15, 20, 1, -0.4, -0.1, -0.2, 7);
        motor[c]=0;
        move(b, 290, 12);
        motor[b]=0;
        move(c, 182, 12);
        startTask(up);
        wait1Msec(400);
        movestr(b, c, 0, 400, -30, 1, -0.4, -0.1, -0.2, 10);
        movestr(c, b, 100, 260, 30, 1, -0.3, -0.1, 0, 30);
        movecol(c, b, 100, S3, -30, 18, 1, -0.3, -0.1);
        motor(b)=0;
        motor(c)=0;
      }

    }
    //222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222//
  }

  //--------------------------------------------------------------------------------------------------------------------------------------
  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //**************************************************************************************************************************************



  lfport=S3; offset=30; Tp=35; kp=-0.27; kd=-29;
  resetMotorEncoder(c);
  while(getMotorEncoder(c)<100) lfpdf();
    motor[c]=0;
    motor[b]=0;
  Tp=70; kp=-0.57; kd=-53;
  for(int i=0; i<3; i++){
    resetMotorEncoder(c);
    while(getMotorEncoder(c)<100) lfpdf();
    LFstop(S2, -20);
  }
  motor(b)=0;
  motor(c)=0;


  //prasino
  //if
  if (cubenum[3]==1||cubenum[3]==3){
    movestr(b, c, 0, 163, 30, 1, -0.4, -0.1, -0.2, 10);
    motor[c]=0;
    motor[b]=0;

    movestr(b, c, 100, 265, 40, 1, -0.4, -0.1, -0.2, 10);
    motor[c]=0;
    motor[b]=0;
    stall();
    startTask(shake);
    if (cubenum[3]==1)
    {
      //place green 1
      placeElement(0);
    }
    else
    {
      //place green 3
      placeElementRev(0);
    }
    movestr(b, c, 0, 140, -30, 1, -0.4, -0.1, -0.2, 10);
    movestr(c, b, 50, 485, 35, 1, 0, 0, 0.2, 7);
    lfport=S3; offset=30; Tp=35; kp=-0.27; kd=-29;
    LFstop(S2, -20);
    movestr(b, c, 0, 110, 25, 1, 0.3, 0.1, 0.2, 8);
    movestr(b, c, 100, 255, 25, 1, 0.3, 0.1, 0.2, 8);

    motor[c]=0;
    motor[b]=0;


    lfport=S3; offset=30; Tp=35; kp=-0.27; kd=-29;
    LFstop(S2, -20);
    LFstop(S2, 40);
    LFstop(S2, -20);
    LFstop(S2, 40);
    movestr(b, c, 0, 175, 25, 1, 0.4, 0.1, 0.2, 8);
    failcheck=true;
    setMotorSync(b, c, 0, 30);
    while(SensorValue[S1]<40){}
    while(SensorValue[S1]>20){}
    while(SensorValue[S1]<40){}


    lfport=S1;offset=18;Tp=-16; kp=0.4; kd=45;
    resetMotorEncoder(b);
    resetMotorEncoder(c);
    while(getMotorEncoder(c)>-410)
    {lfpdf();}
    setMotorSpeed(b, 0);
    setMotorSpeed(c, 0);
    failcheck=false;
  }
  else{
    resetMotorEncoder(c);
    while(getMotorEncoder(c)<100) lfpdf();
    LFstop(S2, -20);
    movestr(b, c, 0, 150, 30, 1, -0.4, -0.1, -0.2, 10);
    motor[c]=0;
    motor[b]=0;

    movestr(b, c, 100, 210, 30, 1, -0.4, -0.1, -0.2, 10);
    movecol(b, c, 100, S3, -20, 30, 1, -0.2, 10);
    movecol(b, c, 100, S3, 40, 30, 1, -0.2, 10);
    motor[c]=0;
    motor[b]=0;
    lfport=S3; offset=30; Tp=35; kp=-0.27; kd=-29;
    LFstop(S2, -20);
    movestr(b, c, 0, 180, 25, 1, -0.3, -0.1, -0.2, 8);
    motor[b]=0;
    motor[c]=0;
    movestr(b, c, 100, 245, 25, 1, -0.3, -0.1, -0.2, 8);
    motor[b]=0;
    motor[c]=0;
    resetMotorEncoder(c);
    startTask(shake);
    //dialegei 2 h 4

    if (cubenum[3]==2)//222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222//
    {
      //place green 2
      offset=35; Tp=10; kp=-0.4; kd=-20;
      while(getMotorEncoder(c)<110)
      { lfpd2();}

      Tp=13; kp=-0.2; kd=-8;
      while(getMotorEncoder(c)<210)
      { lfpd2();
      }
      setMotorSpeed(b, 0);
      setMotorSpeed(c, 0);

      //to afinei, paei kato gia 95
      resetMotorEncoder(d);
      setMotorSpeed(d, 10);
      while(nMotorEncoder(d)<160){}
      setMotorSpeed(d, 0);

      //paei 243 mprosta
      while(getMotorEncoder(c)<285)
      { lfpd2();
      }
      setMotorSpeed(b, 0);
      setMotorSpeed(c, 0);

      //teleionei tin kinisi pou to afinei
      setMotorSpeed(d, 10);
      while(nMotorEncoder(d)<270){}
      setMotorSpeed(d, 0);

      //anoigokleinei
      resetMotorEncoder(d);
      setMotorSpeed(d, -30);
      while(nMotorEncoder(d)>-120){}
      setMotorSpeed(d, 0);

      setMotorSpeed(d, 30);
      while(nMotorEncoder(d)<=0){}
      setMotorSpeed(d, 0);
      movestr(b, c, 0, 90, -30, 1, -0.4, -0.1, -0.2, 10);
    }

    //dialegei 2 h 4
    else//222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222//
    {
      //place green 4
      offset=25; Tp=10; kp=-0.4; kd=-20;
      while(getMotorEncoder(c)<95)
      { lfpd2();
      }

      Tp=13; kp=-0.2; kd=-8;
      while(getMotorEncoder(c)<160)
      { lfpd2();
      }
      setMotorSpeed(b, 0);
      setMotorSpeed(c, 0);


      //paei 242 mprosta
      while(getMotorEncoder(c)<285)
      { lfpd2();
      }
      setMotorSpeed(b, 0);
      setMotorSpeed(c, 0);
      //to afinei, paei kato gia 85
      resetMotorEncoder(d);
      setMotorSpeed(d, 10);
      while(nMotorEncoder(d)<150){}
      setMotorSpeed(d, 0);

      //paei 60 piso
      movestr(c, b, 0, 53, -12, 1, 0.4, 0.1, 0.2, -2);
      motor(c)=0;
      motor(b)=0;

      //teleionei tin kinisi pou to afinei
      setMotorSpeed(d, 10);
      while(nMotorEncoder(d)<280){}
      setMotorSpeed(d, 0);

      //anoigokleinei
      resetMotorEncoder(d);
      setMotorSpeed(d, -30);
      while(nMotorEncoder(d)>-120){}
      setMotorSpeed(d, 0);

      setMotorSpeed(d, 30);
      while(nMotorEncoder(d)<=0){}
      setMotorSpeed(d, 0);
      movestr(b, c, 0, 90, -30, 1, -0.4, -0.1, -0.2, 10);
    }

    movestr(c, b, 100, 255, 25, 1, 0.3, 0.1, 0.2, 8);

    setMotorSpeed(b, 0);
    setMotorSpeed(c, 0);
    waitUntilMotorStop(c);
    waitUntilMotorStop(b);

    movecol(b, c, 0, S2, 40, 20, 1, 0.4, 0.1);
    movecol(b, c, 0, S2, -20, 20, 1, 0.4, 0.1);
    movecol(b, c, 0, S2, 40, 20, 1, 0.4, 0.1);

    movestr(b, c, 0, 180, 25, 1, 0.4, 0.1, 0.2, 8);

    setMotorSpeed(b, 0);
    setMotorSpeed(c, 0);
    failcheck=true;
    movecol(c, b, 100, S1, 40, 20, 1, 0.3, 0.1);
    movecol(c, b, 100, S1, -20, 20, 1, 0.3, 0.1);
    movecol(c, b, 100, S1, 40, 20, 1, 0.3, 0.1);

    motor[b]=0;
    motor[c]=0;

    lfport=S1;offset=18;Tp=-16; kp=0.4; kd=45;
    resetMotorEncoder(b);
    resetMotorEncoder(c);
    while(getMotorEncoder(c)>-185)
    {lfpdf();}
    setMotorSpeed(b, 0);
    setMotorSpeed(c, 0);
  }

  resetMotorEncoder(c);
  setMotorSpeed(c, -30);
  while(getMotorEncoder(c)>-115){}
  setMotorSpeed(c, 0);

  resetMotorEncoder(b);
  setMotorSpeed(b, 0);
  setMotorSpeed(b, -30);
  while(getMotorEncoder(b)<115){}
  setMotorSpeed(b, 0);

  startTask(downoi);
  clearTimer(2);
  lfport=S1; offset=25; Tp=-15; kp=0.15; kd=4.2;
  while(colorSensor.color!=6&&time1[2]<=1000)
  {
    readSensor(&colorSensor);
    lfpdf();
  }
  setMotorSpeed(b, 0);
  setMotorSpeed(c, 0);
  downoiextra();

  startTask(up);
  failcheck=false;
  movestr(b, c, 0, 550, 45, 1, 0.4, 0.1, 0.2, 10);
  movestr(b, c, 100, 260, 30, 1, -0.3, -0.1, -0.2, 10);
  movestr(b, c, 0, 490, 50, 0, 0, 00, 0, 0);
  movecol(b, c, 0, S2, -20, 30, 0, 0, 0);

  movestr(b, c, 0, 120, 25, 1, -0.4, -0.1, -0.2, 6);

  movestr(b, c, 100, 255, 30, 1, -0.3, -0.1, -0.2, 10);
  setMotorSpeed(b, 0);
  setMotorSpeed(c, 0);

  lfport=S3; offset=32; Tp=25; kp=-0.18; kd=-20;
  resetMotorEncoder(c);
  while(getMotorEncoder(c)<150)
  {lfpdf();}

  lfport=S3; offset=32; Tp=70; kp=-0.70; kd=-47;
  while(getMotorEncoder(c)<1600)
  {lfpdf();}
  motor(b)=0;
  motor(c)=0;

  LFstop(S2, -17);
  motor(b)=0;
  motor(c)=0;
  startTask(lastupoi);

  //if

  lfport=S3; offset=30; Tp=25; kp=-0.18; kd=-20;

  if (comb2!=3)//22222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222//
  {
    if (cubenum[0]<3)//33333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333//
    {
      //take yellow 1 normal
      movestr(b, c, 0, 185, 25, 1, -0.4, -0.1, -0.2, 7);
      movestr(b, c, 100, 215, 30, 0, -0.3, -0.1, 0, 0);
      startTask(down);
      movestr(b, c, 100, 45, 30, 1, -0.3, -0.1, -0.2, 10);
      motor(b)=0;
      motor(c)=0;
      movestr(b, c, 0, 30, -20, 1, -0.4, -0.1, -0.2, 7);
      motor(b)=0;
      motor(c)=0;
      movestr(c, b, 50, 195, 30, 1, 0, 0, -0.2, 10);
      movestr(c, b, 50, 20, 15, 1, 0, 0, -0.2, 10);
      movestr(b, c, 0, 7, 15, 1, 0, 0, -0.2, 10);
      motor(b)=0;
      motor(c)=0;
      startTask(up);
      wait1Msec(700);
      movestr(b, c, 0, 115, -25, 1, -0.4, -0.1, -0.2, 7);
      motor(b)=0;
      motor(c)=0;
      movestr(b, c, 100, 297, 30, 1, -0.3, -0.1, -0.2, 10);
      movecol(b, c, 100, S2, -20, 15, 1, -0.3, -0.1);
    }

    else//33333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333//
    {
      //take yellow 1 reverse
      startTask(down);
      movestr(b, c, 0, 70, 25, 1, -0.4, -0.1, -0.2, 7);
      movestr(b, c, 50, 410, 25, 1, -0.3, -0.1, -0.2, 10);
      motor(b)=0;
      motor(c)=0;
      startTask(up);
      wait1Msec(700);
      movestr(b, c, 0, 125, -25, 1, -0.4, -0.1, -0.2, 7);
      movestr(b, c, 100, 220, 25, 1, -0.3, -0.1, -0.2, 8);
      movecol(b, c, 100, S2, -18, 15, 1, -0.3, -0.1);
      motor[c]=0;
      motor[b]=0;
    }
  }
  else
  {
    if (cubenum[0]<3)
    {
      //take yellow 2 normal
      movestr(b, c, 0, 185, 25, 1, -0.4, -0.1, -0.2, 7);
      movestr(b, c, 100, 260, 30, 1, -0.3, -0.1, -0.2, 10);
      motor(b)=0;
      motor(c)=0;
      startTask(down);
      movestr(b, c, 0, 315, 20, 1, -0.4, -0.1, -0.2, 7);
      motor(b)=0;
      motor(c)=0;
      movestr(c, b, 50, 215, 20, 1, 0, 0, -0.2, 10);
      motor(b)=0;
      motor(c)=0;
      startTask(up);
      wait1Msec(400);
      movestr(c, b, 50, 225, -20, 1, 0, 0, -0.2, 10);
      movestr(b, c, 0, 350, -20, 1, -0.4, -0.1, -0.2, 7);
      movestr(b, c, 100, 200, 30, 1, -0.3, -0.1, -0.2, 8);
      movecol(b, c, 100, S2, -30, 15, 1, -0.3, -0.1);
      motor[c]=0;
      motor[b]=0;
    }
    else
    {
      //take yellow 2 reverse
      movestr(b, c, 0, 125, 25, 1, -0.4, -0.1, -0.2, 7);
      startTask(down);
      movestr(b, c, 100, 265, 30, 1, -0.3, -0.1, -0.2, 10);
      motor(b)=0;
      motor(c)=0;
      LFstop(S2, -20);
      LFstop(S2, 40);
      move(c, 290, 20);
      move(b, 187, 20);
      startTask(up);
      wait1Msec(400);
      movestr(b, c, 0, 440, -30, 1, -0.4, -0.1, -0.2, 10);
      movestr(b, c, 100, 260, 30, 1, -0.3, -0.1, 0, 30);
      movecol(b, c, 100, S2, -30, 18, 1, -0.3, -0.1);
      motor(b)=0;
      motor(c)=0;
    }

  }


  lfport=S2; offset=30; Tp=35; kp=0.27; kd=29;
  resetMotorEncoder(c);
  while(getMotorEncoder(c)<100) lfpdf();

  lfport=S2; offset=32; Tp=70; kp=0.70; kd=47;
  resetMotorEncoder(c);

  LFstop(S3, -30);
  resetMotorEncoder(c);
  while(getMotorEncoder(c)<70) lfpdf();
  LFstop(S3, -30);
  resetMotorEncoder(c);
  while(getMotorEncoder(c)<70) lfpdf();



  //if
  if(cubenum[0]==2||cubenum[0]==4){
    lfport=S2; offset=30; Tp=35; kp=0.27; kd=29;
    LFstop(S3, -30);


    movestr(b, c, 0, 183, 30, 1, -0.4, -0.1, -0.2, 10);
    motor(c)=0;
    motor(b)=0;

    movestr(c, b, 100, 268, 40, 1, -0.4, -0.1, -0.2, 10);
    motor(c)=0;
    motor(b)=0;

    //stall
    stall();
    startTask(shake);
    if (cubenum[0]<3){
      //place yellow 2
      placeElement(1);
    }
    else
    {
      //place yellow 4
      placeElementRev(1);
    }

    movestr(b, c, 0, 50, -20, 1, 0.3, 0.1, 0.2, 6);
    startTask(up);
    movestr(b, c, 100, 200, 25, 1, 0.3, 0.1, 0.2, 8);
    movecol(b, c, 100, S3, -15, 30, 1, -0.3, 0.1);
    motor[b]=0;
    motor[c]=0;


    lfport=S3; offset=16; Tp=50; kp=-0.4; kd=-37;
    LFstop(S2, -20);
    startTask(up);
    movestr(b, c, 0, 330, 40, 0, 0.3, 0.1, 0.2, 6);

  }
  else
  {
    LFstop(S3, -30);
    resetMotorEncoder(c);
    while(getMotorEncoder(c)<70) lfpdf();
    lfport=S2; offset=30; Tp=35; kp=0.27; kd=29;
    LFstop(S3, -30);

    movestr(b, c, 0, 150, 30, 1, -0.4, -0.1, -0.2, 10);
    motor[c]=0;
    motor[b]=0;

    movestr(c, b, 100, 210, 30, 1, -0.4, -0.1, -0.2, 10);
    movecol(c, b, 100, S2, -20, 30, 1, -0.3, 0.1);
    movecol(c, b, 100, S2, 40, 30, 1, -0.3, 0.1);
    motor[c]=0;
    motor[b]=0;
    lfport=S2; offset=30; Tp=35; kp=0.27; kd=29;
    LFstop(S3, -20);
    movestr(b, c, 0, 195, 25, 1, -0.3, -0.1, -0.2, 8);
    motor[b]=0;
    motor[c]=0;
    movestr(c, b, 100, 250, 25, 1, -0.3, -0.1, -0.2, 8);
    motor[b]=0;
    motor[c]=0;
    resetMotorEncoder(c);
    startTask(shake);

    if (cubenum[0]==1)//222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222//
    {
      //place yellow 1
      offset=-30; Tp=11; kp=-0.42; kd=-21;
      while(getMotorEncoder(c)<110)
      { lfpd2();}

      Tp=13; kp=-0.2; kd=-8;
      while(getMotorEncoder(c)<215)
      { lfpd2();
      }
      setMotorSpeed(b, 0);
      setMotorSpeed(c, 0);

      //to afinei, paei kato gia 95
      resetMotorEncoder(d);
      setMotorSpeed(d, 10);
      while(nMotorEncoder(d)<160){}
      setMotorSpeed(d, 0);

      //paei 243 mprosta
      while(getMotorEncoder(c)<295)
      { lfpd2();
      }
      setMotorSpeed(b, 0);
      setMotorSpeed(c, 0);

      //teleionei tin kinisi pou to afinei
      setMotorSpeed(d, 10);
      while(nMotorEncoder(d)<280){}
      setMotorSpeed(d, 0);

      //anoigokleinei
      resetMotorEncoder(d);
      setMotorSpeed(d, -30);
      while(nMotorEncoder(d)>-120){}
      setMotorSpeed(d, 0);

      setMotorSpeed(d, 30);
      while(nMotorEncoder(d)<=0){}
      setMotorSpeed(d, 0);
      movestr(b, c, 0, 90, -30, 1, -0.4, -0.1, -0.2, 10);
    }

    //dialegei 2 h 4
    else//222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222//
    {
      //place yellow 3
      offset=-25; Tp=11; kp=-0.42; kd=-21;
      while(getMotorEncoder(c)<95)
      { lfpd2();
      }

      Tp=14; kp=-0.2; kd=-8;
      while(getMotorEncoder(c)<160)
      { lfpd2();
      }
      setMotorSpeed(b, 0);
      setMotorSpeed(c, 0);


      //paei 242 mprosta
      while(getMotorEncoder(c)<308)
      { lfpd2();
      }
      setMotorSpeed(b, 0);
      setMotorSpeed(c, 0);
      //to afinei, paei kato gia 85
      resetMotorEncoder(d);
      setMotorSpeed(d, 10);
      while(nMotorEncoder(d)<150){}
      setMotorSpeed(d, 0);

      //paei 60 piso
      movestr(c, b, 0, 53, -12, 1, 0.4, 0.1, 0.2, -2);
      motor(c)=0;
      motor(b)=0;

      //teleionei tin kinisi pou to afinei
      setMotorSpeed(d, 10);
      while(nMotorEncoder(d)<280){}
      setMotorSpeed(d, 0);

      //anoigokleinei
      resetMotorEncoder(d);
      setMotorSpeed(d, -30);
      while(nMotorEncoder(d)>-120){}
      setMotorSpeed(d, 0);

      setMotorSpeed(d, 30);
      while(nMotorEncoder(d)<=0){}
      setMotorSpeed(d, 0);
      movestr(b, c, 0, 90, -30, 1, -0.4, -0.1, -0.2, 10);
    }
      startTask(upfull);
      failcheck=true;
    movecol(b, c, 0, S1, 45, -80, 0, 0, 0);
    movestr(b, c, 0, 18, -40, 0, 0, 0, 0, 0);
    motor[b]=0;
    motor[c]=0;
    failcheck=false;
    movestr(c, b, 30.2, 975, -60, 0, 0, 0, 0, 0);
    movestr(b, c, 0, 200, -50, 1, -0.4, -0.1, -0.2, 10);
    motor[b]=0;
    motor[c]=0;

  }
end=true;
}

task failsafe(){
  while(true){
    if(SensorRaw[S1]==0){
    motor[b]=0;
    motor[c]=0;
    suspendTask(basic);
    waitUntil(SensorRaw[S1]!=0);
    resumeTask(basic);
    }
  }
}

task main(){
  if(nImmediateBatteryLevel<7850)
  {
  for(int i=0; i<3; i++)
  {
    playTone(700, 10);
    wait1Msec(180);
    playTone(700, 10);
    wait1Msec(800);
  }
  }
  startTask(basic);
  while(end==false){
  waitUntil(failcheck==true||end==true);
  startTask(failsafe);
  waitUntil(failcheck==false||end==true);
  stopTask(failsafe);
  }

}
