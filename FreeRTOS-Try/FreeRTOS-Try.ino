//Note
//Pin 4 = Direction control for Motor 2
//Pin 5 = PWM control for Motor 2
//Pin 6 = PWM control for Motor 1
//Pin 7 = Direction control for Motor 1

#include <Arduino_FreeRTOS.h>
#include <Wire.h>
#include <LSM303.h>
#include<math.h>
#include <NewPing.h>

// end of including
#define MAX_DISTANCE 25
#define DISTANCE_BETWEEN_WHEEL 16.0
#define PROPORTION 2
#define INTEGTAL 0 //0.1
#define DERIVATIVE 0.05 //0.05
#define interruptPin1  3
#define interruptPin2  2
#define TARGETSPEED 25
#define RATIODEG 0.190782
#define WHEEL_DIAMETER 6.46

//end of define

typedef struct PID { 
double SetPoint; // 设定目标Desired value 
double Proportion; // 比例常数Proportional Const 
double Integral; // 积分常数Integral Const 
double Derivative; // 微分常数Derivative Const 
double LastError; // Error[-1] 
double PrevError; // Error[-2] 
double SumError; // Sums of Errors 
} PID;

//end of struct

LSM303 compass;
NewPing frontsonar(12, 11, MAX_DISTANCE);
NewPing frontleftsonar(13, 8, MAX_DISTANCE);
NewPing frontrightsonar(10, 9, MAX_DISTANCE);

volatile unsigned long wheel1turn=1;
volatile unsigned long wheel2turn=1;
volatile unsigned long wheel1turn_new=0;
volatile unsigned long wheel2turn_new=0;
unsigned long oldwheel1=0;
unsigned long oldwheel2=0;
int speed_1=0,speed_2=0;
int distance_front=0;
int distance_frontleft=0;
int distance_frontright=0;
int state=0;//0 stop 1 going 2 stoped
int targetspeed=0;
float targetangle;
double power1;
double power2;
float ang_calc_init;
float ang_calc;
double x,y;
double targetx,targety;
double northat;
double deg;
int diff;
int finished;
//unsigned long count=0;
//unsigned long lastreadcomtime=0;

TaskHandle_t pidHandle = NULL;


//end of defining global variable

void Task_sonar( void *pvParameters );
void Task_calcspeed( void *pvParameters );
void Task_main( void *pvParameters );
void Task_pid( void *pvParameters );
void Task_calcang(void *pvParameters);
void Task_getserial3(void *pvParameters);
void Task_getserial(void *pvParameters);
void Task_pid(void *pvParameters);

void mydelay(int millis);
//end of defining functions

double calcang_raw(double x1,double x2,double y1,double y2){
  double tmp_rad,tmp_deg;
  //char tmp[200];
  if(abs(x1-x2)<0.0001 && abs(y1-y2)<0.0001)
    return 0;
  if(abs(y1-y2)<0.001){
    if(x2-x1>0){
      return 90;
    }else{
      return 270;
    }
  }else{
    //sprintf(tmp,"x1:%d y1:%d x2:%d y2:%d\n",(int)x1,(int)y1,(int)x2,(int)y2);
    //Serial.write(tmp);
    tmp_rad=atan((x2-x1)/(y2-y1));
    tmp_deg=tmp_rad*180/3.14159;
    if(tmp_deg<0){
      if(y2-y1>0){
        tmp_deg=tmp_deg+360;
      }else{
        tmp_deg=180+tmp_deg;
      }
    }else{
      if(y2-y1<0){
        tmp_deg=tmp_deg+180;
      }
    }
  }
  return tmp_deg;
}


double calcang(double x1,double x2,double y1,double y2,double arg_northat){
  double rawang;
  rawang=calcang_raw(x1,x2,y1,y2);
  
  rawang=rawang+360-arg_northat;
  if(rawang>=360){
    rawang=rawang-360;
  }
  return rawang;
}

float get_compass_ang(){

  compass.read();
  return(compass.heading());
}

float get_calc_ang(){
  return(ang_calc);
}

void setInitAng(double ang){
  ang_calc_init=ang;
  wheel1turn=0;
  wheel2turn=0;
}

void setNorthAt(double ang){
  northat=ang;
}

void setXY(double arg_x,double arg_y){
  x=arg_x;
  y=arg_y;
}

double get_real_ang(double ang){
  ang=ang+360-northat;
  if(ang>=360){
    ang=ang-360;
  }
  return ang;
}

double get_virtul_ang(double ang){
  ang=ang-360+northat;
  if(ang>=360){
    ang-=360;
  }else if(ang<0){
    ang+=360;
  }
  return ang;
}

double PIDCalc( PID *pp, double NextPoint ) 
{
    double dError, Error; 
    Error = pp->SetPoint - NextPoint; // 偏差 
    pp->SumError += Error; // 积分 
    dError = pp->LastError - pp->PrevError; // 当前微分 
    pp->PrevError = pp->LastError; 
    pp->LastError = Error; 
    return (pp->Proportion * Error // 比例项 
      + pp->Integral * pp->SumError // 积分项 
      + pp->Derivative * dError );// 微分项 
}

void PIDInit (PID *pp) 
{
  memset ( pp,0,sizeof(PID)); 
}

void inline mydelay(int millis){
  TickType_t xDelay = millis / portTICK_PERIOD_MS;
  vTaskDelay( xDelay );
}

void wheel1(){
  wheel1turn+=1;
  wheel1turn_new+=1;
  if(wheel1turn%500==0){
    wheel1turn+=1;
    wheel1turn_new+=1;
  }
}

void wheel2(){
  wheel2turn+=1;
  wheel2turn_new+=1;
}

void wheel1control(int arg){
  digitalWrite(4,1);
  if(arg>255){
     arg=255;
  }else if(arg<0){
     arg=0;
  }
  analogWrite(5, arg);
}

void wheel2control(int arg){
  digitalWrite(7,1);
  if(arg>255){
    arg=255;
  }else if(arg<0){
    arg=0;
  }
  analogWrite(6, arg);
}

void stop(){
  analogWrite(5, 0);
  analogWrite(6, 0);
}

int getdiff(int ang1,int ang2){
  int diff;
  diff=ang1-ang2;
  if(diff>180){
    return 360-diff;
  }else if(diff<-180){
    return 360+diff;
  }else if(diff<0){
    return 0-diff;
  }else{
    return diff;
  }
}

int getdiff2(int ang1,int ang2){
  int diff;
  diff=ang2-ang1;
  if(diff>180){
    return diff-360;
  }else if(diff<-180){
    return 360+diff;
  }else{
    return diff;
  }
}

void turn(int left,int angle){
  //char tmp[100];
  int nowang;
  int dstang;
  int oldang;

  nowang=get_calc_ang();
  oldang=nowang;

  if(left){
    dstang=nowang-angle;
  }else{
    dstang=nowang+angle;
  }
  if(dstang<0){
    dstang=360+dstang;
  }
  if(dstang>=360){
    dstang=dstang-360;
  }
  //sprintf(tmp,"#turn %d  %d degree,now ang %d,dstang %d\n",left,angle,nowang,dstang);
  //Serial.write(tmp);
  
  while(getdiff(nowang,oldang)<=angle-5){
    if(angle-getdiff(nowang,oldang)>20){
      if(left==0){
        digitalWrite(4,1);
        analogWrite(5, 255);
      }else{
        digitalWrite(7,1);
        analogWrite(6, 255);
      }
    }else{
      if(left==0){
        digitalWrite(4,1);
        analogWrite(5, 255);
      }else{
        digitalWrite(7,1);
        analogWrite(6, 255);
      }
      
    }
    mydelay(1);
    //nowang=(int)get_compass_ang();
    nowang=(int)get_calc_ang();
    //nowang=ang_calc;
    //sprintf(tmp,"now ang %d\n",nowang);
    //Serial.write(tmp);
  }
  analogWrite(5, 0);
  analogWrite(6, 0);
  //Serial.write("@t");
}

int adjusttoang(int dstang){
  int angtoturn;
  int nowang;
  int left;

  //nowang=get_compass_ang();
  nowang=get_calc_ang();
  
  angtoturn=dstang-nowang;
  if(angtoturn>180){
    left=1;
  }else if(angtoturn<-180){
    left=0;
  }else if(angtoturn<0){
    left=1;
  }else{
    left=0;
  }
  angtoturn=getdiff(nowang,dstang);
  if(getdiff(nowang,dstang)>5){
    turn(left,angtoturn);
    return 1;
  }else{
    return 0;
  }
}

void setup() {

  Serial.begin(9600);
  Serial3.begin(9600);
  for (int i = 4; i <= 7; i++) //Pin 4 to 7 are used
    pinMode(i, OUTPUT);
  pinMode(interruptPin1, INPUT_PULLUP);
  pinMode(interruptPin2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin1), wheel1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptPin2), wheel2, CHANGE);
  Wire.begin();
  compass.init();
  compass.enableDefault();
  compass.m_min = (LSM303::vector<int16_t>){-1954, -2714, -2718};
  compass.m_max = (LSM303::vector<int16_t>){+2202, +1326, +1626};

  targetspeed=TARGETSPEED;
  targetangle=0;

  setInitAng(0);
  setNorthAt(0);
  setXY(0.0,0.0);
  targetx=0;
  targety=1000;
  state=0;
  
  xTaskCreate(Task_main,(const portCHAR *) "main" ,256 ,NULL, 1,NULL);
  //xTaskCreate(Task_sonar,(const portCHAR *) "sonar" ,128 ,NULL, 1,NULL);
  //xTaskCreate(Task_calcspeed,(const portCHAR *) "speed" ,128 ,NULL ,1,NULL);
  xTaskCreate(Task_getserial,(const portCHAR *) "serial" ,256 ,NULL ,1,NULL);

  xTaskCreate(Task_calcang,(const portCHAR *) "angle" ,150 ,NULL ,2,NULL);
  xTaskCreate(Task_getserial,(const portCHAR *) "serial" ,128 ,NULL ,1,NULL);
  xTaskCreate(Task_getserial3,(const portCHAR *) "serial3" ,300 ,NULL ,1,NULL);
  xTaskCreate(Task_pid,(const portCHAR *) "pid" ,200 ,NULL ,3,&pidHandle);
  
  //wheel1control ( 127);
 
}

void loop(){}

void Task_getserial(void *pvParameters){
  while(1){
  if (Serial.available()) {
    char input = Serial.read();
    if(input=='w'){
      if(state==0){
        state=1;
        
      }
    }else if(input=='s'){
      if(state==1){
        if(state==1){
          finished=0;
          state=0;
          while(!finished){
            mydelay(1);
          }
          //vTaskDelete(pidHandle);
          stop();
        }
      }
    }else if(input=='a'){
      state=0;
      stop();
      turn(1,90);
      
    }else if(input=='d'){
      state=0;
      stop();
      turn(0,90);
    }
    //Serial.write(input);
  }
  mydelay(100);
  }
}

void Task_getserial3(void *pvParameters){
  char cmd[20]={0};
  char tmp[20];
  int i;
  int tmpang;
  while(1){
    if (Serial3.available()) {
      char input = Serial3.read();
      if(input=='@'){
        while(!Serial3.available()){
        }
        input = Serial3.read();
        i=0;
        while(input!='@' && i<15){
          cmd[i++]=input;
          while(!Serial3.available()){
          }
          input = Serial3.read();
        }
        cmd[i]=0;
      }
      sprintf(tmp,"get cmd %s\n",cmd);
      Serial.write(tmp);

      if(strncmp(cmd,"getr",4)==0){
        sprintf(tmp,"@a%d@",(int)get_compass_ang());
        Serial3.write(tmp);
      }else if(strncmp(cmd,"getv",4)==0){
        sprintf(tmp,"@a%d@",(int)ang_calc);
        Serial3.write(tmp);
      }else if(strncmp(cmd,"seta",4)==0){
        sscanf(&cmd[4],"%d",&tmpang);
        setInitAng(tmpang);
        Serial3.write("@void@");
      }else if(strncmp(cmd,"setn",4)==0){
        sscanf(&cmd[4],"%d",&tmpang);
        setNorthAt(tmpang);
        Serial3.write("@void@");
      }else if(strncmp(cmd,"turl",4)==0){
        sscanf(&cmd[4],"%d",&tmpang);
        turn(1,tmpang);
        Serial3.write("@tufi");//turn finished
      }else if(strncmp(cmd,"turr",4)==0){
        sscanf(&cmd[4],"%d",&tmpang);
        turn(0,tmpang);
        Serial3.write("@tufi@");//turn finished
      }else if(strncmp(cmd,"adja",4)==0){
        sscanf(&cmd[4],"%d",&tmpang);
        adjusttoang(tmpang);
        Serial3.write("@adfi@");//turn finished
      }else if(strncmp(cmd,"getx",4)==0){
        
        sprintf(tmp,"@x%d@",(int)x);
        Serial3.write(tmp);
      }else if(strncmp(cmd,"gety",4)==0){

        sprintf(tmp,"@y%d@",(int)y);
        Serial3.write(tmp);
      }else if(strncmp(cmd,"setx",4)==0){
        sscanf(&cmd[4],"%d",&tmpang);
        x=tmpang;
        Serial3.write("@void@");
      }else if(strncmp(cmd,"sety",4)==0){
        sscanf(&cmd[4],"%d",&tmpang);
        y=tmpang;
        Serial3.write("@void@");
      }else if(strncmp(cmd,"go",2)==0){
        if(state==0){
          state=1;
        }
        Serial3.write("@void@");
      }else if(strncmp(cmd,"stop",4)==0){
        if(state==1){
          finished=0;
          state=0;
          while(!finished){
            mydelay(1);
          }
          //vTaskDelete(pidHandle);
          stop();
        }
        Serial3.write("@void@");
      }else if(strncmp(cmd,"tarx",4)==0){
        sscanf(&cmd[4],"%d",&tmpang);
        targetx=tmpang;
        Serial3.write("@void@");
      }else if(strncmp(cmd,"tary",4)==0){
        sscanf(&cmd[4],"%d",&tmpang);
        targety=tmpang;
        Serial3.write("@void@");
      }
    }
    mydelay(50);
  }
}

void Task_main(void *pvParameters){
  char tmp[100];
  while(1){
        sprintf(tmp,"speed1 %d speed2 %d h_real %d h_calc %d diff %d turn1 %d turn2 %d x:%d y:%d\n",(int)speed_1,(int)speed_2,(int)get_compass_ang(),(int)(ang_calc),diff,(int)(wheel1turn/48.0/8*3.14159*6.5),(int)(wheel2turn/48.0/8*3.14159*6.5),(int)(x),(int)(y));
        //sprintf(tmp,"turn1 %ld turn2 %ld\n",wheel1turn,wheel2turn);
        Serial.write(tmp);
        mydelay(200);
  }
}
void Task_pid(void *pvParameters){
    PID sPID_1; // PID Control Structure 
    PID sPID_2; // PID Control Structure 
    PID sPID_3; // PID Control Structure 
    double rIn_1; // PID Feedback (Input) 
    double rIn_2; // PID Feedback (Input) 
    double rIn_3; // PID Feedback (Input) 
    double rOut_1; // PID Response (Output)
    double rOut_2; // PID Response (Output)
    double rOut_3; // PID Response (Output)
    unsigned long wheel1=0, wheel2=0;
    int tmp=0;

    PIDInit ( &sPID_1 ); // Initialize Structure 
    PIDInit ( &sPID_2 ); // Initialize Structure 
    PIDInit ( &sPID_3 ); // Initialize Structure 
    sPID_1.Proportion = PROPORTION; // Set PID Coefficients 
    sPID_1.Integral = INTEGTAL; 
    sPID_1.Derivative =DERIVATIVE; 
    

    sPID_2.Proportion = PROPORTION; // Set PID Coefficients 
    sPID_2.Integral =INTEGTAL; 
    sPID_2.Derivative =DERIVATIVE; 

    sPID_3.Proportion = 0.08; // Set PID Coefficients 
    sPID_3.Integral =0; 
    sPID_3.Derivative =0.01; 
    //sPID_3.Derivative =0; 
    sPID_3.SetPoint = 0;
    wheel1turn_new=0;
    wheel2turn_new=0;
    for (;;)
    { // Mock Up of PID Processing 
          speed_1=wheel1turn_new-wheel1;
          speed_2=wheel2turn_new-wheel2;
          wheel1+=speed_1;
          wheel2+=speed_2;
        if(state==1){
          sPID_1.SetPoint = targetspeed; // Set PID Setpoint 
          sPID_2.SetPoint = targetspeed; // Set PID Setpoint 
          
          rIn_1 = speed_1; // Read Input 
          rIn_2 = speed_2; // Read Input 


          targetangle=calcang(x,targetx,y,targety,northat);
          
          rIn_3 = getdiff2(targetangle,get_calc_ang());
          rIn_3=rIn_3/RATIODEG;//transform the angle to the turn
          
          //rIn_3 = ((long)wheel1turn_new-(long)wheel2turn_new); // Read Input 
          
          rOut_1 = PIDCalc ( &sPID_1,rIn_1 ); // Perform PID Interation 
          rOut_2 = PIDCalc ( &sPID_2,rIn_2 ); // Perform PID Interation 
          rOut_3 = PIDCalc ( &sPID_3,rIn_3 ); // Perform PID Interation 
          
          /*if(rOut_3>20){
            rOut_3=20;
          }else if(rOut_3<-20){
            rOut_3=-20;
          }*/
          //rOut_3=0;
          
          power1+=rOut_1+rOut_3;
          power2+=rOut_2-rOut_3;

          if(tmp==0){
            wheel1control ( power1); // Effect Needed Changes 
            wheel2control ( power2); // Effect Needed Changes 
          tmp=1;
          }else{
            wheel2control ( power2); // Effect Needed Changes 
            wheel1control ( power1); // Effect Needed Changes 
            tmp=0;
          }
        }else{
          wheel1turn_new=0;
          wheel2turn_new=0;
          wheel1=0;
          wheel2=0;
          finished=1;
        }
        mydelay(100);
    }
}



void Task_sonar(void *pvParameters)  // This is a task.
{
  int tmp1,tmp2;
  while(1){
    while(1){
      tmp1=frontsonar.ping_cm();
      mydelay(10);
      tmp2=frontsonar.ping_cm();
      if(abs(tmp1-tmp2)<5){
        distance_front=tmp1;
        break;
      }
    }
    mydelay(30);
    while(1){
      tmp1=frontrightsonar.ping_cm();
      delay(10);
      tmp2=frontrightsonar.ping_cm();
      if(abs(tmp1-tmp2)<5){
        distance_frontright=tmp1;
        break;
      }
    }
    mydelay(30);
    while(1){
      tmp1=frontleftsonar.ping_cm();
      delay(10);
      tmp2=frontleftsonar.ping_cm();
      if(abs(tmp1-tmp2)<5){
        distance_frontleft=tmp1;
        break;
      }
    }
    mydelay(30);
  }
}

void Task_calcspeed(void *pvParameters)  // Another task
{
  
  while(1){
    compass.read();
    mydelay(10);
  }
}

void Task_calcang(void *pvParameters){
  unsigned long wheel1=0, wheel2=0;
  int wheel1go=0, wheel2go=0;
  double tmpdistance;
  
  while(1){
    wheel1go=wheel1turn-wheel1;
    wheel2go=wheel2turn-wheel2;
    wheel1+=wheel1go;
    wheel2+=wheel2go;

    diff=((long)wheel1turn-(long)wheel2turn);
    
    //deg=360.0*(diff*6.5/384)/DISTANCE_BETWEEN_WHEEL;
    deg=diff*RATIODEG;
    while(deg>=360){
      deg-=360;
    }
    while(deg<0){
      deg+=360;
    }
    ang_calc=ang_calc_init+deg;
    while(ang_calc>=360){
      ang_calc-=360;
    }
    while(ang_calc<0){
      ang_calc+=360;
    }
    deg=get_virtul_ang(ang_calc);
    tmpdistance=(wheel1go+wheel2go)/2*3.14159*WHEEL_DIAMETER/48.0/8.0;
    deg=deg*3.14159/180;
    x=x+sin(deg)*tmpdistance;
    y=y+cos(deg)*tmpdistance;
    mydelay(100);
  }
}

