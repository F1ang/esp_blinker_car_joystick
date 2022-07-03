
#define BLINKER_WIFI
#include <Blinker.h>
//#include <Servo.h>           //暂定 bug 
char auth[] = "ab6399324fd4";
char ssid[] = "F1ang";
char pswd[] = "201913018";

// 新建组件对象(按钮和摇杆组件)
BlinkerJoystick Joystick1("joy-g86");//平移/左手油门   
BlinkerJoystick Joystick2("joy-ygs");//forward,back,left,right 
BlinkerButton Button1("btn-n33"); //伪小陀螺
BlinkerNumber Number1("num-abc");
void Joystick1_callback(uint8_t xAxis, uint8_t yAxis);//摇杆回调
void Joystick2_callback(uint8_t xAxis, uint8_t yAxis);//油门摇杆
void Button1_callback(const String & state);//按钮回调
void dataRead(const String & data);
//function
//Servo server1;  
uint8_t gyroscope=0;
int counter = 0;
uint8_t X1;
uint8_t Y1;
uint8_t X2;
uint8_t Y2;
void setup() {
  //初始化串口，并开启调试信息
  Serial.begin(115200);
  Serial.println("Welcome to Romantic Diy F1ang...");
  BLINKER_DEBUG.stream(Serial);
//  BLINKER_DEBUG.debugAll();
  //初始化blinker
   Blinker.begin(auth, ssid, pswd);
   //Blinker.attachData(dataRead);
  //组件回调
  Button1.attach(Button1_callback);
  Joystick1.attach(Joystick1_callback);
  Joystick2.attach(Joystick2_callback);
  /*function*/  //2 0 4 5   16 14 12 13        
  pinMode(2, OUTPUT);
  pinMode(0, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(15, OUTPUT);//bug--pin16
  pinMode(14, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(13, OUTPUT);
//  server1.attach(1);
//  Serial.println("Welcome to Romantic Diy F1ang...");
}

void loop() {
  Blinker.run();
  if(X1==128&&Y1==128&&X2==128&&Y2==128&&gyroscope==0)
  {
    digitalWrite(2,LOW);//左前
    digitalWrite(0,LOW);//
    digitalWrite(4,LOW);//左后
    digitalWrite(5,LOW);

    digitalWrite(15,LOW);
    digitalWrite(14,LOW);
    digitalWrite(12,LOW);
    digitalWrite(13,LOW);   
  }
}
// 如果未绑定的组件被触发，则会执行其中内容

void dataRead(const String & data)
{
    BLINKER_LOG("Blinker readString: ", data);
    counter++;
    Number1.print(counter);
}

//按下组件即会执行该函数
                                              /*摇杆量化控制*/
void Joystick1_callback(uint8_t xAxis, uint8_t yAxis)//获取摇杆x轴与y轴的数据
 {                /*左手油门*/
  X1=xAxis;   //将x轴获取的数据赋值给X
  Y1=yAxis;   //将y轴获取的数据赋值给Y
//  Serial.print("X1=");
//  Serial.println(X1);
//  Serial.print("Y1=");
//  Serial.println(Y1);
  if(X1==0&&Y1==0){     //油门为0,P-正转,N-反转
//    Serial.println("power is zero");
    //server1.write(0);

    digitalWrite(2,LOW);//左前
    digitalWrite(0,LOW);//
    digitalWrite(4,LOW);//左后
    digitalWrite(5,LOW);

    digitalWrite(15,LOW);
    digitalWrite(14,LOW);;
    digitalWrite(12,LOW);
    digitalWrite(13,LOW);
   }

  else if(X1>=0&&X1<=64&&Y1>=64&&Y1<=191){//forward
    Serial.println("forward");
    //server1.write(90);

    digitalWrite(2,HIGH);//P
    digitalWrite(0,LOW);
    digitalWrite(4,HIGH);//P
    digitalWrite(5,LOW);

    digitalWrite(15,HIGH);//P
    digitalWrite(14,LOW);
    digitalWrite(12,HIGH);//P
    digitalWrite(13,LOW);
   }

  else if(X1>=191&&X1<=255&&Y1>=64&&Y1<=191){//back
    Serial.println("back");
    //server1.write(180);

    digitalWrite(2,LOW);//N
    digitalWrite(0,HIGH);
    digitalWrite(4,LOW);//N
    digitalWrite(5,HIGH);

    digitalWrite(15,LOW);//N
    digitalWrite(14,HIGH);
    digitalWrite(12,LOW);//N
    digitalWrite(13,HIGH);
   }

  else if(X1>64&&X1<191&&Y1>191&&Y1<=255){//left平移
   Serial.println("left");

    digitalWrite(2,LOW);//N
    digitalWrite(0,HIGH);
    digitalWrite(4,HIGH);//P
    digitalWrite(5,LOW);

    digitalWrite(15,HIGH);//p
    digitalWrite(14,LOW);
    digitalWrite(12,LOW);//N
    digitalWrite(13,HIGH);
   }
  
  else if(X1>64&&X1<191&&Y1>=0&&Y1<64){//right平移
    Serial.println("right");

    digitalWrite(2,HIGH);//P
    digitalWrite(0,LOW);
    digitalWrite(4,LOW);//N
    digitalWrite(5,HIGH);

    digitalWrite(15,LOW);//N
    digitalWrite(14,HIGH);
    digitalWrite(12,HIGH);//P
    digitalWrite(13,LOW);
   }

 }
void Joystick2_callback(uint8_t xAxis, uint8_t yAxis)//获取摇杆x轴与y轴的数据
 {                /*右摇杆*/
  X2=xAxis;   //将x轴获取的数据赋值给X
  Y2=yAxis;   //将y轴获取的数据赋值给Y
//  Serial.print("X2=");
//  Serial.println(X2);
//  Serial.print("Y2=");
//  Serial.println(Y2);
   if(X2==0&&Y2==0){     
//    Serial.println("power is zero");
    //server1.write(0);

    digitalWrite(2,LOW);
    digitalWrite(0,LOW);
    digitalWrite(4,LOW);
    digitalWrite(5,LOW);

    digitalWrite(15,LOW);
    digitalWrite(14,LOW);
    digitalWrite(12,LOW);
    digitalWrite(13,LOW);
   }

  else if(X2>=0&&X2<=64&&Y2>=64&&Y2<=191){//forward
    Serial.println("forward");
    //server1.write(90);

    digitalWrite(2,HIGH);//P
    digitalWrite(0,LOW);
    digitalWrite(4,HIGH);//P
    digitalWrite(5,LOW);

    digitalWrite(15,HIGH);//P
    digitalWrite(14,LOW);
    digitalWrite(12,HIGH);//P
    digitalWrite(13,LOW);
   }

  else if(X2>=191&&X2<=255&&Y2>=64&&Y2<=191){//back
    Serial.println("back");
    //server1.write(180);

    digitalWrite(2,LOW);//N
    digitalWrite(0,HIGH);
    digitalWrite(4,LOW);//N
    digitalWrite(5,HIGH);

    digitalWrite(15,LOW);//N
    digitalWrite(14,HIGH);
    digitalWrite(12,LOW);//N
    digitalWrite(13,HIGH);
   }

  else if(X2>64&&X2<191&&Y2>191&&Y2<=255){//left
   Serial.println("left");

    digitalWrite(2,LOW);//N
    digitalWrite(0,HIGH);
    digitalWrite(4,HIGH);//P
    digitalWrite(5,LOW);

    digitalWrite(15,HIGH);//p
    digitalWrite(14,LOW);
    digitalWrite(12,HIGH);//P
    digitalWrite(13,LOW);
   }
  
  else if(X2>64&&X2<191&&Y2>=0&&Y2<64){//right
    Serial.println("right");

    digitalWrite(2,HIGH);//P---左前
    digitalWrite(0,LOW);
    digitalWrite(4,HIGH);//P--左后
    digitalWrite(5,LOW);

    digitalWrite(15,LOW);//N--右前
    digitalWrite(14,HIGH);
    digitalWrite(12,HIGH);//P--右后
    digitalWrite(13,LOW);
   }
 }
                                                      /*伪小陀螺*/
void Button1_callback(const String & state){
        //BLINKER_LOG("get button1 state: ", state);  //tap
        Serial.println(state);
        if(state == "tap"){             //按下
          gyroscope+=1;
          if(gyroscope==1)
          {
            Serial.println("consequent");//逆向旋转

            digitalWrite(2,HIGH);//P
            digitalWrite(0,LOW);
            digitalWrite(4,LOW);//N
            digitalWrite(5,HIGH);

            digitalWrite(15,HIGH);//p
            digitalWrite(14,LOW);
            digitalWrite(12,LOW);//N
            digitalWrite(13,HIGH);
          }        
          else if(gyroscope==2){  
            Serial.println("reverse");    
            digitalWrite(2,LOW);//N
            digitalWrite(0,HIGH);
            digitalWrite(4,HIGH);//P
            digitalWrite(5,LOW);

            digitalWrite(15,LOW);//N
            digitalWrite(14,HIGH);
            digitalWrite(12,HIGH);//P
            digitalWrite(13,LOW);
          }
          else if(gyroscope>3)
          {
            gyroscope=0;
            digitalWrite(2,LOW);
            digitalWrite(0,LOW);
            digitalWrite(4,LOW);
            digitalWrite(5,LOW);

            digitalWrite(15,LOW);
            digitalWrite(14,LOW);
            digitalWrite(12,LOW);
            digitalWrite(13,LOW);
          }
        }
}
