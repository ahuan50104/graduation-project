//#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <SoftwareSerial.h>
SoftwareSerial bluetooth(10, 11); // RX, TX
#include "Wire.h"
MPU6050 mpu;

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define button 3
#define led 13

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint8_t fifoBuffer[64]; // FIFO storage buffer

unsigned long Time;
unsigned long gestureTime;
float duration;
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
char last_action;
int flip_array=8;
bool flip[8];
bool led_state;
int flip_cnt=0;
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
float forward_Detect,Turn_Detect,cw_Detect;
int result; //processing接收手勢感測的結果

//////////////////////修改參數////////////////////
const int  forward_angle=30;
const int  backward_angle=-30;
const int  right_angle=25;
const int  left_angle=-25;
const int  Right_turn_angle=30;
const int  left_turn_angle=-30;
const int  transfer_speed=200;// unit mS
//////////////////////修改參數////////////////////



///////////////////////////////////////////////////////////////
void dmpDataReady() {
    mpuInterrupt = true;
}
void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    bluetooth.begin(9600);
    
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately
    
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);
    pinMode(button, INPUT);
    pinMode(led, OUTPUT);
    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    read_button();//------------------------------------------------------------------------------------------------------
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
    
    //mpu.setRate(5);
    // supply your own gyro offsets here, scaled for min sensitivity
   /* mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip*/
    
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        
        mpu.PrintActiveOffsets();
        
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
        digitalWrite(led,led_state=false);
    }
    gestureTime=millis();
}

void loop() 
{
    read_mpu6050();
    if((millis()-gestureTime)>=800){
      bluetooth.print(result);
      result=0;
      gestureTime=millis();
    }
}

void read_button()
{
  unsigned long led_Time;
  float led_duration;
  Serial.println("wait button.....");
  while(!digitalRead(button)) //等待按按鈕，此狀態下，LED會0.5秒閃爍一次
    {
      led_duration = (millis() - led_Time);
      if(led_duration>=500)
        {
           digitalWrite(led,led_state=!led_state);
           led_Time=millis();
        }
     }
  while(digitalRead(button));
  digitalWrite(led,led_state=false);
  Serial.println("一秒後校準");
  delay(1000);
}

void move_flip()
{
  for(int cnt=1;cnt<=flip_array-1;cnt++)
    {
      flip[cnt-1]=flip[cnt]; //擷取下一位的值，只有第7位不會變
    }
}

void gesture()
{
  duration = (millis() - Time);
  if(duration>transfer_speed)
    {
      digitalWrite(led,led_state=!led_state);
      Serial.print("ypr\t");
      Serial.print(cw_Detect);
      Serial.print("\t");
      Serial.print(Turn_Detect);
      Serial.print("\t");
      Serial.print(forward_Detect);
      Serial.print("\t");
      int movement=0;
      Time=millis();
      move_flip();
      
      //判斷是否有旋(YAW)的動作//
      if(cw_Detect>=Right_turn_angle) //旋轉角度>=30(預設右旋)
        {
          result=0;//無指令
          Serial.println("無指令");
        }
      else if(cw_Detect<=left_turn_angle) //旋轉角度<=-30(預設左旋)
        {
          if(Turn_Detect>=right_angle) //傾斜角度>=25(預設右傾斜)
            {
              result=3;//查看行程
              Serial.println("查看行程");
            }
          else
            {
              result=0;//無指令
              Serial.println("無指令");
            }
        }
      else movement++; 
      
      //判斷是否有左右傾斜(ROLL)的動作//
      if(movement==1)
        {
          flip[flip_array-1]=1;
          if(Turn_Detect>=right_angle) //左右傾斜角度>=30(預設右傾斜)
            { 
              result=0;//無指令
              Serial.println("無指令");
              last_action='3';

            }
          else if(Turn_Detect<=left_angle) //左右傾斜角度<=-30(預設左傾斜)
            {
              result=0;//無指令
              Serial.println("無指令");
              last_action='4';
            }
          else movement++; 
        }
        
      //判斷是否有前後傾斜(PITCH)的動作//
      if(movement==2)
        {
          flip[flip_array-1]=1;
          if(forward_Detect>=forward_angle) //前後傾斜角度>=30(預設向前)
            {
              result=0;//無指令
              Serial.println("無指令");
              last_action='1';
            }
          else if(forward_Detect<=backward_angle) //前後傾斜角度<=-30(預設向後)
            {
              result=0;//無指令
              Serial.println("無指令");
            }
          else movement++; 
        }
        
      if(movement>=3)
        {
          flip_cnt=0; //次數
          flip[flip_array-1]=0; //第7位為0
          for(int cnt=0;cnt<=flip_array-2;cnt++) //檢查第0~6位 @@為什麼不是檢查0~7? >>因為第6位已經跟第7位檢查到了，而且第七位沒有下一位
            {
              if(flip[cnt]!=flip[cnt+1]) //若相鄰兩位不相同
                flip_cnt++; //則次數加一
            }
          if(flip_cnt>=4) //當次數>=4，且不需要連續，因為人難以做到要求的動作(電腦判斷間隔太短)
            {  
              switch(last_action)
                { 
                  case '1':
                    result=2;//更換點滴
                    Serial.println("更換點滴");
                    break;
                  case '3':
                    result=1;//緊急求救
                    Serial.println("緊急求救");
                    break;
                  case '4':
                    result=1;//緊急求救
                    Serial.println("緊急求救");
                    break;
                } //3&4 -->> 因為以上面的判斷標準來看，好像是只看最後一個動作來分辨翻滾的方向，而不是原本的連續兩次相同方向的要求，所以為了方便記憶，就將緊急求救設定為「將手左右甩動」
            }
          else
            {
              result=0;//無指令
              Serial.println("無指令");
            }
          
        }
   }
}
void read_mpu6050()
{
  if (!dmpReady) return;
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        Turn_Detect=asin(2*q.x*q.z-2*q.w*q.y)*57.3;
        forward_Detect=atan2(2*q.y*q.z+2*q.w*q.x,-2*q.x*q.x-2*q.y*q.y+1)*57.3;
        cw_Detect=atan2(2*(q.x*q.y+q.w*q.z),q.w*q.w+q.x*q.x-q.y*q.y-q.z*q.z)*57.3*-1;
        gesture();
   }
}
