
/*
UNO 
타이머1 : 16.1kHz  - D9,D10
타이머2 : 62.5kHz  - D11
* 타이머1 사용시 arduino 함수중 delay() 사용하면 안됨.단, detach 하고나서는 가능.
*/

// Serial.available()는 루프를 돌때마다 줄어들어 최종 0이 된다.!!!
// 줄어들게 만드는 함수는 Serial.parseInt(), Serial.parseFloat(), Serial.read() 이다.
// 특히, parseFloat()는 한번에 여러개의 Byte를 뺀다.
// Serial.peak() 는 줄어들지 않음. 시리얼포트로부터 더 이상 읽어들일 데이터가 없으면 -1 리턴
// RAM --> ROM 으로 const int 배열 저장하고 뺴는법
// https://www.arduino.cc/reference/en/language/variables/utilities/progmem


#define VOLUMECOUNT 2
#define DATACOUNT 36
#define BAUDRATE 115200

#define USER_INT_PIN 12


#define OFFSET_AAC1 2147483136
#define OFFSET_AAC2 2147483136
#define OFFSET_ANC  2147483136


#include <Arduino.h>
#include <TimerOne.h>

#include <stddef.h>

#include "rtwtypes.h"
#include <avr/pgmspace.h>

static int volume = 0;
static float data[VOLUMECOUNT][DATACOUNT] = {
  
     {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 1, 2, 3, 4, 5, 6},
     {2, 2, 3, 4, 5, 6, 7, 8, 9, 10, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 1, 2, 3, 4, 5, 6}
};

// 1  2  3  4  5  6  7  8  9  10  11  12  13  14  15  16  17  18  19  20  21  22  23  24  25  26  27  28  29  30  31  32  33  34  35  36 

const float MEMdata[] PROGMEM = {
  1, 2.2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36,
  2, 2.2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36,
  3, 2.2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36,
  4, 2.2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36,
  5, 2.2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36,
  6, 2.2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36,
  7, 2.2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36,
  8, 2.2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36,
  9, 2.2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36,
  10, 2.2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36
};

 int voltagePin1 = A0,voltagePin2 = A2;  
 int currentPin1 = A1,currentPin2 = A3;            // 전류미터 저항으로 구현 Amp 출력단 연결
 int micPin1 = A4, micPin2 = A5;  // 마이크를 아날로그 A0 핀에 연결
 
const int AACPin2 = 9,AACPin1 = 11;             // 스피커핀 PWM으로 LM386
const int ANCPin = 10;             // Actuator핀 PWM으로 LM386

const int samplingFrequency = 16000;  // 샘플링 주파수: 16kHz

// int output1,output2,output3;  // 스피커의 PWM 조절 파라미터
float pmic1 = 1, pcurrent1 = 1, pvoltage1 = 1;   // calibration 항목
float pmic2 = 1, pcurrent2 = 1, pvoltage2 = 1;   // calibration 항목

int select;
boolean bDataOut = true;

/* Block signals and states (default storage) for system '<Root>' */
typedef struct {
  real_T Delay_DSTATE;                 /* '<S3>/Delay' */
  real_T Delay1_DSTATE;                /* '<S3>/Delay1' */
  real_T Delay_DSTATE_c;               /* '<S5>/Delay' */
  real_T Delay1_DSTATE_e;              /* '<S5>/Delay1' */
  real_T Delay_DSTATE_d;               /* '<S4>/Delay' */
  real_T Delay1_DSTATE_j;              /* '<S4>/Delay1' */
  real_T Delay_DSTATE_p;               /* '<S6>/Delay' */
  real_T Delay1_DSTATE_i;              /* '<S6>/Delay1' */
  real_T Delay_DSTATE_o;               /* '<S1>/Delay' */
  real_T Delay1_DSTATE_f;              /* '<S1>/Delay1' */
  real_T Delay_DSTATE_c4;              /* '<S2>/Delay' */
  real_T Delay1_DSTATE_a;              /* '<S2>/Delay1' */
} DW;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T Mic1;                         /* '<Root>/Mic1' */
  real_T Current1;                     /* '<Root>/Current1' */
  real_T Mic2;                         /* '<Root>/Mic2' */
  real_T Current2;                     /* '<Root>/Current2' */
  real_T Gain_Velocity1;               /* '<Root>/Gain_Velocity1' */
  real_T Gain_Pressure1;               /* '<Root>/Gain_Pressure1' */
  real_T Gain_Velocity2;               /* '<Root>/Gain_Velocity2' */
  real_T Gain_Pressure2;               /* '<Root>/Gain_Pressure2' */
  real_T Gain_BPF1;                    /* '<Root>/Gain_BPF1' */
  real_T Gain_BPF2;                    /* '<Root>/Gain_BPF2' */
  real_T ma1_Velocity1;                /* '<Root>/ma1_Velocity1' */
  real_T ma2_Velocity1;                /* '<Root>/ma2_Velocity1' */
  real_T b0_Velocity1;                 /* '<Root>/b0_Velocity1' */
  real_T b1_Velocity1;                 /* '<Root>/b1_Velocity1' */
  real_T b2_Velocity1;                 /* '<Root>/b2_Velocity1' */
  real_T ma1_Voltage1;                 /* '<Root>/ma1_Voltage1' */
  real_T ma2_Voltage1;                 /* '<Root>/ma2_Voltage1' */
  real_T b0_Voltage1;                  /* '<Root>/b0_Voltage1' */
  real_T b1_Voltage1;                  /* '<Root>/b1_Voltage1' */
  real_T b2_Voltage1;                  /* '<Root>/b2_Voltage1' */
  real_T ma1_Velocity2;                /* '<Root>/ma1_Velocity2' */
  real_T ma2_Velocity2;                /* '<Root>/ma2_Velocity2' */
  real_T b0_Velocity2;                 /* '<Root>/b0_Velocity2' */
  real_T b1_Velocity2;                 /* '<Root>/b1_Velocity2' */
  real_T b2_Velocity2;                 /* '<Root>/b2_Velocity2' */
  real_T ma1_Voltage2;                 /* '<Root>/ma1_Voltage2' */
  real_T ma2_Voltage2;                 /* '<Root>/ma2_Voltage2' */
  real_T b0_Voltage2;                  /* '<Root>/b0_Voltage2' */
  real_T b1_Voltage2;                  /* '<Root>/b1_Voltage2' */
  real_T b2_Voltage2;                  /* '<Root>/b2_Voltage2' */
  real_T ma1_MHHC1;                    /* '<Root>/ma1_MHHC1' */
  real_T ma2_MHHC1;                    /* '<Root>/ma2_MHHC1' */
  real_T b0_MHHC1;                     /* '<Root>/b0_MHHC1' */
  real_T b1_MHHC1;                     /* '<Root>/b1_MHHC1' */
  real_T b2_MHHC1;                     /* '<Root>/b2_MHHC1' */
  real_T ma1_MHHC2;                    /* '<Root>/ma1_MHHC2' */
  real_T ma2_MHHC2;                    /* '<Root>/ma2_MHHC2' */
  real_T b0_MHHC2;                     /* '<Root>/b0_MHHC2' */
  real_T b1_MHHC2;                     /* '<Root>/b1_MHHC2' */
  real_T b2_MHHC2;                     /* '<Root>/b2_MHHC2' */
  real_T Voltage1;                     /* '<Root>/Voltage1' */
  real_T Voltage2;                     /* '<Root>/Voltage2' */
} ExtU;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  // real_T AAC1;                         /* '<Root>/AAC1' */
  // real_T AAC2;                         /* '<Root>/AAC2' */
  // real_T ANC;                          /* '<Root>/ANC' */
  int32_T AAC1;                         /* '<Root>/AAC1' */
  int32_T AAC2;                         /* '<Root>/AAC2' */
  int32_T ANC;                          /* '<Root>/ANC' */


} ExtY;

/* Block signals and states (default storage) */
DW rtDW;

/* External inputs (root inport signals with default storage) */
ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
ExtY rtY;

void ParseInputTo2DArray();
void SignalProcessing(int micValue1, int micValue2, int currentValue1, int currentValue2,int voltageValue1, int voltageValue2 );
void UImenu();
void ReadAndProcessIRQ();
void SetVolumeData();
void GetVolumeData();
void printAllVolumeData();

void Timer2Setup();

void AXC_step();
void SignalProcessing2();

// ********************************************************************** //
void setup() {

  Serial.begin(BAUDRATE);  // 시리얼 통신 속도 설정
  
  pinMode(USER_INT_PIN, INPUT_PULLUP);                        // 사용자 인터럽트 버튼

  pinMode(voltagePin1, INPUT);                       // 전압,   A0 핀을 입력 모드로 설정 
  pinMode(voltagePin2, INPUT);                       // 전압,   A1 핀을 입력 모드로 설정
  pinMode(currentPin1, INPUT);                       // 전류,   A2 핀을 입력 모드로 설정 
  pinMode(currentPin2, INPUT);                       // 전류,   A3 핀을 입력 모드로 설정 
  pinMode(micPin1, INPUT);                          // 마이크, A4 핀을 입력 모드로 설정
  pinMode(micPin2, INPUT);                          // 마이크, A5 핀을 입력 모드로 설정    
  pinMode(AACPin2, OUTPUT);                     // 앰프, D9 핀 PWM 출력
  pinMode(AACPin1, OUTPUT);                     // 앰프, D10 핀 PWM 출력
  pinMode(ANCPin, OUTPUT);                     // 앰프, D11 핀 PWM 출력
  pinMode(13, OUTPUT);                              // 내장 LED핀

  Timer1.initialize(1000000 / samplingFrequency);  // 타이머 설정
  
  Serial.println("Serial Port Connected ...");
  Serial.println("Ver: 1.4.0  230531");
  Serial.println("--note------------------");
  Serial.println("remove output offset ");

  
  Serial.println("--Pin map------------------------");
  Serial.println("V1 : A0, V2 : A2");
  Serial.println("A1 : A1, A2 : A3");   
  Serial.println("MIC1 : A4, MIC2 : A5");
  Serial.println("AAC2      : D9, ANC  : D10");
  Serial.println("AAC1      : D11, STOP : D12");


  UImenu();
  // delay(100); Timer 쓸때, delay()사용하면 안됨!!!!!!! Timer2 안걸림.
  Timer2Setup();

}
// ********************************************************************** //


// ********************************************************************** //
void loop() {
  
  if (digitalRead(USER_INT_PIN) == LOW) {  // 사용자 인터럽트
    Timer1.detachInterrupt();
    delay(1000); // Timer1 멈추고는 delay() 사용해도 됨.
    UImenu();
  }
}

  
     
// ********************************************************************** //


//--------------------------------------------------------
void Timer2Setup() {
  cli();
  // 타이머2 설정
  TCCR2A = 0;  // 모든 출력 비활성화
  TCCR2B = 0;  // 카운터 중지
  TCNT2 = 0;   // 카운터 초기화

  // Timer2 설정 16kHz
  TCCR2B = B00000001;  // 분주 비율을 1로 설정 --> 62.5 KHz
  //TCCR2B = B00000010;  // 분주 비율을 8로 설정 --> 8.4 KHz
  
  //TCCR2A = B11110011;  // 반전 모드 (Fast PWM 모드)
  TCCR2A = B10100011;  // 비반전 모드 (Fast PWM 모드)
  
  OCR2A = 127;          // Duty 50% 초기값
  TIMSK2 = B00000010;  // OC2A 인터럽트 Enable --> 삭제시 인터럽트 안걸림. ISR(TIMER2_COMPA_vect)호출 안함.!!
  sei();
}
//--------------------------------------------------------



//--------------------------------------------------------
ISR(TIMER2_COMPA_vect) {
  // TO DO : 타이머2 인터럽트시 진행할 동작 정의

  OCR2A = rtY.AAC1;  //듀티 결정 0~255(100%)
  // OCR2A = output3;
  //OCR2A = map(analogRead(A0),0,1023,0,255); // 맵은 오래걸려서 클럭낮추면서, 시리얼포트 Disable 해야함!!!
  // Timer2 걸리는지 확인용
  //Serial.println("----------------------------------------");
}
//--------------------------------------------------------



//--------------------------------------------------------
void ReadAndProcessIRQ() {

  // int micValue1 = pmic1 * analogRead(micPin1);              // 마이크 입력 값 읽기
  // int micValue2 = pmic2 * analogRead(micPin2);              // 마이크 입력 값 읽기
  // int currentValue1 = pcurrent1 * analogRead(currentPin1);  // 전류 입력 값 읽기
  // int currentValue2 = pcurrent2 * analogRead(currentPin2);  // 전류 입력 값 읽기
  // int voltageValue1 = pvoltage1 * analogRead(voltagePin1);  // 전류 입력 값 읽기
  // int voltageValue2 = pvoltage2 * analogRead(voltagePin2);  // 전류 입력 값 읽기 

  // rtU.Mic1 = pmic1 * analogRead(micPin1);              // 마이크 입력 값 읽기
  // rtU.Mic2 = pmic2 * analogRead(micPin2);              // 마이크 입력 값 읽기
  // rtU.Current1 = pcurrent1 * (analogRead(currentPin1)-analogRead(voltagePin1));  // 전류 입력 값 읽기
  // rtU.Current2 = pcurrent2 * (analogRead(currentPin2)-analogRead(voltagePin2));  // 전류 입력 값 읽기
  // rtU.Voltage1 = pvoltage1 * analogRead(voltagePin1);  // 전류 입력 값 읽기
  // rtU.Voltage2 = pvoltage2 * analogRead(voltagePin2);  // 전류 입력 값 읽기 



  rtU.Mic1 = pmic1 * (analogRead(micPin1)-512);              // 마이크 입력 값 읽기
  rtU.Mic2 = pmic2 * (analogRead(micPin2)-512);              // 마이크 입력 값 읽기
  rtU.Current1 = pcurrent1 * (analogRead(currentPin1)-analogRead(voltagePin1));  // 전류 입력 값 읽기
  rtU.Current2 = pcurrent2 * (analogRead(currentPin2)-analogRead(voltagePin2));  // 전류 입력 값 읽기
  rtU.Voltage1 = pvoltage1 * (analogRead(voltagePin1)-512);  // 전류 입력 값 읽기
  rtU.Voltage2 = pvoltage2 * (analogRead(voltagePin2)-512);  // 전류 입력 값 읽기 


  // SignalProcessing -----------------------------
  //SignalProcessing(1, 1, 1,1,1,1);
  SignalProcessing2();



  // 시리얼모니터 출력 (Optional)
  if (bDataOut) {
    // Serial.print("$ ");
    Serial.print(rtU.Voltage1);
    Serial.print(",");
    Serial.print(rtU.Voltage2);
    Serial.print(",");  
    Serial.print(rtU.Current1);
    Serial.print(",");
    Serial.print(rtU.Current2);
    Serial.print(",");
    Serial.print(rtU.Mic1);
    Serial.print(",");
    Serial.print(rtU.Mic2);
    Serial.print(",");
    Serial.print(rtY.AAC1);
    Serial.print(",");
    Serial.print(rtY.AAC2);
    Serial.print(",");
    Serial.print(rtY.ANC);Serial.println("");    
    // Serial.println(";");  
  }
  
}
//--------------------------------------------------------


//--------------------------------------------------------
void SignalProcessing(int micValue1, int micValue2, int currentValue1, int currentValue2,int voltageValue1, int voltageValue2 ) {


  // TO DO : 두 신호 받아 처리블록 넣으시오
  // // SAMPLE 연산, 출력 
  // // 곱셈연산 40, 더하기 연산 30
  // randomcalc();
  // // 임시 comment 처리하시오.
  // output1 = 256;  // 주파수 16.1kHz, 1.25V
  // output2 = 512;  // 주파수 16.1kHz, 2.5V
  // output3 = 127;  // 주파수 62.5kHz, 2.5V

  // 파라미터 입력
  /* Block signals and states (default storage) */





  // 앰프제어 SPEAKER PWM 듀티 사이클을 조정
  // 주파수는 고정으므로 LC 필터 거치면 값을 결정하게 된다.
  // Timer1.setPwmDuty(AACPin2, output1); //D9, High구간값으로 1~1023 int 값
  // Timer1.setPwmDuty(ANCPin, output2); //D10, High구간값으로 1~1023 int 값 

  
}
//--------------------------------------------------------

void SignalProcessing2() 
{
  
  AXC_step();
  // 앰프제어 SPEAKER PWM 듀티 사이클을 조정
  // 주파수는 고정으므로 LC 필터 거치면 값을 결정하게 된다.

  // rtY.AAC1 = (rtY.AAC1+32768)/256; // 0 ~ 255
  // rtY.ANC = (rtY.ANC+32768)/64;    // 0 ~ 1023
  // rtY.AAC2 = (rtY.AAC2+32768)/64;  // 0 ~ 1023

  rtY.AAC1 = rtY.AAC1+OFFSET_AAC1 + 128; // 0 ~ 255
  rtY.ANC = rtY.ANC+OFFSET_ANC + 512;    // 0 ~ 1023
  rtY.AAC2 = rtY.AAC2+OFFSET_AAC2 + 512;  // 0 ~ 1023


  // Timer1.setPwmDuty(AACPin2, 512); //D9, High구간값으로 1~1023 int 값
  // Timer1.setPwmDuty(ANCPin, 512); //D10, High구간값으로 1~1023 int 값 

  Timer1.setPwmDuty(AACPin2, rtY.AAC2); //D9, High구간값으로 1~1023 int 값
  Timer1.setPwmDuty(ANCPin, rtY.ANC); //D10, High구간값으로 1~1023 int 값 

}

//--------------------------------------------------------
void UImenu() {

  // UI 구성
  Serial.println("---------------UI V0.5 -----------------");
  Serial.println("[1] Set Params(2 x 36)");
  Serial.println("[2] Calibration(TBD)");
  Serial.println("[3] Monitoring (defaut: printing)(V1,V2,A1,A2,Mic1,Mic2,AAC1,AAC2,ANC");
  Serial.println("[4] Start");
  Serial.println("-----------------------------------------");
  Serial.println("");
  UserInput();
}
//--------------------------------------------------------


//--------------------------------------------------------
void ParseInputTo2DArray() {

  int MaxDataCount = DATACOUNT;

  if (Serial.available() > 0) {
    for (byte i = 0; i < MaxDataCount * 100; i++) {
      if (i == 0) {
        volume = Serial.parseInt();
        Serial.print("volume: ");
        Serial.println(volume);
        Serial.print("Params: ");

      } else if (Serial.peek() == -1) {
        break;
      } else {
        // data[volume][i - 1] = Serial.parseInt(); //0428

        Serial.print(data[volume][i - 1]);
        Serial.print(" ");
      }
    }
    Serial.println("");
  }
}
//----------------------------------------------------------











//--------------------------------------------------------
void UserInput() {
  Serial.println("Select Menu");
  // 사용자 입력
  while (Serial.available() == 0) {
  }
  select = Serial.parseInt();
  Serial.println(select);
  Serial.println("");


  switch (select) {


    //-------------------------
    case 1: /* 파라미터 설정 */
      select = 0;
      Serial.println("SELECT.");
      Serial.println("1)Input Params.     2)Print Params.     3) Print all Params.");

      while (Serial.available() == 0) {
      }
      select = Serial.parseInt();
      Serial.println(select);

      if (select == 1) {
        SetVolumeData();
      } else if (select == 2) {
        GetVolumeData();
      } else if (select == 3) {
        printAllVolumeData();
      } else {
        Serial.println("wrong input....");
      }
  

  UImenu();
  break;
  //---------------------------

  //---------------------------
  case 2: /* Calibration */

    Serial.println("Calibration TBD.");

    UImenu();
    break;
  //---------------------------

  //---------------------------
  case 3: /* 시리얼모니터 출력 */

    bDataOut = !bDataOut;

    if (bDataOut) {
      Serial.println("print Serial data.");
    } else {
      Serial.println("not print Serial data.");
    }

    UImenu();
    break;
  //---------------------------

  //---------------------------
  case 4: /* 시작 종료 */

    Serial.print("Input VOLUME for application: ");
    while (Serial.available() == 0) {
      }
    select = Serial.parseInt();
    Serial.println(select);
    SetAppParameters(select);
    // SetAppParametersFromMEM(select);   // ROM 메모리로부터 파라미터를 입력받을 경우

    Serial.println("36 params applied.");
    
    Serial.println("check params ------------");
    printAppParameters(select);
    // printAppParametersMEM(select);    // ROM 메모리로부터 파라미터를 입력받을 경우

    Serial.println(" ");
    Serial.println("Input 1 to start !!! ");
    while (Serial.available() == 0) { }
    select = Serial.parseInt();

    if (select==1) { 
    // Timer1.initialize(1000000 / samplingFrequency);  // 타이머 설정 -->  1000000us/fs
    Timer1.attachInterrupt(ReadAndProcessIRQ);  // IRQ 함수 설정
    Timer1.pwm(AACPin2, 0);             // D9핀,  듀티 ( 0 0% ~ 1023 100%) SETUP 초기값
    Timer1.pwm(ANCPin, 0);             // D10핀, 듀티 ( 0 0% ~ 1023 100%) SETUP 초기값
    break;
    }
    UImenu();
    break;
    //---------------------------

}  // switch
}  // textUI
//--------------------------------------------------------





// -----------------------------------------------------
void SetVolumeData() {

  int MaxDataCount = DATACOUNT;
  Serial.println("first one is VOLUME, the others (36 ea) are params.(* Insert SPACE between data )");
  Serial.println("ex) 1 1.1 2 3.3 400 ...");

  // wait for data input
  while (Serial.available() == 0) {
  }



  if (Serial.available() > 0) {

    for (byte i = 0; i < MaxDataCount * 100; i++) {
      if (i == 0) {
        volume = Serial.parseInt();
        Serial.print("volume: ");
        Serial.println(volume);
        Serial.print("Params: ");

      } else if (Serial.peek() == -1) {
        break;
      } else {

        // data[volume][i - 1] = Serial.parseInt();  //0508
        data[volume][i - 1] = Serial.parseFloat();  //0508

        Serial.print(data[volume][i - 1]);
        Serial.print(" ");
      }
    }
    Serial.println("");
  }
}
// -----------------------------------------------------


// -----------------------------------------------------
void GetVolumeData() {
  Serial.println("Volume: ");
  // wait for data input
  while (Serial.available() == 0) {
  }
  volume = Serial.parseInt();

  for (int i = 0; i < VOLUMECOUNT; i++) {
    Serial.print(data[volume][i]);
    Serial.print(" ");
  }
  Serial.println(" ");
}
// -----------------------------------------------------

// -----------------------------------------------------
void SetAppParameters(int volume) {
  rtU.Gain_Velocity1 = data[volume][0];               /* '<Root>/Gain_Velocity1' */
  rtU.Gain_Pressure1 = data[volume][1 ];               /* '<Root>/Gain_Pressure1' */
  rtU.Gain_Velocity2 = data[volume][2 ];               /* '<Root>/Gain_Velocity2' */
  rtU.Gain_Pressure2 = data[volume][3 ];               /* '<Root>/Gain_Pressure2' */
  rtU.Gain_BPF1 = data[volume][4 ];                    /* '<Root>/Gain_BPF1' */
  rtU.Gain_BPF2 = data[volume][5 ];                    /* '<Root>/Gain_BPF2' */
  rtU.ma1_Velocity1 = data[volume][6 ];                /* '<Root>/ma1_Velocity1' */
  rtU.ma2_Velocity1 = data[volume][7 ];                /* '<Root>/ma2_Velocity1' */
  rtU.b0_Velocity1 = data[volume][8 ];                 /* '<Root>/b0_Velocity1' */
  rtU.b1_Velocity1 = data[volume][9 ];                 /* '<Root>/b1_Velocity1' */
  rtU.b2_Velocity1 = data[volume][10 ];                 /* '<Root>/b2_Velocity1' */
  rtU.ma1_Voltage1 = data[volume][11 ];                 /* '<Root>/ma1_Voltage1' */
  rtU.ma2_Voltage1 = data[volume][12 ];                 /* '<Root>/ma2_Voltage1' */
  rtU.b0_Voltage1 = data[volume][13 ];                  /* '<Root>/b0_Voltage1' */
  rtU.b1_Voltage1 = data[volume][14 ];                  /* '<Root>/b1_Voltage1' */
  rtU.b2_Voltage1 = data[volume][15 ];                  /* '<Root>/b2_Voltage1' */
  rtU.ma1_Velocity2 = data[volume][16 ];                /* '<Root>/ma1_Velocity2' */
  rtU.ma2_Velocity2 = data[volume][ 17];                /* '<Root>/ma2_Velocity2' */
  rtU.b0_Velocity2 = data[volume][ 18];                 /* '<Root>/b0_Velocity2' */
  rtU.b1_Velocity2 = data[volume][ 19];                 /* '<Root>/b1_Velocity2' */
  rtU.b2_Velocity2 = data[volume][ 20];                 /* '<Root>/b2_Velocity2' */
  rtU.ma1_Voltage2 = data[volume][21 ];                 /* '<Root>/ma1_Voltage2' */
  rtU.ma2_Voltage2 = data[volume][22 ];                 /* '<Root>/ma2_Voltage2' */
  rtU.b0_Voltage2 = data[volume][23 ];                  /* '<Root>/b0_Voltage2' */
  rtU.b1_Voltage2 = data[volume][24 ];                  /* '<Root>/b1_Voltage2' */
  rtU.b2_Voltage2 = data[volume][25 ];                  /* '<Root>/b2_Voltage2' */
  rtU.ma1_MHHC1 = data[volume][26 ];                    /* '<Root>/ma1_MHHC1' */
  rtU.ma2_MHHC1 = data[volume][27 ];                    /* '<Root>/ma2_MHHC1' */
  rtU.b0_MHHC1 = data[volume][28 ];                     /* '<Root>/b0_MHHC1' */
  rtU.b1_MHHC1 = data[volume][29 ];                     /* '<Root>/b1_MHHC1' */
  rtU.b2_MHHC1 = data[volume][30 ];                     /* '<Root>/b2_MHHC1' */
  rtU.ma1_MHHC2 = data[volume][31 ];                    /* '<Root>/ma1_MHHC2' */
  rtU.ma2_MHHC2 = data[volume][32 ];                    /* '<Root>/ma2_MHHC2' */
  rtU.b0_MHHC2 = data[volume][33 ];                     /* '<Root>/b0_MHHC2' */
  rtU.b1_MHHC2 = data[volume][34 ];                     /* '<Root>/b1_MHHC2' */
  rtU.b2_MHHC2 = data[volume][35 ];   
}
// -----------------------------------------------------

// -----------------------------------------------------
void SetAppParametersFromMEM(int volume) {

// https://onlinedocs.microchip.com/oxy/GUID-BD1C16C8-7FA3-4D73-A4BE-241EE05EF592-en-US-5/GUID-90FCF448-97F2-404D-ABF3-8B5A4AFDACBD.html
// pgm_read_word_near 정수
// pgm_read_float_near float경우

  
  rtU.Gain_Velocity1 = pgm_read_float_near(volume*36 + MEMdata);               /* '<Root>/Gain_Velocity1' */
  rtU.Gain_Pressure1 = pgm_read_float_near(volume*36 + MEMdata+1);               /* '<Root>/Gain_Pressure1' */
  rtU.Gain_Velocity2 = pgm_read_float_near(volume*36 + MEMdata+2);               /* '<Root>/Gain_Velocity2' */
  rtU.Gain_Pressure2 = pgm_read_float_near(volume*36 + MEMdata+3);               /* '<Root>/Gain_Pressure2' */
  rtU.Gain_BPF1 = pgm_read_float_near(volume*36 + MEMdata+4 );                    /* '<Root>/Gain_BPF1' */
  rtU.Gain_BPF2 = pgm_read_float_near(volume*36 + MEMdata+5 );                    /* '<Root>/Gain_BPF2' */
  rtU.ma1_Velocity1 = pgm_read_float_near(volume*36 + MEMdata+6 );                /* '<Root>/ma1_Velocity1' */
  rtU.ma2_Velocity1 = pgm_read_float_near(volume*36 + MEMdata+7 );                /* '<Root>/ma2_Velocity1' */
  rtU.b0_Velocity1 = pgm_read_float_near(volume*36 + MEMdata+8 );                 /* '<Root>/b0_Velocity1' */
  rtU.b1_Velocity1 = pgm_read_float_near(volume*36 + MEMdata+9 );                 /* '<Root>/b1_Velocity1' */
  rtU.b2_Velocity1 = pgm_read_float_near(volume*36 + MEMdata+10 );                 /* '<Root>/b2_Velocity1' */
  rtU.ma1_Voltage1 = pgm_read_float_near(volume*36 + MEMdata+11 );                 /* '<Root>/ma1_Voltage1' */
  rtU.ma2_Voltage1 = pgm_read_float_near(volume*36 + MEMdata+12 );                 /* '<Root>/ma2_Voltage1' */
  rtU.b0_Voltage1 = pgm_read_float_near(volume*36 + MEMdata+13 );                  /* '<Root>/b0_Voltage1' */
  rtU.b1_Voltage1 = pgm_read_float_near(volume*36 + MEMdata+14 );                  /* '<Root>/b1_Voltage1' */
  rtU.b2_Voltage1 = pgm_read_float_near(volume*36 + MEMdata+15 );                  /* '<Root>/b2_Voltage1' */
  rtU.ma1_Velocity2 = pgm_read_float_near(volume*36 + MEMdata+16 );                /* '<Root>/ma1_Velocity2' */
  rtU.ma2_Velocity2 = pgm_read_float_near(volume*36 + MEMdata+ 17);                /* '<Root>/ma2_Velocity2' */
  rtU.b0_Velocity2 = pgm_read_float_near(volume*36 + MEMdata+ 18);                 /* '<Root>/b0_Velocity2' */
  rtU.b1_Velocity2 = pgm_read_float_near(volume*36 + MEMdata+ 19);                 /* '<Root>/b1_Velocity2' */
  rtU.b2_Velocity2 = pgm_read_float_near(volume*36 + MEMdata+ 20);                 /* '<Root>/b2_Velocity2' */
  rtU.ma1_Voltage2 = pgm_read_float_near(volume*36 + MEMdata+21 );                 /* '<Root>/ma1_Voltage2' */
  rtU.ma2_Voltage2 = pgm_read_float_near(volume*36 + MEMdata+22 );                 /* '<Root>/ma2_Voltage2' */
  rtU.b0_Voltage2 = pgm_read_float_near(volume*36 + MEMdata+23 );                  /* '<Root>/b0_Voltage2' */
  rtU.b1_Voltage2 = pgm_read_float_near(volume*36 + MEMdata+24 );                  /* '<Root>/b1_Voltage2' */
  rtU.b2_Voltage2 = pgm_read_float_near(volume*36 + MEMdata+25 );                  /* '<Root>/b2_Voltage2' */
  rtU.ma1_MHHC1 = pgm_read_float_near(volume*36 + MEMdata+26 );                    /* '<Root>/ma1_MHHC1' */
  rtU.ma2_MHHC1 = pgm_read_float_near(volume*36 + MEMdata+27 );                    /* '<Root>/ma2_MHHC1' */
  rtU.b0_MHHC1 = pgm_read_float_near(volume*36 + MEMdata+28 );                     /* '<Root>/b0_MHHC1' */
  rtU.b1_MHHC1 = pgm_read_float_near(volume*36 + MEMdata+29 );                     /* '<Root>/b1_MHHC1' */
  rtU.b2_MHHC1 = pgm_read_float_near(volume*36 + MEMdata+30 );                     /* '<Root>/b2_MHHC1' */
  rtU.ma1_MHHC2 = pgm_read_float_near(volume*36 + MEMdata+31 );                    /* '<Root>/ma1_MHHC2' */
  rtU.ma2_MHHC2 = pgm_read_float_near(volume*36 + MEMdata+32 );                    /* '<Root>/ma2_MHHC2' */
  rtU.b0_MHHC2 = pgm_read_float_near(volume*36 + MEMdata+33 );                     /* '<Root>/b0_MHHC2' */
  rtU.b1_MHHC2 = pgm_read_float_near(volume*36 + MEMdata+34 );                     /* '<Root>/b1_MHHC2' */
  rtU.b2_MHHC2 = pgm_read_float_near(volume*36 + MEMdata+35 );   
}
// -----------------------------------------------------

void printAppParameters(int volume) {
  Serial.print(rtU.Gain_Velocity1);Serial.print(" : ");Serial.println(data[volume][0]);               /* '<Root>/Gain_Velocity1' */
  Serial.print(rtU.Gain_Pressure1);Serial.print(" : ");Serial.println(data[volume][1 ]);               /* '<Root>/Gain_Pressure1' */
  Serial.print(rtU.Gain_Velocity2);Serial.print(" : ");Serial.println(data[volume][2 ]);               /* '<Root>/Gain_Velocity2' */
  Serial.print(rtU.Gain_Pressure2);Serial.print(" : ");Serial.println(data[volume][3 ]);               /* '<Root>/Gain_Pressure2' */
  Serial.print(rtU.Gain_BPF1);Serial.print(" : ");Serial.println(data[volume][4 ]);                    /* '<Root>/Gain_BPF1' */
  Serial.print(rtU.Gain_BPF2);Serial.print(" : ");Serial.println(data[volume][5 ]);                    /* '<Root>/Gain_BPF2' */
  Serial.print(rtU.ma1_Velocity1);Serial.print(" : ");Serial.println(data[volume][6 ]);                /* '<Root>/ma1_Velocity1' */
  Serial.print(rtU.ma2_Velocity1);Serial.print(" : ");Serial.println(data[volume][7 ]);                /* '<Root>/ma2_Velocity1' */
  Serial.print(rtU.b0_Velocity1);Serial.print(" : ");Serial.println(data[volume][8 ]);                 /* '<Root>/b0_Velocity1' */
  Serial.print(rtU.b1_Velocity1);Serial.print(" : ");Serial.println(data[volume][9 ]);                 /* '<Root>/b1_Velocity1' */
  Serial.print(rtU.b2_Velocity1);Serial.print(" : ");Serial.println(data[volume][10 ]);                 /* '<Root>/b2_Velocity1' */
  Serial.print(rtU.ma1_Voltage1);Serial.print(" : ");Serial.println(data[volume][11 ]);                 /* '<Root>/ma1_Voltage1' */
  Serial.print(rtU.ma2_Voltage1);Serial.print(" : ");Serial.println(data[volume][12 ]);                 /* '<Root>/ma2_Voltage1' */
  Serial.print(rtU.b0_Voltage1);Serial.print(" : ");Serial.println(data[volume][13 ]);                  /* '<Root>/b0_Voltage1' */
  Serial.print(rtU.b1_Voltage1);Serial.print(" : ");Serial.println(data[volume][14 ]);                  /* '<Root>/b1_Voltage1' */
  Serial.print(rtU.b2_Voltage1);Serial.print(" : ");Serial.println(data[volume][15 ]);                  /* '<Root>/b2_Voltage1' */
  Serial.print(rtU.ma1_Velocity2);Serial.print(" : ");Serial.println(data[volume][16 ]);                /* '<Root>/ma1_Velocity2' */
  Serial.print(rtU.ma2_Velocity2);Serial.print(" : ");Serial.println(data[volume][ 17]);                /* '<Root>/ma2_Velocity2' */
  Serial.print(rtU.b0_Velocity2);Serial.print(" : ");Serial.println(data[volume][ 18]);                 /* '<Root>/b0_Velocity2' */
  Serial.print(rtU.b1_Velocity2);Serial.print(" : ");Serial.println(data[volume][ 19]);                 /* '<Root>/b1_Velocity2' */
  Serial.print(rtU.b2_Velocity2);Serial.print(" : ");Serial.println(data[volume][ 20]);                 /* '<Root>/b2_Velocity2' */
  Serial.print(rtU.ma1_Voltage2);Serial.print(" : ");Serial.println(data[volume][21 ]);                 /* '<Root>/ma1_Voltage2' */
  Serial.print(rtU.ma2_Voltage2);Serial.print(" : ");Serial.println(data[volume][22 ]);                 /* '<Root>/ma2_Voltage2' */
  Serial.print(rtU.b0_Voltage2);Serial.print(" : ");Serial.println(data[volume][23 ]);                  /* '<Root>/b0_Voltage2' */
  Serial.print(rtU.b1_Voltage2);Serial.print(" : ");Serial.println(data[volume][24 ]);                  /* '<Root>/b1_Voltage2' */
  Serial.print(rtU.b2_Voltage2);Serial.print(" : ");Serial.println(data[volume][25 ]);                  /* '<Root>/b2_Voltage2' */
  Serial.print(rtU.ma1_MHHC1);Serial.print(" : ");Serial.println(data[volume][26 ]);                    /* '<Root>/ma1_MHHC1' */
  Serial.print(rtU.ma2_MHHC1);Serial.print(" : ");Serial.println(data[volume][27 ]);                    /* '<Root>/ma2_MHHC1' */
  Serial.print(rtU.b0_MHHC1);Serial.print(" : ");Serial.println(data[volume][28 ]);                     /* '<Root>/b0_MHHC1' */
  Serial.print(rtU.b1_MHHC1);Serial.print(" : ");Serial.println(data[volume][29 ]);                     /* '<Root>/b1_MHHC1' */
  Serial.print(rtU.b2_MHHC1);Serial.print(" : ");Serial.println(data[volume][30 ]);                     /* '<Root>/b2_MHHC1' */
  Serial.print(rtU.ma1_MHHC2);Serial.print(" : ");Serial.println(data[volume][31 ]);                    /* '<Root>/ma1_MHHC2' */
  Serial.print(rtU.ma2_MHHC2);Serial.print(" : ");Serial.println(data[volume][32 ]);                    /* '<Root>/ma2_MHHC2' */
  Serial.print(rtU.b0_MHHC2);Serial.print(" : ");Serial.println(data[volume][33 ]);                     /* '<Root>/b0_MHHC2' */
  Serial.print(rtU.b1_MHHC2);Serial.print(" : ");Serial.println(data[volume][34 ]);                     /* '<Root>/b1_MHHC2' */
  Serial.print(rtU.b2_MHHC2);Serial.print(" : ");Serial.println(data[volume][35 ]);   
}


void printAppParametersMEM(int volume) {
  Serial.print(rtU.Gain_Velocity1);Serial.print(" : ");Serial.println(pgm_read_float_near(volume*36 + MEMdata+0));              /* '<Root>/Gain_Velocity1' */
  Serial.print(rtU.Gain_Pressure1);Serial.print(" : ");Serial.println(pgm_read_float_near(volume*36 + MEMdata+1 ));              /* '<Root>/Gain_Pressure1' */
  Serial.print(rtU.Gain_Velocity2);Serial.print(" : ");Serial.println(pgm_read_float_near(volume*36 + MEMdata+2 ));              /* '<Root>/Gain_Velocity2' */
  Serial.print(rtU.Gain_Pressure2);Serial.print(" : ");Serial.println(pgm_read_float_near(volume*36 + MEMdata+3 ));              /* '<Root>/Gain_Pressure2' */
  Serial.print(rtU.Gain_BPF1);Serial.print(" : ");Serial.println(pgm_read_float_near(volume*36 + MEMdata+4 ));                   /* '<Root>/Gain_BPF1' */
  Serial.print(rtU.Gain_BPF2);Serial.print(" : ");Serial.println(pgm_read_float_near(volume*36 + MEMdata+5 ));                   /* '<Root>/Gain_BPF2' */
  Serial.print(rtU.ma1_Velocity1);Serial.print(" : ");Serial.println(pgm_read_float_near(volume*36 + MEMdata+6 ));               /* '<Root>/ma1_Velocity1' */
  Serial.print(rtU.ma2_Velocity1);Serial.print(" : ");Serial.println(pgm_read_float_near(volume*36 + MEMdata+7 ));               /* '<Root>/ma2_Velocity1' */
  Serial.print(rtU.b0_Velocity1);Serial.print(" : ");Serial.println(pgm_read_float_near(volume*36 + MEMdata+8 ));                /* '<Root>/b0_Velocity1' */
  Serial.print(rtU.b1_Velocity1);Serial.print(" : ");Serial.println(pgm_read_float_near(volume*36 + MEMdata+9 ));                /* '<Root>/b1_Velocity1' */
  Serial.print(rtU.b2_Velocity1);Serial.print(" : ");Serial.println(pgm_read_float_near(volume*36 + MEMdata+10 ));                /* '<Root>/b2_Velocity1' */
  Serial.print(rtU.ma1_Voltage1);Serial.print(" : ");Serial.println(pgm_read_float_near(volume*36 + MEMdata+11 ));                /* '<Root>/ma1_Voltage1' */
  Serial.print(rtU.ma2_Voltage1);Serial.print(" : ");Serial.println(pgm_read_float_near(volume*36 + MEMdata+12 ));                /* '<Root>/ma2_Voltage1' */
  Serial.print(rtU.b0_Voltage1);Serial.print(" : ");Serial.println(pgm_read_float_near(volume*36 + MEMdata+13 ));                 /* '<Root>/b0_Voltage1' */
  Serial.print(rtU.b1_Voltage1);Serial.print(" : ");Serial.println(pgm_read_float_near(volume*36 + MEMdata+14 ));                 /* '<Root>/b1_Voltage1' */
  Serial.print(rtU.b2_Voltage1);Serial.print(" : ");Serial.println(pgm_read_float_near(volume*36 + MEMdata+15 ));                 /* '<Root>/b2_Voltage1' */
  Serial.print(rtU.ma1_Velocity2);Serial.print(" : ");Serial.println(pgm_read_float_near(volume*36 + MEMdata+16 ));               /* '<Root>/ma1_Velocity2' */
  Serial.print(rtU.ma2_Velocity2);Serial.print(" : ");Serial.println(pgm_read_float_near(volume*36 + MEMdata+ 17));               /* '<Root>/ma2_Velocity2' */
  Serial.print(rtU.b0_Velocity2);Serial.print(" : ");Serial.println(pgm_read_float_near(volume*36 + MEMdata+ 18));                /* '<Root>/b0_Velocity2' */
  Serial.print(rtU.b1_Velocity2);Serial.print(" : ");Serial.println(pgm_read_float_near(volume*36 + MEMdata+ 19));                /* '<Root>/b1_Velocity2' */
  Serial.print(rtU.b2_Velocity2);Serial.print(" : ");Serial.println(pgm_read_float_near(volume*36 + MEMdata+ 20));                /* '<Root>/b2_Velocity2' */
  Serial.print(rtU.ma1_Voltage2);Serial.print(" : ");Serial.println(pgm_read_float_near(volume*36 + MEMdata+21 ));                /* '<Root>/ma1_Voltage2' */
  Serial.print(rtU.ma2_Voltage2);Serial.print(" : ");Serial.println(pgm_read_float_near(volume*36 + MEMdata+22 ));                /* '<Root>/ma2_Voltage2' */
  Serial.print(rtU.b0_Voltage2);Serial.print(" : ");Serial.println(pgm_read_float_near(volume*36 + MEMdata+23 ));                 /* '<Root>/b0_Voltage2' */
  Serial.print(rtU.b1_Voltage2);Serial.print(" : ");Serial.println(pgm_read_float_near(volume*36 + MEMdata+24 ));                 /* '<Root>/b1_Voltage2' */
  Serial.print(rtU.b2_Voltage2);Serial.print(" : ");Serial.println(pgm_read_float_near(volume*36 + MEMdata+25 ));                 /* '<Root>/b2_Voltage2' */
  Serial.print(rtU.ma1_MHHC1);Serial.print(" : ");Serial.println(pgm_read_float_near(volume*36 + MEMdata+26 ));                   /* '<Root>/ma1_MHHC1' */
  Serial.print(rtU.ma2_MHHC1);Serial.print(" : ");Serial.println(pgm_read_float_near(volume*36 + MEMdata+27 ));                   /* '<Root>/ma2_MHHC1' */
  Serial.print(rtU.b0_MHHC1);Serial.print(" : ");Serial.println(pgm_read_float_near(volume*36 + MEMdata+28 ));                    /* '<Root>/b0_MHHC1' */
  Serial.print(rtU.b1_MHHC1);Serial.print(" : ");Serial.println(pgm_read_float_near(volume*36 + MEMdata+29 ));                    /* '<Root>/b1_MHHC1' */
  Serial.print(rtU.b2_MHHC1);Serial.print(" : ");Serial.println(pgm_read_float_near(volume*36 + MEMdata+30 ));                    /* '<Root>/b2_MHHC1' */
  Serial.print(rtU.ma1_MHHC2);Serial.print(" : ");Serial.println(pgm_read_float_near(volume*36 + MEMdata+31 ));                   /* '<Root>/ma1_MHHC2' */
  Serial.print(rtU.ma2_MHHC2);Serial.print(" : ");Serial.println(pgm_read_float_near(volume*36 + MEMdata+32 ));                   /* '<Root>/ma2_MHHC2' */
  Serial.print(rtU.b0_MHHC2);Serial.print(" : ");Serial.println(pgm_read_float_near(volume*36 + MEMdata+33 ));                    /* '<Root>/b0_MHHC2' */
  Serial.print(rtU.b1_MHHC2);Serial.print(" : ");Serial.println(pgm_read_float_near(volume*36 + MEMdata+34 ));                    /* '<Root>/b1_MHHC2' */
  Serial.print(rtU.b2_MHHC2);Serial.print(" : ");Serial.println(pgm_read_float_near(volume*36 + MEMdata+35 ));  
}


// -----------------------------------------------------
void printAllVolumeData() {
  Serial.println("------Params @ VOLUME-------------------- ");

  for (int i = 0; i < VOLUMECOUNT; i++) {
    Serial.print("VOLUME ");
    Serial.print(i);
    Serial.print(": ");
    for (int j = 0; j < DATACOUNT; j++) {
      Serial.print(data[i][j]);
      Serial.print(" ");
    }
    Serial.println(" ");
    Serial.println(" ");
  }
}
// -----------------------------------------------------

// -----------------------------------------------------

void randomcalc() {
  float signal1[10] = {0.5, 0.3, 0.1, 0.7, 0.2, 0.6, 0.8, 0.4, 0.9, 0.2}; // 첫 번째 신호
  float signal2[10] = {0.2, 0.9, 0.8, 0.1, 0.3, 0.7, 0.5, 0.6, 0.4, 0.2}; // 두 번째 신호
  float signal3[10] = {0.5, 0.3, 0.1, 0.7, 0.2, 0.6, 0.8, 0.4, 0.9, 0.2}; // 첫 번째 신호
  float signal4[10] = {0.2, 0.9, 0.8, 0.1, 0.3, 0.7, 0.5, 0.6, 0.4, 0.2}; // 두 번째 신호
  float result = 0.0; // 결과값 초기화

  // 두 신호의 곱셈을 40회 수행
  for (int i = 0; i < 40; i++) {
    int index = random(0, VOLUMECOUNT); // 무작위 인덱스 생성
    result += signal1[index] * signal2[index]; // 두 신호를 곱하여 결과값에 더함
  }

  // 두 신호의 덧셈을 30회 수행
  for (int i = 0; i < 30; i++) {
    int index = random(0, VOLUMECOUNT); // 무작위 인덱스 생성
    result += signal3[index] + signal4[index]; // 두 신호를 더하여 결과값에 더함
  }

}
// -----------------------------------------------------


/* Model step function */
void AXC_step()
{
  real_T rtb_Delay;
  real_T rtb_Delay_c;
  real_T rtb_Delay_cr;
  real_T rtb_Delay_d;
  real_T rtb_Delay_j;
  real_T rtb_Delay_l;
  real_T rtb_Sum3;
  real_T rtb_Sum3_c;
  real_T rtb_Sum3_e;
  real_T rtb_Sum3_h;
  real_T rtb_Sum3_n;
  real_T rtb_Sum3_o;

  /* Delay: '<S3>/Delay' */
  rtb_Delay = rtDW.Delay_DSTATE;

  /* Sum: '<S3>/Sum3' incorporates:
   *  Delay: '<S3>/Delay'
   *  Delay: '<S3>/Delay1'
   *  Inport: '<Root>/Current1'
   *  Inport: '<Root>/ma1_Velocity1'
   *  Inport: '<Root>/ma2_Velocity1'
   *  Product: '<S3>/Product'
   *  Product: '<S3>/Product2'
   *  Sum: '<S3>/Sum2'
   */
  rtb_Sum3 = (rtDW.Delay_DSTATE * rtU.ma1_Velocity1 + rtDW.Delay1_DSTATE *
              rtU.ma2_Velocity1) + rtU.Current1;

  /* Delay: '<S5>/Delay' */
  rtb_Delay_d = rtDW.Delay_DSTATE_c;

  /* Sum: '<S5>/Sum3' incorporates:
   *  Delay: '<S3>/Delay'
   *  Delay: '<S3>/Delay1'
   *  Delay: '<S5>/Delay'
   *  Delay: '<S5>/Delay1'
   *  Gain: '<Root>/Gain'
   *  Gain: '<Root>/Gain1'
   *  Inport: '<Root>/Gain_Pressure1'
   *  Inport: '<Root>/Gain_Velocity1'
   *  Inport: '<Root>/Mic1'
   *  Inport: '<Root>/Voltage1'
   *  Inport: '<Root>/b0_Velocity1'
   *  Inport: '<Root>/b1_Velocity1'
   *  Inport: '<Root>/b2_Velocity1'
   *  Inport: '<Root>/ma1_Voltage1'
   *  Inport: '<Root>/ma2_Voltage1'
   *  Product: '<Root>/Product'
   *  Product: '<Root>/Product1'
   *  Product: '<S3>/Product3'
   *  Product: '<S3>/Product4'
   *  Product: '<S3>/Product5'
   *  Product: '<S5>/Product'
   *  Product: '<S5>/Product2'
   *  Sum: '<Root>/Sum'
   *  Sum: '<Root>/Sum1'
   *  Sum: '<S3>/Sum'
   *  Sum: '<S3>/Sum1'
   *  Sum: '<S5>/Sum2'
   */
  rtb_Sum3_n = (-(rtU.Voltage1 - ((rtDW.Delay_DSTATE * rtU.b1_Velocity1 +
    rtDW.Delay1_DSTATE * rtU.b2_Velocity1) + rtb_Sum3 * rtU.b0_Velocity1)) *
                rtU.Gain_Velocity1 + -rtU.Mic1 * rtU.Gain_Pressure1) +
    (rtDW.Delay_DSTATE_c * rtU.ma1_Voltage1 + rtDW.Delay1_DSTATE_e *
     rtU.ma2_Voltage1);

  /* Outport: '<Root>/AAC1' incorporates:
   *  Delay: '<S5>/Delay'
   *  Delay: '<S5>/Delay1'
   *  Inport: '<Root>/b0_Voltage1'
   *  Inport: '<Root>/b1_Voltage1'
   *  Inport: '<Root>/b2_Voltage1'
   *  Product: '<S5>/Product3'
   *  Product: '<S5>/Product4'
   *  Product: '<S5>/Product5'
   *  Sum: '<S5>/Sum'
   *  Sum: '<S5>/Sum1'
   */
  rtY.AAC1 = (rtDW.Delay_DSTATE_c * rtU.b1_Voltage1 + rtDW.Delay1_DSTATE_e *
              rtU.b2_Voltage1) + rtb_Sum3_n * rtU.b0_Voltage1;

  /* Delay: '<S4>/Delay' */
  rtb_Delay_cr = rtDW.Delay_DSTATE_d;

  /* Sum: '<S4>/Sum3' incorporates:
   *  Delay: '<S4>/Delay'
   *  Delay: '<S4>/Delay1'
   *  Inport: '<Root>/Current2'
   *  Inport: '<Root>/ma1_Velocity2'
   *  Inport: '<Root>/ma2_Velocity2'
   *  Product: '<S4>/Product'
   *  Product: '<S4>/Product2'
   *  Sum: '<S4>/Sum2'
   */
  rtb_Sum3_e = (rtDW.Delay_DSTATE_d * rtU.ma1_Velocity2 + rtDW.Delay1_DSTATE_j *
                rtU.ma2_Velocity2) + rtU.Current2;

  /* Delay: '<S6>/Delay' */
  rtb_Delay_c = rtDW.Delay_DSTATE_p;

  /* Sum: '<S6>/Sum3' incorporates:
   *  Delay: '<S4>/Delay'
   *  Delay: '<S4>/Delay1'
   *  Delay: '<S6>/Delay'
   *  Delay: '<S6>/Delay1'
   *  Gain: '<Root>/Gain2'
   *  Gain: '<Root>/Gain3'
   *  Inport: '<Root>/Gain_Pressure2'
   *  Inport: '<Root>/Gain_Velocity2'
   *  Inport: '<Root>/Mic2'
   *  Inport: '<Root>/Voltage2'
   *  Inport: '<Root>/b0_Velocity2'
   *  Inport: '<Root>/b1_Velocity2'
   *  Inport: '<Root>/b2_Velocity2'
   *  Inport: '<Root>/ma1_Voltage2'
   *  Inport: '<Root>/ma2_Voltage2'
   *  Product: '<Root>/Product2'
   *  Product: '<Root>/Product3'
   *  Product: '<S4>/Product3'
   *  Product: '<S4>/Product4'
   *  Product: '<S4>/Product5'
   *  Product: '<S6>/Product'
   *  Product: '<S6>/Product2'
   *  Sum: '<Root>/Sum2'
   *  Sum: '<Root>/Sum3'
   *  Sum: '<S4>/Sum'
   *  Sum: '<S4>/Sum1'
   *  Sum: '<S6>/Sum2'
   */
  rtb_Sum3_o = (-(rtU.Voltage2 - ((rtDW.Delay_DSTATE_d * rtU.b1_Velocity2 +
    rtDW.Delay1_DSTATE_j * rtU.b2_Velocity2) + rtb_Sum3_e * rtU.b0_Velocity2)) *
                rtU.Gain_Velocity2 + -rtU.Mic2 * rtU.Gain_Pressure2) +
    (rtDW.Delay_DSTATE_p * rtU.ma1_Voltage2 + rtDW.Delay1_DSTATE_i *
     rtU.ma2_Voltage2);

  /* Outport: '<Root>/AAC2' incorporates:
   *  Delay: '<S6>/Delay'
   *  Delay: '<S6>/Delay1'
   *  Inport: '<Root>/b0_Voltage2'
   *  Inport: '<Root>/b1_Voltage2'
   *  Inport: '<Root>/b2_Voltage2'
   *  Product: '<S6>/Product3'
   *  Product: '<S6>/Product4'
   *  Product: '<S6>/Product5'
   *  Sum: '<S6>/Sum'
   *  Sum: '<S6>/Sum1'
   */
  rtY.AAC2 = (rtDW.Delay_DSTATE_p * rtU.b1_Voltage2 + rtDW.Delay1_DSTATE_i *
              rtU.b2_Voltage2) + rtb_Sum3_o * rtU.b0_Voltage2;

  /* Delay: '<S1>/Delay' */
  rtb_Delay_l = rtDW.Delay_DSTATE_o;

  /* Sum: '<S1>/Sum3' incorporates:
   *  Delay: '<S1>/Delay'
   *  Delay: '<S1>/Delay1'
   *  Inport: '<Root>/Mic2'
   *  Inport: '<Root>/ma1_MHHC1'
   *  Inport: '<Root>/ma2_MHHC1'
   *  Product: '<S1>/Product'
   *  Product: '<S1>/Product2'
   *  Sum: '<S1>/Sum2'
   */
  rtb_Sum3_h = (rtDW.Delay_DSTATE_o * rtU.ma1_MHHC1 + rtDW.Delay1_DSTATE_f *
                rtU.ma2_MHHC1) + rtU.Mic2;

  /* Delay: '<S2>/Delay' */
  rtb_Delay_j = rtDW.Delay_DSTATE_c4;

  /* Sum: '<S2>/Sum3' incorporates:
   *  Delay: '<S2>/Delay'
   *  Delay: '<S2>/Delay1'
   *  Inport: '<Root>/Mic2'
   *  Inport: '<Root>/ma1_MHHC2'
   *  Inport: '<Root>/ma2_MHHC2'
   *  Product: '<S2>/Product'
   *  Product: '<S2>/Product2'
   *  Sum: '<S2>/Sum2'
   */
  rtb_Sum3_c = (rtDW.Delay_DSTATE_c4 * rtU.ma1_MHHC2 + rtDW.Delay1_DSTATE_a *
                rtU.ma2_MHHC2) + rtU.Mic2;

  /* Outport: '<Root>/ANC' incorporates:
   *  Delay: '<S1>/Delay'
   *  Delay: '<S1>/Delay1'
   *  Delay: '<S2>/Delay'
   *  Delay: '<S2>/Delay1'
   *  Inport: '<Root>/Gain_BPF1'
   *  Inport: '<Root>/Gain_BPF2'
   *  Inport: '<Root>/b0_MHHC1'
   *  Inport: '<Root>/b0_MHHC2'
   *  Inport: '<Root>/b1_MHHC1'
   *  Inport: '<Root>/b1_MHHC2'
   *  Inport: '<Root>/b2_MHHC1'
   *  Inport: '<Root>/b2_MHHC2'
   *  Product: '<Root>/Product4'
   *  Product: '<Root>/Product5'
   *  Product: '<S1>/Product3'
   *  Product: '<S1>/Product4'
   *  Product: '<S1>/Product5'
   *  Product: '<S2>/Product3'
   *  Product: '<S2>/Product4'
   *  Product: '<S2>/Product5'
   *  Sum: '<Root>/Sum4'
   *  Sum: '<S1>/Sum'
   *  Sum: '<S1>/Sum1'
   *  Sum: '<S2>/Sum'
   *  Sum: '<S2>/Sum1'
   */
  rtY.ANC = ((rtDW.Delay_DSTATE_o * rtU.b1_MHHC1 + rtDW.Delay1_DSTATE_f *
              rtU.b2_MHHC1) + rtb_Sum3_h * rtU.b0_MHHC1) * rtU.Gain_BPF1 +
    ((rtDW.Delay_DSTATE_c4 * rtU.b1_MHHC2 + rtDW.Delay1_DSTATE_a * rtU.b2_MHHC2)
     + rtb_Sum3_c * rtU.b0_MHHC2) * rtU.Gain_BPF2;

  /* Update for Delay: '<S3>/Delay' */
  rtDW.Delay_DSTATE = rtb_Sum3;

  /* Update for Delay: '<S3>/Delay1' */
  rtDW.Delay1_DSTATE = rtb_Delay;

  /* Update for Delay: '<S5>/Delay' */
  rtDW.Delay_DSTATE_c = rtb_Sum3_n;

  /* Update for Delay: '<S5>/Delay1' */
  rtDW.Delay1_DSTATE_e = rtb_Delay_d;

  /* Update for Delay: '<S4>/Delay' */
  rtDW.Delay_DSTATE_d = rtb_Sum3_e;

  /* Update for Delay: '<S4>/Delay1' */
  rtDW.Delay1_DSTATE_j = rtb_Delay_cr;

  /* Update for Delay: '<S6>/Delay' */
  rtDW.Delay_DSTATE_p = rtb_Sum3_o;

  /* Update for Delay: '<S6>/Delay1' */
  rtDW.Delay1_DSTATE_i = rtb_Delay_c;

  /* Update for Delay: '<S1>/Delay' */
  rtDW.Delay_DSTATE_o = rtb_Sum3_h;

  /* Update for Delay: '<S1>/Delay1' */
  rtDW.Delay1_DSTATE_f = rtb_Delay_l;

  /* Update for Delay: '<S2>/Delay' */
  rtDW.Delay_DSTATE_c4 = rtb_Sum3_c;

  /* Update for Delay: '<S2>/Delay1' */
  rtDW.Delay1_DSTATE_a = rtb_Delay_j;
}
