
// Serial.available()는 루프를 돌때마다 줄어들어 최종 0이 된다.!!!
// 줄어들게 만드는 함수는 Serial.parseInt(), Serial.parseFloat(), Serial.read() 이다.
// 특히, parseFloat()는 한번에 여러개의 Byte를 뺀다.
// Serial.peak() 는 줄어들지 않음. 시리얼포트로부터 더 이상 읽어들일 데이터가 없으면 -1 리턴

#define VOLUMECOUNT 10
#define DATACOUNT 40
#define BAUDRATE 115200



static int volume = 0;
static float data[VOLUMECOUNT][DATACOUNT];


// -----------------------------------------------------
void setup() {
  Serial.begin(BAUDRATE);
  while (!Serial)
    ;
  Serial.println("Ready");
}
// -----------------------------------------------------


// -----------------------------------------------------
void loop() {
  int select = 0;
  // Set 또는 Get에 따라 함수 호출.
  Serial.println("선택하세요.");
  Serial.println("1)볼륨별파라미터입력     2)볼륨별파라미터출력     3) 전체파라미터출력");
  // wait for data input
  while (Serial.available() == 0) {
  }

  select = Serial.parseInt();

  if (select == 1) {
    SetVolumeData();
  } else if (select == 2) {
    GetVolumeData();
  } else if (select == 3) {
    printAllVolumeData();
  } else {
    Serial.println("잘못된 입력입니다.");
  }
}
// -----------------------------------------------------


// -----------------------------------------------------
void SetVolumeData() {

  int MaxDataCount = DATACOUNT;
  Serial.println("첫번째 숫자 1개는 볼륨, 나머지 40개는 ANC 파라미터임.(*구분자는 빈킨임)");
  Serial.println("예) 2 1.2 33 4444 0.5 0.6 0.7 0.8 0.9 1.0");

  // wait for data input
  while (Serial.available() == 0) {
  }



  if (Serial.available() > 0) {

    for (byte i = 0; i < MaxDataCount * 100; i++) {
      if (i == 0) {
        volume = Serial.parseInt();
        Serial.print("volume: ");
        Serial.println(volume);
        Serial.print("파라미터: ");

      } else if (Serial.peek() == -1) {
        break;
      } else {
        data[volume][i - 1] = Serial.parseFloat();

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
  Serial.println("볼륨: ");
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
void printAllVolumeData() {
  Serial.println("------볼륨별 파라미터(전체)-------------------- ");

  for (int i = 0; i < VOLUMECOUNT; i++) {
    Serial.print("볼륨 ");
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
