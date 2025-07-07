/*
  인텔7기 AI+SW 아카데미
  김민우, 김민수
  관리실 중앙 통제 시스템
  (RFID, Bluetooth, LCD, OLED)
*/
//rfid 라이브러리 : MFRC522 by GithubCommunity
/*
 연결
 -------------------------------------------------
 Arduino UNO RFID-RC522
 -------------------------------------------------
 10 SPI SS SDA
 13 SPI SCK SCK
 11 SPI MOSI MOSI
 12 SPI MISO MISO
 9 SPI RESET RST 
 3.3V
 GND
 -------------------------------------------------
 Arduino UNO Bluetooth Module
 -------------------------------------------------
 7 RXD
 8 TXD
 5V
 GND
  -------------------------------------------------
 Arduino UNO LCD
 -------------------------------------------------
 SDA SDA 우측 맨 위에서 두번 째 핀
 SCL SCL 우측 맨 위에서 첫번 째 핀
 5V
 GND
  -------------------------------------------------
 Arduino UNO OLED
 -------------------------------------------------
*/
//include
#include <SoftwareSerial.h>
#include <Wire.h>
#include <SPI.h>
#include <MFRC522.h>
#include <MsTimer2.h>
#include <LiquidCrystal_I2C.h>

// define
#define DEBUG // 디버그용
#define LED_BUILTIN_PIN 13 // 빌트인 LED
#define BT_RXD 8 //블루투스 모듈 TXD -> 8핀
#define BT_TXD 7 //블루투스 모듈 RXD -> 7핀
#define RST_PIN 9 // 
#define SS_PIN 10 //

#define ARR_CNT 6
#define CMD_SIZE 60

// variables
char UID[20];
char officeName[10] = "KMW_SQL";
char officeNameInv[10] = "";
char lcd1Line1[17] = "Scan ID CARD";
char lcd1Line2[17] = "";
char lcd2Line1[17] = "KMW_SQL"; // 오피스 이름
char lcd2Line2[17] = "total member: 0";
char sendBuf[CMD_SIZE];
char recvId[10] = "KMW_SQL";  // SQL 저장 클라이이언트 ID
int total_member = 0; // 전체 멤버 수 카운트
int Fire_flag = 0; // 화재 플래그
int Eq_flag = 0; // 지진 플래그
int Inv_flag = 0; // 침입 플래그

LiquidCrystal_I2C lcd1(0x27, 16, 2); // 인원 출입 LCD
LiquidCrystal_I2C lcd2(0x26, 16, 2); // 재난 상황 LCD
MFRC522 mfrc522(SS_PIN, RST_PIN); // Create MFRC522 instance
SoftwareSerial BTSerial(BT_RXD, BT_TXD); // bt모듈 TXD -> 8, RXD -> 7

void setup() {
#ifdef DEBUG
  Serial.begin(115200);
  Serial.println("setup() start!");
  Serial.println(F("Scan PICC to see UID, type, and data blocks..."));
#endif
  BTSerial.begin(9600); // set the data rate for the SoftwareSerial port
  lcd1.init();
  lcd1.backlight();
  lcd1Display(0, 0, lcd1Line1);
  lcd1Display(0, 1, lcd1Line2);
  lcd2.init();
  lcd2.backlight();
  lcd2Display(0, 0, lcd2Line1);
  lcd2Display(0, 1, lcd2Line2);  
  SPI.begin(); // Init SPI bus
  mfrc522.PCD_Init(); // Init MFRC522
  pinMode(LED_BUILTIN_PIN, OUTPUT);

  BTSerial.write("[KMW_SQL]SETDB@INIT@STATE\n");
}

void loop() {
  if (BTSerial.available())
    bluetoothEvent();
  if (!Fire_flag && !Eq_flag && !Inv_flag) // 재난 상황 종료 (평상시)
  {
    sprintf(lcd2Line1, "KMW_SQL");
    sprintf(lcd2Line2, "total member: %d", total_member);
    lcd2Display(0, 0, lcd2Line1);
    lcd2Display(0, 1, lcd2Line2);
    // RFID 읽기    
    if ( ! mfrc522.PICC_IsNewCardPresent()) {
      return;
    }
    if ( ! mfrc522.PICC_ReadCardSerial()) {
      return;
    }
    // 읽은 카드의 고유 UID 값 저장
    Serial.print("UID tag : ");
    sprintf(UID, "%d %d %d %d",mfrc522.uid.uidByte[0], mfrc522.uid.uidByte[1], 
                               mfrc522.uid.uidByte[2], mfrc522.uid.uidByte[3]);
    Serial.println(UID);
    lcd1Display(0, 1, UID);
    sprintf(sendBuf, "[KMW_SQL]GETDB@%s\n", UID); 
    BTSerial.write(sendBuf); // 데이터베이스에 UID확인 명령보내기
  }
  else if (Fire_flag && !Eq_flag) // 재난 상황 발생 (화재)
  {
    sprintf(lcd2Line1, "%s ON FIRE", officeName);
    sprintf(lcd2Line2, "call 119 & check");
    lcd2Display(0, 0, lcd2Line1);
    lcd2Display(0, 1, lcd2Line2);       
  }
  else if (Eq_flag && !Fire_flag) // 재난 상황 발생 (지진)
  {
    sprintf(lcd2Line1, "EARTH QUAKE!");
    sprintf(lcd2Line2, "EVACUATION!");
    lcd2Display(0, 0, lcd2Line1);
    lcd2Display(0, 1, lcd2Line2);
  }
  else if (Eq_flag && Fire_flag) // 재난 상황 발생 (지진 & 화재)
  {
    sprintf(lcd2Line1, "%s ON FIRE", officeName);
    sprintf(lcd2Line2, "EARTH QUAKE!");
    lcd2Display(0, 0, lcd2Line1);
    lcd2Display(0, 1, lcd2Line2);
  }  
  else if (Inv_flag) // 외부인 침입 발생
  {
    sprintf(lcd2Line1, "%s INTRUDER", officeName);
    sprintf(lcd2Line2, "PLEASE CHECK");
    lcd2Display(0, 0, lcd2Line1);
    lcd2Display(0, 1, lcd2Line2);
    if ( ! mfrc522.PICC_IsNewCardPresent()) {
      return;
    }
    // Select one of the cards
    if ( ! mfrc522.PICC_ReadCardSerial()) {
      return;
    }
    Serial.print("UID tag : ");
    sprintf(UID, "%d %d %d %d",mfrc522.uid.uidByte[0], mfrc522.uid.uidByte[1], mfrc522.uid.uidByte[2], mfrc522.uid.uidByte[3]);
    Serial.println(UID);
    if (!strcmp(UID,"51 243 71 150"))
    {
      sprintf(sendBuf, "[KMS_STM]INV@OFF\n"); // 오피스에 문제 없음 확인 하고 침입자 없음 확인 메세지 보냄
      BTSerial.write(sendBuf);      
    }
    Inv_flag = 0;
  }  
}

/* 
  블루투스 이벤트 발생 함수
  기능 : 
  - RFID찍었을 때 등록된 멤버인지 아닌지 인식
  - 전체 인원 수 카운트
  - 재난 상황 발생 인식
*/
void bluetoothEvent()
{
  int i = 0;
  char * pToken;
  char * pArray[ARR_CNT] = {0};
  char recvBuf[CMD_SIZE] = {0};
  int len = BTSerial.readBytesUntil('\n', recvBuf, sizeof(recvBuf) - 1);

#ifdef DEBUG
  Serial.print("Recv : ");
  Serial.println(recvBuf);
#endif

  pToken = strtok(recvBuf, "[@]");
  while (pToken != NULL)
  {
    pArray[i] =  pToken;
    if (++i >= ARR_CNT)
      break;
    pToken = strtok(NULL, "[@]");
  }

  if (!strcmp(pArray[0], "KMW_ARD")) {
    return ;
  }

#ifdef DEBUG
  else if (!strcmp(pArray[1], "LED")) {
    if (!strcmp(pArray[2], "ON")) {
      digitalWrite(LED_BUILTIN_PIN, HIGH);
    }
    else if (!strcmp(pArray[2], "OFF")) {
      digitalWrite(LED_BUILTIN_PIN, LOW);
    }
    return;
  }
#endif

  else if (!strcmp(pArray[1], "GETDB")) { // 
    if (!strcmp(pArray[3], "NO")) {
      sprintf(lcd1Line2, "INVALID CARD!!");
      lcd1Display(0, 0, lcd1Line2);
    }
    else if (!strcmp(pArray[3], "YES")) {
      if (!strcmp(pArray[4], "0"))
      {
        Serial.println("hi");
        if (total_member == 0)
        {
          sprintf(sendBuf,"[KMS_STM]OFFI@ON\n");
          BTSerial.write(sendBuf);
          sprintf(sendBuf,"[KMW_STM]OFFI@ON\n");
          BTSerial.write(sendBuf);          
        }
        sprintf(lcd1Line1, "HI! GOODMORNING");
        sprintf(lcd1Line2, "%s", pArray[5]);        
        lcd1Display(0, 0, lcd1Line1);
        lcd1Display(0, 1, lcd1Line2);
        total_member++;
      }
      else if (!strcmp(pArray[4], "1"))
      {
        Serial.println("bye");
        if (total_member == 1)
        {
          sprintf(sendBuf,"[KMS_STM]OFFI@OFF\n");
          BTSerial.write(sendBuf);
          sprintf(sendBuf,"[KMW_STM]OFFI@OFF\n");
          BTSerial.write(sendBuf);          
        }             
        sprintf(lcd1Line1, "GOOD BYE!!");
        sprintf(lcd1Line2, "%s", pArray[5]);        
        lcd1Display(0, 0, lcd1Line1);
        lcd1Display(0, 1, lcd1Line2);
        total_member--;        
      }
    }
    sprintf(sendBuf, "[KMW_SQL]SETDB@TOTAL@%d\n",total_member); 
    BTSerial.write(sendBuf); // 총 인원수를 현황에 보냄
    sprintf(lcd2Line1, "%s", pArray[0]);
    sprintf(lcd2Line2, "total member: %d", total_member);
    lcd2Display(0, 0, lcd2Line1);
    lcd2Display(0, 1, lcd2Line2);    
    return;
  }
  else if (!strcmp(pArray[1], "DIS")) { // 재난 상황 발생 시
    if (!strcmp(pArray[0], "KMS_STM") || !strcmp(pArray[0], "KMW_STM"))
      strcpy(officeName, "KMW_SQL");
    if (!strcmp(pArray[2], "FIRE"))
    {
		  if(!strcmp(pArray[3], "ON"))
		  {
        total_member = 0;
        BTSerial.write("[KMW_SQL]SETDB@INIT@STATE\n");
			  Fire_flag = 1;
		  }
		  else if(!strcmp(pArray[3], "OFF"))
		  {
			  Fire_flag = 0;
		  }
    }
    else if (!strcmp(pArray[2], "EQ"))
    {
		  if(!strcmp(pArray[3], "ON"))
		  {
        BTSerial.write("[KMW_SQL]SETDB@INIT@STATE\n");
        total_member = 0;
			  Eq_flag = 1;
		  }
		  else if(!strcmp(pArray[3], "OFF"))
		  {
			  Eq_flag = 0;
		  }
    }    
    return;
  }  
  else if (!strcmp(pArray[1], "INV")) { // 외부인 침입 발생 시
    if (!strcmp(pArray[0], "KMS_STM"))
      strcpy(officeName, "KMW_SQL");
    Inv_flag = 1;
    return;
  }  
  else if (!strncmp(pArray[1], " New", 4)) // New Connected
  {
    return ;
  }
  else if (!strncmp(pArray[1], " Alr", 4)) //Already logged
  {
    return ;
  }
  else 
  { 
    return;
  }

#ifdef DEBUG
  Serial.print("Send : ");
  Serial.print(sendBuf);
#endif
  BTSerial.write(sendBuf);
}

/*
  LCD 디스플레이 함수
*/
void lcd1Display(int x, int y, char * str) // 인원 출입 LCD Display
{
  int len = 16 - strlen(str);
  lcd1.setCursor(x, y);
  lcd1.print(str);
  for (int i = len; i > 0; i--)
    lcd1.write(' ');
}
void lcd2Display(int x, int y, char * str) // 재난 상황 LCD Display
{
  int len = 16 - strlen(str);
  lcd2.setCursor(x, y);
  lcd2.print(str);
  for (int i = len; i > 0; i--)
    lcd2.write(' ');
}