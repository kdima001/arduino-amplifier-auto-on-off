#include <Streaming.h>
#include <avr/wdt.h>
#include "ACS712.h"

// Контроллер включения отключения аудиосборки по наличию/отсутсвия аудиосигнала
// слушает аналог на A1 и A2
// при обнаружении сигнала
// 	1. запускается цикл включения
//		включает канал 1 (фильтры/преды/ЦАП)
//		взводит таймер1 до включения канала 2 (усилители)
//		после обнуления таймера1 если сигнал есть переходит к включает канал2
//		взводит таймер2 (защита от выключения сразу)
//	при пропадании звука
//	запускаем цикл выключения (если таймер2 истёк)
//		выключаем канал 2
//		взводим таймер1
//		после обнуления таймера1 выключаем канал2
//		взводим таймер2 (защитный)
//
//Состояния:
//				нет звука								(светодио не светится)
//				есть звук, цикл включения				(светодиод медленно мигает)
//				есть звук, включено						(светится непрерывно)
//				нет звука, цикл выключения				(светодиод быстро мигает)


//индикаторы (исп. одноцветный диод)
// LED_BUILTIN

#define PRE 2  // нагрузка 1 - преды
#define AMP 3  // нагрузка 2 - усилители

//константы интервалов в мсек
#define POOL_INT 100

// порог тишины
#define MIN_LEVEL 175
#define HISTERESIL_LEVEL 5
// порог тока
#define MIN_CURRENT 100
#define HISTERESIL_CURRENT 50

// Переменные пуллинга
unsigned long curMillis;
unsigned long prevMillisLED = 0;
unsigned long prevMillis1 = 0;

//---------------------------------------------------------------------------
uint8_t SerialDBG, TimeON_RO;
int level, currentMillis;

ACS712 acSensor(ACS712_05B, A1);

//---------------------------------------------------------------------------
enum mSTATE_t {  //режим работы
  WAIT,          //слушаем и ждем (выключены все каналы)
  ON_01,         //включен 1 выход
  ON_11,         //включен 1+2 выход
  OFF_10,        //цикл выключения 2 выход отключен
  OFF_00         //цикл выключения 2+1 выходы отключены
};
mSTATE_t mSTATE;

void PrintStatus(void) {
    switch (mSTATE) {
    case WAIT: Serial << F("WAIT") << endl; break;
    case ON_01: Serial << F("ON_01") << endl; break;
    case ON_11: Serial << F("ON_11") << endl; break;
    case OFF_10: Serial << F("OFF_10") << endl; break;
    case OFF_00: Serial << F("OFF_00") << endl; break;
  };
}

//---------------------------------------------------------------------------
void (*Reset)(void) = 0;  // Reset CON function

//---------------------------------------------------------------------------
// длинная буфера усреднения
#define BUF_LENGTH 256
// буфер усреднения
int valuesBuffer[BUF_LENGTH];
int bufferPosition = 0;
void ProcessSensors(void) {
  if (bufferPosition >= BUF_LENGTH) {
    bufferPosition = 0;
  }
  //холостые чтения
  analogRead(A0);
  analogRead(A0);
  analogRead(A0);
  // реальный сигнал в кольцевой буфер
  valuesBuffer[bufferPosition] = analogRead(A0);
  bufferPosition++;

  uint32_t val = 0;
  for (int i = 0; i < BUF_LENGTH; i++) {
    val += valuesBuffer[i];
  }
  level = val / BUF_LENGTH;

  currentMillis = round(1000 * acSensor.getCurrentAC());
}

//---------------------------------------------------------------------------
struct LED_MODE_t {
  bool blink;
  uint16_t t1, t2;  // свечения, пауза
  uint8_t r, g, b;
};

enum LM_t {
  LM_WAIT,
  LM_ON_01,
  LM_ON_11,
  LM_OFF_10,
  LM_OFF_00
};
LM_t LED_MODE;

LED_MODE_t lm[5] = {
  { 0, 0, 0,      0, 0, 0 },        //LM_WAIT,    не горит
  { 1, 50, 500,  255, 0, 0 },  //LM_ON_01, 	редко мигает 0,5сек
  { 0, 0, 0,      255, 0, 0 },      //LM_ON_11, 	горит
  { 1, 50, 250,  255, 0, 0 },  //LM_OFF_10,  часто мигает
  { 1, 50, 50,  255, 0, 0 }   //LM_OFF_11,  очень часто мигает
};

//---------------------------------------------------------------------------
void setRGB(uint8_t p_Y, uint8_t p_G, uint8_t p_B) {
  //инвертнем ибо общий анод и 0 зажигает, а 255 гасит диод
  digitalWrite(LED_BUILTIN, p_Y);
}

//---------------------------------------------------------------------------
void ProcessLED() {
  if (lm[LED_MODE].blink) {
    //Если цикл режима светодиода кончился - ставим цикл в начало
    if (curMillis - prevMillisLED > lm[LED_MODE].t1 + lm[LED_MODE].t2)
      prevMillisLED = curMillis;

    if ((prevMillisLED < curMillis) && (curMillis <= (prevMillisLED + lm[LED_MODE].t1))) {  //фаза удержания яркости
      setRGB(lm[LED_MODE].r, lm[LED_MODE].g, lm[LED_MODE].b);
    } else {
      if (prevMillisLED + lm[LED_MODE].t1 < curMillis && curMillis <= prevMillisLED + lm[LED_MODE].t1 + lm[LED_MODE].t2) {  //фаза выключения
        setRGB(0, 0, 0);
      }
    }
  } else
    setRGB(lm[LED_MODE].r, lm[LED_MODE].g, lm[LED_MODE].b);
}

//---------------------------------------------------------------------------
void PrintSensors() {
  Serial << F("state:") << mSTATE << F(", ");
  Serial << F("led:") << LED_MODE << F(", ");
  Serial << F("current:") << currentMillis << F(", ");
  Serial << F("level:") << level;
  Serial << endl;
}
//---------------------------------------------------------------------------
bool isSensorLevelUp(void) {
  bool sound = level > MIN_LEVEL + HISTERESIL_LEVEL;
  bool current = false; //currentMillis > MIN_CURRENT + HISTERESIL_CURRENT;
  return sound || current;
}

bool isSensorLevelDown(void) {
  bool sound = level < MIN_LEVEL - HISTERESIL_LEVEL;
  bool current = true; //urrentMillis < MIN_CURRENT - HISTERESIL_CURRENT;
  return sound && current;
}

uint8_t checkCount = 0;
#define WAIT_CHECK_COUNT 200   // (2 сек)
#define ON_11_CHECK_COUNT 20  // (2 сек)
#define OFF_CHECK_COUNT 200   // (20 сек)
void ProcessState(void) {
  switch (mSTATE) {
    //********************
    case WAIT:
      if (isSensorLevelUp()) {
        checkCount++;
      } else {
        checkCount = 0;
      }
      if (checkCount >= WAIT_CHECK_COUNT) {
        checkCount = 0;
        digitalWrite(PRE, HIGH);
        mSTATE = ON_01;
      }
      break;
    //********************
    case ON_01:
      checkCount++;
      if (checkCount >= ON_11_CHECK_COUNT) {
        checkCount = 0;
        digitalWrite(AMP, HIGH);
        mSTATE = ON_11;
      }
      break;
    //********************
    case ON_11:
      if (isSensorLevelDown()) {
        checkCount++;
      } else {
        checkCount = 0;
      }
      if (checkCount >= WAIT_CHECK_COUNT) {
        digitalWrite(AMP, LOW);
        mSTATE = OFF_10;
        checkCount = 0;
      }
      break;
    //********************
    case OFF_10:
      checkCount++;
      if (checkCount >= OFF_CHECK_COUNT) {
        checkCount = 0;
        digitalWrite(PRE, LOW);
        mSTATE = OFF_00;
      }
      break;
    //********************
    case OFF_00:
      checkCount++;
      if (checkCount >= WAIT_CHECK_COUNT) {
        checkCount = 0;
        mSTATE = WAIT;
      }
      break;
  }
}

//---------------------------------------------------------------------------
String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete

void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    // Пришлось придумать символ-заменитель переводу строки - в кач-ве параметра в cmd \n передать не удалось
    if (inChar == '\n' || inChar == '@' || inChar == ';') {
      inputString.replace('@', '\n');
      inputString.replace(';', '\n');
      stringComplete = true;
    }
  }
}

//--------------------------------------------------------------------------
void SelfTest(void) {
  Serial << endl;
  ProcessSensors();

  PrintSensors();
  Serial << F("Test outputs:") << endl;

  wdt_reset();  // не забываем сбросить смотрящую собаку

  Serial << F("PRE channel ON...");
  digitalWrite(PRE, HIGH);
  delay(1000);
  Serial << F("OFF") << endl;
  digitalWrite(PRE, LOW);
  Serial << F("AMP channel ON...");
  digitalWrite(AMP, HIGH);
  delay(1000);
  Serial << F("OFF") << endl;
  digitalWrite(AMP, LOW);

  wdt_reset();  // не забываем сбросить смотрящую собаку

  Serial << F("Self test complete") << endl;
}

//---------------------------------------------------------------------------
void setup() {
  uint8_t tmpi;

  wdt_enable(WDTO_8S);

  pinMode(PRE, OUTPUT);
  digitalWrite(PRE, LOW);
  pinMode(AMP, OUTPUT);
  digitalWrite(AMP, LOW);  

  pinMode(LED_BUILTIN, OUTPUT);

  analogReference(DEFAULT);

  SerialDBG = 1;
  Serial.begin(115200);
  inputString.reserve(200);

  mSTATE = WAIT;

  LED_MODE = LM_WAIT;

  for (int i = 0; i < BUF_LENGTH; i++) {
    valuesBuffer[i] = MIN_LEVEL;
  }

  PrintStatus();
}

//--------------------------------------------------------------------------
void loop() {

  wdt_reset();  // не забываем сбросить смотрящую собаку

  ProcessLED();

  curMillis = millis();

  //проверим на переполнение
  if (prevMillis1 > curMillis || prevMillisLED > curMillis) {
    prevMillis1 = 0;   
    prevMillisLED = 0;
  }

  //Еже-действия
  if (curMillis - prevMillis1 > POOL_INT) {
    prevMillis1 = curMillis;

    ProcessSensors();

    ProcessState();

    // print the string when a newline arrives:
    if (stringComplete) {
      inputString.toLowerCase();
      inputString.trim();
      Serial << inputString << endl;
      if (inputString.equals(F("help"))) {
        Serial << F("comands:") << endl;
        Serial << F("diag") << endl;
        Serial << F("debug=on") << endl;
        Serial << F("debug=off") << endl;
      }
      if (inputString.equals(F("status")) || inputString.equals(F("diag"))) {
        PrintSensors();
      }
      if (inputString.equals(F("debug=on")) || inputString.equals(F("debug on"))) SerialDBG = 1;
      if (inputString.equals(F("debug=off")) || inputString.equals(F("debug off"))) SerialDBG = 0;
      // clear the string:
      inputString = "";
      stringComplete = false;
    }

    if (SerialDBG) PrintSensors();
  }

  switch (mSTATE) {
    case WAIT: LED_MODE = LM_WAIT; break;
    case ON_01: LED_MODE = LM_ON_01; break;
    case ON_11: LED_MODE = LM_ON_11; break;
    case OFF_10: LED_MODE = LM_OFF_10; break;
    case OFF_00: LED_MODE = LM_OFF_00; break;
  }
}
