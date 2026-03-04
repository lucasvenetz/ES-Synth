#include <Arduino.h>
#include <U8g2lib.h>
#include <bitset>
#include <STM32FreeRTOS.h>
#include <ES_CAN.h>
#include "../include/helper.hpp"

// Global variables
volatile uint32_t currentStepSize;
HardwareTimer sampleTimer(TIM1);
struct {
  std::bitset<32> inputs;
  uint8_t RX_Message[8];
  SemaphoreHandle_t mutex;
} sysState;
Knob knob3(0, 8, 0); // lower, upper, initial value
Knob knob2(0, 3, 0);
const char* currentNote = "";  
QueueHandle_t msgInQ;
QueueHandle_t msgOutQ;
SemaphoreHandle_t CAN_TX_Semaphore;
bool isSender;
int8_t sineTable[256];

//Constants
  const uint32_t interval = 100; //Display update interval

  // equal temperament (12root2) with fs=22kHz
  constexpr uint32_t stepSizes [12] = {
    51076057,   // C
    54113197,   // C#
    57330935,   // D
    60740010,   // D#
    64351799,   // E
    68178356,   // F
    72232452,   // F#
    76527617,   // G
    81078186,   // G#
    85899346,   // A 440 Hz
    91007187,   // A#
    96418756    // B
  };

  const char* noteNames[12] = {
    "C", "C#", "D", "D#", "E", "F",
    "F#", "G", "G#", "A", "A#", "B"
  };

//Pin definitions
  //Row select and enable
  const int RA0_PIN = D3;
  const int RA1_PIN = D6;
  const int RA2_PIN = D12;
  const int REN_PIN = A5;

  //Matrix input and output
  const int C0_PIN = A2;
  const int C1_PIN = D9;
  const int C2_PIN = A6;
  const int C3_PIN = D1;
  const int OUT_PIN = D11;

  //Audio analogue out
  const int OUTL_PIN = A4;
  const int OUTR_PIN = A3;

  //Joystick analogue in
  const int JOYY_PIN = A0;
  const int JOYX_PIN = A1;

  //Output multiplexer bits
  const int KNOB_MODE = 2;
  const int DEN_BIT = 3;
  const int DRST_BIT = 4;
  const int HKOW_BIT = 5;
  const int HKOE_BIT = 6;

//Display driver object
U8G2_SSD1305_128X32_ADAFRUIT_F_HW_I2C u8g2(U8G2_R0);

//Function to set outputs using key matrix
void setOutMuxBit(const uint8_t bitIdx, const bool value) {
      digitalWrite(REN_PIN,LOW);
      digitalWrite(RA0_PIN, bitIdx & 0x01);
      digitalWrite(RA1_PIN, bitIdx & 0x02);
      digitalWrite(RA2_PIN, bitIdx & 0x04);
      digitalWrite(OUT_PIN,value);
      digitalWrite(REN_PIN,HIGH);
      delayMicroseconds(2);
      digitalWrite(REN_PIN,LOW);
}

void autoConfig() {
  bool leftBoardExists = digitalRead(HKOW_BIT);
  if (!leftBoardExists) {
      isSender = true;
      setOutMuxBit(HKOE_BIT, HIGH);
  } else {
      isSender = false;
    }
}

void sampleISR(){
  static uint32_t phaseAcc = 0;
  phaseAcc += currentStepSize;
  int32_t Vout;
  int32_t waveform = knob2.getAtomicRotation();

  switch (waveform) {
    case 0: // sawtooth
      Vout = (phaseAcc >> 24) - 128;
      break;
    case 1: // square
      Vout = (phaseAcc >> 31) ? 127 : -128;
      break;
    case 2: // triangle
      Vout = (phaseAcc >> 23) - 256;
      if(Vout > 127) Vout = 256 - Vout;
      break;
    case 3: // sine
      Vout = sineTable[phaseAcc >> 24];
      break;
  }

  int32_t localVol = knob3.getAtomicRotation();
  Vout = Vout >> (8 - localVol);
  analogWrite(OUTR_PIN, Vout + 128);
}

void CAN_RX_ISR (void) {
  uint8_t RX_Message_ISR[8];
	uint32_t ID;
	CAN_RX(ID, RX_Message_ISR);
	xQueueSendFromISR(msgInQ, RX_Message_ISR, NULL);
}

void CAN_TX_ISR (void) {
	xSemaphoreGiveFromISR(CAN_TX_Semaphore, NULL);
}

std::bitset<4> readCols(){
  std::bitset<4> result;
  result[0] = digitalRead(C0_PIN);
  result[1] = digitalRead(C1_PIN);
  result[2] = digitalRead(C2_PIN);
  result[3] = digitalRead(C3_PIN);
  return result;
}

void setRow(uint8_t rowIdx){
  digitalWrite(REN_PIN,LOW);
  digitalWrite(RA0_PIN, rowIdx & 0x01);
  digitalWrite(RA1_PIN, rowIdx & 0x02);
  digitalWrite(RA2_PIN, rowIdx & 0x04); 
  digitalWrite(REN_PIN,HIGH);
}

void scanKeysTask(void * pvParameters) {
  const TickType_t xFrequency = 20/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  uint8_t TX_Message[8] = {0};
  std::bitset<12> prevKeyState;
  prevKeyState.set();

  while (1) {
    vTaskDelayUntil( &xLastWakeTime, xFrequency );

    for (int row=0; row<4; row++) {
      setRow(row);
      delayMicroseconds(3);
      std::bitset<4> col_out = readCols();
      xSemaphoreTake(sysState.mutex, portMAX_DELAY);
      for (int i=0; i<4; i++){
        sysState.inputs[row*4 + i] = col_out[i];
      }
      xSemaphoreGive(sysState.mutex);
    }

    // Decoding knob3 rotation
    xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    bool A3 = sysState.inputs[12];
    bool B3 = sysState.inputs[13];
    bool A2 = sysState.inputs[14];
    bool B2 = sysState.inputs[15];
    xSemaphoreGive(sysState.mutex);
    knob3.updateRotation(A3,B3);
    knob2.updateRotation(A2,B2);

    uint32_t localCurrentStepSize = 0;
    const char* localCurrentNote = ""; 
    for (int key=0; key<12; key++) {
      xSemaphoreTake(sysState.mutex, portMAX_DELAY);
      bool key_pressed = !sysState.inputs[key];
      xSemaphoreGive(sysState.mutex);
      bool stateChange = 0;

      if(!prevKeyState[key] && key_pressed) { /* If prev state is unpressed and pressed now true*/
        TX_Message[0] = 0x50;
        TX_Message[1] = 4;
        TX_Message[2] = key;
        localCurrentStepSize = stepSizes[key];
        localCurrentNote = noteNames[key];
        stateChange = 1;
      } else if(prevKeyState[key] && !key_pressed) { /* key released */
        TX_Message[0] = 0x52;
        TX_Message[1] = 4;
        TX_Message[2] = key;
        stateChange = 1;
      } else if (key_pressed) { /* key held */
        localCurrentStepSize = stepSizes[key];
        localCurrentNote = noteNames[key];
        stateChange = 1;
      }
      if(stateChange) {
        if (isSender) {
          xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
        } else {
          __atomic_store_n(&currentStepSize, localCurrentStepSize, __ATOMIC_RELAXED);
          __atomic_store_n(&currentNote, noteNames[key], __ATOMIC_RELAXED);
        }
      }
      
      prevKeyState[key] = key_pressed; 
    }
  }
}

void updateDisplayTask(void * pvParameters) {
  const TickType_t xFrequency = 100/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  uint32_t ID;
  uint8_t localRX[8] = {0};
  const char* waveNames[4] = {"Saw", "Square", "Triangle", "Sine"};

  while (1) {
    vTaskDelayUntil( &xLastWakeTime, xFrequency );

    u8g2.clearBuffer();         // clear the internal memory
    u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
    //u8g2.drawStr(0,10,"Helllo World!");  // write something to the internal memory
    
    xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    uint32_t inputSnapshot = sysState.inputs.to_ulong();
    memcpy(localRX, sysState.RX_Message, 8);
    xSemaphoreGive(sysState.mutex);

    u8g2.setCursor(2,20);
    u8g2.print(inputSnapshot, HEX);
    u8g2.setCursor(2,30);
    const char* localCurrentnote = __atomic_load_n(&currentNote, __ATOMIC_RELAXED);
    u8g2.print(localCurrentnote);

    // Volume knob
    u8g2.setCursor(66, 20);
    u8g2.print("Vol: ");
    u8g2.print(knob3.getRotation());

    // waveform knob
    u8g2.setCursor(0, 10);
    u8g2.print("Wave: ");
    u8g2.print(waveNames[knob2.getRotation()]);

    u8g2.setCursor(66,30);
    u8g2.print((char) localRX[0]);
    u8g2.print(localRX[1]);
    u8g2.print(localRX[2]);
    u8g2.sendBuffer();          // transfer internal memory to the display
    digitalToggle(LED_BUILTIN);
  }
}

void decodeTask(void * pvParameters) {
  uint8_t localMessage[8];
  while(1) {
    xQueueReceive(msgInQ, localMessage, portMAX_DELAY);
    if (localMessage[0] == 0x52) {
      __atomic_store_n(&currentStepSize, 0, __ATOMIC_RELAXED);
    } else if (localMessage[0] == 0x50) {
      uint8_t octave = localMessage[1];
      uint8_t note = localMessage[2];
      uint32_t step = stepSizes[note];
      // current board is at 4th octave
      if (octave >= 4) {
        step = step << (octave - 4);
      } else {
        step = step >> (4 - octave);
      }
      __atomic_store_n(&currentStepSize, step, __ATOMIC_RELAXED);
    }

    xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    memcpy(sysState.RX_Message, localMessage, 8);
    xSemaphoreGive(sysState.mutex);
  }
}

void CAN_TX_Task (void * pvParameters) {
  uint8_t msgOut[8];
  while (1) {
		xQueueReceive(msgOutQ, msgOut, portMAX_DELAY);
		xSemaphoreTake(CAN_TX_Semaphore, portMAX_DELAY);
		CAN_TX(0x123, msgOut);
	}
}

void setup() {
  for(int i = 0; i < 256; i++){
    sineTable[i] = (int8_t)(127.0f * sin(2.0f * PI * i / 256.0f)); 
  }

  //Set pin directions
  pinMode(RA0_PIN, OUTPUT);
  pinMode(RA1_PIN, OUTPUT);
  pinMode(RA2_PIN, OUTPUT);
  pinMode(REN_PIN, OUTPUT);
  pinMode(OUT_PIN, OUTPUT);
  pinMode(OUTL_PIN, OUTPUT);
  pinMode(OUTR_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(C0_PIN, INPUT);
  pinMode(C1_PIN, INPUT);
  pinMode(C2_PIN, INPUT);
  pinMode(C3_PIN, INPUT);
  pinMode(JOYX_PIN, INPUT);
  pinMode(JOYY_PIN, INPUT);

  //Initialise display
  setOutMuxBit(DRST_BIT, LOW);  //Assert display logic reset
  delayMicroseconds(2);
  setOutMuxBit(DRST_BIT, HIGH);  //Release display logic reset
  u8g2.begin();
  setOutMuxBit(DEN_BIT, HIGH);  //Enable display power supply
  setOutMuxBit(KNOB_MODE, HIGH);  //Read knobs through key matrix

  //Initialise UART
  Serial.begin(9600);
  Serial.println("Hello World");

  // Change Init to false if attaching second keyboard
  CAN_Init(true);
  setCANFilter(0x123,0x7ff);
  #ifndef DISABLE_ISR
    CAN_RegisterRX_ISR(CAN_RX_ISR);
    CAN_RegisterTX_ISR(CAN_TX_ISR);
    CAN_Start();
  #endif

  msgInQ = xQueueCreate(36,8);
  msgOutQ = xQueueCreate(36,8);
  CAN_TX_Semaphore = xSemaphoreCreateCounting(3,3);

  sampleTimer.setOverflow(22000, HERTZ_FORMAT);
  #ifndef DISABLE_ISR
    sampleTimer.attachInterrupt(sampleISR);
  #endif
  sampleTimer.resume();

  autoConfig();
  sysState.mutex = xSemaphoreCreateMutex();
  knob3.initMutex();
  knob2.initMutex();

  #ifndef DISABLE_THREADS
    TaskHandle_t scanKeysHandle = NULL;
    xTaskCreate(
      scanKeysTask,		/* Function that implements the task */
      "scanKeys",		  /* Text name for the task */
      64,      		    /* Stack size in words, not bytes */
      NULL,			      /* Parameter passed into the task */
      2,			        /* Task priority */
      &scanKeysHandle /* Pointer to store the task handle */
    );

    TaskHandle_t updateDisplayHandle = NULL;
    xTaskCreate(
      updateDisplayTask, "updateDisplay", 256,
      NULL, 1, &updateDisplayHandle
    );

    TaskHandle_t decodeHandle = NULL;
    xTaskCreate(
      decodeTask, "decode", 64,
      NULL, 3, &decodeHandle
    );

    TaskHandle_t CAN_TX_Handle = NULL;
    xTaskCreate(
      CAN_TX_Task, "CAN_TX", 64,
      NULL, 3, &CAN_TX_Handle
    );
  #endif

  vTaskStartScheduler();
}

void loop() {
  return; 
}