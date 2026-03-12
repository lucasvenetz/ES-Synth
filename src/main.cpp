#include <Arduino.h>
#include <U8g2lib.h>
#include <bitset>
#include <STM32FreeRTOS.h>
#include <ES_CAN.h>
#include <Wire.h>
#include "../include/helper.hpp"

#define MAX_KEYS 12

//Constants
  constexpr uint32_t FS = 22000;
  constexpr float LOOP_SECONDS = 2.0; //Recording duration
  constexpr uint32_t LOOP_MAX_SAMPLES = static_cast<uint32_t>(FS * LOOP_SECONDS);
  constexpr uint32_t RECORD_COUNTDOWN_SECONDS = 3;
  constexpr uint32_t RECORD_COUNTDOWN_SAMPLES = FS * RECORD_COUNTDOWN_SECONDS;
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

// Global variables
  volatile int8_t loopBuffer[LOOP_MAX_SAMPLES] = {0};
  volatile uint32_t g_recIdx = 0;
  volatile uint32_t g_playIdx = 0;
  volatile uint32_t g_loopLength = 0;
  volatile uint32_t currentStepSizes[MAX_KEYS] = {0}; // Local key scan
  volatile uint32_t remoteStepSizes[MAX_KEYS] = {0}; // Remote CAN key scan
  HardwareTimer sampleTimer(TIM1);
  struct {
    std::bitset<32> inputs;
    uint8_t RX_Message[8];
    SemaphoreHandle_t mutex;
  } sysState;
  Knob knob3(0, 8, 4); // volume control (lower, upper, initial value)
  Knob knob2(0, 3, 0); // waveform control
  Knob knob1(0, 8, 4); // octave control
  Knob knob0(0, 2, 0); // sysMode control (3 modes)
  const char* currentNote = "";
  QueueHandle_t msgInQ;
  QueueHandle_t msgOutQ;
  SemaphoreHandle_t CAN_TX_Semaphore;
  SemaphoreHandle_t i2cMutex;
  SemaphoreHandle_t knobSemaphore;
  bool isSender;
  int8_t sineTable[256];
  enum class sysMode : uint8_t {
    LIVE = 0,
    RECORD = 1,
    PLAYBACK = 2
  };
  volatile uint8_t g_sysMode = static_cast<uint8_t>(sysMode::LIVE);
  volatile uint32_t g_recordCountdownSamples = 0;

  // ATTACK_RATE  : how fast amplitude rises
  // DECAY_RATE   : how fast amplitude falls
  // SUSTAIN_LEVEL: amplitude held while key down after decay
  // RELEASE_RATE : how fast amplitude falls after key release
  constexpr uint16_t FP_ONE       = 4096;
  constexpr uint16_t ATTACK_INC   = max(1u, (unsigned)(FP_ONE / (0.01f  * FS))); // per sample
  constexpr uint32_t DECAY_PERIOD = (uint32_t)((5.0f * FS) / (FP_ONE * (1.0f - 0.1f)));
  constexpr uint16_t SUSTAIN_FP   = (uint16_t)(0.3f * FP_ONE);  // = 410
  constexpr uint16_t RELEASE_DEC  = max(1u, (unsigned)(FP_ONE / (0.3f   * FS)));

  volatile uint16_t envAmp[MAX_KEYS] = {0};
  volatile uint16_t remoteEnvAmp[MAX_KEYS] = {0};
  volatile uint8_t envStage[MAX_KEYS] = {0};
  volatile uint8_t remoteEnvStage[MAX_KEYS] = {0};

  enum class EnvStage : uint8_t { IDLE, ATTACK, DECAY, SUSTAIN, RELEASE };

//Display driver object
U8G2_SSD1305_128X32_ADAFRUIT_F_HW_I2C u8g2(U8G2_R0);

inline uint16_t stepEnvelope(volatile uint16_t& amp, volatile uint8_t& stage) {
  EnvStage s = static_cast<EnvStage>(stage);
  switch (s) {
    case EnvStage::ATTACK:
      if (amp + ATTACK_INC >= FP_ONE) {
        amp = FP_ONE;
        stage = static_cast<uint8_t>(EnvStage::DECAY);
      } else {
        amp += ATTACK_INC;
      }
      break;
    case EnvStage::DECAY: {
      static uint32_t decayCounter = 0;
      if (++decayCounter >= DECAY_PERIOD) {
        decayCounter = 0;
        if (amp <= SUSTAIN_FP + 1) {
          amp = SUSTAIN_FP;
          stage = static_cast<uint8_t>(EnvStage::SUSTAIN);
        } else {
          amp -= 1;
        }
      }
      break;
    }
    case EnvStage::SUSTAIN:
      break;
    case EnvStage::RELEASE:
      if (amp <= RELEASE_DEC) {
        amp = 0;
        stage = static_cast<uint8_t>(EnvStage::IDLE);
      } else {
        amp -= RELEASE_DEC;
      }
      break;
    default:
      break;
  }
  return amp;
}

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

inline sysMode getSysMode() {
  return static_cast<sysMode>(__atomic_load_n(&g_sysMode, __ATOMIC_RELAXED));
}

inline void setSysMode(sysMode m) {
  __atomic_store_n(&g_sysMode, static_cast<uint8_t>(m), __ATOMIC_RELAXED);
}

const char* loopModeToText(sysMode m) {
  switch (m) {
    case sysMode::LIVE:     return "LIVE"; //LIVE
    case sysMode::RECORD:   return "RECORDING"; //RECORD
    case sysMode::PLAYBACK: return "PLAYBACK"; //PLAYBACK
    default:                 return "?";
  }
}

void sampleISR(){
  static uint32_t localPhaseAcc[MAX_KEYS] = {0};
  static uint32_t remotePhaseAcc[MAX_KEYS] = {0};
  static uint32_t localLastStep[MAX_KEYS] = {0};
  static uint32_t remoteLastStep[MAX_KEYS] = {0};
  int32_t Vout = 0;
  int32_t waveform = knob2.getAtomicRotation();
  int32_t octave = knob1.getAtomicRotation();
  int32_t activeKeyCount = 0;

  auto sampleFromPhase = [&](uint32_t phase) -> int32_t {
    switch (waveform) {
      case 0: return (phase >> 24) - 128;                 // saw
      case 1: return (phase >> 31) ? 127 : -128;          // square
      case 2: { int32_t t = (phase >> 23) - 256;          // triangle
                if (t > 127) t = 256 - t;
                return t; }
      default: return sineTable[phase >> 24];             // sine
    }
  };

  for (int i = 0; i < MAX_KEYS; i++) {
    uint32_t remoteStepSize = __atomic_load_n(&remoteStepSizes[i], __ATOMIC_RELAXED);
    uint32_t localStepSize = __atomic_load_n(&currentStepSizes[i], __ATOMIC_RELAXED);
    localStepSize = (octave >= 4) ? localStepSize << (octave - 4) : localStepSize >> (4 - octave);

    uint16_t localGain = stepEnvelope(envAmp[i], envStage[i]);
    EnvStage localStage = static_cast<EnvStage>(envStage[i]);

    // Cache the step size while the key is held
    if (localStepSize) localLastStep[i] = localStepSize;

    // Use cached step during release so the oscillator keeps running
    uint32_t effectiveLocalStep = localStepSize;
    if (localStage == EnvStage::RELEASE) effectiveLocalStep = localLastStep[i];

    if (effectiveLocalStep || localStage != EnvStage::IDLE) {
      localPhaseAcc[i] += effectiveLocalStep;
      int32_t sample = sampleFromPhase(localPhaseAcc[i]);
      Vout += (sample * localGain) >> 12;
    } else {
      localPhaseAcc[i] = 0;
      localLastStep[i] = 0;
    }

    uint16_t remoteGain = stepEnvelope(remoteEnvAmp[i], remoteEnvStage[i]);
    EnvStage remoteStage = static_cast<EnvStage>(remoteEnvStage[i]);

    if (remoteStepSize) remoteLastStep[i] = remoteStepSize;
    uint32_t effectiveRemoteStep = remoteStepSize;
    if (remoteStage == EnvStage::RELEASE) effectiveRemoteStep = remoteLastStep[i];

    if (effectiveRemoteStep || remoteStage != EnvStage::IDLE) {
      remotePhaseAcc[i] += effectiveRemoteStep;
      int32_t sample = sampleFromPhase(remotePhaseAcc[i]);
      Vout += (sample * remoteGain) >> 12;
    } else {
      remotePhaseAcc[i] = 0;
      remoteLastStep[i] = 0;
    }
  }

  static sysMode prevMode = sysMode::LIVE;
  sysMode mode = getSysMode();

  if (mode != prevMode) {
    if (mode == sysMode::RECORD) {
      __atomic_store_n(&g_recIdx, 0, __ATOMIC_RELAXED);
      __atomic_store_n(&g_playIdx, 0, __ATOMIC_RELAXED);
      __atomic_store_n(&g_loopLength, 0, __ATOMIC_RELAXED);
      __atomic_store_n(&g_recordCountdownSamples, RECORD_COUNTDOWN_SAMPLES, __ATOMIC_RELAXED);
    } else if (mode == sysMode::PLAYBACK) {
      __atomic_store_n(&g_playIdx, 0, __ATOMIC_RELAXED);
      __atomic_store_n(&g_recordCountdownSamples, 0, __ATOMIC_RELAXED);
    } else {
    __atomic_store_n(&g_recordCountdownSamples, 0, __ATOMIC_RELAXED);
    }
    prevMode = mode;
  }

  uint32_t cd = __atomic_load_n(&g_recordCountdownSamples, __ATOMIC_RELAXED);
  uint32_t localRecIdx = __atomic_load_n(&g_recIdx, __ATOMIC_RELAXED);
  uint32_t localLoopLength = __atomic_load_n(&g_loopLength, __ATOMIC_RELAXED);
  uint32_t localplayIdx = __atomic_load_n(&g_playIdx, __ATOMIC_RELAXED);
  if (mode == sysMode::RECORD) {
    if (cd > 0) { // Still waiting for countdown
      __atomic_store_n(&g_recordCountdownSamples, cd - 1, __ATOMIC_RELAXED); // Do not write to loopBuffer yet
    } else if (localRecIdx < LOOP_MAX_SAMPLES) { // Start recording
      Vout = constrain(Vout, -128, 127);
      loopBuffer[localRecIdx++] = static_cast<int8_t>(Vout);
      __atomic_store_n(&g_recIdx, localRecIdx, __ATOMIC_RELAXED);
      __atomic_store_n(&g_loopLength, localRecIdx, __ATOMIC_RELAXED);
    }
  } else if (mode == sysMode::PLAYBACK) {
    if (localLoopLength > 0) {
      Vout = loopBuffer[localplayIdx++];
      if (localplayIdx >= localLoopLength) localplayIdx = 0;
      __atomic_store_n(&g_playIdx, localplayIdx, __ATOMIC_RELAXED);
    } else {
      Vout = 0;
    }
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
  if (rowIdx == 2)
    digitalWrite(OUT_PIN, LOW);
  else
    digitalWrite(OUT_PIN, HIGH);
  digitalWrite(REN_PIN, HIGH);
}

void knobISR() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(knobSemaphore, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void knobTask(void * pvParameters) {
  while (1) {
    xSemaphoreTake(knobSemaphore, portMAX_DELAY);
    xSemaphoreTake(i2cMutex, portMAX_DELAY);

    Wire.beginTransmission(0x21);
    Wire.write(0x00);
    Wire.endTransmission();
    Wire.requestFrom(0x21, (uint8_t)1);
    uint8_t pcalData = 0xFF;
    while (Wire.available()) pcalData = Wire.read();

    xSemaphoreGive(i2cMutex);

    // 4. Extract individual A/B signals
    // PCAL pin assignment:
    //   P0 = Knob3A, P1 = Knob3B
    //   P2 = Knob2A, P3 = Knob2B
    //   P4 = Knob1A, P5 = Knob1B
    //   P6 = Knob0A, P7 = Knob0B
    bool k3A = (pcalData >> 0) & 1;
    bool k3B = (pcalData >> 1) & 1;
    bool k2A = (pcalData >> 2) & 1;
    bool k2B = (pcalData >> 3) & 1;
    bool k1A = (pcalData >> 4) & 1;
    bool k1B = (pcalData >> 5) & 1;
    bool k0A = (pcalData >> 6) & 1;
    bool k0B = (pcalData >> 7) & 1;

    knob3.updateRotation(k3A, k3B);
    knob2.updateRotation(k2A, k2B);
    knob1.updateRotation(k1A, k1B);
    knob0.updateRotation(k0A, k0B);

    static int32_t prevMode = -1;
    int32_t modeIdx = knob0.getRotation();
    if (modeIdx != prevMode) {
      setSysMode(static_cast<sysMode>(modeIdx));
      prevMode = modeIdx;
    }
  }
}

void scanKeysTask(void * pvParameters) {
  const TickType_t xFrequency = 20/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  uint8_t TX_Message[8] = {0};
  std::bitset<12> prevKeyState;
  prevKeyState.set();

  while (1) {
    vTaskDelayUntil( &xLastWakeTime, xFrequency );

    for (int row=0; row<3; row++) {
      setRow(row);
      delayMicroseconds(3);
      std::bitset<4> col_out = readCols();
      xSemaphoreTake(sysState.mutex, portMAX_DELAY);
      for (int i=0; i<4; i++){
        sysState.inputs[row*4 + i] = col_out[i];
      }
      xSemaphoreGive(sysState.mutex);
    }

    uint32_t localStepSizes[MAX_KEYS] = {0};
    const char* localCurrentNote = "";
    for (int key=0; key<12; key++) {
      xSemaphoreTake(sysState.mutex, portMAX_DELAY);
      bool key_pressed = !sysState.inputs[key];
      xSemaphoreGive(sysState.mutex);

      if(key_pressed && !prevKeyState[key]) { /* If prev state is unpressed and pressed now true*/
        __atomic_store_n(&envStage[key],
        static_cast<uint8_t>(EnvStage::ATTACK), __ATOMIC_RELAXED);

        TX_Message[0] = 0x50;
        TX_Message[1] = knob1.getAtomicRotation();
        TX_Message[2] = key;
        if (isSender) xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
      } else if(!key_pressed && prevKeyState[key]) { /* key released */
        __atomic_store_n(&envStage[key],
        static_cast<uint8_t>(EnvStage::RELEASE), __ATOMIC_RELAXED);

        TX_Message[0] = 0x52;
        TX_Message[1] = knob1.getAtomicRotation();
        TX_Message[2] = key;
        if (isSender) xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
      }

      if (key_pressed) { /* key held */
        localStepSizes[key] = stepSizes[key];
        localCurrentNote = noteNames[key];
      }

      prevKeyState[key] = key_pressed;
    }
    for (int i=0; i<MAX_KEYS; i++){
      __atomic_store_n(&currentStepSizes[i], localStepSizes[i], __ATOMIC_RELAXED);
    }
    __atomic_store_n(&currentNote, localCurrentNote, __ATOMIC_RELAXED);
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

    // sysInput and currentNote
    // u8g2.setCursor(70,10);
    // u8g2.print("sys: ");
    // u8g2.print(inputSnapshot, HEX);
    u8g2.setCursor(2,30);
    const char* localCurrentnote = __atomic_load_n(&currentNote, __ATOMIC_RELAXED);
    u8g2.print(localCurrentnote);

    // sysMode
    u8g2.setCursor(0, 10);
    sysMode mode = getSysMode();
    uint32_t cd = __atomic_load_n(&g_recordCountdownSamples, __ATOMIC_RELAXED);
    uint32_t recIdx = __atomic_load_n(&g_recIdx, __ATOMIC_RELAXED);
    uint32_t playIdx = __atomic_load_n(&g_playIdx, __ATOMIC_RELAXED);
    uint32_t loopLen = __atomic_load_n(&g_loopLength, __ATOMIC_RELAXED);
    if (mode == sysMode::RECORD && cd > 0) {
      uint32_t secs = (cd + FS - 1) / FS;
      u8g2.print("Recording in ");
      u8g2.print(secs);
    } else {
      u8g2.print(loopModeToText(mode));
    }

    const uint8_t dotStartX = 88;
    const uint8_t dotY = 5;
    const uint8_t dotSpacing = 6;
    const uint8_t dotRadius = 2;
    const uint8_t dotCount = 6;
    for (uint8_t i = 0; i < dotCount; i++) {
      u8g2.drawCircle(dotStartX + i * dotSpacing, dotY, dotRadius);
    }

    if (mode == sysMode::RECORD && cd > 0) {
      uint8_t countdownDots = static_cast<uint8_t>((cd + FS - 1) / FS);
      if (countdownDots > 3) countdownDots = 3;
      for (uint8_t i = 0; i < countdownDots; i++) {
        u8g2.drawDisc(dotStartX + i * dotSpacing, dotY, dotRadius);
      }
    } else if (mode == sysMode::RECORD) {
      uint32_t remaining = (LOOP_MAX_SAMPLES > recIdx) ? (LOOP_MAX_SAMPLES - recIdx) : 0;
      uint8_t remainingDots = static_cast<uint8_t>((remaining * dotCount + LOOP_MAX_SAMPLES - 1) / LOOP_MAX_SAMPLES);
      if (remainingDots > dotCount) remainingDots = dotCount;
      for (uint8_t i = 0; i < remainingDots; i++) {
        u8g2.drawDisc(dotStartX + i * dotSpacing, dotY, dotRadius);
      }
    } else if (mode == sysMode::PLAYBACK) {
      if (loopLen > 0) {
        uint8_t pos = static_cast<uint8_t>((playIdx * dotCount) / loopLen);
        if (pos >= dotCount) pos = dotCount - 1;
        u8g2.drawDisc(dotStartX + pos * dotSpacing, dotY, dotRadius);
      }
    }

    // Octave knob
    u8g2.setCursor(0, 20);
    u8g2.print("Oct: ");
    u8g2.print(knob1.getRotation());

    // Volume knob
    u8g2.setCursor(40, 20);
    u8g2.print("Vol: ");
    u8g2.print(knob3.getRotation());

    // Waveform knob
    u8g2.setCursor(40, 30);
    u8g2.print("Wave: ");
    u8g2.print(waveNames[knob2.getRotation()]);

    u8g2.setCursor(86,20);
    u8g2.print("RX: ");
    u8g2.print((char) localRX[0]);
    u8g2.print(localRX[1]);
    u8g2.print(localRX[2]);

    xSemaphoreTake(i2cMutex, portMAX_DELAY);
    u8g2.sendBuffer();          // transfer internal memory to the display
    xSemaphoreGive(i2cMutex);

    digitalToggle(LED_BUILTIN);
  }
}

void decodeTask(void * pvParameters) {
  uint8_t localMessage[8];
  while(1) {
    xQueueReceive(msgInQ, localMessage, portMAX_DELAY);
    uint8_t note = localMessage[2];

    if (localMessage[0] == 0x52) { // Key released
      __atomic_store_n(&remoteStepSizes[note], 0, __ATOMIC_RELAXED);
      __atomic_store_n(&remoteEnvStage[note],
      static_cast<uint8_t>(EnvStage::RELEASE), __ATOMIC_RELAXED);
    } else if (localMessage[0] == 0x50) { // Key pressed
      uint8_t octave = localMessage[1];
      uint32_t step = stepSizes[note];
      // current board is at 4th octave
      if (octave >= 4) {
        step = step << (octave - 4);
      } else {
        step = step >> (4 - octave);
      }
      __atomic_store_n(&remoteStepSizes[note], step, __ATOMIC_RELAXED);
      __atomic_store_n(&remoteEnvStage[note],
      static_cast<uint8_t>(EnvStage::ATTACK), __ATOMIC_RELAXED);
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
  pinMode(PA10, INPUT);

  //Initialise display
  setOutMuxBit(DRST_BIT, LOW);  //Assert display logic reset
  delayMicroseconds(2);
  setOutMuxBit(DRST_BIT, HIGH);  //Release display logic reset
  u8g2.begin();
  setOutMuxBit(DEN_BIT, HIGH);  //Enable display power supply
  setOutMuxBit(KNOB_MODE, LOW);  //Read knobs through key matrix

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

  sampleTimer.setOverflow(FS, HERTZ_FORMAT);
  #ifndef DISABLE_ISR
    sampleTimer.attachInterrupt(sampleISR);
  #endif
  sampleTimer.resume();

  autoConfig();
  sysState.mutex = xSemaphoreCreateMutex();
  knob3.initMutex();
  knob2.initMutex();
  knob1.initMutex();
  knob0.initMutex();

  i2cMutex = xSemaphoreCreateMutex();
  knobSemaphore = xSemaphoreCreateBinary();

  Wire.begin();

  // Configure PCAL6408A
  xSemaphoreTake(i2cMutex, portMAX_DELAY);

  Wire.beginTransmission(0x21);
  Wire.write(0x03); Wire.write(0xFF);
  Wire.endTransmission();

  Wire.beginTransmission(0x21);
  Wire.write(0x43); Wire.write(0xFF);
  Wire.endTransmission();

  Wire.beginTransmission(0x21);
  Wire.write(0x44); Wire.write(0xFF);
  Wire.endTransmission();

  Wire.beginTransmission(0x21);
  Wire.write(0x42); Wire.write(0xFF);
  Wire.endTransmission();

  Wire.beginTransmission(0x21);
  Wire.write(0x45); Wire.write(0x00);
  Wire.endTransmission();

  // Clear any pending interrupt before attaching ISR
  Wire.beginTransmission(0x21);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.requestFrom(0x21, (uint8_t)1);
  while (Wire.available()) Wire.read();

  xSemaphoreGive(i2cMutex);

  attachInterrupt(digitalPinToInterrupt(PA10), knobISR, FALLING);

  xTaskCreate(knobTask, "knobTask", 128, NULL, 3, NULL);

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
