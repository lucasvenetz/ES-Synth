class Knob {
private:
  int32_t rotation;
  int32_t upperLimit;
  int32_t lowerLimit;
  uint8_t prevStateBA;
  SemaphoreHandle_t mutex;

  static constexpr int8_t rotationLookup[16] = {
    0, +1, 0, 0, -1, 0, 0, 0, 0, 0, 0, -1, 0, 0, +1, 0
  };

public:
  Knob(int32_t lower, int32_t upper, int32_t initial=0) :
    lowerLimit(lower), upperLimit(upper), rotation(initial),
    prevStateBA(0), mutex(nullptr) {}

  void initMutex(){
    mutex = xSemaphoreCreateMutex();
  }

  void updateRotation(bool A, bool B) {
    uint8_t currStateBA = (B << 1) | A;
    int8_t stateLookup = (prevStateBA << 2) | currStateBA;
    int32_t change = rotationLookup[stateLookup];
    prevStateBA = currStateBA;

    xSemaphoreTake(mutex, portMAX_DELAY);
    rotation += change;
    if (rotation > upperLimit) rotation = upperLimit;
    if (rotation < lowerLimit) rotation = lowerLimit;
    xSemaphoreGive(mutex);
  }

  int32_t getRotation() {
    xSemaphoreTake(mutex, portMAX_DELAY);
    int32_t localRotation = rotation;
    xSemaphoreGive(mutex);
    return localRotation;
  }

  int32_t getAtomicRotation() {
    // for ISR
    return __atomic_load_n(&rotation, __ATOMIC_RELAXED);
  }

  void setLimits(int32_t lower, int32_t upper) {
      xSemaphoreTake(mutex, portMAX_DELAY);
      lowerLimit = lower;
      upperLimit = upper;
      if (rotation > upperLimit) rotation = upperLimit;
      if (rotation < lowerLimit) rotation = lowerLimit;
      xSemaphoreGive(mutex);
    }

};

