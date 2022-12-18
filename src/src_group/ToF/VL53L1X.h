#include "Adafruit_VL53L1X.h"

#define IRQ_PIN 37
#define XSHUT_PIN 36
void ToFsetup();
void ToFloop();
int16_t distance;
float distance_LP_param = 0.5;
float distancePrev;
float distance_LP;

Adafruit_VL53L1X vl53 = Adafruit_VL53L1X(XSHUT_PIN, IRQ_PIN);

void VL53L1Xsetup() {
  Wire.begin();
  if (! vl53.begin(0x29, &Wire)) {
    while (1)       delay(10);
  }

  if (! vl53.startRanging()) {
    while (1)       delay(10);
  }

  // Valid timing budgets: 15, 20, 33, 50, 100, 200 and 500ms!
  vl53.setTimingBudget(100);
  /*
  vl.VL53L1X_SetDistanceThreshold(100, 300, 3, 1);
  vl.VL53L1X_SetInterruptPolarity(0);
  */
}

void VL35L1Xloop() {
  if (vl53.dataReady()) {
    // new measurement for the taking!
    distance = vl53.distance();
    if (distance == -1) {
      return;
    }
    // data is read out, time for another reading!
    distance_LP = (1.0 - distance_LP_param) * distancePrev + distance_LP_param * distance;
    distancePrev = distance_LP;
    vl53.clearInterrupt();
  }
}