/*
 * ═══════════════════════════════════════════════════════════════════
 *  SMART ENERGY-GENERATING FITNESS SHOES
 *  ESP32 Firmware v1.0
 *
 *  Features:
 *    - Dual piezoelectric sensor reading (heel + toe)
 *    - Pressure-based step detection (no motion estimation)
 *    - Energy harvesting calculation from piezo voltage peaks
 *    - Step count, calories burned, distance walked
 *    - BLE GATT notify every 200ms → SmartSole app
 *    - Serial debug output at 115200 baud
 *
 *  Hardware:
 *    - ESP32 (any variant)
 *    - 2× Piezo disc sensors (27mm recommended)
 *    - 4× 1N4148 diodes (bridge rectifier per sensor)
 *    - 2× 10kΩ resistors (voltage divider)
 *    - 1× TP4056 Li-Po charger module (energy harvesting)
 *    - 1× 3.7V Li-Po battery (500–1000mAh)
 *
 *  Wiring:
 *    Heel piezo (+) → Bridge rectifier (+) → 10kΩ → GPIO34
 *    Heel piezo (-) → Bridge rectifier (-) → GND
 *    Toe  piezo (+) → Bridge rectifier (+) → 10kΩ → GPIO35
 *    Toe  piezo (-) → Bridge rectifier (-) → GND
 *    Add 3.3V Zener diode from each GPIO pin to GND for protection
 * ═══════════════════════════════════════════════════════════════════
 */

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// ───────────────────────────────────────────────────────────────────
//  BLE Configuration
// ───────────────────────────────────────────────────────────────────
#define DEVICE_NAME   "SmartSole"
#define SVC_UUID      "aaaa1234-aaaa-aaaa-aaaa-aaaaaaaaaaaa"
#define CHR_UUID      "bbbb1234-bbbb-bbbb-bbbb-bbbbbbbbbbbb"

// ───────────────────────────────────────────────────────────────────
//  Hardware Pin Definitions
// ───────────────────────────────────────────────────────────────────
#define PIN_PIEZO_HEEL  34    // ADC1_CH6 — heel sensor
#define PIN_PIEZO_TOE   35    // ADC1_CH7 — toe sensor
#define PIN_LED_STATUS   2    // onboard LED (optional)

// ───────────────────────────────────────────────────────────────────
//  Step Detection Parameters
// ───────────────────────────────────────────────────────────────────
#define STEP_THRESHOLD_ADC  750    // ADC counts to register a step (0–4095)
#define STEP_RELEASE_RATIO  0.45f  // fraction of threshold for release detection
#define STEP_COOLDOWN_MS    320    // minimum ms between two consecutive steps
#define DEBOUNCE_SAMPLES    3      // samples above threshold before confirming step

// ───────────────────────────────────────────────────────────────────
//  Physical Calibration Constants
// ───────────────────────────────────────────────────────────────────
#define PIEZO_LOAD_OHM     10000.0f   // load resistor in ohms
#define ADC_REF_VOLT       3.3f       // ESP32 ADC reference voltage
#define ADC_RESOLUTION     4095.0f    // 12-bit ADC max

#define STRIDE_LENGTH_M    0.78f      // average walking stride in metres
#define KCAL_PER_STEP      0.04f      // kilocalories burned per step (avg adult)

// Energy harvesting: typical piezo disc generates ~0.1–5 mW per step
// We calculate from the measured peak voltage across the load resistor.
// Actual mWh = (V²/R) × pulse_duration_in_hours
#define PULSE_DURATION_S   0.05f      // estimated pressure pulse duration (50ms)

// ───────────────────────────────────────────────────────────────────
//  BLE Objects
// ───────────────────────────────────────────────────────────────────
BLEServer*          pServer = nullptr;
BLECharacteristic*  pChar   = nullptr;
volatile bool       deviceConnected = false;

// ───────────────────────────────────────────────────────────────────
//  Fitness & Energy State
// ───────────────────────────────────────────────────────────────────
uint32_t      stepCount      = 0;
double        totalEnergyMWh = 0.0;

// Step detection state machine
bool          inStep          = false;
uint8_t       aboveCount      = 0;       // consecutive samples above threshold
unsigned long lastStepMs      = 0;

// ───────────────────────────────────────────────────────────────────
//  BLE Connection Callbacks
// ───────────────────────────────────────────────────────────────────
class SmartSoleCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* s) override {
    deviceConnected = true;
    digitalWrite(PIN_LED_STATUS, HIGH);
    Serial.println("[BLE] Client connected");
  }
  void onDisconnect(BLEServer* s) override {
    deviceConnected = false;
    digitalWrite(PIN_LED_STATUS, LOW);
    Serial.println("[BLE] Client disconnected — restarting advertising");
    BLEDevice::startAdvertising();
  }
};

// ───────────────────────────────────────────────────────────────────
//  Energy Calculation
//  Input : peak ADC reading during step
//  Return: energy in milliwatt-hours for that single step
// ───────────────────────────────────────────────────────────────────
double calcStepEnergy(int peakADC) {
  float voltage    = (peakADC / ADC_RESOLUTION) * ADC_REF_VOLT;  // V
  float powerWatts = (voltage * voltage) / PIEZO_LOAD_OHM;       // P = V²/R
  float hours      = PULSE_DURATION_S / 3600.0f;                  // s → h
  return (double)(powerWatts * 1000.0f * hours);                  // mWh
}

// ───────────────────────────────────────────────────────────────────
//  Read & average both piezo channels
// ───────────────────────────────────────────────────────────────────
void readPiezo(int &heel, int &toe) {
  // Oversample ×4 for noise reduction
  long heelSum = 0, toeSum = 0;
  for (int i = 0; i < 4; i++) {
    heelSum += analogRead(PIN_PIEZO_HEEL);
    toeSum  += analogRead(PIN_PIEZO_TOE);
    delayMicroseconds(200);
  }
  heel = (int)(heelSum / 4);
  toe  = (int)(toeSum  / 4);
}

// ───────────────────────────────────────────────────────────────────
//  Step Detection (pressure-peak state machine)
// ───────────────────────────────────────────────────────────────────
bool detectStep(int heel, int toe, int &peakOut) {
  int peak = max(heel, toe);
  unsigned long now = millis();

  if (!inStep) {
    // Rising edge: count consecutive samples above threshold
    if (peak > STEP_THRESHOLD_ADC) {
      aboveCount++;
      if (aboveCount >= DEBOUNCE_SAMPLES &&
          (now - lastStepMs) > STEP_COOLDOWN_MS) {
        inStep   = true;
        peakOut  = peak;
        lastStepMs = now;
        aboveCount = 0;
        return true;          // ← step confirmed
      }
    } else {
      aboveCount = 0;
    }
  } else {
    // Falling edge: wait for pressure release
    if (peak < (int)(STEP_THRESHOLD_ADC * STEP_RELEASE_RATIO)) {
      inStep = false;
    }
  }
  peakOut = peak;
  return false;
}

// ───────────────────────────────────────────────────────────────────
//  Build & transmit BLE packet
//  Format: "steps,calories,distKm,energyMWh,heelADC,toeADC"
// ───────────────────────────────────────────────────────────────────
void transmitBLE(int heel, int toe) {
  float calories = stepCount * KCAL_PER_STEP;
  float distKm   = (stepCount * STRIDE_LENGTH_M) / 1000.0f;

  char buf[96];
  snprintf(buf, sizeof(buf),
    "%lu,%.2f,%.4f,%.5f,%d,%d",
    (unsigned long)stepCount,
    calories,
    distKm,
    (float)totalEnergyMWh,
    heel,
    toe
  );

  pChar->setValue(buf);
  pChar->notify();
}

// ───────────────────────────────────────────────────────────────────
//  SETUP
// ───────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial.println("\n╔══════════════════════════════╗");
  Serial.println("║   SmartSole ESP32 Firmware   ║");
  Serial.println("╚══════════════════════════════╝\n");

  // GPIO
  pinMode(PIN_LED_STATUS, OUTPUT);
  digitalWrite(PIN_LED_STATUS, LOW);

  // ADC config — 12-bit, full 3.3V range
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  Serial.printf("[ADC] Heel pin: GPIO%d  Toe pin: GPIO%d\n",
                PIN_PIEZO_HEEL, PIN_PIEZO_TOE);
  Serial.printf("[STEP] Threshold: %d ADC counts  Cooldown: %d ms\n",
                STEP_THRESHOLD_ADC, STEP_COOLDOWN_MS);

  // ── BLE Initialisation ──────────────────────────────────────────
  BLEDevice::init(DEVICE_NAME);
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new SmartSoleCallbacks());

  BLEService* pSvc = pServer->createService(SVC_UUID);

  pChar = pSvc->createCharacteristic(
    CHR_UUID,
    BLECharacteristic::PROPERTY_NOTIFY |
    BLECharacteristic::PROPERTY_READ
  );
  pChar->addDescriptor(new BLE2902());

  pSvc->start();

  BLEAdvertising* pAdv = BLEDevice::getAdvertising();
  pAdv->addServiceUUID(SVC_UUID);
  pAdv->setScanResponse(true);
  pAdv->setMinPreferred(0x06);   // iPhone compatibility
  pAdv->setMinPreferred(0x12);
  BLEDevice::startAdvertising();

  Serial.println("[BLE] Advertising as \"SmartSole\"");
  Serial.println("[SYS] Ready — waiting for footsteps...\n");
}

// ───────────────────────────────────────────────────────────────────
//  LOOP
// ───────────────────────────────────────────────────────────────────
void loop() {
  static unsigned long lastTxMs = 0;
  const unsigned long TX_INTERVAL_MS = 200;   // BLE notify rate

  int heel = 0, toe = 0, peak = 0;
  readPiezo(heel, toe);

  // Step detection
  if (detectStep(heel, toe, peak)) {
    stepCount++;
    double stepEnergy = calcStepEnergy(peak);
    totalEnergyMWh   += stepEnergy;

    Serial.printf("[STEP #%lu] heel=%-4d toe=%-4d peak=%-4d | "
                  "E_step=%.5f mWh | E_total=%.4f mWh | "
                  "Cal=%.1f kcal | Dist=%.3f km\n",
                  (unsigned long)stepCount,
                  heel, toe, peak,
                  (float)stepEnergy,
                  (float)totalEnergyMWh,
                  stepCount * KCAL_PER_STEP,
                  (stepCount * STRIDE_LENGTH_M) / 1000.0f);

    // Flash LED on step
    digitalWrite(PIN_LED_STATUS, HIGH);
    delay(20);
    if (!deviceConnected) digitalWrite(PIN_LED_STATUS, LOW);
  }

  // BLE transmit at fixed interval
  if (deviceConnected && (millis() - lastTxMs >= TX_INTERVAL_MS)) {
    transmitBLE(heel, toe);
    lastTxMs = millis();
  }

  delay(10);   // ~100 Hz sensor polling
}
