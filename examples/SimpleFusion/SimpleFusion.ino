// Example: SimpleFusion.ino
// Demonstrace jednoduché fúze senzorů s využitím nové kalibrace.

#include <TriSense.h>

TriSense sensor;
SimpleTriFusion fusion(&sensor.imu, &sensor.mag);

unsigned long lastPrint = 0;
const unsigned long printInterval = 50000; // 20Hz (us)

void setup() {
  Serial.begin(115200);
  delay(500);

  // 1. Inicializace senzorů
  if (!sensor.beginAll(MODE_HYBRID, 17)) {
    Serial.println("Failed to initialize sensors!");
    while (1);
  }
    IMU.setODR(ODR_4KHZ); 


  // =============================================================
  // 2. KALIBRACE (Sem vložte hodnoty z TriSense_Calibration.ino)
  // =============================================================
  
  // A) Akcelerometr (Bias + Scale)
  // Příklad:
  // sensor.imu.setAccelOffset(0.012, -0.005, 0.001);
  // sensor.imu.setAccelScale(1.001, 0.999, 1.002);

  // B) Gyroskop
  // Můžete buď nastavit pevné hodnoty z minula:
  // sensor.imu.setGyroOffset(0.52, -0.12, 0.05);
  
  // ... NEBO provést rychlou kalibraci při každém startu (doporučeno):
  Serial.println("Calibrating Gyro... Keep still!");
  sensor.autoCalibrateGyro(500); 

  // C) Magnetometr (Hard/Soft Iron) - stále se nastavuje do fusion objektu
  fusion.setMagHardIron(-46.02, -0.85, -46.00);
  float softIron[3][3] = {
    {0.965, 0.008, -0.002},
    {0.008, 0.981, 0.139},
    {-0.002, 0.139, 1.077}
  };
  fusion.setMagSoftIron(softIron);
  fusion.setDeclination(5.0 + 1.0 / 60.0); // Změňte dle vaší lokality

  // =============================================================

  // Inicializace orientace
  Serial.println("Initializing orientation...");
  fusion.initOrientation();
  Serial.println("Ready.");
}

void loop() {
  if (fusion.update()) {
    unsigned long now = micros();
    if (now - lastPrint >= printInterval) {
      float roll, pitch, yaw;
      fusion.getOrientationDegrees(roll, pitch, yaw);
      
      Serial.print("Roll: "); Serial.print(roll, 2);
      Serial.print(" Pitch: "); Serial.print(pitch, 2);
      Serial.print(" Yaw: "); Serial.println(yaw, 2);
      
      lastPrint = now;
    }
  }
}