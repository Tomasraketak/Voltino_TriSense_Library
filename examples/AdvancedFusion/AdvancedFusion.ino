// Example: AdvancedFusion.ino
// Pokročilá fúze (Complementary Filter) s přesnou kalibrací.

#include <TriSense.h>

TriSense sensor;
AdvancedTriFusion fusion(&sensor.imu, &sensor.mag);

unsigned long lastPrint = 0;
const unsigned long printInterval = 20000; // 50Hz output

void setup() {
  Serial.begin(115200);
  delay(500);

  // 1. Inicializace
  if (!sensor.beginAll(MODE_HYBRID, 17)) {
    Serial.println("Sensor init failed!");
    while (1);
  }

  IMU.setODR(ODR_4KHZ); 

  // =============================================================
  // 2. KALIBRAČNÍ DATA
  // =============================================================

  // -- IMU (ICM-42688-P) --
  // Hodnoty získejte pomocí příkladu 'TriSense_Calibration'
  // sensor.imu.setAccelOffset(...);
  // sensor.imu.setAccelScale(...);
  
  // Gyro kalibrujeme při startu pro maximální přesnost driftu
  sensor.autoCalibrateGyro(800);

  // -- MAG (AK09918C) --
  fusion.setMagHardIron(-46.02, -0.85, -46.00);
  float softIron[3][3] = {
    {1.0, 0.0, 0.0}, // Zde vložte matici Soft Iron kalibrace
    {0.0, 1.0, 0.0},
    {0.0, 0.0, 1.0}
  };
  fusion.setMagSoftIron(softIron);
  fusion.setDeclination(5.0); // Magnetická deklinace

  // -- Parametry Fúze (Ladění) --
  // Určuje, jak moc věříme akcelerometru vs. gyroskopu
  fusion.setAccelGaussian(1.0, 0.05); // Ref 1G, Sigma (šum)
  fusion.setMagGaussian(50.0, 4.0);   // Ref uT, Sigma
  
  // Zisk korekcí (0.0 až 1.0)
  fusion.setMaxGains(0.1, 0.1); 
  fusion.setYawKi(0.005); // Integrální složka pro opravu driftu yaw

  // =============================================================

  Serial.println("Calibrating initial orientation...");
  fusion.initOrientation();
  Serial.println("System Running.");
}

void loop() {
  // Update fúze (čte data, integruje gyro, koriguje podle accel/mag)
  if (fusion.update()) {
    
    // Výpis dat
    unsigned long now = micros();
    if (now - lastPrint >= printInterval) {
      float roll, pitch, yaw;
      fusion.getOrientationDegrees(roll, pitch, yaw);
      
      Serial.print("R: "); Serial.print(roll, 1);
      Serial.print("\tP: "); Serial.print(pitch, 1);
      Serial.print("\tY: "); Serial.println(yaw, 1);
      
      lastPrint = now;
    }
  }
}