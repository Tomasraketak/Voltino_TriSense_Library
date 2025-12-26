/*
 * Example: AdvancedFusion_Improved.ino
 * Pokročilá fúze (Complementary Filter) s využitím přesné 6-bodové kalibrace.
 * * HW: Raspberry Pi Pico 2 + Voltino TriSense
 */

#include <TriSense.h>

TriSense sensor;
AdvancedTriFusion fusion(&sensor.imu, &sensor.mag);

unsigned long lastPrint = 0;
const unsigned long printInterval = 20000; // 50Hz výstup na Serial

void setup() {
  Serial.begin(115200);
  delay(500);

  // 1. Inicializace senzorů
  // CS pin 17 pro Raspberry Pi Pico
  if (!sensor.beginAll(MODE_HYBRID, 17)) {
    Serial.println("Sensor init failed!");
    while (1);
  }

  // Nastavení ODR pro IMU (Turbo mode pro SPI)
  sensor.imu.setODR(ODR_4KHZ);

  // =============================================================
  // 2. KALIBRAČNÍ DATA (DŮLEŽITÉ)
  // =============================================================

  // --- A) AKCELEROMETR (ICM-42688-P) ---
  // Hodnoty získejte pomocí příkladu 'IMUcalibration.ino' (volba 'a' v menu).
  // Příklad výstupu kalibrace:
  // IMU.setAccelOffset(0.01234, -0.00567, 0.00123);
  // IMU.setAccelScale(1.00100, 0.99950, 1.00200);
  
  // SEM VLOŽTE VAŠE HODNOTY:
  sensor.imu.setAccelOffset(0.0, 0.0, 0.0); // <-- Nahraďte vlastními čísly
  sensor.imu.setAccelScale(1.0, 1.0, 1.0);  // <-- Nahraďte vlastními čísly

  // --- B) GYROSKOP ---
  // Gyro kalibrujeme při každém startu pro maximální přesnost (odstranění driftu po zapnutí)
  Serial.println("Calibrating Gyro... Keep still!");
  sensor.autoCalibrateGyro(800);

  // --- C) MAGNETOMETR (AK09918C) ---
  // Hodnoty získejte pomocí MotionCal a příkladu 'MotionCal.ino'
  fusion.setMagHardIron(-46.02, -0.85, -46.00); // <-- Nahraďte
  
  float softIron[3][3] = {
    {1.0, 0.0, 0.0}, // <-- Nahraďte maticí Soft Iron
    {0.0, 1.0, 0.0},
    {0.0, 0.0, 1.0}
  };
  fusion.setMagSoftIron(softIron);
  
  fusion.setDeclination(5.0); // Magnetická deklinace (cca 5.0 pro ČR)

  // =============================================================
  // 3. PARAMETRY FÚZE (LADĚNÍ)
  // =============================================================
  
  // Určuje, jak moc věříme akcelerometru vs. gyroskopu
  // Ref 1G, Sigma (šum)
  fusion.setAccelGaussian(1.0, 0.05);
  
  // Ref uT, Sigma
  fusion.setMagGaussian(50.0, 4.0);   
  
  // Zisk korekcí (0.0 až 1.0) - čím vyšší, tím rychlejší korekce, ale více šumu
  fusion.setMaxGains(0.1, 0.1);
  
  // Integrální složka pro opravu trvalého driftu yaw
  fusion.setYawKi(0.005); 

  // =============================================================

  Serial.println("Calibrating initial orientation...");
  // Funkce initOrientation() využije už zkalibrovaná data z akcelerometru a magnetometru
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
      
      // Výstup ve formátu pro Serial Plotter nebo terminál
      Serial.print("R:"); Serial.print(roll, 1);
      Serial.print("\tP:"); Serial.print(pitch, 1);
      Serial.print("\tY:"); Serial.println(yaw, 1);
      
      lastPrint = now;
    }
  }
}