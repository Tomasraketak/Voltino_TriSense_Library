/*
 * Příklad: TriSense_Calibration.ino
 * Tento nástroj slouží ke kalibraci senzorů na desce Voltino TriSense.
 * * INSTRUKCE:
 * 1. Otevřete Serial Monitor (115200 baud).
 * 2. Odesílejte příkazy:
 * 'g' -> Automatická kalibrace gyroskopu (Senzor musí být v klidu!)
 * 'a' -> 6-bodová kalibrace akcelerometru (Postupujte podle pokynů v terminálu)
 * 'x' -> Reset všech HW offsetů (tovární nastavení biasu v čipu)
 * * 3. Získané hodnoty (kód) zkopírujte do funkce setup() ve vašem projektu.
 */

#include <TriSense.h>

TriSense sensor;

void setup() {
  Serial.begin(115200);
  while (!Serial);
  delay(500);

  Serial.println("Inicializuji TriSense...");

  // Inicializace v hybridním módu (nejčastější použití)
  // Pokud používáš I2C mód, změň na: sensor.beginAll(MODE_I2C);
  if (!sensor.beginAll(MODE_HYBRID, 17)) {
    Serial.println("Chyba inicializace! Zkontrolujte zapojení.");
    while (1);
  }
    IMU.setODR(ODR_500HZ); 


  Serial.println("--------------------------------------");
  Serial.println("TriSense Kalibracni Nastroj");
  Serial.println("--------------------------------------");
  Serial.println("[g] -> Kalibrovat GYRO (Nehybat!)");
  Serial.println("[a] -> Kalibrovat AKCELEROMETR (6 poloh)");
  Serial.println("[x] -> RESET offsetu v cipu");
  Serial.println("--------------------------------------");
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    
    // --- KALIBRACE GYRA ---
    if (c == 'g') {
      // Zavolá wrapper v TriSense, který spustí logiku v ICM42688P
      sensor.autoCalibrateGyro(1000); 
    } 
    
    // --- KALIBRACE AKCELEROMETRU ---
    else if (c == 'a') {
      // Spustí interaktivního průvodce (vyžaduje potvrzování 'y' pro každou polohu)
      sensor.autoCalibrateAccel();
    }
    
    // --- RESET OFFSETŮ ---
    else if (c == 'x') {
      sensor.resetHardwareOffsets();
      Serial.println("Hardwarove registry vymazany (Offset = 0).");
    }
  }
}