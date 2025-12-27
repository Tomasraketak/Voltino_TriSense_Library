/*
 * Example: AdvancedFusion_GlobalAccel.ino
 *
 * Popis:
 * Ukázka použití pokročilé fúze (AdvancedTriFusion) spolu s výpočtem
 * globální akcelerace (World Frame Acceleration).
 *
 * Funkce:
 * - Fúze dat z gyroskopu, akcelerometru a magnetometru.
 * - Výstup orientace (Roll, Pitch, Yaw).
 * - Výstup globální akcelerace (očištěné o náklon senzoru).
 * -> Pokud senzor leží v klidu (jakýkoliv náklon), osa Z by měla ukazovat cca 1.00 G.
 */

#include <TriSense.h>

TriSense sensor;
AdvancedTriFusion fusion(&sensor.imu, &sensor.mag);

unsigned long lastPrint = 0;
const unsigned long printInterval = 40000; // 25Hz výstup (40ms)

void setup() {
  Serial.begin(115200);
  delay(1000); // Počkej na naběhnutí sériové linky

  // 1. Inicializace senzorů
  // CS pin 17 pro Raspberry Pi Pico
  if (!sensor.beginAll(MODE_HYBRID, 17)) {
    Serial.println("Sensor init failed!");
    while (1) {
      delay(100); // Blikání LED nebo čekání
    }
  }

  // Nastavení ODR pro IMU (Turbo mode pro SPI)
  sensor.imu.setODR(ODR_4KHZ);

  // =============================================================
  // 2. KALIBRAČNÍ DATA
  // (Zde vlož své naměřené hodnoty z kalibračních skriptů)
  // =============================================================

  // -- AKCELEROMETR (ICM-42688-P) --
  // Příklad: sensor.imu.setAccelOffset(0.012, -0.005, 0.001);
  sensor.imu.setAccelOffset(0.0, 0.0, 0.0); 
  sensor.imu.setAccelScale(1.0, 1.0, 1.0);

  // -- GYROSKOP --
  // Rychlá kalibrace při startu (senzor musí být v klidu!)
  Serial.println("Calibrating Gyro... Keep still!");
  sensor.autoCalibrateGyro(1000);

  // -- MAGNETOMETR (AK09918C) --
  fusion.setMagHardIron(-46.02, -0.85, -46.00);
  
  float softIron[3][3] = {
    {1.0, 0.0, 0.0},
    {0.0, 1.0, 0.0},
    {0.0, 0.0, 1.0}
  };
  fusion.setMagSoftIron(softIron);
  
  // Magnetická deklinace (cca 5.6° pro Třeboň/ČR)
  fusion.setDeclination(5.6); 

  // =============================================================
  // 3. PARAMETRY FÚZE
  // =============================================================
  
  // Důvěra v akcelerometr (Ref 1G, Sigma šum)
  fusion.setAccelGaussian(1.0, 0.05); 
  
  // Důvěra v magnetometr (Ref uT, Sigma)
  fusion.setMagGaussian(48.0, 4.0);   
  
  // Zisky korekce (0.0 - 1.0)
  fusion.setMaxGains(0.2, 0.2);
  fusion.setYawKi(0.005); 

  // Inicializace orientace
  Serial.println("Calibrating initial orientation...");
  fusion.initOrientation();
  
  Serial.println("System Running. Formát: R, P, Y | Ax_G, Ay_G, Az_G");
}

void loop() {
  // Update fúze (čte data, počítá orientaci)
  if (fusion.update()) {
    
    unsigned long now = micros();
    if (now - lastPrint >= printInterval) {
      
      // 1. Získání orientace (stupně)
      float roll, pitch, yaw;
      fusion.getOrientationDegrees(roll, pitch, yaw);

      // 2. Získání GLOBÁLNÍ AKCELERACE [G]
      // (Tato funkce je nyní součástí knihovny díky tvé úpravě)
      float ax_g, ay_g, az_g;
      fusion.getGlobalAcceleration(ax_g, ay_g, az_g);

      // --- Volitelné: Čisté lineární zrychlení (Linear Acceleration) ---
      // Pokud chceš odstranit gravitaci, odkomentuj tento řádek:
      // az_g -= 1.0f; 

      // Výpis dat na Serial Plotter / Monitor
      // Orientace
      Serial.print("R:"); Serial.print(roll, 1);
      Serial.print("\tP:"); Serial.print(pitch, 1);
      Serial.print("\tY:"); Serial.print(yaw, 1);
      
      // Globální akcelerace
      // V klidu by mělo být X a Y blízko 0.00 a Z blízko 1.00
      Serial.print("\t| GA_X:"); Serial.print(ax_g, 2);
      Serial.print("\tGA_Y:"); Serial.print(ay_g, 2);
      Serial.print("\tGA_Z:"); Serial.println(az_g, 2);
      
      lastPrint = now;
    }
  }
}