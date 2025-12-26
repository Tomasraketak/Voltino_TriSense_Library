/*
 * Příklad: MotionCal_Bridge.ino
 * Odesílá data ze senzorů Voltino TriSense do aplikace MotionCal.
 * * INSTRUKCE:
 * 1. Nahrajte tento program do Raspberry Pico 2.
 * 2. Spusťte aplikaci MotionCal na PC.
 * 3. Vyberte příslušný COM port.
 * 4. Otáčejte senzorem ve všech směrech, dokud se "brambora" (sféra) nevyplní body.
 * 5. Výsledné hodnoty (Hard Iron, Soft Iron) opište do svého kódu (SimpleFusion/AdvancedFusion).
 */

#include <TriSense.h>

TriSense sensor;

// Proměnné pro uložení posledních načtených hodnot
float ax, ay, az;
float gx, gy, gz;

void setup() {
  // Preferovaný baud rate
  Serial.begin(115200);
  delay(500);

  // Inicializace senzorů (Hybrid mode: AK+BMP na I2C, ICM na SPI)
  // CS pin 17 pro Raspberry Pi Pico
  if (!sensor.beginAll(MODE_HYBRID, 17)) {
    Serial.println("Chyba: Senzory nenalezeny!");
    while (1) delay(10);
  }

  // DŮLEŽITÉ: Pro MotionCal chceme co "nejsurovější" data.
  // Proto resetujeme HW offsety v ICM, aby nám do toho čip nezasahoval.
  sensor.resetHardwareOffsets();

  // Nastavíme ODR magnetometru na maximum (100 Hz), aby bylo překreslování plynulé
  sensor.mag.setODR(100);
}

void loop() {
  // 1. Načtení dat z IMU (Akcelerometr + Gyro)
  // Používáme readFIFO, které vrací true, pokud jsou nová data
  // Data si uložíme do globálních proměnných, abychom je měli připravená,
  // až bude hotový i magnetometr.
  float tax, tay, taz, tgx, tgy, tgz;
  if (sensor.imu.readFIFO(tax, tay, taz, tgx, tgy, tgz)) {
    ax = tax; ay = tay; az = taz;
    gx = tgx; gy = tgy; gz = tgz;
  }

  // 2. Načtení dat z Magnetometru
  // Magnetometr je pomalejší (100Hz) než IMU. Data posíláme do PC
  // jen ve chvíli, kdy máme nový vzorek magnetometru, aby se v MotionCal
  // nemnožily duplicitní body.
  if (sensor.mag.readData()) {
    
    // 3. Formátování pro MotionCal
    // MotionCal očekává formát: "Raw:ax,ay,az,gx,gy,gz,mx,my,mz"
    // Očekává celá čísla (int), takže floaty převedeme zpět na "pseudo-raw" hodnoty.
    
    Serial.print("Raw:");
    
    // Accel: Knihovna vrací [g]. Vynásobíme 8192 (cca odpovídá raw hodnotě při rozsahu 4G)
    Serial.print((int)(ax * 8192)); Serial.print(",");
    Serial.print((int)(ay * 8192)); Serial.print(",");
    Serial.print((int)(az * 8192)); Serial.print(",");

    // Gyro: Knihovna vrací [deg/s]. Vynásobíme 16 (cca odpovídá raw sensitivity)
    // MotionCal gyro používá jen pro vizuální rotaci modelu, není kritické pro kalibraci magu.
    Serial.print((int)(gx * 16));   Serial.print(",");
    Serial.print((int)(gy * 16));   Serial.print(",");
    Serial.print((int)(gz * 16));   Serial.print(",");

    // Mag: Knihovna vrací [uT]. Vynásobíme 10, abychom dostali rozumná celá čísla (např. 40 uT -> 400)
    // AK09918C vrací raw data 16-bit, TriSense je násobí 0.15. Zde to vracíme řádově zpět.
    Serial.print((int)(sensor.mag.x * 10)); Serial.print(",");
    Serial.print((int)(sensor.mag.y * 10)); Serial.print(",");
    Serial.print((int)(sensor.mag.z * 10));
    
    Serial.println();
	delay(50);
  }
}