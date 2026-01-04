# Voltino TriSense Library

## Popis
**Voltino TriSense** je vysoce výkonná Arduino knihovna navržená pro senzorovou desku **Voltino TriSense Pro**. Integruje podporu pro následující senzory:
* **BMP580** (Vysoce přesný senzor tlaku a teploty)
* **AK09918C** (3-osý magnetometr)
* **ICM-42688-P** (6-osá IMU s akcelerometrem a gyroskopem)

Tato knihovna umožňuje snadnou inicializaci všech senzorů současně, a to buď v režimu **I2C**, nebo v režimu **Hybrid** (AK09918C a BMP580 na I2C, ICM42688P na SPI pro maximální rychlost).

Hlavní předností je robustní engine pro **Senzorovou Fúzi** (Advanced Sensor Fusion), který využívá adaptivní algoritmus (vycházející z principů Madgwick/Mahony filtrů) s dynamickým Gaussovým přizpůsobením zisku. To zajišťuje stabilní orientaci (Roll, Pitch, Yaw) a vektory globálního zrychlení i v náročných podmínkách.

## Klíčové vlastnosti
* **Sjednocená inicializace:** Jeden řádek kódu pro nastavení všech senzorů.
* **Podpora Hybridního režimu:** Provozuje IMU na sběrnici SPI (až 10 MHz), zatímco ostatní senzory zůstávají na I2C.
* **Kalibrace "Sphere Fit":** Vestavěný algoritmus pro 6-bodovou kalibraci akcelerometru.
* **Pokročilá Senzorová Fúze:**
    * Kvaternionový odhad orientace.
    * **Globální zrychlení:** Výpočet lineárního zrychlení v zemském referenčním rámci (odstranění náklonu senzoru).
    * **Adaptivní filtrace:** Použití Gaussových funkcí pro snížení důvěry v senzory během dynamického pohybu nebo magnetických anomálií.
    * **Korekce driftu Yaw:** Integrální složka (Ki) pro minimalizaci dlouhodobého driftu kurzu.
* **Kompatibilita s MotionCal:** Dedikovaný příklad pro snadnou kalibraci magnetometru pomocí vizuálních nástrojů.

## Instalace
1.  Stáhněte si nejnovější vydání jako ZIP soubor z GitHub repozitáře.
2.  Otevřete Arduino IDE.
3.  Přejděte na **Sketch > Include Library > Add .ZIP Library...** a vyberte stažený ZIP soubor.
4.  Knihovna by nyní měla být dostupná v **Sketch > Include Library > Voltino TriSense**.

## Závislosti
* `Wire` (Standardní Arduino I2C)
* `SPI` (Standardní Arduino SPI)

## Použití

### 1. Vložení knihovny
```cpp
#include <TriSense.h>
```

### 2. Inicializace senzorů
Všechny senzory můžete inicializovat najednou pomocí funkce `beginAll()`.

**Parametry:**
* `mode`: `MODE_HYBRID` (Doporučeno) nebo `MODE_I2C`.
* `spiCsPin`: Chip Select pin pro IMU (Výchozí je 17 pro Voltino/Pico).
* `spiFreq`: Frekvence SPI v Hz (Výchozí 10000000 = 10 MHz).

```cpp
TriSense sensor;

void setup() {
  Serial.begin(115200);

  // Inicializace v Hybridním režimu:
  // - AK09918C & BMP580 na I2C
  // - ICM42688P na SPI (Pin 17, 10 MHz)
  if (!sensor.beginAll(MODE_HYBRID, 17, 10000000)) {
    Serial.println("Chyba inicializace senzorů!");
    while (1);
  }

  // Nastavení ODR (Output Data Rate)
  sensor.imu.setODR(ODR_4KHZ); // Turbo mód pro SPI
}
```

### 3. Čtení surových dat (Raw Data)
Přistupujte k jednotlivým objektům senzorů přes `sensor.bmp`, `sensor.mag` a `sensor.imu`.

```cpp
// BMP580
float temp = sensor.bmp.readTemperature();
float press = sensor.bmp.readPressure();
float alt = sensor.bmp.readAltitude(101325);

// AK09918C
if (sensor.mag.readData()) {
    float mx = sensor.mag.x;
    float my = sensor.mag.y;
    float mz = sensor.mag.z;
}

// ICM-42688-P
float ax, ay, az, gx, gy, gz;
if (sensor.imu.readFIFO(ax, ay, az, gx, gy, gz)) {
    // Data jsou dostupná v proměnných
}
```

## Senzorová Fúze a Orientace (AdvancedTriFusion)

Knihovna poskytuje třídu `AdvancedTriFusion` pro získání vysoce kvalitních dat o orientaci.

### Jak funguje algoritmus Advanced Fusion?
Algoritmus kombinuje data z gyroskopu (rychlá odezva), akcelerometru (korekce náklonu) a magnetometru (korekce kurzu) do jednoho kvaternionu orientace. Na rozdíl od jednoduchých filtrů používá **Adaptivní Gaussovu Váhu**:

1.  **Predikce (Gyroskop):** Orientace je primárně integrována z gyroskopu. To zajišťuje okamžitou odezvu na pohyb.
2.  **Korekce (Akcelerometr):** Algoritmus porovnává naměřený vektor gravitace s očekávaným (1.0 G).
    * *Adaptivní zisk:* Pokud je celkové zrychlení výrazně odlišné od 1 G (např. při vibracích nebo prudkém pohybu), algoritmus sníží vliv akcelerometru na korekci pomocí Gaussovy křivky. Tím se zabrání chybnému náklonu horizontu při pohybu.
3.  **Korekce (Magnetometr):** Algoritmus porovnává naměřený magnetický sever.
    * *Magnetická ochrana:* Pokud se velikost magnetického pole liší od kalibrované referenční hodnoty (např. v blízkosti motorů nebo kovů), vliv magnetometru je dočasně potlačen.
    * *Tilt Compensation:* Zisk magnetometru je také dynamicky upravován podle náklonu (Pitch/Roll), aby se předešlo chybám v Yaw při strmých úhlech.
4.  **Gyro Bias Learning (Ki):** Algoritmus obsahuje integrální složku, která se "učí" drift gyroskopu v ose Z (Yaw) a automaticky jej kompenzuje.

### Nastavení Fúze

```cpp
AdvancedTriFusion fusion(&sensor.imu, &sensor.mag);

void setup() {
    // ... init senzorů ...

    // Nastavení kalibračních dat (Viz sekce Kalibrace)
    sensor.imu.setAccelOffset(0.01, -0.02, 0.05);
    sensor.imu.setAccelScale(1.00, 1.00, 1.00);
    fusion.setMagHardIron(-40.0, 12.5, 5.0);
    
    // Nastavení magnetické deklinace (např. 5.6 stupňů pro ČR)
    fusion.setDeclination(5.6);

    // Inicializace orientace (chvíli počká na ustálení)
    fusion.initOrientation();
}
```

### Hlavní smyčka

```cpp
void loop() {
    if (fusion.update()) {
        float roll, pitch, yaw;
        fusion.getOrientationDegrees(roll, pitch, yaw);
        
        Serial.print("Roll: "); Serial.print(roll);
        Serial.print(" Pitch: "); Serial.print(pitch);
        Serial.print(" Yaw: "); Serial.println(yaw);
    }
}
```

## Globální Zrychlení (Global Acceleration)
Tato funkce vypočítá zrychlení relativně k Zemi (odstraní gravitaci a náklon senzoru).
* **Stacionární stav:** Z ≈ 1.0 G, X/Y ≈ 0.0 G (nebo 0 ve všech osách po odečtení 1G).
* **Lineární pohyb:** Ukazuje čisté zrychlení pohybu bez ohledu na to, jak je senzor natočen.

```cpp
float ax_g, ay_g, az_g;
fusion.getGlobalAcceleration(ax_g, ay_g, az_g);
```

## Kalibrace
Pro získání přesných dat je nutné senzory kalibrovat. Knihovna obsahuje nástroje, které tento proces usnadňují.

### 1. Gyroskop a Akcelerometr
Použijte příklad: `TriSense_Calibration.ino`
1.  Nahrajte sketch.
2.  Otevřete Serial Monitor.
3.  Odešlete `'g'` pro automatickou kalibraci driftu gyroskopu (senzor musí být v klidu).
4.  Odešlete `'a'` pro spuštění 6-bodové kalibrace akcelerometru. Postupujte podle pokynů na obrazovce a otáčejte senzorem do požadovaných poloh. Algoritmus "Sphere Fit" vypočítá přesný Offset a Scale.
5.  Zkopírujte vygenerovaný kód do `setup()` vašeho projektu.

### 2. Magnetometr
Použijte příklad: `MotionCal_Bridge.ino`
1.  Nahrajte sketch.
2.  Stáhněte si nástroj **MotionCal** (od Paula Stoffregena).
3.  Spusťte MotionCal a připojte se k sériovému portu.
4.  Otáčejte senzorem ve všech směrech, dokud se sféra v programu nezaplní body.
5.  MotionCal vypočítá hodnoty **Hard Iron** a **Soft Iron**.
6.  Zkopírujte tyto hodnoty do funkcí `fusion.setMagHardIron(...)` a `fusion.setMagSoftIron(...)`.

## Přiložené příklady
* **SimpleFusion:** Základní nastavení fúze.
* **AdvancedFusion_Improved:** Kompletní implementace s placeholdery pro kalibraci a "best practices".
* **AdvancedFusion_GlobalAccel:** Ukázka získání zrychlení v zemském rámci.
* **TriSense_Calibration:** Nástroj pro kalibraci Gyra a Akcelerometru.
* **MotionCal_Bridge:** Nástroj pro odesílání surových dat do aplikace MotionCal.

## Licence
Tato knihovna je uvolněna pod licencí **MIT License**. Vyvinuto **VoltinoLabs** (Tomas Michal).