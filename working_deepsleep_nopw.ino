/*
  ESP32-C3 - Stazione meteo solare (deep sleep)
  ------------------------------------------------
  Sensori:
    - Temperatura / Umidita' / Pressione: BME280 (I2C)
    - Anemometro: interruttore reed a impulsi, campionato per pochi
      secondi solo al momento della pubblicazione (non e' critico non
      perdere impulsi, ci interessa una lettura periodica)
    - Pluviometro a bascula: interruttore reed, DEVE contare ogni
      impulso -> sveglia il chip dal deep sleep tramite (RTC GPIO) 
    - Banderuola: partitore resistivo letto via ADC

  Alimentazione: batteria + pannello solare con charger BQ25185, che ha
  un timer di sicurezza di carica di 6 ore non disattivabile via
  configurazione. Ad ogni ciclo di pubblicazione diamo un impulso al
  pin CE del charger per resettare quel timer, cosi' non scatta mai.

  Librerie richieste (Library Manager):
    - PubSubClient       (Nick O'Leary)
    - ArduinoJson         (Benoit Blanchon)
    - Adafruit BME280 Library + Adafruit Unified Sensor

  ASSUNZIONI DA VERIFICARE / ADATTARE:
    - Sensore temp/umidita/pressione: BME280 su I2C. Se usi un sensore
      diverso, sostituisci solo la sezione "LETTURA BME280".
    - Costanti di calibrazione anemometro/pluviometro/banderuola: uso i
      valori tipici del kit SparkFun/Argent Data Systems come punto di
      partenza plausibile, ma vanno confermati con il datasheet dei
      tuoi sensori reali.
    - Pin RTC-capable su ESP32-C3: GPIO0-GPIO5. Il pluviometro deve
      stare su uno di questi (qui uso GPIO3).
*/

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "driver/gpio.h"
#include "esp_sleep.h"
#include <sys/time.h>

// ...



// ---------- CONFIGURAZIONE DA MODIFICARE ----------
const char* WIFI_SSID     = "";
const char* WIFI_PASSWORD = "";

const char* MQTT_HOST     = "192.168.1.11";
const int   MQTT_PORT     = 1883;
const char* MQTT_USER     = "esp32";
const char* MQTT_PASSWORD = "";

const char* DEVICE_ID = "esp32_meteo";

// Ogni quanto pubblichiamo un dato completo
const uint32_t PUBLISH_INTERVAL_SEC = 30; // 15 minuti

// Quanto campioniamo l'anemometro prima di ogni pubblicazione
const uint32_t WIND_SAMPLE_MS = 2500; // 10s

// Pin (mappati sulle etichette D0-D5 della XIAO ESP32-C3:
// GPIO0 e GPIO1 non sono portati fuori su questa scheda)
#define RAIN_PIN        GPIO_NUM_5   // D3 - RTC-capable, sveglia dal deep sleep
#define WIND_PIN        2             // D0 - impulsi anemometro
#define WIND_VANE_PIN   3             // D1 - ADC, partitore banderuola
#define CHARGER_CE_PIN  4             // D2 - collegato al pin CE del BQ25185
#define I2C_SDA_PIN     6             // D4 - BME280 SDA
#define I2C_SCL_PIN     7             // D5 - BME280 SCL

// Calibrazione (valori tipici SparkFun/Argent, DA VERIFICARE con i tuoi sensori)
const float WIND_KMH_PER_PULSE_PER_SEC = 2.4;   // km/h per (impulsi/secondo)
const float RAIN_MM_PER_TIP            = 0.2794; // mm per basculata
// ----------------------------------------------------

WiFiClient espClient;
PubSubClient mqtt(espClient);
Adafruit_BME280 bme;

String availabilityTopic = String(DEVICE_ID) + "/status";
String dataTopic         = String(DEVICE_ID) + "/data";

// Sopravvivono al deep sleep (si azzerano solo con un vero power-on)
RTC_DATA_ATTR uint32_t rainPulses = 0;
RTC_DATA_ATTR int64_t lastPublishTimeUs = 0; // 


volatile uint32_t windPulsesSample = 0;
void IRAM_ATTR onWindPulse() {
  windPulsesSample++;
}

// ---------- Banderuola: mappa lettura ADC -> gradi ----------
// Tabella a 16 direzioni, calcolata da resistenze reali del sensore
// con R_FIXED = 10000 ohm e ADC a 12 bit (0-4095):
// rawADC = 4095 * R_FIXED / (outResistance + R_FIXED)
// Se il tuo resistore fisso e' diverso da 10k, questi valori vanno ricalcolati.
struct VaneReading { int adcValue; int degrees; const char* name; };
VaneReading readWindVane(); 
VaneReading vaneTable[16] = {
  { 952,   0, "N"   },
  {2472,  23, "NNE" },
  {2250,  45, "NE"  },
  {3760,  68, "ENE" },
  {3723,  90, "E"   },
  {3831, 113, "ESE" },
  {3357, 135, "SE"  },
  {3589, 158, "SSE" },
  {2946, 180, "S"   },
  {3116, 203, "SSW" },
  {1575, 225, "SW"  },
  {1698, 248, "WSW" },
  { 315, 270, "W"   },
  { 786, 293, "WNW" },
  { 445, 315, "NW"  },
  {1053, 338, "NNW" }
};

VaneReading readWindVane() {
  int raw = analogRead(WIND_VANE_PIN);
  int bestDelta = 99999;
  VaneReading best = vaneTable[0];
  for (int i = 0; i < 16; i++) {
    int delta = abs(raw - vaneTable[i].adcValue);
    if (delta < bestDelta) {
      bestDelta = delta;
      best = vaneTable[i];
    }
  }
  Serial.printf("[Sensori] Banderuola - ADC Raw: %d -> %d gradi (%s)\n", raw, best.degrees, best.name);
  return best;
}

int64_t getRtcTimeUs() {
  struct timeval tv;
  gettimeofday(&tv, nullptr);

  return (int64_t)tv.tv_sec * 1000000LL +
         (int64_t)tv.tv_usec;
}
// ---------- Charger BQ25185: reset del timer di sicurezza a 6h ----------
void resetChargerSafetyTimer() {
  pinMode(CHARGER_CE_PIN, OUTPUT);
  digitalWrite(CHARGER_CE_PIN, HIGH);
  delayMicroseconds(100);
  digitalWrite(CHARGER_CE_PIN, LOW); // torna ad abilitare la carica
}

// ---------- WiFi / MQTT ----------
bool connectWiFi() {
  Serial.printf("[WiFi] Connessione a \"%s\" ", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
    delay(200);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print(" OK, IP: ");
    Serial.println(WiFi.localIP());
    return true;
  }
  Serial.printf(" FALLITO (status=%d)\n", WiFi.status());
  return false;
}

bool connectMQTT() {
  Serial.printf("[MQTT] Connessione a %s:%d ", MQTT_HOST, MQTT_PORT);
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  unsigned long start = millis();
  while (!mqtt.connected() && millis() - start < 10000) {
    mqtt.connect(
      DEVICE_ID,
      MQTT_USER,
      MQTT_PASSWORD,
      availabilityTopic.c_str(),
      1,
      true,
      "offline"
    );
    if (!mqtt.connected()) {
      Serial.print(".");
      delay(500);
    }
  }
  if (mqtt.connected()) {
    Serial.println(" OK");
    return true;
  }
  Serial.printf(" FALLITO (rc=%d)\n", mqtt.state());
  return false;
}

// ---------- Ciclo completo: leggi tutto e pubblica ----------
void readAndPublish() {
  Serial.println("\n[Pubblica] --- Inizio a leggere e pubblicare ---");

  //resetChargerSafetyTimer();
  //Serial.println("[Charger] Impulso CE inviato (reset timer sicurezza 6h)");

  // --- Vento: campionato solo ora, per pochi secondi ---
  Serial.printf("[Sensori] Campionamento vento per %lus...\n", WIND_SAMPLE_MS / 1000UL);
  windPulsesSample = 0;
  pinMode(WIND_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(WIND_PIN), onWindPulse, FALLING);
  delay(WIND_SAMPLE_MS);
  detachInterrupt(digitalPinToInterrupt(WIND_PIN));

  float windSpeedKmh = (windPulsesSample / (WIND_SAMPLE_MS / 1000.0)) * WIND_KMH_PER_PULSE_PER_SEC;
  Serial.printf("[Sensori] Vento - impulsi: %u -> %.1f km/h\n", windPulsesSample, windSpeedKmh);

  VaneReading vane = readWindVane();

  // --- Pioggia: accumulata dagli impulsi contati durante il deep sleep ---
  float rainMm = rainPulses * RAIN_MM_PER_TIP;
  Serial.printf("[Sensori] Pioggia - impulsi accumulati: %u -> %.1f mm\n", rainPulses, rainMm);

  // --- Temp / umidita' / pressione ---
  float temperature = NAN, humidity = NAN, pressure = NAN;
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  if (bme.begin(0x76) || bme.begin(0x77)) {
    temperature = bme.readTemperature();
    humidity    = bme.readHumidity();
    pressure    = bme.readPressure() / 100.0F; // Pa -> hPa
    Serial.printf("[Sensori] BME280 - temp: %.1fC  umid: %.1f%%  press: %.1fhPa\n",
                  temperature, humidity, pressure);
  } else {
    Serial.println("[Sensori] ATTENZIONE: BME280 non trovato sul bus I2C (controlla i collegamenti)");
  }

  // --- Connetti e pubblica ---
  if (connectWiFi() && connectMQTT()) {
    mqtt.publish(availabilityTopic.c_str(), "online", true);

    JsonDocument doc;
    if (!isnan(temperature)) doc["temperature"] = roundf(temperature * 10) / 10.0;
    if (!isnan(humidity))    doc["humidity"]    = roundf(humidity * 10) / 10.0;
    if (!isnan(pressure))    doc["pressure"]    = roundf(pressure * 10) / 10.0;
    doc["wind_speed"]           = roundf(windSpeedKmh * 10) / 10.0;
    doc["wind_direction"]       = vane.degrees;
    doc["wind_direction_label"] = vane.name;
    doc["rain"]                 = roundf(rainMm * 10) / 10.0;

    char buffer[256];
    size_t n = serializeJson(doc, buffer);
    Serial.print("[MQTT] Payload: ");
    Serial.println(buffer);

    bool published = mqtt.publish(dataTopic.c_str(), (const uint8_t*)buffer, n, false);
    Serial.println(published ? "[MQTT] Pubblicazione dati OK" : "[MQTT] Pubblicazione dati FALLITA");

    mqtt.publish(availabilityTopic.c_str(), "offline", true);
    mqtt.disconnect();

    // Azzera SOLO ora: la pubblicazione e' confermata riuscita.
    // Se WiFi/MQTT fossero falliti, restano intatti: al prossimo
    // risveglio si ritenta, senza aver perso nulla nel frattempo.
    rainPulses = 0;
    lastPublishTimeUs = getRtcTimeUs();
    Serial.println("[Pubblica] Contatori azzerati, ciclo completato con successo");
  } else {
    Serial.println("[Pubblica] WiFi/MQTT falliti: contatori NON azzerati, si ritenta al prossimo risveglio");
  }

  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
}

// ---------- Prepara le sorgenti di sveglia e dormi ----------
void goToSleep() {
  // Pluviometro: sveglia se il pin va LOW (contatto chiuso)
  pinMode(RAIN_PIN, INPUT);
  //gpio_pullup_en(RAIN_PIN);
  //gpio_pulldown_dis(RAIN_PIN);

  esp_err_t err = esp_deep_sleep_enable_gpio_wakeup(
    (1ULL << RAIN_PIN),
    ESP_GPIO_WAKEUP_GPIO_LOW
);
  
  Serial.printf("[SLEEP] GPIO wake config: %s\n", esp_err_to_name(err));
  int64_t nowUs = getRtcTimeUs();
  // Quanto manca DAVVERO alla prossima pubblicazione, calcolato sul tempo
  // assoluto: cosi' anche se la pioggia ci sveglia spesso nel frattempo,
  // il conto alla rovescia verso i 15 minuti non riparte mai da capo
  int64_t elapsedUs =
      (lastPublishTimeUs == 0)
          ? 0
          : nowUs - lastPublishTimeUs;
  int64_t remainingUs = (int64_t)PUBLISH_INTERVAL_SEC * 1000000LL - elapsedUs;
  if (remainingUs < 1000000LL) remainingUs = 1000000LL; // minimo 1s, mai 0 o negativo

  esp_sleep_enable_timer_wakeup((uint64_t)remainingUs);

  Serial.printf("[Sleep] Deep sleep per max %llds (prima se piove)\n", remainingUs / 1000000LL);
  Serial.flush(); // assicura che il log esca dalla seriale prima di spegnersi

  esp_deep_sleep_start();
}

void setup() {
  Serial.begin(115200);
  delay(2000); // tempo alla seriale USB di stabilizzarsi dopo il wake

  int64_t nowUs = getRtcTimeUs(); //mi prendo il tempo

  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();

  Serial.println("\n==================== WAKE ====================");
  switch (cause) {
    case ESP_SLEEP_WAKEUP_GPIO:
      Serial.println("[Sveglia] Causa: impulso di PIOGGIA");
      break;
    case ESP_SLEEP_WAKEUP_TIMER:
      Serial.println("[Sveglia] Causa: TIMER (controllo periodico)");
      break;
    default:
      Serial.println("[Sveglia] Causa: PRIMO AVVIO (power-on)");
      break;
  }

  bool firstBoot =
    (cause == ESP_SLEEP_WAKEUP_UNDEFINED || lastPublishTimeUs == 0);


  
  if (cause == ESP_SLEEP_WAKEUP_GPIO) {
    // Sveglia da un impulso di pioggia
    rainPulses++;
    Serial.printf("[Pioggia] Impulso contato, totale accumulato: %u\n", rainPulses);
    // Debounce: aspetta che il contatto si riapra, altrimenti si
    // risveglierebbe di nuovo all'istante mentre e' ancora chiuso
    pinMode(RAIN_PIN, INPUT_PULLUP);
    unsigned long waitStart = millis();
    while (digitalRead(RAIN_PIN) == LOW && millis() - waitStart < 100) {
      delay(2);
    }
    if (digitalRead(RAIN_PIN) == LOW) {
      Serial.println("[Pioggia] ATTENZIONE: pin ancora LOW dopo 100ms, possibile contatto bloccato");
    }
  }
  // cause == ESP_SLEEP_WAKEUP_UNDEFINED -> primo avvio (power-on reale)
  // cause == ESP_SLEEP_WAKEUP_TIMER -> il countdown verso i 15 min e' scaduto

  
 
  int64_t elapsedUs = firstBoot
                  ? 0
                  : nowUs - lastPublishTimeUs;

                  
  Serial.printf(
    "[Timing] Trascorsi %llds dall'ultima pubblicazione (soglia: %us)\n",
    elapsedUs / 1000000LL,
    PUBLISH_INTERVAL_SEC
  );
 Serial.printf(
    "[DEBUG] esp_timer=%lld lastPublish=%lld\n",
    nowUs,
    lastPublishTimeUs
);


  if (firstBoot || elapsedUs >= (int64_t)PUBLISH_INTERVAL_SEC * 1000000LL) {
    readAndPublish();
  } else {
    Serial.println("[Timing] Non ancora ora di pubblicare, torno a dormire");
  }

  goToSleep();
  // Da qui non si torna: il chip si riavvia da setup() al prossimo wake
}

void loop() {
  // Non usato: tutta la logica gira in setup() ad ogni ciclo di wake
}