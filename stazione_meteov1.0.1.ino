/*
  ESP32-C3 (XIAO) - Stazione meteo solare (deep sleep)
  -----------------------------------------------------
  Novita' rispetto alla versione precedente:
   1. Backoff su errore WiFi/MQTT (60s, 2m, 5m, 10m, 15m) invece del
      retry ogni secondo.
   2. Watchdog software (esp_timer): se la fase sveglia dura piu' di
      WATCHDOG_SEC, il chip va in deep sleep forzato con backoff.
   3. Pluviometro: interrupt attivo per TUTTA la fase sveglia, contatore
      in RTC memory, pubblicazione con sottrazione dello snapshot
      (nessuna basculata persa durante WiFi/MQTT).
   4. Anemometro campionato MENTRE si connette il WiFi, con durata
      misurata realmente.
   5. WiFi veloce: canale + BSSID salvati in RTC e IP statico.

  Librerie: PubSubClient, ArduinoJson (v7), Adafruit BME280 + Unified Sensor
*/

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "driver/gpio.h"
#include "esp_sleep.h"
#include "esp_system.h"
#include "esp_timer.h"
#include <sys/time.h>
#include "secrets.h"

// Definito qui in alto: l'Arduino IDE genera i prototipi delle funzioni prima del
// primo codice, quindi un tipo usato come valore di ritorno deve essere gia' noto.
struct VaneReading { int adcValue; int degrees; };

// Metti 0 in produzione: niente Serial, niente delay di debug
#define DEBUG 1
#if DEBUG
  #define LOG(...) Serial.printf(__VA_ARGS__)
#else
  #define LOG(...)
#endif

// ---------- CONFIGURAZIONE DA MODIFICARE ----------


// IP statico: velocizza molto la connessione (niente DHCP)
const bool  USE_STATIC_IP = true;
IPAddress STATIC_IP(192, 168, 1, 254);
IPAddress GATEWAY  (192, 168, 1, 1);
IPAddress SUBNET   (255, 255, 255, 0);
IPAddress DNS      (192, 168, 1, 1);

const char* MQTT_HOST     = "192.168.1.11";
const int   MQTT_PORT     = 1883;
const char* MQTT_USER     = "esp32";


const char* DEVICE_ID = "esp32_meteo";
const char* FW_VERSION = "1.0.1";   // aggiornala a ogni build

// Intervallo di pubblicazione (900 = 15 minuti; metti 30 solo per i test)
const uint32_t PUBLISH_INTERVAL_SEC = 30;

// Backoff dopo un fallimento di WiFi/MQTT (secondi), poi resta sull'ultimo
const uint32_t BACKOFF_SEC[] = { 60, 120, 300, 600, 900 };
const int      BACKOFF_STEPS = sizeof(BACKOFF_SEC) / sizeof(BACKOFF_SEC[0]);

// Se la fase sveglia supera questo tempo, si forza il deep sleep
const uint32_t WATCHDOG_SEC = 40;

// Finestra minima di campionamento del vento (il WiFi gira in parallelo)
const uint32_t WIND_MIN_MS = 3000;

// Quota della stazione sul livello del mare, in metri (per la pressione ridotta)
const float ALTITUDE_M = 0.0;   // <-- METTI LA TUA QUOTA

// Pin (etichette XIAO ESP32-C3)
#define RAIN_PIN        5    // D3 - RTC-capable, sveglia dal deep sleep
#define WIND_PIN        2    // D0 - impulsi anemometro
#define WIND_VANE_PIN   3    // D1 - ADC banderuola (punto centrale del partitore)
#define VANE_POWER_PIN  10   // D10 - alimenta il partitore solo durante la lettura
#define I2C_SDA_PIN     6    // D4
#define I2C_SCL_PIN     7    // D5

// Tempo di assestamento del partitore dopo l'accensione (alza a 20 se il cavo e' lungo)
const uint32_t VANE_SETTLE_MS = 20;

// Calibrazione (valori tipici SparkFun/Argent, DA VERIFICARE)
const float WIND_KMH_PER_PULSE_PER_SEC = 2.4;
const float RAIN_MM_PER_TIP            = 0.2794;
// ----------------------------------------------------

WiFiClient espClient;
PubSubClient mqtt(espClient);
Adafruit_BME280 bme;

String dataTopic         = String(DEVICE_ID) + "/data";

// ---------- Stato che sopravvive al deep sleep ----------
RTC_DATA_ATTR volatile uint32_t rainPulses = 0;      // basculate non ancora pubblicate
RTC_DATA_ATTR volatile uint32_t rainTotalPulses = 0; // totale cumulativo (mai azzerato)
RTC_DATA_ATTR int64_t  lastPublishUs = 0;            // ultima pubblicazione riuscita (per la media)
RTC_DATA_ATTR volatile int64_t lastTipRtcUs = 0;     // istante dell'ultima basculata
RTC_DATA_ATTR volatile int64_t minTipIntervalUs = INT64_MAX; // intervallo minimo tra due basculate (per l'intensita' di picco)
RTC_DATA_ATTR int64_t  nextAttemptUs = 0;            // quando tentare la prossima pubblicazione
RTC_DATA_ATTR uint32_t seq = 0;                      // numero di sequenza delle pubblicazioni riuscite
RTC_DATA_ATTR bool     wdtFired = false;             // il watchdog software ha forzato lo sleep
RTC_DATA_ATTR char     pendingReset[16] = "";        // causa di reset ancora da comunicare
RTC_DATA_ATTR uint8_t  failCount = 0;                // fallimenti consecutivi
RTC_DATA_ATTR uint8_t  wifiBssid[6];
RTC_DATA_ATTR uint8_t  wifiChannel = 0;
RTC_DATA_ATTR bool     wifiSaved = false;

// ---------- ISR ----------
// Anemometro: totale impulsi + conteggio per ogni secondo (per la raffica)
const int WIND_BUCKETS = 16;                  // finestra massima coperta: 16 s
volatile uint32_t windPulsesSample = 0;
volatile uint16_t windBuckets[WIND_BUCKETS];
volatile uint32_t windStartUs = 0;
volatile uint32_t lastRainUs = 0;

void IRAM_ATTR onWindPulse() {
  windPulsesSample++;
  uint32_t idx = (micros() - windStartUs) / 1000000UL;
  if (idx < WIND_BUCKETS) windBuckets[idx]++;
}

// Differenza RTC_time - esp_timer al boot: permette di avere nell'ISR un
// orologio coerente con quello che sopravvive al deep sleep
volatile int64_t rtcOffsetUs = 0;

// Registra l'istante di una basculata e tiene l'intervallo minimo
inline void IRAM_ATTR registerTipTime(int64_t nowRtcUs) {
  if (lastTipRtcUs > 0) {
    int64_t dt = nowRtcUs - lastTipRtcUs;
    if (dt > 50000 && dt < minTipIntervalUs) minTipIntervalUs = dt;
  }
  lastTipRtcUs = nowRtcUs;
}

void IRAM_ATTR onRainPulse() {
  uint32_t now = micros();
  if (now - lastRainUs > 50000) {   // debounce 50 ms
    rainPulses++;
    rainTotalPulses++;
    lastRainUs = now;
    registerTipTime(rtcOffsetUs + esp_timer_get_time());
  }
}

esp_timer_handle_t watchdogTimer = nullptr;

// ---------- Utilita' ----------
int64_t getRtcTimeUs() {
  struct timeval tv;
  gettimeofday(&tv, nullptr);
  return (int64_t)tv.tv_sec * 1000000LL + (int64_t)tv.tv_usec;
}

uint32_t backoffSeconds() {
  int idx = (int)failCount - 1;
  if (idx < 0) idx = 0;
  if (idx >= BACKOFF_STEPS) idx = BACKOFF_STEPS - 1;
  return BACKOFF_SEC[idx];
}

// ---------- Banderuola ----------
VaneReading vaneTable[16] = {
  { 952,   0 }, {2472,  23 }, {2250,  45 }, {3760,  68 },
  {3723,  90 }, {3831, 113 }, {3357, 135 }, {3589, 158 },
  {2946, 180 }, {3116, 203 }, {1575, 225 }, {1698, 248 },
  { 315, 270 }, { 786, 293 }, { 445, 315 }, {1053, 338 }
};

VaneReading readWindVane() {
  // Alimenta il partitore, aspetta che si assesti, legge, poi lo spegne
  pinMode(VANE_POWER_PIN, OUTPUT);
  digitalWrite(VANE_POWER_PIN, HIGH);
  delay(VANE_SETTLE_MS);

  analogSetAttenuation(ADC_11db);
  uint32_t sum = 0;
  for (int i = 0; i < 8; i++) { sum += analogRead(WIND_VANE_PIN); delayMicroseconds(200); }
  int raw = sum / 8;

  digitalWrite(VANE_POWER_PIN, LOW);   // nessuna corrente nel partitore fino alla prossima lettura

  int bestDelta = 99999;
  VaneReading best = vaneTable[0];
  for (int i = 0; i < 16; i++) {
    int delta = abs(raw - vaneTable[i].adcValue);
    if (delta < bestDelta) { bestDelta = delta; best = vaneTable[i]; }
  }
  LOG("[Sensori] Banderuola ADC: %d -> %d gradi\n", raw, best.degrees);
  return best;
}

// ---------- Deep sleep ----------
// Unico punto di ingresso in deep sleep. Chiamabile anche dal watchdog.
void enterDeepSleep(int64_t sleepUs) {
  if (sleepUs < 1000000LL) sleepUs = 1000000LL;   // minimo 1 s

  // Aspetta che il reed si riapra (l'interrupt e' ancora attivo, quindi
  // eventuali rimbalzi vengono contati): evita il doppio conteggio al wake.
  unsigned long t = millis();
  while (digitalRead(RAIN_PIN) == LOW && millis() - t < 200) delay(2);

  detachInterrupt(digitalPinToInterrupt(RAIN_PIN));

  pinMode(RAIN_PIN, INPUT_PULLUP);   
  esp_err_t err = esp_deep_sleep_enable_gpio_wakeup(1ULL << RAIN_PIN, ESP_GPIO_WAKEUP_GPIO_LOW);
  LOG("[SLEEP] GPIO wake config: %s, pin=%d livello=%d\n", esp_err_to_name(err), RAIN_PIN, digitalRead(RAIN_PIN));
  esp_sleep_enable_timer_wakeup((uint64_t)sleepUs);

  LOG("[Sleep] Deep sleep per max %llds (prima se piove)\n", sleepUs / 1000000LL);
#if DEBUG
  Serial.flush();
#endif
  esp_deep_sleep_start();
}

// Imposta il prossimo tentativo dopo un fallimento e dorme
void scheduleRetryAndSleep() {
  failCount++;
  uint32_t wait = backoffSeconds();
  nextAttemptUs = getRtcTimeUs() + (int64_t)wait * 1000000LL;
  LOG("[Retry] Fallimento #%u, riprovo tra %us\n", failCount, wait);
  enterDeepSleep((int64_t)wait * 1000000LL);
}

// Dorme fino alla prossima pubblicazione gia' programmata
void sleepUntilNextAttempt() {
  int64_t remainingUs = nextAttemptUs - getRtcTimeUs();
  // Protezione: se l'orologio e' andato fuori fase, ricomincia da un intervallo
  int64_t maxUs = (int64_t)((PUBLISH_INTERVAL_SEC > 900 ? PUBLISH_INTERVAL_SEC : 900)) * 1000000LL;
  if (remainingUs > maxUs) remainingUs = (int64_t)PUBLISH_INTERVAL_SEC * 1000000LL;
  enterDeepSleep(remainingUs);
}

// ---------- Watchdog ----------
void watchdogCallback(void*) {
  LOG("[WATCHDOG] Fase sveglia troppo lunga, forzo deep sleep\n");
  wdtFired = true;
  scheduleRetryAndSleep();
}

void startWatchdog() {
  esp_timer_create_args_t args = {};
  args.callback = &watchdogCallback;
  args.name = "awake_wdt";
  esp_timer_create(&args, &watchdogTimer);
  esp_timer_start_once(watchdogTimer, (uint64_t)WATCHDOG_SEC * 1000000ULL);
}

void stopWatchdog() {
  if (watchdogTimer) esp_timer_stop(watchdogTimer);
}

// ---------- WiFi / MQTT ----------
bool connectWiFiFast() {
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  if (USE_STATIC_IP) WiFi.config(STATIC_IP, GATEWAY, SUBNET, DNS);

  for (int attempt = 0; attempt < 2; attempt++) {
    bool fast = (wifiSaved && attempt == 0);
    if (fast) WiFi.begin(WIFI_SSID, WIFI_PASSWORD, wifiChannel, wifiBssid, true);
    else      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    unsigned long t = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t < 8000) delay(10);

    if (WiFi.status() == WL_CONNECTED) {
      memcpy(wifiBssid, WiFi.BSSID(), 6);
      wifiChannel = WiFi.channel();
      wifiSaved = true;
      LOG("[WiFi] OK (%s) in %lums, RSSI %d\n", fast ? "veloce" : "scansione",
          millis() - t, WiFi.RSSI());
      return true;
    }
    // il BSSID salvato non vale piu': riprova con scansione completa
    wifiSaved = false;
    WiFi.disconnect();
    if (!fast) break;   // la scansione completa e' gia' stata provata
  }
  LOG("[WiFi] FALLITO\n");
  return false;
}

bool connectMQTT() {
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setSocketTimeout(5);
  mqtt.setBufferSize(512);   // il default (256) e' troppo piccolo per il payload
  for (int i = 0; i < 3 && !mqtt.connected(); i++) {
    mqtt.connect(DEVICE_ID, MQTT_USER, MQTT_PASSWORD);
    if (!mqtt.connected()) delay(500);
  }
  LOG(mqtt.connected() ? "[MQTT] OK\n" : "[MQTT] FALLITO (rc=%d)\n", mqtt.state());
  return mqtt.connected();
}

// ---------- Ciclo completo. Ritorna true se la pubblicazione e' riuscita ----------
bool readAndPublish() {
  LOG("\n[Pubblica] --- Inizio ---\n");

  // 1) Sensori veloci prima di accendere la radio
  VaneReading vane = readWindVane();

  float temperature = NAN, humidity = NAN, pressure = NAN;
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  if (bme.begin(0x76) || bme.begin(0x77)) {
    bme.setSampling(Adafruit_BME280::MODE_FORCED,
                    Adafruit_BME280::SAMPLING_X1,
                    Adafruit_BME280::SAMPLING_X1,
                    Adafruit_BME280::SAMPLING_X1,
                    Adafruit_BME280::FILTER_OFF);
    bme.takeForcedMeasurement();
    temperature = bme.readTemperature();
    humidity    = bme.readHumidity();
    pressure    = bme.readPressure() / 100.0F;
    LOG("[Sensori] BME280 - %.1fC  %.1f%%  %.1fhPa\n", temperature, humidity, pressure);
  } else {
    LOG("[Sensori] ATTENZIONE: BME280 non trovato\n");
  }

  // 2) Vento: l'interrupt conta mentre il WiFi si connette
  windPulsesSample = 0;
  for (int i = 0; i < WIND_BUCKETS; i++) windBuckets[i] = 0;
  windStartUs = micros();
  pinMode(WIND_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(WIND_PIN), onWindPulse, FALLING);
  uint32_t t0 = millis();

  bool wifiOk = connectWiFiFast();

  while (millis() - t0 < WIND_MIN_MS) delay(10);       // finestra minima
  uint32_t dtMs = millis() - t0;
  detachInterrupt(digitalPinToInterrupt(WIND_PIN));
  uint32_t windPulses = windPulsesSample;

  float windSpeedKmh = (windPulses / (dtMs / 1000.0f)) * WIND_KMH_PER_PULSE_PER_SEC;

  // Raffica: massimo su sotto-finestre di 1 s complete
  uint32_t fullSec = dtMs / 1000;
  if (fullSec > WIND_BUCKETS) fullSec = WIND_BUCKETS;
  uint16_t maxBucket = 0;
  for (uint32_t i = 0; i < fullSec; i++) {
    if (windBuckets[i] > maxBucket) maxBucket = windBuckets[i];
  }
  float windGustKmh = maxBucket * WIND_KMH_PER_PULSE_PER_SEC;
  if (windGustKmh < windSpeedKmh) windGustKmh = windSpeedKmh;   // la raffica non scende sotto la media
  LOG("[Sensori] Vento - %u impulsi in %ums -> media %.1f km/h, raffica %.1f km/h\n",
      windPulses, dtMs, windSpeedKmh, windGustKmh);

  if (!wifiOk || !connectMQTT()) return false;

  // 3) Pioggia: snapshot (l'interrupt continua a contare in background)
  noInterrupts();
  uint32_t sent  = rainPulses;
  uint32_t total = rainTotalPulses;
  int64_t  snapMinTip = minTipIntervalUs;
  minTipIntervalUs = INT64_MAX;          // riparte da zero per il prossimo intervallo
  interrupts();
  float rainMm      = sent  * RAIN_MM_PER_TIP;
  float rainTotalMm = total * RAIN_MM_PER_TIP;
  LOG("[Sensori] Pioggia - %u basculate -> %.2f mm (totale %.2f mm)\n", sent, rainMm, rainTotalMm);

  // Intensita' pioggia (mm/h)
  //  - media: pioggia dall'ultima pubblicazione riuscita / ore trascorse
  //  - picco: dalla coppia di basculate piu' ravvicinata nell'intervallo
  float rateAvg = 0, ratePeak = 0;
  if (sent > 0) {
    float hours = (lastPublishUs > 0) ? (getRtcTimeUs() - lastPublishUs) / 3.6e9f : 0;
    if (hours > 1.0f / 60.0f) rateAvg = rainMm / hours;
    if (snapMinTip != INT64_MAX) ratePeak = RAIN_MM_PER_TIP * 3.6e9f / (float)snapMinTip;
    if (ratePeak < rateAvg) ratePeak = rateAvg;   // il picco non scende sotto la media
  }
  LOG("[Sensori] Intensita' pioggia - media %.1f mm/h, picco %.1f mm/h\n", rateAvg, ratePeak);

  // 4) Pubblica
  JsonDocument doc;
  if (!isnan(temperature)) doc["temperature"] = roundf(temperature * 10) / 10.0;
  if (!isnan(humidity))    doc["humidity"]    = roundf(humidity * 10) / 10.0;
  if (!isnan(pressure)) {
    // Pressione ridotta al livello del mare (formula barometrica standard)
    float pressureMsl = pressure / powf(1.0f - (ALTITUDE_M / 44330.0f), 5.255f);
    doc["pressure"]         = roundf(pressureMsl * 10) / 10.0;   // livello del mare
    doc["pressure_station"] = roundf(pressure * 10) / 10.0;      // misurata alla quota della stazione
  }
  doc["wind_speed"]           = roundf(windSpeedKmh * 10) / 10.0;
  doc["wind_gust"]            = roundf(windGustKmh * 10) / 10.0;
  doc["wind_direction"]       = vane.degrees;
  doc["rain"]                 = roundf(rainMm * 100) / 100.0;       // delta dall'ultima pubblicazione
  doc["rain_total"]           = roundf(rainTotalMm * 100) / 100.0;  // cumulativo
  doc["rain_rate"]            = roundf(ratePeak * 10) / 10.0;       // intensita' di picco, mm/h
  doc["rain_rate_avg"]        = roundf(rateAvg * 10) / 10.0;        // intensita' media, mm/h
  doc["rssi"]                 = WiFi.RSSI();
  doc["fail_count"]           = failCount;                          // fallimenti prima di questo invio
  doc["fw"]                   = FW_VERSION;
  doc["seq"]                  = seq;                                // salti = pacchetti persi
  if (pendingReset[0]) doc["reset_reason"] = pendingReset;          // solo finche' non e' stato comunicato

  char buffer[384];
  size_t n = serializeJson(doc, buffer);
  LOG("[MQTT] Payload: %s\n", buffer);

  bool published = mqtt.publish(dataTopic.c_str(), (const uint8_t*)buffer, n, false);
  mqtt.loop();
  delay(50);                       // lascia svuotare il buffer TCP
  mqtt.disconnect();

  if (published) {
    seq++;
    pendingReset[0] = 0;   // causa di reset comunicata
    // Sottrae SOLO cio' che e' stato inviato: le basculate arrivate
    // durante la pubblicazione restano nel contatore.
    noInterrupts();
    rainPulses -= sent;
    interrupts();
    LOG("[Pubblica] OK, restano %u basculate non inviate\n", rainPulses);
  } else {
    // Pubblicazione fallita: non perdere il picco gia' misurato
    noInterrupts();
    if (snapMinTip < minTipIntervalUs) minTipIntervalUs = snapMinTip;
    interrupts();
  }
  return published;
}

// ---------- Setup (tutta la logica gira qui ad ogni wake) ----------
void setup() {
  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();

  // Pluviometro per primo: meno tempo possibile "ciechi"
  pinMode(RAIN_PIN, INPUT_PULLUP);
  lastRainUs = micros();
  rtcOffsetUs = getRtcTimeUs() - esp_timer_get_time();
  if (cause == ESP_SLEEP_WAKEUP_GPIO) {
    rainPulses++;
    rainTotalPulses++;
    registerTipTime(getRtcTimeUs());   // approssimata: la basculata e' avvenuta poco prima del boot
  }
  attachInterrupt(digitalPinToInterrupt(RAIN_PIN), onRainPulse, FALLING);

  startWatchdog();

  // Causa del reset: si memorizza se anomala e si comunica al primo invio riuscito
  esp_reset_reason_t rr = esp_reset_reason();
  if (wdtFired) {
    strcpy(pendingReset, "AWAKE_WDT");
    wdtFired = false;
  } else {
    switch (rr) {
      case ESP_RST_BROWNOUT: strcpy(pendingReset, "BROWNOUT"); break;
      case ESP_RST_PANIC:    strcpy(pendingReset, "PANIC");    break;
      case ESP_RST_INT_WDT:  strcpy(pendingReset, "INT_WDT");  break;
      case ESP_RST_TASK_WDT: strcpy(pendingReset, "TASK_WDT"); break;
      case ESP_RST_WDT:      strcpy(pendingReset, "WDT");      break;
      case ESP_RST_POWERON:  strcpy(pendingReset, "POWERON");  break;
      default: break;        // DEEPSLEEP (normale) e altri: niente da segnalare
    }
  }

#if DEBUG
  Serial.begin(115200);
  if (cause != ESP_SLEEP_WAKEUP_GPIO) delay(2000);   // solo per vedere il log
  Serial.println("\n==================== WAKE ====================");
  Serial.printf("[Sveglia] Causa: %s\n",
                cause == ESP_SLEEP_WAKEUP_GPIO  ? "PIOGGIA" :
                cause == ESP_SLEEP_WAKEUP_TIMER ? "TIMER" : "PRIMO AVVIO");
  Serial.printf("[Pioggia] In attesa di invio: %u basculate\n", rainPulses);
#endif

  int64_t nowUs = getRtcTimeUs();

  // Dopo un brownout la batteria e' probabilmente debole: NON accendere subito il
  // WiFi (rischio di loop di reset). Si dorme un intervallo e si ritenta.
  if (rr == ESP_RST_BROWNOUT) {
    LOG("[Reset] BROWNOUT: salto la pubblicazione e dormo\n");
    nextAttemptUs = nowUs + (int64_t)PUBLISH_INTERVAL_SEC * 1000000LL;
    stopWatchdog();
    sleepUntilNextAttempt();
  }

  bool firstBoot = (cause == ESP_SLEEP_WAKEUP_UNDEFINED || nextAttemptUs == 0);

  if (firstBoot || nowUs >= nextAttemptUs) {
    if (readAndPublish()) {
      failCount = 0;
      lastPublishUs = getRtcTimeUs();
      nextAttemptUs = getRtcTimeUs() + (int64_t)PUBLISH_INTERVAL_SEC * 1000000LL;
      WiFi.disconnect(true);
      WiFi.mode(WIFI_OFF);
      stopWatchdog();
      sleepUntilNextAttempt();
    } else {
      WiFi.disconnect(true);
      WiFi.mode(WIFI_OFF);
      stopWatchdog();
      scheduleRetryAndSleep();   // backoff progressivo
    }
  } else {
    LOG("[Timing] Non ancora ora di pubblicare, torno a dormire\n");
    stopWatchdog();
    sleepUntilNextAttempt();
  }
}

void loop() {
  // Non usato
}
