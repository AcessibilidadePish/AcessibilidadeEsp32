/*******************************************************************
 * ESP32 + NEO-6M + Azure IoT Hub MQTT
 * LED RGB: amarelo piscando (busy) | verde (sucesso) | vermelho (erro)
 *******************************************************************/
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <PubSubClient.h>
#include <TinyGPS.h>

/* ────────── PINOS ────────── */
#define PIN_GPS_RX 16
#define PIN_GPS_TX 17
#define PIN_BUTTON 23

/* ────────── LED RGB COMUM CÁTODO ────────── */
#define LED_R 25
#define LED_G 26
#define LED_B 27

/* ────────── REDE / API ────────── */
const char* WIFI_SSID = "philipicalt";
const char* WIFI_PASS = "190600phi";
const char* API_URL   = "https://othergreyroof11.conveyor.cloud/Teste/TestePost";

/* ────────── MQTT (Azure IoT Hub) ────────── */
const char* host     = "iot-pish.azure-devices.net";
const int   port     = 8883;
const char* deviceId = "esp32_gps_01";
const char* sasToken = "SharedAccessSignature sr=iot-pish.azure-devices.net%2Fdevices%2Fesp32_gps_01&sig=M6La7DmNu05agqb%2F8oKQyLceRpvGVSU7SYKhMVdYdDA%3D&se=1780345218";

String  mqttUsername   = String(host) + "/" + deviceId + "/?api-version=2021-04-12";
const char* mqttClientId = deviceId;
const char* topicTelemetry = "devices/esp32_gps_01/messages/events/";
const char* topicC2D       = "devices/esp32_gps_01/messages/devicebound/#";

const char* ACK_OK           = "EN_ROUTE";
const uint32_t ACK_TIMEOUTMS = 10000;

/* ────────── DEBUG GPS MOCK ────────── */
#define USE_GPS_MOCK true
const float MOCK_LAT = -19.9275497f;
const float MOCK_LNG = -43.9941561f;

/* ────────── GLOBAIS ────────── */
HardwareSerial gpsSerial(1);
TinyGPS         gps;
WiFiClientSecure net;
PubSubClient     mqtt(net);
HTTPClient       http;

enum State { IDLE, WAIT_GPS, POSTING, WAIT_ACK } state = IDLE;
uint32_t ackStart = 0;

/* ────────── LED: cores lógicas ────────── */
enum LedColor { LED_ON, LED_OFF, LED_YELLOW, LED_GREEN, LED_RED };

/* estado atual do LED */
LedColor  curColor   = LED_OFF;
bool      curBlink   = false;
uint32_t  blinkTimer = 0;
bool      blinkOn    = false;

/* define cor + piscada desejada */
void setLed(LedColor c, bool blink = false)
{
  curColor  = c;
  curBlink  = blink;
  blinkOn   = true;          // força ligar logo no primeiro update
  blinkTimer = millis();
  
  updateLed();  
}

/* efetiva (e pisca se necessário) – chame em loop() */
void updateLed()
{
  if (curBlink && millis() - blinkTimer >= 300) {
    blinkOn    = !blinkOn;
    blinkTimer = millis();
  } else if (!curBlink) {
    blinkOn = true;          // mantém ligado fixo
  }

  /* escolhe quais pinos acender */
  bool r = LOW, g = LOW, b = LOW;          // LOW = aceso em comum-cátodo
  if (blinkOn) {
    switch (curColor) {
      case LED_RED:    r = HIGH;  g = LOW; b = LOW; break;
      case LED_GREEN:  r = LOW; g = HIGH;  b = LOW; break;
      case LED_YELLOW: r = LOW; g = LOW; b = HIGH;  break;  
      case LED_ON: r = HIGH; g = HIGH; b = HIGH;  break; 
      default: /* OFF */  r = g = b = LOW; break;
    }
  } else {
    r = g = b = LOW;        // fase “apagada” do piscar
  }

  digitalWrite(LED_R, r);
  digitalWrite(LED_G, g);
  digitalWrite(LED_B, b);
}

/* ────────── MQTT callbacks ────────── */
void onMqttMessage(char* topic, byte* payload, unsigned int len) {
  payload[len] = '\0';
  String msg = (char*)payload;
  Serial.printf("[MQTT] Mensagem recebida <%s>: %s\n", topic, msg.c_str());

  if (msg == "EN_ROUTE") {
    Serial.println("[MQTT] ACK OK ➜ sucesso.");
    setLed(LED_GREEN);
    state = IDLE;
    delay(5000);
  }
}

void ensureMqtt() {
  if (mqtt.connected()) return;

  Serial.print("[MQTT] Conectando...");
  net.setInsecure();               // ⚠️ teste
  mqtt.setServer(host, port);

  if (mqtt.connect(mqttClientId, mqttUsername.c_str(), sasToken)) {
    Serial.println(" conectado!");
    mqtt.subscribe(topicC2D);
  } else {
    Serial.printf(" falha (%d)\n", mqtt.state());
    delay(2000);
  }
}

/* ────────── GPS helpers ────────── */
bool waitForGPSFix(uint32_t t_ms = 30000) {
  Serial.println("[GPS] Aguardando fix...");
  uint32_t t0 = millis();
  float lat = 0, lng = 0;

  while (millis() - t0 < t_ms) {
    while (gpsSerial.available())
      gps.encode(gpsSerial.read());

    gps.f_get_position(&lat, &lng);
    if (lat != TinyGPS::GPS_INVALID_F_ANGLE &&
        lng != TinyGPS::GPS_INVALID_F_ANGLE) {
      Serial.printf("[GPS] Fix! %.6f, %.6f\n", lat, lng);
      return true;
    }
    setLed(LED_YELLOW, true);                 // pisca enquanto espera
    delay(10);
  }
  return false;
}

bool postLocation(float lat, float lng) {
  String body = "{\"lat\":" + String(lat,6) + ",\"lng\":" + String(lng,6) + "}";
  Serial.println("[HTTP] POST: " + body);

  http.begin(API_URL);
  http.addHeader("Content-Type", "application/json");
  int code = http.POST(body);
  http.end();

  Serial.printf("[HTTP] Status: %d\n", code);
  return code >= 200 && code < 300;
}

/* ────────── SETUP ────────── */
void setup() {
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
  setLed(LED_ON);
  updateLed();
  pinMode(PIN_BUTTON, INPUT_PULLUP);

  Serial.begin(115200);
  gpsSerial.begin(9600, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) { Serial.print('.'); delay(500); }
  Serial.println("\nWi-Fi OK");

  mqtt.setCallback(onMqttMessage);
  
  setLed(LED_OFF);
  updateLed();
}

/* ────────── LOOP ────────── */
void loop() {
  while (gpsSerial.available()) gps.encode(gpsSerial.read());
  updateLed();  
  mqtt.loop();
  ensureMqtt();


  bool btn = digitalRead(PIN_BUTTON);
  if (btn == LOW) {
    Serial.println("[BOTÃO] Pressionado.");
    state = WAIT_GPS;
  }

  /* ─ Máquina de estados ─ */
  switch (state) {
    case WAIT_GPS: {
      setLed(LED_YELLOW, true);   
      float lat = 0, lng = 0;

      bool gotFix = USE_GPS_MOCK ? true : waitForGPSFix();
      if (gotFix) {
        if (!USE_GPS_MOCK) gps.f_get_position(&lat, &lng);
        else { lat = MOCK_LAT; lng = MOCK_LNG; }

        state = POSTING;
        /* cai direto no POSTING na mesma iteração */
      } else {
        Serial.println("[ERRO] Sem fix.");
        setLed(LED_RED);
        delay(5000);
        state = IDLE;
      }
      break;
    }

    case POSTING: {
      /* Piscando amarelo enquanto envia */
      setLed(LED_YELLOW, true);

      float lat = USE_GPS_MOCK ? MOCK_LAT : 0;
      float lng = USE_GPS_MOCK ? MOCK_LNG : 0;
      if (!USE_GPS_MOCK) gps.f_get_position(&lat, &lng);
      
      bool okPost = postLocation(lat, lng);

      if (okPost) {
        Serial.println("[POST] OK, aguardando ACK...");
        ackStart = millis();
        state = WAIT_ACK;
      } else {
        Serial.println("[ERRO] POST falhou.");
        setLed(LED_RED);
        delay(5000);
        state = IDLE;
      }
      break;
    }

    case WAIT_ACK:
      setLed(LED_YELLOW, true);                              // continua piscando
      if (millis() - ackStart > ACK_TIMEOUTMS) {
        Serial.println("[TIMEOUT] Sem ACK.");
        setLed(LED_RED);
        delay(5000);
        state = IDLE;
      }
      else{
        state = IDLE;
      }
      break;

    case IDLE:
    default:
      setLed(LED_OFF);
      delay(10);
      break;
  }
}
