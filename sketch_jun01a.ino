#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <MAX30100_PulseOximeter.h>

// Wi-Fi
const char* wifi_ssid = "";
const char* wifi_password = "";

// MQTT
const int mqtt_port = 1883;
const char* mqtt_broker = "broker.hivemq.com";
const char* mqtt_topic = "sensor/leitura/app1234";
const char* mqtt_alert_topic = "sensor/alerta/app1234";

// LED
const int led_pin = 2;

// Sensores
#define SDA_PIN 21
#define SCL_PIN 22
Adafruit_MPU6050 mpu;
PulseOximeter pox;

// MQTT
WiFiClient client;
PubSubClient mqtt_client(client);

// Controle de tempo
unsigned long ultima_tentativa_reconexao = 0;
unsigned long ultimo_alerta = 0;
const unsigned long intervalo_alerta = 60 * 1000;

void conectar_wifi() {
  Serial.println("Conectando \u00e0 rede Wi-Fi...");
  WiFi.begin(wifi_ssid, wifi_password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    digitalWrite(led_pin, LOW);
  }
  Serial.println("\nWi-Fi conectado!");
  Serial.print("IP local: ");
  Serial.println(WiFi.localIP());
  digitalWrite(led_pin, HIGH);
}

void reconectar_mqtt() {
  Serial.println("Conectando ao broker MQTT...");
  if (mqtt_client.connect("esp32_80989")) {
    Serial.println("Conectado ao MQTT!");
  } else {
    Serial.print("Falha na conex\u00e3o MQTT, rc=");
    Serial.println(mqtt_client.state());
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(led_pin, OUTPUT);

  conectar_wifi();
  mqtt_client.setServer(mqtt_broker, mqtt_port);
  ArduinoOTA.begin();
  Serial.println("OTA pronto");

  // Inicializa I2C e sensores
  Wire.begin(SDA_PIN, SCL_PIN);

  Serial.println("Inicializando MPU6050...");
  if (!mpu.begin()) {
    Serial.println("Erro: MPU6050 n\u00e3o encontrado.");
    while (true);
  }
  Serial.println("MPU6050 pronto.");

  Serial.println("Inicializando MAX30100...");
  if (!pox.begin()) {
    Serial.println("Erro: MAX30100 n\u00e3o encontrado.");
    while (true);
  }
  pox.setIRLedCurrent(MAX30100_LED_CURR_24MA);
  Serial.println("MAX30100 pronto.");
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    conectar_wifi();
  }

  if (!mqtt_client.connected()) {
    unsigned long agora = millis();
    if (agora - ultima_tentativa_reconexao > 5000) {
      ultima_tentativa_reconexao = agora;
      reconectar_mqtt();
    }
  } else {
    mqtt_client.loop();
    ArduinoOTA.handle();
    pox.update();  // Atualiza sensor MAX30100

    static unsigned long ultima_mensagem = 0;
    if (millis() - ultima_mensagem > 2000) {
      ultima_mensagem = millis();

      // Leitura MAX30100
      float batimentos = pox.getHeartRate();
      float oximetria = pox.getSpO2();

      // Leitura MPU6050
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);

      // Acelera\u00e7\u00e3o total (m\u00f3dulo)
      float aceleracao_total = sqrt(pow(a.acceleration.x, 2) +
                                    pow(a.acceleration.y, 2) +
                                    pow(a.acceleration.z, 2));

      // Rota\u00e7\u00e3o total (m\u00f3dulo)
      float rotacao_total = sqrt(pow(g.gyro.x, 2) +
                                 pow(g.gyro.y, 2) +
                                 pow(g.gyro.z, 2));

      // Monta JSON
      String mensagem_json = "{";
      mensagem_json += "\"dispositivo\":\"ESP32\",";
      mensagem_json += "\"batimentos\":" + String(batimentos, 1) + ",";
      mensagem_json += "\"oximetria\":" + String(oximetria, 1) + ",";
      mensagem_json += "\"aceleracao\":" + String(aceleracao_total, 2) + ",";
      mensagem_json += "\"rotacao\":" + String(rotacao_total, 2);
      mensagem_json += "}";

      Serial.print("Enviando JSON MQTT: ");
      Serial.println(mensagem_json);

      mqtt_client.publish(mqtt_topic, mensagem_json.c_str());

      // Envia alerta, se necess\u00e1rio
      if ((batimentos > 95 || batimentos < 5 || aceleracao_total < 0.2) &&
          (millis() - ultimo_alerta > intervalo_alerta)) {
        mqtt_client.publish(mqtt_alert_topic, mensagem_json.c_str());
        ultimo_alerta = millis();
      }
    }
  }
}
