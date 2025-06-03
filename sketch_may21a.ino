#include <Wire.h>
#include "MAX30100_PulseOximeter.h"

// Pinos I2C padrão do ESP32 (você pode ajustar se necessário)
#define I2C_SDA 21
#define I2C_SCL 22

#define REPORTING_PERIOD_MS 1000

uint32_t tsLastReport = 0;
PulseOximeter pox;

void onBeatDetected() {
  Serial.println("Batimento detectado");
}

void setup() {
  Serial.begin(115200); // Taxa mais alta comum no ESP32

  // Inicializa I2C com pinos definidos
  Wire.begin(I2C_SDA, I2C_SCL);

  Serial.print("Iniciando funcionamento do sensor... ");

  if (!pox.begin()) {
    Serial.println("Falhou na inicialização do sensor!");
    while (1);
  } else {
    Serial.println("Sensor iniciado com sucesso!");
  }

  pox.setIRLedCurrent(MAX30100_LED_CURR_7_6MA);
  pox.setOnBeatDetectedCallback(onBeatDetected);
}

void loop() {
  pox.update();

  if (millis() - tsLastReport > REPORTING_PERIOD_MS) {
    Serial.print("Taxa de Batimento : ");
    Serial.print(pox.getHeartRate());
    Serial.print(" bpm / Saturação de Oxigênio (SpO2): ");
    Serial.print(pox.getSpO2());
    Serial.println(" %");

    tsLastReport = millis();
  }
}
