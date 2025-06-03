#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <MAX30100_PulseOximeter.h>

// Definições dos pinos I2C
#define SDA_PIN 21
#define SCL_PIN 22

// Instâncias dos sensores
Adafruit_MPU6050 mpu;
PulseOximeter pox;

// Controle de tempo
uint32_t tsLastReport = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Inicializa I2C
  Wire.begin(SDA_PIN, SCL_PIN);

  // Inicializa MPU6050
  Serial.println("Inicializando MPU6050...");
  if (!mpu.begin()) {
    Serial.println("Falha ao encontrar MPU6050. Verifique a conexão.");
    while (true);
  }
  Serial.println("MPU6050 pronto.");

  // Inicializa MAX30100
  Serial.println("Inicializando MAX30100...");
  if (!pox.begin()) {
    Serial.println("Falha ao iniciar MAX30100. Verifique a conexão.");
    while (true);
  }
  pox.setIRLedCurrent(MAX30100_LED_CURR_24MA);  // Corrente ajustável
  Serial.println("MAX30100 pronto.");
}

void loop() {
  // Atualiza o estado do sensor MAX30100
  pox.update();

  // Lê dados do MPU6050
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Exibe os dados a cada 1 segundo
  if (millis() - tsLastReport > 1000) {
    Serial.println("----- Leitura dos Sensores -----");

    // Dados do MAX30100
    Serial.print("BPM: ");
    Serial.print(pox.getHeartRate());
    Serial.print(" | SpO2: ");
    Serial.println(pox.getSpO2());

    // Dados do MPU6050
    Serial.print("Aceleração (m/s^2): X=");
    Serial.print(a.acceleration.x);
    Serial.print(" Y=");
    Serial.print(a.acceleration.y);
    Serial.print(" Z=");
    Serial.println(a.acceleration.z);

    Serial.print("Giroscópio (rad/s): X=");
    Serial.print(g.gyro.x);
    Serial.print(" Y=");
    Serial.print(g.gyro.y);
    Serial.print(" Z=");
    Serial.println(g.gyro.z);

    Serial.println("-------------------------------");
    tsLastReport = millis();
  }
}
