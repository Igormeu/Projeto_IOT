#include <Wire.h>

void setup() {
  Wire.begin(21, 22); // SDA, SCL (ajuste se necessário)
  Serial.begin(115200);
  Serial.println("\nEscaneando dispositivos I2C...");

  byte count = 0;
  for (byte i = 1; i < 127; i++) {
    Wire.beginTransmission(i);
    if (Wire.endTransmission() == 0) {
      Serial.print("Dispositivo I2C encontrado no endereço 0x");
      Serial.println(i, HEX);
      count++;
      delay(5);
    }
  }

  if (count == 0) Serial.println("Nenhum dispositivo I2C encontrado.");
  else Serial.println("Scan concluído.");
}

void loop() {}
