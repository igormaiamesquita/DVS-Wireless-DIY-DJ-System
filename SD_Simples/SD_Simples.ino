/*
  SD_Simples - teste minimo do cartao SD
  Usa o objeto SPI global + velocidade baixa (jeito mais "compativel" no S3).
  Serial 115200 (UART). Pinos conforme sua solda: MISO=11, MOSI=12.
*/

#include <SPI.h>
#include <SD.h>

#define PIN_SCK  13
#define PIN_MISO 11
#define PIN_MOSI 12
#define PIN_CS   10

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("\n=== SD_Simples ===");

  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);

  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);

  Serial.println("Tentando SD.begin @ 400 kHz ...");
  if (SD.begin(PIN_CS, SPI, 400000)) {
    Serial.println(">>> SD OK! <<<");

    uint8_t t = SD.cardType();
    Serial.print("Tipo: ");
    if (t == CARD_NONE) Serial.println("NENHUM");
    else if (t == CARD_MMC) Serial.println("MMC");
    else if (t == CARD_SD) Serial.println("SDSC");
    else if (t == CARD_SDHC) Serial.println("SDHC/SDXC");
    else Serial.println("?");
    Serial.printf("Tamanho: %llu MB\n", SD.cardSize() / (1024ULL * 1024ULL));

    Serial.println("Raiz:");
    File root = SD.open("/");
    File e = root.openNextFile();
    while (e) {
      Serial.printf("  %s%s\n", e.name(), e.isDirectory() ? "/" : "");
      e = root.openNextFile();
    }
    root.close();
  } else {
    Serial.println("!!! Falhou.");
  }
}

void loop() {}
