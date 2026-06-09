/*
========================================================
  SD_Test - DIAGNOSTICO DO CARTAO SD
========================================================
  Tenta iniciar o SD em varias velocidades E nas duas combinacoes
  de MISO/MOSI, e diz qual funcionou. Use o Serial (UART, 115200).

  Config Arduino IDE: pode usar a sua normal
  (USB Mode: Hardware CDC and JTAG | CDC On Boot: Disabled).

  Pinos fixos: SCK=13, CS=10. MISO/MOSI sao testados nos dois jeitos.
========================================================
*/

#include <SPI.h>
#include <SD.h>

#define SD_SCK 13
#define SD_CS  10
#define PIN_A  11      // um dos dois (voce soldou aqui)
#define PIN_B  12      // o outro

SPIClass spiSD(FSPI);

uint32_t freqs[] = {20000000, 10000000, 4000000, 1000000, 400000};
const int NF = sizeof(freqs) / sizeof(freqs[0]);

bool tentar(int miso, int mosi, uint32_t freq) {
  SD.end();
  spiSD.end();
  delay(20);
  spiSD.begin(SD_SCK, miso, mosi, SD_CS);
  return SD.begin(SD_CS, spiSD, freq);
}

void mostrarCartao() {
  uint8_t t = SD.cardType();
  Serial.print("  Tipo: ");
  if (t == CARD_NONE)      Serial.println("NENHUM");
  else if (t == CARD_MMC)  Serial.println("MMC");
  else if (t == CARD_SD)   Serial.println("SDSC");
  else if (t == CARD_SDHC) Serial.println("SDHC/SDXC");
  else                     Serial.println("DESCONHECIDO");
  Serial.printf("  Tamanho: %llu MB\n", SD.cardSize() / (1024ULL * 1024ULL));

  Serial.println("  Conteudo da raiz:");
  File root = SD.open("/");
  if (root) {
    File e = root.openNextFile();
    while (e) {
      Serial.printf("    %s%s\n", e.name(), e.isDirectory() ? "/" : "");
      e = root.openNextFile();
    }
    root.close();
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== SD_Test: procurando combinacao que funciona ===");

  struct { int miso, mosi; const char* desc; } combos[] = {
    { PIN_A, PIN_B, "MISO=11  MOSI=12  (o que esta na v10)" },
    { PIN_B, PIN_A, "MISO=12  MOSI=11  (invertido)" },
  };

  for (int c = 0; c < 2; c++) {
    for (int i = 0; i < NF; i++) {
      Serial.printf("Tentando %s  @ %lu Hz ... ", combos[c].desc, freqs[i]);
      if (tentar(combos[c].miso, combos[c].mosi, freqs[i])) {
        Serial.println("OK! <<<<<<");
        Serial.printf(">>> USE: MISO=%d  MOSI=%d  SD_FREQ=%lu\n",
                      combos[c].miso, combos[c].mosi, freqs[i]);
        mostrarCartao();
        Serial.println("=== Achou. Pode parar aqui. ===");
        return;
      }
      Serial.println("falhou");
      delay(100);
    }
  }

  Serial.println("\n!!! Nenhuma combinacao funcionou.");
  Serial.println("Cheque: cartao inserido? formatado FAT32? VCC no 5V? GND comum?");
}

void loop() {}
