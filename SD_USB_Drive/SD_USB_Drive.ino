/*
========================================================
  SD_USB_Drive - "MODO PENDRIVE" (USB MSC)
========================================================
  Expoe o cartao SD do ScratchPad como um PENDRIVE no PC, sem precisar
  tirar o cartao da carcaca. Pluga o USB no PC -> aparece um drive novo ->
  arraste os WAV pras pastas /scratch e /beats -> ejete -> desligue.

  Depois e so gravar o ScratchPad_S3_v10 de volta pra usar o instrumento.

  >>> CONFIG OBRIGATORIA no Arduino IDE pra ESTE sketch <<<
   - Tools -> USB Mode: "USB-OTG (TinyUSB)"
   - Tools -> USB CDC On Boot: "Enabled"   (pra ver o Serial)
   (Na v10 voce volta pra "Hardware CDC and JTAG" / CDC On Boot Disabled.)

  Pinos do SD = iguais a v10 (MISO/MOSI invertidos conforme a solda):
========================================================
*/

#include "USB.h"
#include "USBMSC.h"
#include <SPI.h>
#include <SD.h>

#define SD_SCK  13
#define SD_MISO 11
#define SD_MOSI 12
#define SD_CS   10
#define SD_FREQ 10000000

SPIClass spiSD(FSPI);
USBMSC msc;

static const uint32_t SECTOR = 512;

// ---- callbacks: ponte USB <-> setores do cartao ----
int32_t onWrite(uint32_t lba, uint32_t offset, uint8_t *buffer, uint32_t bufsize) {
  uint32_t count = bufsize / SECTOR;
  for (uint32_t i = 0; i < count; i++)
    if (!SD.writeRAW(buffer + i * SECTOR, lba + i)) return -1;
  return bufsize;
}

int32_t onRead(uint32_t lba, uint32_t offset, void *buffer, uint32_t bufsize) {
  uint32_t count = bufsize / SECTOR;
  for (uint32_t i = 0; i < count; i++)
    if (!SD.readRAW((uint8_t *)buffer + i * SECTOR, lba + i)) return -1;
  return bufsize;
}

bool onStartStop(uint8_t power_condition, bool start, bool load_eject) {
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(800);
  Serial.println("=== SD_USB_Drive (modo pendrive) ===");

  spiSD.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  if (!SD.begin(SD_CS, spiSD, SD_FREQ)) {
    Serial.println("SD nao iniciou! Confira fiacao/cartao.");
    return;
  }
  Serial.printf("SD OK: %llu MB\n", SD.cardSize() / (1024ULL * 1024ULL));

  msc.vendorID("ScratchP");
  msc.productID("SD Card");
  msc.productRevision("1.0");
  msc.onRead(onRead);
  msc.onWrite(onWrite);
  msc.onStartStop(onStartStop);
  msc.mediaPresent(true);
  msc.begin(SD.numSectors(), SD.sectorSize());

  USB.begin();
  Serial.println("Pronto! O cartao deve aparecer como drive no PC.");
}

void loop() {
  delay(100);
}
