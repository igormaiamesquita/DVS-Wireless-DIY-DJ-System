/*
========================================================
  ScratchPad S3 - v10  (SD + ENCODER EC11)
========================================================
  Base: v8 (scratch de sample real + motor slip/low-heat + crossfader).
  Novo nesta versao:
   - Cartao SD (modulo HW-125) lendo a pasta /scratch
   - Encoder EC11: GIRAR navega os samples, CLICAR (SW) carrega o selecionado
   - 6 botoes (leitura simples com debounce) p/ funcoes ao vivo (provisorio)
   - Fallback: se o SD falhar ou /scratch estiver vazia, gera sample de teste

  Pinos do SD foram ajustados pro jeito que voce soldou:
        MISO = 11   |   MOSI = 12   (invertidos de proposito - ok no S3)

  IMPORTANTE (carregar arquivos sem tirar o cartao):
   Use o sketch separado "SD_USB_Drive.ino" (modo pendrive USB MSC) pra copiar
   os WAV pro cartao pelo PC. Estrutura no cartao:
        /scratch/   -> samples curtos pra esfregar (WAV 16-bit mono 22050 Hz)
        /beats/     -> batidas (usado na proxima etapa, streaming)

  Config Arduino IDE desta v10: igual a sua de sempre
   (USB Mode: Hardware CDC and JTAG | CDC On Boot: Disabled).
========================================================
*/

#include <SimpleFOC.h>
#include <driver/i2s.h>
#include <math.h>
#include "esp_heap_caps.h"
#include <SPI.h>
#include <SD.h>
#include "freertos/stream_buffer.h"
#include "freertos/semphr.h"
#include <WiFi.h>
#include <WebServer.h>
#include <Update.h>

// =====================================================
// Motor / DRV8313 / AS5600
// =====================================================
#define DRV_IN1 4
#define DRV_IN2 5
#define DRV_IN3 6
#define AS5600_SDA 8
#define AS5600_SCL 9

BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(DRV_IN1, DRV_IN2, DRV_IN3);
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

// Velocidades padrao de toca-disco (rad/s; negativo = sentido). Long-press no botao 3 alterna.
float SPEED_33 = -3.49f;   // 33 1/3 RPM  (~1.8s de audio por volta)
float SPEED_45 = -4.71f;   // 45 RPM
float gTargetVel = SPEED_33;          // alvo atual do prato (alterna 33/45)
#define NOMINAL_VEL SPEED_33          // referencia de TOM 1x (sempre o 33; no 45 toca mais agudo)

// >>>>>>>>>>>>>>>> AJUSTES RAPIDOS (mexa aqui) <<<<<<<<<<<<<<<<
float MOTOR_TORQUE   = 3.5;   // FORCA/firmeza do motor (volts). Maior = mais firme/preso. 2-6.
float RETORNO        = 6.0;   // rapidez do retorno ao soltar. Maior=firme/direto; baixo=suave
float DEADBAND       = 0.5;   // folga antes de ceder ao toque. Menor = mais firme/preso ao giro
float FIRMEZA        = 0.20;  // rigidez do controle (PID P). Maior = mais preso/responsivo (cuidado: chia)
float SCRATCH_PITCH  = 1.0;   // trim fino de tom (1.0 = normal)
float SCRATCH_PARADA = 0.02;  // congela o som qdo o prato esta quase parado (anti-ruido)
float MOTOR_FILTRO   = 0.02;  // suavidade do controle do motor (nao afeta o tom). 0.01 a 0.05
float VOLUME_MESTRE  = 0.90;  // volume geral (0.0 a ~1.2)
float BEAT_VOL       = 0.60;  // volume da BATIDA relativo ao scratch (0.0 a 1.0)
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

// =====================================================
// Hall / crossfader
// =====================================================
#define HALL_PIN 1
int hallMin = 4095, hallMax = 0;
const int  CUT_THRESH = 30;
const bool CUT_INVERT = true;
const int  HYST = 5;

// =====================================================
// SD (modulo HW-125)  -- MISO/MOSI invertidos conforme a solda
// =====================================================
#define SD_SCK  13
#define SD_MISO 11
#define SD_MOSI 12
#define SD_CS   10
#define SD_FREQ 10000000        // 10 MHz; se der erro de leitura, baixe p/ 4000000

SPIClass spiSD(FSPI);

// =====================================================
// Encoder EC11
// =====================================================
#define ENC_A  40
#define ENC_B  41
#define ENC_SW 42

// =====================================================
// Botoes (GND comum)  -- GPIO 0 e strapping (nao segurar no boot)
// =====================================================
const uint8_t BTN_PINS[6] = {21, 47, 48, 14, 2, 0};
#define MAINT_BTN 21   // segurar este botao ao LIGAR -> modo manutencao (WiFi/OTA/upload)

// =====================================================
// I2S / UDA1334A
// =====================================================
#define I2S_BCLK 15
#define I2S_WSEL 16
#define I2S_DIN  17
#define SAMPLE_RATE 22050

// =====================================================
// Estado de audio (compartilhado)
// =====================================================
volatile float gVol         = VOLUME_MESTRE;   // volume mestre (botoes ajustam em runtime)
volatile float gSpeedFactor = 0.0f;    // ratio de reproducao (do prato)
volatile bool  gCut         = false;   // crossfader
volatile bool  gMuteScratch = false;   // mute manual (botao)

const float MAX_RATIO_FWD = 1.10f;
const float MAX_RATIO_REV = 3.00f;

// =====================================================
// Buffer do sample (PSRAM)
// =====================================================
int16_t* gSample = nullptr;
int32_t  gSampleLen = 0;
volatile bool gAudioReady = false;     // false durante troca de sample -> silencio
volatile int32_t scratchPos = 0;       // (legado, nao usado no position-lock)

// --- POSITION-LOCK: audio colado na POSICAO do prato (agulha fixa) ---
// gFramesPerRad: tom normal (1.0x no giro nominal)
double gFramesPerRad = (double)SAMPLE_RATE / NOMINAL_VEL;
portMUX_TYPE posMux = portMUX_INITIALIZER_UNLOCKED;
volatile double gScratchTarget = 0.0;  // posicao do prato em frames (cumulativo)

// =====================================================
// Lista de arquivos /scratch
// =====================================================
#define MAX_FILES 64
String scratchFiles[MAX_FILES];
int    scratchCount = 0;
int    selIndex     = 0;

// =====================================================
// Batidas /beats (streaming do SD)
// =====================================================
String beatFiles[MAX_FILES];
int    beatCount = 0;
int    beatIndex = 0;
volatile bool  gBeatPlaying = true;        // play/pause da batida
volatile int   gBeatReq     = 0;           // pedido: +1 = proxima, -1 = anterior
volatile float gBeatVol     = BEAT_VOL;    // volume da batida (runtime)

StreamBufferHandle_t beatStream = NULL;    // fila de audio da batida (produtor->audio)
SemaphoreHandle_t    sdMutex    = NULL;    // serializa acesso ao SD (scratch x batida)

File     beatFile;                         // arquivo da batida atual (aberto)
bool     beatOpen     = false;
uint16_t beatChannels = 1;
uint32_t beatDataPos  = 0;                 // inicio do PCM no arquivo
uint32_t beatDataEnd  = 0;                 // fim do PCM

// =====================================================
// Gravacao /records (grava o MIX final no SD)
// =====================================================
StreamBufferHandle_t recStream = NULL;     // audio -> gravador
volatile bool gRecording = false;          // status
volatile int  gRecCmd    = 0;              // 1 = comecar, -1 = parar (pedido do botao)

// =====================================================
// WRAP / interpolacao
// =====================================================
static inline int32_t wrapPos(int32_t p) {
  int32_t maxp = gSampleLen << 8;
  if (maxp <= 0) return 0;
  while (p >= maxp) p -= maxp;
  while (p < 0)     p += maxp;
  return p;
}
static inline int16_t interpMono(int32_t pos) {
  int frame = pos >> 8;
  int next  = frame + 1;
  if (next >= gSampleLen) next = 0;
  float frac = (pos & 0xFF) / 256.0f;
  float s = gSample[frame] + (gSample[next] - gSample[frame]) * frac;
  return (int16_t)s;
}

// =====================================================
// Carrega um WAV (16-bit) do SD pra PSRAM (scan de chunks)
// =====================================================
int16_t* loadWavToPSRAM(const char* path, int32_t* lenOut) {
  File f = SD.open(path, FILE_READ);
  if (!f) { Serial.printf("Nao abriu %s\n", path); return nullptr; }

  char tag[4];
  f.read((uint8_t*)tag, 4);                 // "RIFF"
  f.seek(8);
  f.read((uint8_t*)tag, 4);                 // "WAVE"

  uint16_t channels = 1, bits = 16;
  uint32_t rate = SAMPLE_RATE, dataSize = 0, dataPos = 0;

  while (f.available()) {
    char id[4];
    if (f.read((uint8_t*)id, 4) != 4) break;
    uint8_t sb[4]; f.read(sb, 4);
    uint32_t sz = sb[0] | (sb[1] << 8) | (sb[2] << 16) | (sb[3] << 24);

    if (memcmp(id, "fmt ", 4) == 0) {
      uint8_t fmt[16]; f.read(fmt, 16);
      channels = fmt[2] | (fmt[3] << 8);
      rate     = fmt[4] | (fmt[5] << 8) | (fmt[6] << 16) | (fmt[7] << 24);
      bits     = fmt[14] | (fmt[15] << 8);
      if (sz > 16) f.seek(f.position() + (sz - 16));
    } else if (memcmp(id, "data", 4) == 0) {
      dataSize = sz; dataPos = f.position(); break;
    } else {
      f.seek(f.position() + sz + (sz & 1));  // pula chunk (+ padding impar)
    }
  }

  if (dataSize == 0 || bits != 16) {
    Serial.println("WAV invalido (precisa ser 16-bit PCM).");
    f.close(); return nullptr;
  }
  if (rate != SAMPLE_RATE)
    Serial.printf("AVISO: %s a %u Hz (esperado %d) -> pitch desloca.\n", path, rate, SAMPLE_RATE);

  f.seek(dataPos);
  uint32_t totalSamples = dataSize / 2;
  int32_t len = (channels == 2) ? totalSamples / 2 : totalSamples;

  int16_t* buf = (int16_t*)heap_caps_malloc(len * sizeof(int16_t), MALLOC_CAP_SPIRAM);
  if (!buf) {
    // sem PSRAM (ou cheia)? tenta RAM interna - sample curto ainda toca
    buf = (int16_t*)heap_caps_malloc(len * sizeof(int16_t), MALLOC_CAP_8BIT);
    if (buf) Serial.println("AVISO: sample na RAM interna (PSRAM indisponivel?)");
  }
  if (!buf) {
    Serial.printf("Sample grande demais: precisa %d KB, livre na PSRAM %d KB.\n",
                  (int)(len * 2 / 1024),
                  (int)(heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM) / 1024));
    Serial.println("Use WAV mais curto (16-bit MONO 22050 Hz).");
    f.close(); return nullptr;
  }

  if (channels == 2) {
    int16_t st[2];
    for (int32_t i = 0; i < len; i++) {
      f.read((uint8_t*)st, 4);
      buf[i] = (int16_t)(((int)st[0] + st[1]) / 2);
    }
  } else {
    uint8_t* p = (uint8_t*)buf;
    uint32_t toRead = (uint32_t)len * 2;
    while (toRead) {
      int n = f.read(p, toRead > 4096 ? 4096 : toRead);
      if (n <= 0) break;
      p += n; toRead -= n;
    }
  }
  f.close();

  // suaviza as pontas (~5ms) p/ o LOOP nao dar estalo na emenda
  int fade = SAMPLE_RATE / 200;
  if (fade * 2 < len) {
    for (int i = 0; i < fade; i++) {
      float g = (float)i / fade;
      buf[i]           = (int16_t)(buf[i] * g);
      buf[len - 1 - i] = (int16_t)(buf[len - 1 - i] * g);
    }
  }

  if (lenOut) *lenOut = len;
  return buf;
}

// =====================================================
// Mapeamento angulo->sample (TOM NORMAL = 1.0x no giro nominal)
// =====================================================
void recalcMap() {
  gFramesPerRad = (double)SAMPLE_RATE / NOMINAL_VEL;
}

// =====================================================
// Lista os .wav de uma pasta
// =====================================================
int scanFolder(const char* dir, String* list, int maxN) {
  File d = SD.open(dir);
  if (!d || !d.isDirectory()) { Serial.printf("Pasta %s nao existe.\n", dir); return 0; }

  int n = 0;
  File e = d.openNextFile();
  while (e && n < maxN) {
    if (!e.isDirectory()) {
      String nm = e.name();
      // so o nome do arquivo (sem caminho), p/ filtrar lixo do macOS
      String base = nm;
      int slash = base.lastIndexOf('/');
      if (slash >= 0) base = base.substring(slash + 1);

      String low = base; low.toLowerCase();
      // ignora ocultos/AppleDouble (._arquivo, .Spotlight...) e pega so .wav
      if (!base.startsWith(".") && low.endsWith(".wav")) {
        list[n++] = nm.startsWith("/") ? nm : (String(dir) + "/" + nm);
      }
    }
    e = d.openNextFile();
  }
  d.close();
  return n;
}

// =====================================================
// Sample de teste (fallback sem SD)
// =====================================================
void gerarSampleTeste() {
  float secs = 1.0f;
  gSampleLen = (int32_t)(secs * SAMPLE_RATE);
  gSample = (int16_t*)heap_caps_malloc(gSampleLen * sizeof(int16_t), MALLOC_CAP_SPIRAM);
  if (!gSample)  // sem PSRAM? gera na RAM interna (1s = ~44KB, cabe)
    gSample = (int16_t*)heap_caps_malloc(gSampleLen * sizeof(int16_t), MALLOC_CAP_8BIT);
  if (!gSample) { gSampleLen = 0; return; }
  for (int32_t i = 0; i < gSampleLen; i++) {
    float t = (float)i / SAMPLE_RATE;
    float vib = 1.0f + 0.02f * sinf(2.0f * PI * 6.0f * t);
    float base = 220.0f * vib;
    float v = 0.5f * sinf(2*PI*base*t) + 0.3f * sinf(2*PI*base*2*t) + 0.2f * sinf(2*PI*base*3*t);
    float env = 1.0f;
    if (t < 0.02f) env = t / 0.02f;
    if (t > secs - 0.02f) env = (secs - t) / 0.02f;
    gSample[i] = (int16_t)(v * env * 27500.0f);
  }
  Serial.println("Fallback: sample de teste gerado.");
}

// =====================================================
// Troca o sample em runtime (com swap seguro p/ a audioTask)
// =====================================================
void aplicarSample(int idx) {
  if (idx < 0 || idx >= scratchCount) return;
  Serial.printf("Carregando [%d]: %s\n", idx, scratchFiles[idx].c_str());

  int32_t newLen = 0;
  if (sdMutex) xSemaphoreTake(sdMutex, portMAX_DELAY);   // nao colide com o streaming da batida
  int16_t* nb = loadWavToPSRAM(scratchFiles[idx].c_str(), &newLen);
  if (sdMutex) xSemaphoreGive(sdMutex);
  if (!nb) { Serial.println("Falha ao carregar."); return; }

  gAudioReady = false;          // audioTask passa a mandar silencio
  delay(30);                    // garante que ela saiu do bloco atual
  int16_t* old = gSample;
  gSample = nb;
  gSampleLen = newLen;
  scratchPos = 0;
  recalcMap();                  // re-cola o novo sample na volta do prato
  gAudioReady = true;
  if (old) free(old);
  Serial.printf("OK: %d frames (%.2fs)\n", newLen, (float)newLen / SAMPLE_RATE);
}

// =====================================================
// Abre uma batida /beats (so cabecalho; PCM fica streaming)
// =====================================================
bool openBeat(int idx) {
  if (idx < 0 || idx >= beatCount) return false;
  if (beatOpen) { beatFile.close(); beatOpen = false; }

  beatFile = SD.open(beatFiles[idx].c_str(), FILE_READ);
  if (!beatFile) { Serial.printf("Batida nao abriu: %s\n", beatFiles[idx].c_str()); return false; }

  char tag[4];
  beatFile.read((uint8_t*)tag, 4); beatFile.seek(8); beatFile.read((uint8_t*)tag, 4);

  uint16_t ch = 1, bits = 16; uint32_t rate = SAMPLE_RATE, dataSize = 0, dataPos = 0;
  while (beatFile.available()) {
    char id[4]; if (beatFile.read((uint8_t*)id, 4) != 4) break;
    uint8_t sb[4]; beatFile.read(sb, 4);
    uint32_t sz = sb[0] | (sb[1] << 8) | (sb[2] << 16) | (sb[3] << 24);
    if (memcmp(id, "fmt ", 4) == 0) {
      uint8_t fmt[16]; beatFile.read(fmt, 16);
      ch = fmt[2] | (fmt[3] << 8);
      rate = fmt[4] | (fmt[5] << 8) | (fmt[6] << 16) | (fmt[7] << 24);
      bits = fmt[14] | (fmt[15] << 8);
      if (sz > 16) beatFile.seek(beatFile.position() + (sz - 16));
    } else if (memcmp(id, "data", 4) == 0) {
      dataSize = sz; dataPos = beatFile.position(); break;
    } else {
      beatFile.seek(beatFile.position() + sz + (sz & 1));
    }
  }
  if (dataSize == 0 || bits != 16) {
    Serial.println("Batida WAV invalida (precisa 16-bit PCM)."); beatFile.close(); return false;
  }
  if (rate != SAMPLE_RATE)
    Serial.printf("AVISO: batida a %u Hz (esperado %d) -> ritmo desloca.\n", rate, SAMPLE_RATE);

  beatChannels = ch; beatDataPos = dataPos; beatDataEnd = dataPos + dataSize;
  beatFile.seek(dataPos);
  beatOpen = true;
  Serial.printf("Batida [%d]: %s (%uch)\n", idx, beatFiles[idx].c_str(), ch);
  return true;
}

// =====================================================
// TASK PRODUTORA da batida (le do SD -> fila beatStream)
// =====================================================
void beatTask(void *param) {
  const int CHUNK = 512;                 // frames por leitura
  static int16_t raw[CHUNK * 2];         // estereo cru
  static int16_t mono[CHUNK];

  for (;;) {
    // --- pedido de troca de faixa ---
    if (gBeatReq != 0 && beatCount > 0) {
      int req = gBeatReq; gBeatReq = 0;
      int ni = beatIndex + req;
      while (ni < 0)            ni += beatCount;
      while (ni >= beatCount)   ni -= beatCount;
      beatIndex = ni;
      if (sdMutex) xSemaphoreTake(sdMutex, portMAX_DELAY);
      openBeat(beatIndex);
      if (sdMutex) xSemaphoreGive(sdMutex);
      // (sem reset da fila p/ evitar corrida entre tasks; ~0.27s da faixa antiga
      //  ainda toca antes da nova - troca suave)
      continue;
    }

    if (!gBeatPlaying || !beatOpen || beatCount == 0) { vTaskDelay(pdMS_TO_TICKS(10)); continue; }

    // so le se tem espaco na fila (nao bloqueia o produtor)
    if (xStreamBufferSpacesAvailable(beatStream) < (size_t)(CHUNK * 2)) {
      vTaskDelay(pdMS_TO_TICKS(2)); continue;
    }

    int got = 0;
    if (sdMutex) xSemaphoreTake(sdMutex, portMAX_DELAY);
    if (beatChannels == 2) {
      uint32_t left = beatDataEnd - beatFile.position();
      uint32_t want = CHUNK * 4; if (want > left) want = left;
      int n = beatFile.read((uint8_t*)raw, want);
      int fr = n / 4;
      for (int i = 0; i < fr; i++) mono[i] = (int16_t)(((int)raw[i*2] + raw[i*2+1]) / 2);
      got = fr;
    } else {
      uint32_t left = beatDataEnd - beatFile.position();
      uint32_t want = CHUNK * 2; if (want > left) want = left;
      int n = beatFile.read((uint8_t*)mono, want);
      got = n / 2;
    }
    if ((uint32_t)beatFile.position() >= beatDataEnd) beatFile.seek(beatDataPos);  // loop
    if (sdMutex) xSemaphoreGive(sdMutex);

    if (got > 0) xStreamBufferSend(beatStream, mono, got * 2, portMAX_DELAY);
    else vTaskDelay(pdMS_TO_TICKS(2));
  }
}

// =====================================================
// Gravacao: helpers WAV + task gravadora
// =====================================================
static void wr16(File &f, uint16_t v) { uint8_t b[2] = {(uint8_t)v, (uint8_t)(v >> 8)}; f.write(b, 2); }
static void wr32(File &f, uint32_t v) {
  uint8_t b[4] = {(uint8_t)v, (uint8_t)(v >> 8), (uint8_t)(v >> 16), (uint8_t)(v >> 24)};
  f.write(b, 4);
}
void writeWavHeader(File &f, uint32_t dataBytes) {
  uint32_t rate = SAMPLE_RATE; uint16_t ch = 1, bits = 16;
  f.write((const uint8_t*)"RIFF", 4); wr32(f, 36 + dataBytes); f.write((const uint8_t*)"WAVE", 4);
  f.write((const uint8_t*)"fmt ", 4); wr32(f, 16); wr16(f, 1); wr16(f, ch);
  wr32(f, rate); wr32(f, rate * ch * bits / 8); wr16(f, ch * bits / 8); wr16(f, bits);
  f.write((const uint8_t*)"data", 4); wr32(f, dataBytes);
}
String nextRecName() {
  for (int i = 1; i < 10000; i++) {
    char p[40]; snprintf(p, sizeof(p), "/records/REC_%04d.wav", i);
    if (!SD.exists(p)) return String(p);
  }
  return "/records/REC_9999.wav";
}

void recorderTask(void *param) {
  static File recFile;
  static bool recOpen = false;
  static uint32_t recBytes = 0;
  static String recPath = "";
  static uint32_t lastSync = 0;
  static int16_t rbuf[2048];   // 4KB por escrita (menos overhead no SD)

  for (;;) {
    // --- comecar ---
    if (gRecCmd == 1 && !recOpen) {
      gRecCmd = 0;
      recPath = nextRecName();
      if (sdMutex) xSemaphoreTake(sdMutex, portMAX_DELAY);
      if (!SD.exists("/records")) SD.mkdir("/records");
      recFile = SD.open(recPath.c_str(), FILE_WRITE);
      if (recFile) {
        writeWavHeader(recFile, 0); recBytes = 0; recOpen = true; gRecording = true;
        lastSync = millis();
        Serial.printf("REC iniciado: %s\n", recPath.c_str());
      } else Serial.println("REC: nao abriu arquivo");
      if (sdMutex) xSemaphoreGive(sdMutex);
      continue;
    }
    // --- parar ---
    if (gRecCmd == -1 && recOpen) {
      gRecCmd = 0;
      if (sdMutex) xSemaphoreTake(sdMutex, portMAX_DELAY);
      size_t r;
      while ((r = xStreamBufferReceive(recStream, rbuf, sizeof(rbuf), 0)) > 0) {
        recFile.write((uint8_t*)rbuf, r); recBytes += r;     // drena o que sobrou
      }
      recFile.seek(0); writeWavHeader(recFile, recBytes);    // corrige tamanhos
      recFile.close(); recOpen = false; gRecording = false;
      if (sdMutex) xSemaphoreGive(sdMutex);
      Serial.printf("REC parado: %.1fs gravados\n", (float)recBytes / 2 / SAMPLE_RATE);
      continue;
    }
    gRecCmd = 0;

    if (!recOpen) { vTaskDelay(pdMS_TO_TICKS(20)); continue; }

    // --- grava chunk ---
    size_t r = xStreamBufferReceive(recStream, rbuf, sizeof(rbuf), pdMS_TO_TICKS(40));
    if (r > 0) {
      if (sdMutex) xSemaphoreTake(sdMutex, portMAX_DELAY);
      recFile.write((uint8_t*)rbuf, r); recBytes += r;
      // a cada ~1.5s salva cabecalho+flush -> arquivo fica VALIDO mesmo sem parar
      if (millis() - lastSync > 1500) {
        lastSync = millis();
        uint32_t pos = recFile.position();
        recFile.seek(0); writeWavHeader(recFile, recBytes); recFile.seek(pos);
        recFile.flush();
      }
      if (sdMutex) xSemaphoreGive(sdMutex);
    }
  }
}

// =====================================================
// AUDIO TASK (nucleo 0)
// =====================================================
void audioTask(void *param) {
  i2s_config_t cfg = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,     // fila de DMA MENOR = MUITO menos delay no scratch
    .dma_buf_len = 128,     // 4*128 = 512 amostras = ~23ms (era 8*512 = ~186ms!)
    .use_apll = false,
    .tx_desc_auto_clear = true,
    .fixed_mclk = 0
  };
  i2s_pin_config_t pins = {
    .bck_io_num = I2S_BCLK,
    .ws_io_num = I2S_WSEL,
    .data_out_num = I2S_DIN,
    .data_in_num = I2S_PIN_NO_CHANGE
  };
  i2s_driver_install(I2S_NUM_0, &cfg, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pins);
  i2s_zero_dma_buffer(I2S_NUM_0);

  const int FRAMES = 128;   // bloco menor = menos latencia (era 256)
  int16_t buffer[FRAMES * 2];
  static int16_t beatBuf[FRAMES];
  static int16_t recBuf[FRAMES];   // mix mono p/ gravacao

  for (;;) {
    bool  scrReady = gAudioReady && gSampleLen > 0;
    float muteTarget = (gCut || gMuteScratch) ? 0.0f : 1.0f;  // alvo do mute (suavizado)
    float vol  = gVol;
    float bvol = gBeatVol;

    // --- POSITION-LOCK do scratch (sempre atualiza o bookkeeping) ---
    portENTER_CRITICAL(&posMux);
    double target = gScratchTarget;
    portEXIT_CRITICAL(&posMux);

    static double playPos = 0.0;
    static double lastTarget = 0.0;
    static bool aInit = false;
    if (!aInit) { lastTarget = target; aInit = true; }

    double step = (target - lastTarget) / FRAMES;
    lastTarget = target;
    if (fabs(step) < SCRATCH_PARADA) step = 0.0;
    const double MAXSTEP = 32.0;
    if (step >  MAXSTEP) step =  MAXSTEP;
    if (step < -MAXSTEP) step = -MAXSTEP;

    // --- batida: puxa FRAMES amostras da fila (nao bloqueia) ---
    int gotB = 0;
    if (gBeatPlaying && beatStream) {
      size_t r = xStreamBufferReceive(beatStream, beatBuf, FRAMES * 2, 0);
      gotB = r / 2;
    }

    // --- mix scratch + batida ---
    static float scrGain = 1.0f;        // ganho suavizado do scratch (anti-estalo no cut)
    for (int i = 0; i < FRAMES; i++) {
      playPos += step;
      float scratchS = 0.0f;
      if (scrReady) {
        long idx = (long)floor(playPos);
        long m = idx % gSampleLen; if (m < 0) m += gSampleLen;
        long n = m + 1; if (n >= gSampleLen) n = 0;
        double frac = playPos - floor(playPos);
        scratchS = gSample[m] + (gSample[n] - gSample[m]) * frac;
      }
      scrGain += (muteTarget - scrGain) * 0.01f;   // ~5ms de fade no cut (sem estalo)
      scratchS *= scrGain;
      float beatS = (i < gotB) ? (float)beatBuf[i] : 0.0f;
      float mix = (scratchS + beatS * bvol) * vol;
      if (mix >  32767.0f) mix =  32767.0f;
      if (mix < -32768.0f) mix = -32768.0f;
      int16_t s = (int16_t)mix;
      buffer[i * 2]     = s;
      buffer[i * 2 + 1] = s;
      recBuf[i] = s;                 // guarda o mix mono p/ gravar
    }

    // grava o mix (nao bloqueia; se a fila encher, descarta)
    if (gRecording && recStream)
      xStreamBufferSend(recStream, recBuf, FRAMES * 2, 0);

    size_t written;
    i2s_write(I2S_NUM_0, buffer, sizeof(buffer), &written, portMAX_DELAY);
  }
}

// =====================================================
// MODO MANUTENCAO (WiFi AP + pagina web: OTA + upload WAV + download)
// =====================================================
WebServer server(80);
File webFile;

String listaPasta(const char* dir) {
  String s = "";
  File d = SD.open(dir);
  if (d && d.isDirectory()) {
    File e = d.openNextFile();
    while (e) {
      if (!e.isDirectory()) {
        String nm = e.name();
        String base = nm; int sl = base.lastIndexOf('/'); if (sl >= 0) base = base.substring(sl + 1);
        if (!base.startsWith(".")) {
          String full = nm.startsWith("/") ? nm : (String(dir) + "/" + base);
          s += "<li><input type='checkbox' name='del' value='" + full + "'> ";
          s += "<a href='/dl?p=" + full + "'>" + base + "</a> (" + String(e.size() / 1024) + " KB)</li>";
        }
      }
      e = d.openNextFile();
    }
  }
  return s;
}

String paginaHtml() {
  String h = "<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>";
  h += "<title>ScratchPad</title></head><body style='font-family:sans-serif;max-width:640px;margin:auto;padding:10px'>";
  h += "<h2>ScratchPad - Manutencao</h2>";
  // --- diagnostico (sem precisar de Serial) ---
  uint32_t psram = ESP.getPsramSize();
  h += "<p style='padding:8px;border:1px solid #888'>";
  h += "<b>PSRAM:</b> " + String(psram / 1024) + " KB ";
  h += (psram == 0) ? "<b style='color:red'>(DESLIGADA! Tools-&gt;PSRAM: OPI PSRAM e recompile - sem ela o scratch fica MUDO)</b>" : "(ok)";
  h += "<br><b>Heap livre:</b> " + String(ESP.getFreeHeap() / 1024) + " KB";
  h += "<br><b>Versao:</b> " + String(__DATE__) + " " + String(__TIME__);
  h += "</p>";
  h += "<h3>1) Atualizar firmware (.bin)</h3>";
  h += "<form method='POST' action='/update' enctype='multipart/form-data'>";
  h += "<input type='file' name='f' accept='.bin'> <input type='submit' value='Atualizar'></form>";
  h += "<h3>2) Enviar WAV p/ /scratch (pode escolher VARIOS)</h3>";
  h += "<form method='POST' action='/upload?folder=scratch' enctype='multipart/form-data'>";
  h += "<input type='file' name='f' accept='.wav' multiple> <input type='submit' value='Enviar'></form>";
  h += "<h3>3) Enviar WAV p/ /beats (pode escolher VARIOS)</h3>";
  h += "<form method='POST' action='/upload?folder=beats' enctype='multipart/form-data'>";
  h += "<input type='file' name='f' accept='.wav' multiple> <input type='submit' value='Enviar'></form>";
  h += "<hr><form method='POST' action='/delsel'>";
  h += "<b>Marque os arquivos e apague de uma vez:</b>";
  h += "<h3>Gravacoes</h3><ul>" + listaPasta("/records") + "</ul>";
  h += "<h3>/scratch</h3><ul>" + listaPasta("/scratch") + "</ul>";
  h += "<h3>/beats</h3><ul>" + listaPasta("/beats") + "</ul>";
  h += "<input type='submit' value='APAGAR SELECIONADOS' style='color:red;font-weight:bold' ";
  h += "onclick='return confirm(\"Apagar os selecionados?\")'></form>";
  h += "</body></html>";
  return h;
}

void handleDownload() {
  if (!server.hasArg("p")) { server.send(400, "text/plain", "sem p"); return; }
  String p = server.arg("p");
  File f = SD.open(p.c_str(), FILE_READ);
  if (!f) { server.send(404, "text/plain", "nao achou"); return; }
  // manda o NOME com extensao -> navegador salva como .wav
  String base = p; int sl = base.lastIndexOf('/'); if (sl >= 0) base = base.substring(sl + 1);
  server.sendHeader("Content-Disposition", "attachment; filename=\"" + base + "\"");
  server.streamFile(f, "audio/wav");
  f.close();
}

void handleDelete() {
  if (server.hasArg("p")) {
    bool ok = SD.remove(server.arg("p").c_str());
    Serial.printf("Apagar %s -> %s\n", server.arg("p").c_str(), ok ? "ok" : "falhou");
  }
  server.sendHeader("Location", "/");   // volta pra lista atualizada
  server.send(303);
}

void handleDeleteSelected() {
  int n = server.args();
  int apagados = 0;
  for (int i = 0; i < n; i++) {
    if (server.argName(i) == "del") {
      if (SD.remove(server.arg(i).c_str())) apagados++;
      Serial.printf("Apagar %s\n", server.arg(i).c_str());
    }
  }
  Serial.printf("Apagados: %d\n", apagados);
  server.sendHeader("Location", "/");
  server.send(303);
}

void maintenanceMode() {
  Serial.println("=== MODO MANUTENCAO ===");

  spiSD.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  if (SD.begin(SD_CS, spiSD, SD_FREQ)) Serial.println("SD OK."); else Serial.println("SD falhou.");

  WiFi.mode(WIFI_AP);
  WiFi.softAP("ScratchPad", "12345678");
  Serial.print("WiFi 'ScratchPad' (senha 12345678) -> abra  http://");
  Serial.println(WiFi.softAPIP());

  server.on("/", []() { server.send(200, "text/html", paginaHtml()); });
  server.on("/dl", handleDownload);
  server.on("/del", handleDelete);
  server.on("/delsel", HTTP_POST, handleDeleteSelected);

  // --- OTA firmware ---
  server.on("/update", HTTP_POST,
    []() {
      server.send(200, "text/html", Update.hasError() ? "FALHOU" : "OK! Reiniciando...");
      delay(800); ESP.restart();
    },
    []() {
      HTTPUpload& up = server.upload();
      if (up.status == UPLOAD_FILE_START)      { Update.begin(UPDATE_SIZE_UNKNOWN); }
      else if (up.status == UPLOAD_FILE_WRITE) { Update.write(up.buf, up.currentSize); }
      else if (up.status == UPLOAD_FILE_END)   { Update.end(true); }
    });

  // --- upload WAV pro SD ---
  server.on("/upload", HTTP_POST,
    []() { server.send(200, "text/html", "Enviado! <a href='/'>voltar</a>"); },
    []() {
      HTTPUpload& up = server.upload();
      String folder = server.hasArg("folder") ? server.arg("folder") : "scratch";
      String dir = "/" + folder;
      if (up.status == UPLOAD_FILE_START) {
        if (!SD.exists(dir.c_str())) SD.mkdir(dir.c_str());
        webFile = SD.open((dir + "/" + up.filename).c_str(), FILE_WRITE);
      } else if (up.status == UPLOAD_FILE_WRITE) {
        if (webFile) webFile.write(up.buf, up.currentSize);
      } else if (up.status == UPLOAD_FILE_END) {
        if (webFile) webFile.close();
      }
    });

  server.begin();
  Serial.println("Servidor pronto. (segurar o botao no boot foi detectado)");
  while (true) { server.handleClient(); delay(2); }   // fica aqui ate desligar
}

// =====================================================
// SETUP
// =====================================================
void setup() {
  Serial.begin(115200);
  delay(800);

  // ---- modo manutencao: segurar o botao 1 (GPIO21) ao LIGAR ----
  pinMode(MAINT_BTN, INPUT_PULLUP);
  delay(20);
  if (digitalRead(MAINT_BTN) == LOW) {
    maintenanceMode();   // entra no WiFi/web e NAO retorna
  }

  Serial.println("=== ScratchPad S3 v10 (SD + encoder) ===");

  // ---- entradas ----
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  pinMode(ENC_SW, INPUT_PULLUP);
  for (int i = 0; i < 6; i++) pinMode(BTN_PINS[i], INPUT_PULLUP);

  // ---- SD + fila/mutex da batida + fila de gravacao ----
  sdMutex    = xSemaphoreCreateMutex();
  beatStream = xStreamBufferCreate(24576, 1);   // ~0.55s de batida (absorve picos do SD ao gravar)
  recStream  = xStreamBufferCreate(32768, 1);   // ~0.74s de mix p/ gravar

  spiSD.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  if (SD.begin(SD_CS, spiSD, SD_FREQ)) {
    Serial.println("SD OK.");
    scratchCount = scanFolder("/scratch", scratchFiles, MAX_FILES);
    Serial.printf("%d sample(s) em /scratch\n", scratchCount);
    if (scratchCount > 0) {
      gSample = loadWavToPSRAM(scratchFiles[0].c_str(), &gSampleLen);
      selIndex = 0;
    }
    beatCount = scanFolder("/beats", beatFiles, MAX_FILES);
    Serial.printf("%d batida(s) em /beats\n", beatCount);
    if (beatCount > 0) { beatIndex = 0; openBeat(0); }
  } else {
    Serial.println("SD nao iniciou.");
  }
  if (!gSample) gerarSampleTeste();
  recalcMap();                  // cola o sample na volta do prato
  gAudioReady = (gSampleLen > 0);

  // ---- motor ----
  Wire.begin(AS5600_SDA, AS5600_SCL);
  Wire.setClock(400000);
  sensor.init(&Wire);
  motor.linkSensor(&sensor);

  driver.voltage_power_supply = 12;
  driver.voltage_limit = 10;
  driver.pwm_frequency = 20000;
  driver.init();
  motor.linkDriver(&driver);

  motor.controller = MotionControlType::velocity;
  motor.voltage_limit = MOTOR_TORQUE;       // <- ajuste de torque (topo do codigo)
  motor.PID_velocity.P = FIRMEZA;           // <- rigidez (topo do codigo)
  motor.PID_velocity.I = 0.2;
  motor.PID_velocity.output_ramp = 100;
  motor.LPF_velocity.Tf = MOTOR_FILTRO;     // <- suavidade do motor (topo do codigo)

  motor.init();
  motor.initFOC();
  Serial.println("Motor pronto.");

  xTaskCreatePinnedToCore(audioTask,    "audio", 8192, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(beatTask,     "beat",  4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(recorderTask, "rec",   4096, NULL, 1, NULL, 0);
  Serial.println("Rodando!");
}

// =====================================================
// ENCODER (poll de quadratura no loop)
// =====================================================
void lerEncoder() {
  static const int8_t tbl[16] = {0,-1,1,0, 1,0,0,-1, -1,0,0,1, 0,1,-1,0};
  static uint8_t last = 0;
  static int8_t acc = 0;

  uint8_t ab = (digitalRead(ENC_A) << 1) | digitalRead(ENC_B);
  acc += tbl[(last << 2) | ab];
  last = ab;

  if (acc >= 4) {                           // um detente p/ frente
    acc = 0;
    if (selIndex < scratchCount - 1) selIndex++;
    Serial.printf("> sel %d: %s\n", selIndex,
                  scratchCount ? scratchFiles[selIndex].c_str() : "(vazio)");
  } else if (acc <= -4) {                    // um detente p/ tras
    acc = 0;
    if (selIndex > 0) selIndex--;
    Serial.printf("> sel %d: %s\n", selIndex,
                  scratchCount ? scratchFiles[selIndex].c_str() : "(vazio)");
  }

  // clique = carrega o selecionado
  static bool lastSw = HIGH;
  static uint32_t tSw = 0;
  bool sw = digitalRead(ENC_SW);
  if (lastSw == HIGH && sw == LOW && millis() - tSw > 200) {
    tSw = millis();
    aplicarSample(selIndex);
  }
  lastSw = sw;
}

// =====================================================
// BOTOES -- clique CURTO + LONG-PRESS
//   curto: 1=proxima batida 2=batida anterior 3=play/pause 4=mute 5=volume 6=GRAVAR
//   LONG:  3 = alterna 33/45 RPM
// =====================================================
void onButton(int i) {                 // clique CURTO
  switch (i) {
    case 0: gBeatReq = +1; break;                                       // proxima batida
    case 1: gBeatReq = -1; break;                                       // batida anterior
    case 2: gBeatPlaying = !gBeatPlaying; break;                        // play/pause batida
    case 3: gMuteScratch = !gMuteScratch; break;                        // mute scratch
    case 4: gVol += 0.15f; if (gVol > 1.2f) gVol = 0.15f; break;        // volume (cicla)
    case 5: gRecCmd = gRecording ? -1 : 1; break;                       // GRAVAR start/stop
  }
  Serial.printf("BTN%d  (beat=%d play=%d vol=%.1f muteScr=%d rec=%d)\n",
                i + 1, beatIndex, gBeatPlaying, gVol, gMuteScratch, gRecording);
}

void onButtonLong(int i) {             // segurar (long-press)
  if (i == 2) {                        // botao 3 = alterna velocidade
    gTargetVel = (gTargetVel == SPEED_33) ? SPEED_45 : SPEED_33;
    Serial.printf("Velocidade: %s RPM\n", (gTargetVel == SPEED_33) ? "33" : "45");
  }
}

void lerBotoes() {
  static bool last[6] = {HIGH,HIGH,HIGH,HIGH,HIGH,HIGH};
  static uint32_t pressT[6] = {0,0,0,0,0,0};
  static bool longFired[6] = {false,false,false,false,false,false};
  const uint32_t LONGMS = 700;
  for (int i = 0; i < 6; i++) {
    bool s = digitalRead(BTN_PINS[i]);
    if (last[i] == HIGH && s == LOW) {            // pressionou
      pressT[i] = millis(); longFired[i] = false;
    } else if (last[i] == LOW && s == LOW) {      // segurando
      if (!longFired[i] && millis() - pressT[i] >= LONGMS) { longFired[i] = true; onButtonLong(i); }
    } else if (last[i] == LOW && s == HIGH) {     // soltou
      if (!longFired[i] && millis() - pressT[i] >= 30) onButton(i);   // clique curto
    }
    last[i] = s;
  }
}

// =====================================================
// LOOP (nucleo 1): motor + sensores + encoder + botoes
// =====================================================
unsigned long lastHall = 0;
unsigned long lastMicros = 0;
float setVel = 0;

void loop() {
  motor.loopFOC();

  unsigned long now = micros();
  float dt = (lastMicros == 0) ? 0 : (now - lastMicros) * 1e-6f;
  lastMicros = now;

  float v = motor.shaftVelocity();

  // setpoint que escorrega (logica ORIGINAL do v8, comprovada: da partida e cede ao segurar)
  float dir    = (gTargetVel < 0) ? -1.0f : 1.0f;
  float vDir   = v * dir;
  float setDir = setVel * dir;
  float tgtDir = fabs(gTargetVel);

  if (vDir < setDir - DEADBAND) {
    setDir = vDir + DEADBAND;              // prato freado -> setpoint cede (mas mantem torque p/ subir)
  } else {
    setDir += RETORNO * dt;               // livre -> volta ao giro normal (RETORNO alto = quase na hora)
    if (setDir > tgtDir) setDir = tgtDir;
  }
  if (setDir < 0) setDir = 0;

  setVel = setDir * dir;
  motor.move(setVel);

  // POSITION-LOCK: integra o ANGULO real do prato (sem filtro = resposta imediata)
  static float lastAngle = 0.0f;
  static bool angInit = false;
  float ang = motor.shaftAngle();
  if (!angInit) { lastAngle = ang; angInit = true; }
  float dA = ang - lastAngle;
  lastAngle = ang;
  portENTER_CRITICAL(&posMux);
  gScratchTarget += (double)dA * gFramesPerRad * SCRATCH_PITCH;
  portEXIT_CRITICAL(&posMux);

  gSpeedFactor = v / NOMINAL_VEL;   // so p/ debug

  // debug: v=velocidade  cut=crossfader cortando  mute=botao mute  ready=sample carregado
  static uint32_t tDbg = 0;
  if (millis() - tDbg > 500) {
    tDbg = millis();
    Serial.printf("v=%.2f sf=%.2f | cut=%d mute=%d ready=%d  (scratch sai se cut=0 e mute=0)\n",
                  v, gSpeedFactor, gCut, gMuteScratch, gAudioReady);
  }

  // entradas
  lerEncoder();
  lerBotoes();

  // crossfader
  if (millis() - lastHall > 20) {
    lastHall = millis();
    int raw = analogRead(HALL_PIN);
    if (raw < hallMin) hallMin = raw;
    if (raw > hallMax) hallMax = raw;

    int pos = 64;
    if (hallMax - hallMin > 100) {
      pos = map(raw, hallMin, hallMax, 0, 127);
      pos = constrain(pos, 0, 127);
    }
    int p = CUT_INVERT ? (127 - pos) : pos;
    static bool cutState = false;
    if (!cutState && p < CUT_THRESH) cutState = true;
    else if (cutState && p > CUT_THRESH + HYST) cutState = false;
    gCut = cutState;
  }
}
