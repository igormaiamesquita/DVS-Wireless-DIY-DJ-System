/*
  GPIO_Test - isola ESP + fiacao do modulo SD
  Faz os pinos SCK(13), MISO-line(11), MOSI(12), CS(10) piscarem a 2 Hz.
  Meça CADA UM no HEADER DO MODULO com o multimetro (DC):
    deve oscilar entre ~0V e ~3.3V a cada 0,5s.
  - Se um pino NAO oscila no header -> fio/solda/GPIO daquele = problema.
  - Se TODOS oscilam certinho -> ESP e fiacao OK (problema e modulo/alim/terra).
*/

#define PIN_SCK  13
#define PIN_MISO 11
#define PIN_MOSI 12
#define PIN_CS   10

void setup() {
  Serial.begin(115200);
  delay(800);
  Serial.println("\n=== GPIO_Test: pinos piscando a 2Hz ===");
  Serial.println("Meça no header do modulo: deve oscilar 0V <-> 3.3V");
  pinMode(PIN_SCK,  OUTPUT);
  pinMode(PIN_MISO, OUTPUT);
  pinMode(PIN_MOSI, OUTPUT);
  pinMode(PIN_CS,   OUTPUT);
}

void loop() {
  static bool s = false;
  s = !s;
  digitalWrite(PIN_SCK,  s);
  digitalWrite(PIN_MISO, s);
  digitalWrite(PIN_MOSI, s);
  digitalWrite(PIN_CS,   s);
  Serial.printf("Pinos 10/11/12/13 -> %s\n", s ? "HIGH (~3.3V)" : "LOW (0V)");
  delay(500);
}
