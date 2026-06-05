#include <HardwareSerial.h>
#include <U8g2lib.h>

HardwareSerial RF_SERIAL(PC11, PC10);

#define PIN_RF_ENABLE  PB9
#define PIN_RF_POWER   PC13
#define PIN_RF_PTT     PC0
#define PIN_AUDIO      PA4

#define DISP_PIN_SCK PB3
#define DISP_PIN_SDIN PB5
#define DISP_PIN_CS PB6
#define DISP_PIN_DC PB7
#define DISP_PIN_RST PB8
uint8_t DISP_CENTER_POINT = 0;

U8G2_PCD8544_84X48_F_4W_SW_SPI u8g2(U8G2_R0, DISP_PIN_SCK, DISP_PIN_SDIN, DISP_PIN_CS, DISP_PIN_DC, DISP_PIN_RST);

// ---- CHANGE THESE ----
const char* message = ".... . .-.. .-.. --- / ..-. .-. --- -- / - .... . / -. --- -.- .. .- / ...-- ...-- .---- -----";
#define FREQ 433500000
// ----------------------

#define DIT 40
#define DAH (DIT*3)
#define SYM_GAP DIT
#define CHAR_GAP (DIT*3)
#define WORD_GAP (DIT*7)
#define SINE_PERIOD_US 52

const uint8_t sine[32] = {
  128,153,177,199,218,234,245,253,
  255,253,245,234,218,199,177,153,
  128,103,79,57,38,22,11,3,
  1,3,11,22,38,57,79,103
};

void rfWrite(uint8_t reg, uint16_t val) {
  char buf[32];
  sprintf(buf, "AT+POKE=%d,%d\r\n", reg, val);
  RF_SERIAL.print(buf);
  delay(50);
  while(RF_SERIAL.available()) RF_SERIAL.read();
}

uint16_t rfRead(uint8_t reg) {
  char buf[32];
  sprintf(buf, "AT+PEEK=%d\r\n", reg);
  RF_SERIAL.print(buf);
  delay(50);
  String resp = RF_SERIAL.readStringUntil('\n');
  return resp.toInt();
}

void reloadRF() {
  uint16_t val = rfRead(48);
  rfWrite(48, val & ~96);
  rfWrite(48, val);
}

void setFreq(uint32_t freq_hz) {
  uint32_t val = (uint32_t)((freq_hz / 1000.0f) * 16.0f);
  rfWrite(41, (val >> 16) & 0xFFFF);
  rfWrite(42, val & 0xFFFF);
  reloadRF();
}

void playTone(int ms) {
  unsigned long end = millis() + ms;
  while (millis() < end) {
    for (int i = 0; i < 32; i++) {
      analogWrite(PIN_AUDIO, sine[i]);
      delayMicroseconds(SINE_PERIOD_US);
    }
  }
}

void silence(int ms) {
  analogWrite(PIN_AUDIO, 128);
  delay(ms);
}

void sendMorse(const char* msg) {
  int i = 0;
  while (msg[i]) {
    char c = msg[i];
    if (c == '.') {
      playTone(DIT);
      silence(SYM_GAP);
    } else if (c == '-') {
      playTone(DAH);
      silence(SYM_GAP);
    } else if (c == ' ') {
      // peek ahead: "/ " = word gap, else char gap
      if (msg[i+1] == '/') {
        silence(WORD_GAP);
        i += 2; // skip "/ "
        continue;
      } else {
        silence(CHAR_GAP);
      }
    }
    i++;
  }
}

void setup() {
  u8g2.begin();
  u8g2.setContrast(170);
  u8g2.setFontMode(1);
  u8g2.setBitmapMode(1);
  u8g2.setFont(u8g2_font_NokiaLargeBold_tr);
  DISP_CENTER_POINT = u8g2_GetStrWidth(u8g2.getU8g2(), "setting up");    //get the offset of the text to centre it on the screen
  u8g2.setCursor(42 - (DISP_CENTER_POINT / 2), 23);                         //42 = display horizontal midpoint, 23 = vertical point on screen i want to draw at
  u8g2.print("setting up");
  u8g2.sendBuffer();
  u8g2.clearBuffer();
  Serial.begin(9600);
  analogWriteResolution(8);
  analogWrite(PIN_AUDIO, 128);

  pinMode(PIN_RF_ENABLE, OUTPUT);
  pinMode(PIN_RF_POWER, OUTPUT);
  pinMode(PIN_RF_PTT, OUTPUT);

  digitalWrite(PIN_RF_PTT, HIGH);
  digitalWrite(PIN_RF_POWER, LOW);
  digitalWrite(PIN_RF_ENABLE, LOW);

  delay(100);
  digitalWrite(PIN_RF_ENABLE, HIGH);
  delay(100);

  RF_SERIAL.begin(9600);
  delay(100);

  Serial.println("Starting RF init...");
  u8g2.setFont(u8g2_font_NokiaLargeBold_tr);
  DISP_CENTER_POINT = u8g2_GetStrWidth(u8g2.getU8g2(), "starting rf");    //get the offset of the text to centre it on the screen
  u8g2.setCursor(42 - (DISP_CENTER_POINT / 2), 23);                         //42 = display horizontal midpoint, 23 = vertical point on screen i want to draw at
  u8g2.print("starting rf");
  u8g2.sendBuffer();
  u8g2.clearBuffer();

  rfWrite(48, 1);
  delay(160);
  rfWrite(48, 4);
  rfWrite(4, 4048);
  rfWrite(10, 31776);
  rfWrite(19, 41216);
  rfWrite(31, 4097);
  rfWrite(49, 49);
  rfWrite(51, 17573);
  rfWrite(52, 11145);
  rfWrite(65, 16674);
  rfWrite(66, 4178);
  rfWrite(67, 256);
  rfWrite(68, 2047);
  rfWrite(89, 4368);
  rfWrite(71, 32559);
  rfWrite(79, 11362);
  rfWrite(83, 148);
  rfWrite(84, 10812);
  rfWrite(85, 129);
  rfWrite(86, 2818);
  rfWrite(87, 7168);
  rfWrite(88, 40157);
  rfWrite(90, 1755);
  rfWrite(99, 5805);
  rfWrite(103, 1576);
  rfWrite(104, 1509);
  rfWrite(105, 1365);
  rfWrite(106, 1208);
  rfWrite(107, 766);
  rfWrite(108, 477);
  rfWrite(109, 177);
  rfWrite(110, 3970);
  rfWrite(111, 378);
  rfWrite(112, 76);
  rfWrite(113, 3869);
  rfWrite(114, 3473);
  rfWrite(115, 2622);
  rfWrite(116, 2319);
  rfWrite(117, 2099);
  rfWrite(118, 2054);

  rfWrite(48, 16548);
  delay(160);
  rfWrite(48, 16550);
  delay(160);
  rfWrite(48, 16390);
  delay(160);
  rfWrite(64, 49);

  rfWrite(21, 7936);
  rfWrite(50, 30052);
  rfWrite(58, 16579);
  rfWrite(60, 6964);
  rfWrite(63, 10705);
  rfWrite(72, 3939);
  rfWrite(98, 65454);
  rfWrite(127, 1);
  rfWrite(6, 36);
  rfWrite(7, 532);
  rfWrite(8, 548);
  rfWrite(9, 788);
  rfWrite(10, 804);
  rfWrite(11, 836);
  rfWrite(12, 900);
  rfWrite(13, 4996);
  rfWrite(14, 7044);
  rfWrite(15, 16260);
  rfWrite(18, 57579);
  rfWrite(127, 0);
  reloadRF();

  setFreq(FREQ);

  uint16_t reg30 = rfRead(48);
  reg30 = (reg30 & ~96) | 64;
  rfWrite(48, reg30);

  RF_SERIAL.print("AT+AMP=1\r\n");
  delay(50);
  digitalWrite(PIN_RF_PTT, LOW);

  Serial.println("Init done. Sending CW.");
  u8g2.setFont(u8g2_font_NokiaLargeBold_tr);
  DISP_CENTER_POINT = u8g2_GetStrWidth(u8g2.getU8g2(), "sending cw");    //get the offset of the text to centre it on the screen
  u8g2.setCursor(42 - (DISP_CENTER_POINT / 2), 23);                         //42 = display horizontal midpoint, 23 = vertical point on screen i want to draw at
  u8g2.print("sending cw");
  u8g2.sendBuffer();
  u8g2.clearBuffer();
}

void loop() {
  sendMorse(message);
  silence(4000);
}