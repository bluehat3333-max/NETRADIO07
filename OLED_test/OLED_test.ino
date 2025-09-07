#include <Arduino.h>
#include <U8g2lib.h>

// SCL: GPIO6 (D5), SDA: GPIO5 (D4)
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, 6, 5);

void setup(void) {
  u8g2.begin();
}

void loop(void) {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB10_tr);
  u8g2.drawStr(0, 15, "Display Test");
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(0, 40, "Hello, World!");
  u8g2.drawFrame(0, 50, 128, 10);
  u8g2.drawBox(2, 52, 60, 6);
  u8g2.sendBuffer();
  delay(2000);
}
