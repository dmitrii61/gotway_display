/* RELEASE NOTES v3.2:
   1. The code is prepared for Github publication, all comments are translated into English
   2. 30 first loops (~3 sec) startup data is not shown due to deviation of voltage/battery data
   3. Added temperature output to startup data
*/
#include <GyverMAX7219.h>       // LED matrix control library
MAX7219 < 1, 2, 9 > mtrx;       // 2 LED matrixes (1*2), CS pin is 9 (Arduino Nano) or 5 (means D1 for Wemos)

// SETTINGS
String backlight_txt = "";      // Text to scroll
int scrolling_on_euc_speed = 5; // Start scrolling text on backlight if the speed reaches this value
bool debug_mode1 = 0;           // Debug mode 1: synthetic data for variables instead of parcing EUC data
bool debug_mode2 = 0;           // Turn on Serial.print with debugging data
bool useBetterPercents = 1;     // Use Wheellog's better pernects formula
int temperature_alarm = 70;     // Temperature level for overheating alarm
int beeper_pin = 0;             // If beeper is connected, type it's pin number (default: 3). Otherwise, type 0 to avoid listening beeper signals (experimental)
byte brightness_stop = 7;       // Brightness [0-15] while stop
byte brightness_ride = 15;      // Brightness [0-15] while riding
int scroll_x_shift = 7;         // X-coordinate for the scrolling text to appear on the cycle start (recommended = 7)
int scroll_y_shift = 9;         // Y-coordinate for the top pixel of scrolling text (for backlight screen this should be neither 8 or 9)

// Common pixelmaps               http://www.xlr8.at/8x8hexbin/
const uint8_t pixelmap_version[8]             PROGMEM = { 0x00, 0x00, 0x63, 0x21, 0x63, 0x22, 0x6b, 0x00 }; // Firmware version [3.2]
const uint8_t pixelmap_welcome_logo[8]        PROGMEM = { 0xdb, 0xdb, 0x9a, 0xda, 0xda, 0x9a, 0xc3, 0xc3 };
const uint8_t pixelmap_backlight[8]           PROGMEM = { 0xff, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0xff };
const uint8_t pixelmap_brake[8]               PROGMEM = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
// Alarm pixelmaps                http://www.xlr8.at/8x8hexbin/
const uint8_t pixelmap_highpower[8]           PROGMEM = { 0x7e, 0x7e, 0x7e, 0x7e, 0xff, 0x7e, 0x3c, 0x18 };
const uint8_t pixelmap_speed2[8]              PROGMEM = { 0x3c, 0x3c, 0x3c, 0x3c, 0xff, 0x7e, 0x3c, 0x18 };
const uint8_t pixelmap_speed1[8]              PROGMEM = { 0x18, 0x18, 0x18, 0x18, 0x7e, 0x3c, 0x18, 0x00 };
const uint8_t pixelmap_lowvoltage[8]          PROGMEM = { 0x60, 0x92, 0x92, 0x92, 0x92, 0x97, 0x92, 0xf0 };
const uint8_t pixelmap_overvoltage[8]         PROGMEM = { 0x60, 0xf2, 0xf7, 0xf2, 0xf2, 0xf2, 0xf2, 0xf0 };
const uint8_t pixelmap_overtemperature[8]     PROGMEM = { 0xff, 0xeb, 0xef, 0xc7, 0xef, 0xef, 0xe3, 0xff };
const uint8_t pixelmap_hallsensors[8]         PROGMEM = { 0xff, 0xdb, 0xdb, 0xc3, 0xdb, 0xdb, 0xdb, 0xff };
const uint8_t pixelmap_transportmode[8]       PROGMEM = { 0xe0, 0x40, 0x40, 0x51, 0x5b, 0x15, 0x11, 0x11 };
const uint8_t pixelmap_beeper[8]              PROGMEM = { 0xff, 0xe7, 0xe7, 0xe7, 0xe7, 0xff, 0xe7, 0xff };
// Digits 3x5                     http://www.xlr8.at/8x8hexbin/
const uint8_t digit_0[8] PROGMEM = { 0x70, 0x50, 0x50, 0x50, 0x70, 0x0, 0x0, 0x0 };
const uint8_t digit_1[8] PROGMEM = { 0x10, 0x10, 0x10, 0x10, 0x10, 0x0, 0x0, 0x0 };
const uint8_t digit_2[8] PROGMEM = { 0x70, 0x10, 0x70, 0x40, 0x70, 0x0, 0x0, 0x0 };
const uint8_t digit_3[8] PROGMEM = { 0x70, 0x10, 0x70, 0x10, 0x70, 0x0, 0x0, 0x0 };
const uint8_t digit_4[8] PROGMEM = { 0x50, 0x50, 0x70, 0x10, 0x10, 0x0, 0x0, 0x0 };
const uint8_t digit_5[8] PROGMEM = { 0x70, 0x40, 0x70, 0x10, 0x70, 0x0, 0x0, 0x0 };
const uint8_t digit_6[8] PROGMEM = { 0x70, 0x40, 0x70, 0x50, 0x70, 0x0, 0x0, 0x0 };
const uint8_t digit_7[8] PROGMEM = { 0x70, 0x10, 0x10, 0x10, 0x10, 0x0, 0x0, 0x0 };
const uint8_t digit_8[8] PROGMEM = { 0x70, 0x50, 0x70, 0x50, 0x70, 0x0, 0x0, 0x0 };
const uint8_t digit_9[8] PROGMEM = { 0x70, 0x50, 0x70, 0x10, 0x70, 0x0, 0x0, 0x0 };

// Variables
int to_show_startup = 30;     // Loops remained to show startup (>0, until startup information is shown)
float speed, current;
int battery, bat_lvl, temperature, distance, total_distance, prev_total_distance, beeper_state;
byte count, i, data[100], alert, beeper;


void setup() {
  Serial.begin(115200);                               // Baudrate to parse data from EUC
  if (beeper_pin) pinMode(beeper_pin, INPUT);         // Activate input pin for beeper, if it is set up
  mtrx.begin();
  mtrx.setRotation(0);                                // [0...3]
  mtrx.autoPrintln(0);                                // Disable auto line brake for matrix display
  mtrx.setBright(brightness_stop);                    // Minimal brightness on startup
  mtrx.drawBitmap(0, 0, pixelmap_welcome_logo, 8, 8); // Draw welcome logo
  mtrx.update();                                      // Show drawn image on display
  delay(1000);                                        // Logo presentation delay
  mtrx.clear();                                       // Display clear
  mtrx.drawBitmap(0, 0, pixelmap_version, 8, 8);      // Draw version number
  mtrx.update();
  delay(100);                                         // Firmware presentation delay
  mtrx.clear();
  mtrx.update();
}

void loop() {
  //                                                     DATA RECEIVING
  delay(100);                                         // Don't change this! Tried shorter delays, but data were partially broken.
  if (debug_mode2) {
    Serial.print("bytes:"); Serial.println(Serial.available());
  };
  //                                                     DATA PARSING
  if (Serial.available() || debug_mode1) {
    count = Serial.available(); i = 0;                // Counting the bumber of bytes
    while (Serial.available()) {
      data[i] = Serial.read();                        // Writing data as array 'data[]'
      i++;
    }
  }

  mtrx.clear();
  if (count == 48 && data[18] == (byte) 0x00) {       // Checking integrity of data
    int spd = data[4] << 8 | data[5];                 // Speed parsing
    speed = abs(spd * 3.6 / 100);                     // Speed converting to absolute in km/h
    word vbat = data[2] << 8 | data[3];               // Voltage value parsing as of 67v EUC's
    calc_battery(vbat);                               // Voltage converted into battery percentage
    int temp = data[12] << 8 | data[13];              // Temperature parsing
    temperature = temp / 340.0 + 36.53;               // Temperature converted from sensor raw data to Celsium degrees
    unsigned long dist = (unsigned long)data[6] << 24 | (unsigned long)data[7] << 16 | (unsigned long)data[8] << 8 | (unsigned long)data[9];            // Distance parsed from 4 bytes (thus it's so long)
    distance = dist / 1000;                                                                                                                             // Distance converted from meters to kilometers and rounded
    unsigned long total_dist = (unsigned long)data[26] << 24 | (unsigned long)data[27] << 16 | (unsigned long)data[28] << 8 | (unsigned long)data[29];  // Total distance parsed from 4 bytes (thus it's so long
    total_distance = total_dist / 1000;                                                                                                                 // Total distance converted from meters to kilometers and rounded
    int amp = data[10] << 8 | data[11];               // Current/amperage parsing
    current = amp / 100;                              // Current converted to Ampers
    alert = data[36];                                 // Parsing alert flags (if it is supported by EUC Mainboard version)
    beeper = data[38];                                // Parsing beeper flag (if it is supported by EUC Mainboard version)
  }
  if (debug_mode1) {                                  // Synthetical data for variables instead of parcing EUC data for debug_mode1
    speed = 30;
    battery = 60;
    temperature = 37;
    current = -2;
    distance = 61;
    total_distance = 15061;
    alert = 0b00000000;
  }
  if (debug_mode2)                                    // Diagnostical data output for debug_mode2
    Serial.println("\t\t\t" + String(speed) + "kmh \t" + String(battery) + "% \t" + String(temperature) + "°C \t" + String(current) + "A \t" + String(distance) + "km \t" + String(total_distance) + "km_total \tbeep:" + String(beeper) + " \talert:" + String(alert));

  //                                                     DATA OUTPUT
  if (to_show_startup && total_distance && (prev_total_distance = total_distance)) show_startup();          // Running show_startup if it wasn't shown and total distance is sucessfully parsed at seast 2 times on a row (definition of readiness)
  prev_total_distance = total_distance;                                                                     // Saving total_distance as 'previous' for the next loop
  if (alert) {
    for (i = 0; i < 8; i++) {                                                                               // Showing alerts from EUC (if supported by motherboard)
      if (((alert >> i) & 0x01) == 1) show_alert(i);
    }
  }
  else {
    if (beeper_pin) beeper_state = digitalRead(beeper_pin);                                   
    if (beeper_state == HIGH)             show_alert(8);            // Alert for beeper signals, if physically connected (experimental)
    if (beeper)                           show_alert(8);            // Alert for physical beeper, if connected to Arduino
    if (temperature >= temperature_alarm) show_alert(5);            // Temperature alert
  }

  if (speed > 5)  mtrx.setBright(brightness_ride);                  // Brightness management, by speed
  else            mtrx.setBright(brightness_stop);
  if (speed < 1)  draw_digits(distance);                            // Draw trip distance if speed < 1 kmh
  else            draw_digits(speed);                               // Draw speed
  bat_lvl = round(battery * 0.16);                                  // Converting battery percentage into 16-level grade
  if (bat_lvl >= 9) {                                               // Draw battery level for 2-lined scale (if level > 8)
    mtrx.fastLineH(6, 16 - bat_lvl, 7);
    mtrx.fastLineH(7, 0, 7);
  } else if (bat_lvl >= 1) {                                        // Draw battery level for 1-lined scale (if level < 8)
    mtrx.fastLineH(7, 0, bat_lvl - 1);
  }
  if (current > 3)  mtrx.drawBitmap(0, 8, pixelmap_brake, 8, 8);    // Draw stop-signal
  else {
    if (backlight_txt.length() && speed > scrolling_on_euc_speed)   // Draw tail lights
    {
      mtrx.setCursor(scroll_x_shift, scroll_y_shift);
      mtrx.print(backlight_txt);
      if (scroll_x_shift + backlight_txt.length() * 6 - 1 > 0) scroll_x_shift = scroll_x_shift - 1;
      else scroll_x_shift = 7;
    }
    else mtrx.drawBitmap(0, 8, pixelmap_backlight, 8, 8);
  }
  mtrx.update();                                                    // Final output to display
}

void show_alert(int alert_type) {                                   // BLINKING ALERTS OUTPUT
  for (int i = 0; i < 7; i++) {                                     // 10 iterations for blinking effect
    mtrx.clear();
    if (i & 0x01) {                                                 // Alarm is shown on uneven numbers of 'i'
      switch (alert_type) {
        case 0: mtrx.drawBitmap(0, 0, pixelmap_highpower, 8, 8);        mtrx.drawBitmap(0, 8, pixelmap_highpower, 8, 8);        break;
        case 3: mtrx.drawBitmap(0, 0, pixelmap_lowvoltage, 8, 8);       mtrx.drawBitmap(0, 8, pixelmap_lowvoltage, 8, 8);       break;
        case 4: mtrx.drawBitmap(0, 0, pixelmap_overvoltage, 8, 8);      mtrx.drawBitmap(0, 8, pixelmap_overvoltage, 8, 8);      break;
        case 5: mtrx.drawBitmap(0, 0, pixelmap_overtemperature, 8, 8);  mtrx.drawBitmap(0, 8, pixelmap_overtemperature, 8, 8);  break;
        case 6: mtrx.drawBitmap(0, 0, pixelmap_hallsensors, 8, 8);      mtrx.drawBitmap(0, 8, pixelmap_hallsensors, 8, 8);      break;
        case 7: mtrx.drawBitmap(0, 0, pixelmap_transportmode, 8, 8);    mtrx.drawBitmap(0, 8, pixelmap_transportmode, 8, 8);    break;
        case 8: mtrx.drawBitmap(0, 0, pixelmap_beeper, 8, 8);           mtrx.drawBitmap(0, 8, pixelmap_beeper, 8, 8);           break;
      }
    }
    mtrx.update();
    delay(75);                                                      // Delay for blinking effect
  }
  mtrx.clear();
  mtrx.update();
  delay(100);
}
void draw_digits(int digits_value) {                                // MAIN 2-DIGIT OUTPUT
  //  while (digits_value > 99) digits_value = digits_value - 100;
  int digit[2];
  digit[0] = digits_value / 10;                                     // Left symbol
  digit[1] = digits_value - digit[0] * 10;                          // Right symbol
  for (int i = 0; i < 2; i++) {
    switch (digit[i]) {
      case 0: mtrx.drawBitmap(4 * i, 0, digit_0, 4, 8); break;
      case 1: mtrx.drawBitmap(4 * i, 0, digit_1, 4, 8); break;
      case 2: mtrx.drawBitmap(4 * i, 0, digit_2, 4, 8); break;
      case 3: mtrx.drawBitmap(4 * i, 0, digit_3, 4, 8); break;
      case 4: mtrx.drawBitmap(4 * i, 0, digit_4, 4, 8); break;
      case 5: mtrx.drawBitmap(4 * i, 0, digit_5, 4, 8); break;
      case 6: mtrx.drawBitmap(4 * i, 0, digit_6, 4, 8); break;
      case 7: mtrx.drawBitmap(4 * i, 0, digit_7, 4, 8); break;
      case 8: mtrx.drawBitmap(4 * i, 0, digit_8, 4, 8); break;
      case 9: mtrx.drawBitmap(4 * i, 0, digit_9, 4, 8); break;
    }
  }
}

int calc_battery(int voltage) {                                   // BATTERY CALCULATION
  if (useBetterPercents) {                                        // WheelLog's 'use better percents' formula
    if (voltage > 6680) {
      battery = 100;
    } else if (voltage > 5440) {
      battery = (voltage - 5380) / 13;
    } else if (voltage > 5290) {
      battery = (int) round((voltage - 5290) / 32.5);
    } else {
      battery = 0;
    }
  } else {                                                        // WheelLog's normal formula
    if (voltage <= 5290) {
      battery = 0;
    } else if (voltage >= 6580) {
      battery = 100;
    } else {
      battery = (voltage - 5290) / 13;
    }
  }
  return battery;
}

void show_startup() {                                             // STARTUP PERFORMANCE
  if (to_show_startup == 1) {
    String str = String(battery) + "% " + String(temperature) + "C " + String(total_distance) + "km";
    mtrx.clear();
    mtrx.update();
    delay(100);
    for (i = 0; i < str.length() * 6 + 7; i++) {
      mtrx.clear();
      mtrx.drawBitmap(0, 8, pixelmap_backlight, 8, 8);            // Tail light to be turned on
      mtrx.setCursor(7 - i, 0);
      mtrx.print(str);
      mtrx.update();
      delay(50);
    }
    mtrx.clear();
    mtrx.update();
  }
  to_show_startup = to_show_startup - 1;
}


/*

  // настройка
  void begin();                   // запустить
  void setBright(byte value);     // установить яркость [0-15]
  void setPower(bool value);      // переключить питание

  // ориентация
  void setRotation(uint8_t rot);      // поворот МАТРИЦ (8x8): 0, 1, 2, 3 на 90 град по часовой стрелке
  void setFlip(bool x, bool y);       // зеркальное отражение МАТРИЦ (8x8) по x и y
  void setType(bool type);            // конструкция дисплея (тип строчности)
  void setConnection(uint8_t conn);   // точка подключения дисплея

  // рисование
  void clear();                   // очистить
  void fill();                    // залить
  void fillByte(byte data);       // залить байтом
  void dot(int x, int y, byte fill = 1);  // установить точку
  bool get(int x, int y);         // получить точку
  void update();                  // обновить

  // а также наследует всё из GyverGFX:
  void dot(int x, int y, uint8_t fill = 1);                           // точка, fill - GFX_CLEAR/GFX_FILL/GFX_STROKE
  void fastLineH(int y, int x0, int x1, uint8_t fill = 1);            // вертикальная линия, fill - GFX_CLEAR/GFX_FILL/GFX_STROKE
  void fastLineV(int x, int y0, int y1, uint8_t fill = 1);            // горизонтальная линия, fill - GFX_CLEAR/GFX_FILL/GFX_STROKE
  void line(int x0, int y0, int x1, int y1, uint8_t fill = 1);        // линия, fill - GFX_CLEAR/GFX_FILL/GFX_STROKE
  void rect(int x0, int y0, int x1, int y1, uint8_t fill = 1);        // прямоугольник, fill - GFX_CLEAR/GFX_FILL/GFX_STROKE
  void roundRect(int x0, int y0, int x1, int y1, uint8_t fill = 1);   // скруглённый прямоугольник, fill - GFX_CLEAR/GFX_FILL/GFX_STROKE
  void circle(int x, int y, int radius, uint8_t fill = 1);            // окружность, fill - GFX_CLEAR/GFX_FILL/GFX_STROKE
  void bezier(uint8_t* arr, uint8_t size, uint8_t dense, uint8_t fill = 1);   // кривая Безье
  void bezier16(int* arr, uint8_t size, uint8_t dense, uint8_t fill = 1);     // кривая Безье 16 бит. fill - GFX_CLEAR/GFX_FILL/GFX_STROKE
  void drawBitmap(int x, int y, const uint8_t *frame, int width, int height, uint8_t invert = 0, byte mode = 0);  // битмап
  void setCursor(int x, int y);           // установить курсор
  void setScale(uint8_t scale);           // масштаб текста
  void invertText(bool inv);              // инвертировать текст
  void autoPrintln(bool mode);            // автоматический перенос строки
  void textDisplayMode(bool mode);        // режим вывода текста GFX_ADD/GFX_REPLACE

  // и из Print.h
  // print/println любой тип данных



  const uint8_t pixelmap_welcome_logo[8]        PROGMEM = { 0xdb, 0xdb, 0x9a, 0xda, 0xda, 0x9a, 0xc3, 0xc3 }; // Стартовое лого EUC
  const uint8_t pixelmap_backlight[8]           PROGMEM = { 0x18, 0x24, 0x3c, 0x24, 0x24, 0x66, 0x99, 0x66 }; // Габаритный огонь для @SPB_Artem_Prokopenkov
  const uint8_t pixelmap_backlight[8]           PROGMEM = { 0xff, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0xff }; // Габаритный огонь в виде квадрата



  // Цифры для спидометра 3*5 квадратные   http://www.xlr8.at/8x8hexbin/
  const uint8_t digit_0[8] PROGMEM = { 0x70, 0x50, 0x50, 0x50, 0x70, 0x0, 0x0, 0x0 };
  const uint8_t digit_1[8] PROGMEM = { 0x10, 0x10, 0x10, 0x10, 0x10, 0x0, 0x0, 0x0 };
  const uint8_t digit_2[8] PROGMEM = { 0x70, 0x10, 0x70, 0x40, 0x70, 0x0, 0x0, 0x0 };
  const uint8_t digit_3[8] PROGMEM = { 0x70, 0x10, 0x70, 0x10, 0x70, 0x0, 0x0, 0x0 };
  const uint8_t digit_4[8] PROGMEM = { 0x50, 0x50, 0x70, 0x10, 0x10, 0x0, 0x0, 0x0 };
  const uint8_t digit_5[8] PROGMEM = { 0x70, 0x40, 0x70, 0x10, 0x70, 0x0, 0x0, 0x0 };
  const uint8_t digit_6[8] PROGMEM = { 0x70, 0x40, 0x70, 0x50, 0x70, 0x0, 0x0, 0x0 };
  const uint8_t digit_7[8] PROGMEM = { 0x70, 0x10, 0x10, 0x10, 0x10, 0x0, 0x0, 0x0 };
  const uint8_t digit_8[8] PROGMEM = { 0x70, 0x50, 0x70, 0x50, 0x70, 0x0, 0x0, 0x0 };
  const uint8_t digit_9[8] PROGMEM = { 0x70, 0x50, 0x70, 0x10, 0x70, 0x0, 0x0, 0x0 };

*/
