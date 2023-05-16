#include "AiEsp32RotaryEncoder.h"
#include "Arduino.h"

#include <Preferences.h>

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "driver/gpio.h"
#include "driver/twai.h"

#include "sineTab.h"

#define ROTARY_ENCODER_STEPS 4

#define SCREEN_WIDTH 128    // OLED display width, in pixels
#define SCREEN_HEIGHT 32    // OLED display height, in pixels
#define OLED_RESET -1       // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

#define VAL_MODE 0
#define ID_MODE 1
#define BAUD_MODE 2

#define VAL_MAX 1000
#define ADS_MAX 4095

#define SINE_STEPS 1024

/*
#define TWAI_TIMING_CONFIG_25KBITS()    {.brp = 128, .tseg_1 = 16, .tseg_2 = 8, .sjw = 3, .triple_sampling = false}
#define TWAI_TIMING_CONFIG_50KBITS()    {.brp = 80, .tseg_1 = 15, .tseg_2 = 4, .sjw = 3, .triple_sampling = false}
#define TWAI_TIMING_CONFIG_100KBITS()   {.brp = 40, .tseg_1 = 15, .tseg_2 = 4, .sjw = 3, .triple_sampling = false}
#define TWAI_TIMING_CONFIG_125KBITS()   {.brp = 32, .tseg_1 = 15, .tseg_2 = 4, .sjw = 3, .triple_sampling = false}
#define TWAI_TIMING_CONFIG_250KBITS()   {.brp = 16, .tseg_1 = 15, .tseg_2 = 4, .sjw = 3, .triple_sampling = false}
#define TWAI_TIMING_CONFIG_500KBITS()   {.brp = 8, .tseg_1 = 15, .tseg_2 = 4, .sjw = 3, .triple_sampling = false}
#define TWAI_TIMING_CONFIG_800KBITS()   {.brp = 4, .tseg_1 = 16, .tseg_2 = 8, .sjw = 3, .triple_sampling = false}
#define TWAI_TIMING_CONFIG_1MBITS()     {.brp = 4, .tseg_1 = 15, .tseg_2 = 4, .sjw = 3, .triple_sampling = false}
*/
const char *baudrates[5] = {"125kbps", "250kbps", "500kbps", "800kbps", "1mbps"};

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

AiEsp32RotaryEncoder rotaryEncoder = AiEsp32RotaryEncoder(32, 25, 27, -1, ROTARY_ENCODER_STEPS);

uint16_t outVal = 0;
uint16_t outId = 69;
uint16_t outBaud = 500;

uint8_t mode = 0;

bool active0;
uint16_t val0 = 0;
uint16_t id0 = 2;
uint16_t min0 = 0;
uint16_t max0 = 4095;

bool active1;
uint16_t val1 = 0;
uint16_t id1 = 3;
uint16_t min1 = 0;
uint16_t max1 = 4095;
// uint16_t freq = 1;
uint16_t period = 1000; // ms to loop through the sineTab
uint16_t interval = period / SINE_STEPS;
uint16_t idx1;

ulong timer = millis();

uint16_t baudRate = 2;

Preferences prefs;
TaskHandle_t xmitCanTask;

void loadPrefs()
{
  prefs.begin("my-app", true);

  outBaud = prefs.getUShort("outBaud", 3);

  active0 = prefs.getBool("active0", false);
  id0 = prefs.getUShort("id0", 3);
  val0 = prefs.getUShort("val0", 0);
  min0 = prefs.getUShort("min0", 0);
  max0 = prefs.getUShort("max0", 1000);

  active1 = prefs.getBool("active1", false);
  id1 = prefs.getUShort("id1", 3);
  min1 = prefs.getUShort("min1", 0);
  max1 = prefs.getUShort("max1", 1000);
  period = prefs.getUShort("period", 1000);

  prefs.end();
}

void savePrefs()
{
  prefs.begin("my-app", false);

  prefs.putUShort("outBaud", outBaud);

  prefs.putBool("active0", active0);
  prefs.putUShort("id0", id0);
  prefs.putUShort("val0", val0);
  prefs.putUShort("min0", min0);
  prefs.putUShort("max0", max0);

  prefs.putBool("active1", active1);
  prefs.putUShort("id1", id1);
  prefs.putUShort("min1", min1);
  prefs.putUShort("max1", max1);
  prefs.putUShort("period", period);

  prefs.end();
}

void cycleMode()
{
  mode++;

  if (mode > 11)
    mode = 0;

  switch (mode)
  {
  case 0:
    rotaryEncoder.setBoundaries(min0, max0, false);
    rotaryEncoder.setAcceleration(500);
    rotaryEncoder.setEncoderValue(val0);
    break;
  case 1:
    rotaryEncoder.setBoundaries(0, 1, false);
    rotaryEncoder.setAcceleration(0);
    rotaryEncoder.setEncoderValue(active0);
    break;
  case 2:
    rotaryEncoder.setBoundaries(0, 99, false);
    rotaryEncoder.setAcceleration(50);
    rotaryEncoder.setEncoderValue(id0);
    break;
  case 3:
    rotaryEncoder.setBoundaries(0, 4095, false);
    rotaryEncoder.setAcceleration(750);
    rotaryEncoder.setEncoderValue(min0);
    break;
  case 4:
    rotaryEncoder.setBoundaries(0, 4095, false);
    rotaryEncoder.setAcceleration(750);
    rotaryEncoder.setEncoderValue(max0);
    break;
  case 5:
    rotaryEncoder.setBoundaries(0, 1, false);
    rotaryEncoder.setAcceleration(0);
    rotaryEncoder.setEncoderValue(active1);
    break;
  case 6:
    break;
  case 7:
    rotaryEncoder.setBoundaries(0, 99, false);
    rotaryEncoder.setAcceleration(50);
    rotaryEncoder.setEncoderValue(id1);
    break;
  case 8:
    rotaryEncoder.setBoundaries(0, 4095, false);
    rotaryEncoder.setAcceleration(750);
    rotaryEncoder.setEncoderValue(min1);
    break;
  case 9:
    rotaryEncoder.setBoundaries(0, 4095, false);
    rotaryEncoder.setAcceleration(750);
    rotaryEncoder.setEncoderValue(max1);
    break;
  case 10:
    rotaryEncoder.setBoundaries(1000, 10000, false);
    rotaryEncoder.setAcceleration(750);
    rotaryEncoder.setEncoderValue(period);
    break;
  case 11:
    rotaryEncoder.setBoundaries(0, 4, false);
    rotaryEncoder.setAcceleration(0);
    rotaryEncoder.setEncoderValue(outBaud);
    break;
  }
}

void rotary_onButtonClick()
{
  static unsigned long lastTimePressed = 0;
  if (millis() - lastTimePressed < 500)
    return;

  Serial.println("mode change");

  lastTimePressed = millis();
  cycleMode();
}

void rotary_loop()
{
  if (rotaryEncoder.encoderChanged())
  {
    switch (mode)
    {
    case 0:
      val0 = rotaryEncoder.readEncoder();
      break;
    case 1:
      active0 = rotaryEncoder.readEncoder();
      break;
    case 2:
      id0 = rotaryEncoder.readEncoder();
      break;
    case 3:
      min0 = rotaryEncoder.readEncoder();
      break;
    case 4:
      max0 = rotaryEncoder.readEncoder();
      break;
    case 5:
      active1 = rotaryEncoder.readEncoder();
      break;
    case 6:
      val1 = rotaryEncoder.readEncoder();
      break;
    case 7:
      id1 = rotaryEncoder.readEncoder();
      break;
    case 8:
      min1 = rotaryEncoder.readEncoder();
      break;
    case 9:
      max1 = rotaryEncoder.readEncoder();
      break;
    case 10:
      period = rotaryEncoder.readEncoder();
      interval = period / SINE_STEPS;
      break;
    case 11:
      // outBaud= rotaryEncoder.readEncoder(); break;
      break;
    default:
      break;
    }

    // save these values
  }
  if (rotaryEncoder.isEncoderButtonClicked())
  {
    savePrefs();
    rotary_onButtonClick();
  }
}

void IRAM_ATTR readEncoderISR()
{
  rotaryEncoder.readEncoder_ISR();
}

void xmitCan(void *parameter)
{
  for (;;)
  {
    if (active0)
    {
      twai_message_t message0;
      message0.identifier = id0;
      message0.extd = 0;
      message0.data_length_code = 4;

      message0.data[0] = 0;
      message0.data[1] = 0;
      message0.data[2] = (outVal >> 8);
      message0.data[3] = outVal & 0xff;

      twai_transmit(&message0, pdMS_TO_TICKS(1000));
      delayMicroseconds(20);
    }

    if (active1 && (millis() - timer) > interval)
    {
      timer = millis();

      // for the sine wave
      twai_message_t message1;
      message1.identifier = id1;
      message1.extd = 0;
      message1.data_length_code = 4;

      val1 = map(sineTab[idx1], 0, 4095, min1, max1);

      message1.data[0] = 0;
      message1.data[1] = 0;
      message1.data[2] = (val1 >> 8);
      message1.data[3] = val1 & 0xff;

      idx1 += (1 + 0);
      if (idx1 > 1023)
        idx1 = idx1 - 1023;

      twai_transmit(&message1, pdMS_TO_TICKS(1000));
      delayMicroseconds(20);
    }
  }
}

void initCan()
{
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_5, GPIO_NUM_4, TWAI_MODE_NO_ACK);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  // Install TWAI driver
  twai_driver_install(&g_config, &t_config, &f_config);
  twai_start();
}

void setup()
{
  Serial.begin(115200);

  loadPrefs();
  initCan();

  rotaryEncoder.begin();
  rotaryEncoder.setup(readEncoderISR);
  rotaryEncoder.setBoundaries(0, 4095, false);
  rotaryEncoder.setAcceleration(750);

  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS))
  {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ; // Don't proceed, loop forever
  }
  else
    display.setRotation(2);

  xTaskCreatePinnedToCore(xmitCan, "xmitCan", 10000, NULL, 0, &xmitCanTask, 0);
}

void loop()
{
  // in loop call your custom function which will process rotary encoder values
  rotary_loop();
  delay(50); // or do whatever you need to do...

  display.clearDisplay();

  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2); // Draw 2X-scale text
  display.setCursor(1, 1);

  switch (mode)
  {
  case 0:
    display.setTextSize(1);
    display.println("Ch0 Value");
    display.setTextSize(3);
    display.print(val0);
    break;
  case 1:
    display.setTextSize(1);
    display.println("Ch0 Active");
    display.setTextSize(3);
    if (active0)
      display.print("ON");
    else
      display.print("OFF");
    break;
  case 2:
    display.setTextSize(1);
    display.println("Ch0 id");
    display.setTextSize(3);
    display.print(id0);
    break;
  case 3:
    display.setTextSize(1);
    display.println("Ch0 min");
    display.setTextSize(3);
    display.print(min0);
    break;
  case 4:
    display.setTextSize(1);
    display.println("Ch0 max");
    display.setTextSize(3);
    display.print(max0);
    break;
  case 5:
    display.setTextSize(1);
    display.println("Ch1 Active");
    display.setTextSize(3);
    if (active1)
      display.print("ON");
    else
      display.print("OFF");
    break;
  case 6:
    display.setTextSize(1);
    display.println("Ch1 Value");
    display.setTextSize(3);
    display.print(val1);
    break;
  case 7:
    display.setTextSize(1);
    display.println("Ch1 id");
    display.setTextSize(3);
    display.print(id1);
    break;
  case 8:
    display.setTextSize(1);
    display.println("Ch1 min");
    display.setTextSize(3);
    display.print(min1);
    break;
  case 9:
    display.setTextSize(1);
    display.println("Ch1 max");
    display.setTextSize(3);
    display.print(max1);
    break;
  case 10:
    display.setTextSize(1);
    display.println("Ch1 Period");
    display.setTextSize(3);
    display.print(period);
    break;
  case 11:
    display.setTextSize(1);
    display.println("BAUDRATE");
    display.setTextSize(3);
    display.print(baudrates[outBaud]);
    break;
  default:
    break;
  }

  display.display();
}

// to add
// add channel with like a sine wave generator
// for dial channel adjust min,max and accel
// implement baudrate