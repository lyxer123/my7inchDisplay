#include "Arduino_H7_Video.h"
#include "Arduino_GigaDisplayTouch.h"
#include "lvgl.h"
#include <ui.h>

Arduino_H7_Video Display( 800, 480, GigaDisplayShield ); //( 800, 480, GigaDisplayShield );
Arduino_GigaDisplayTouch TouchDetector;

void setup() {
  Display.begin();
  TouchDetector.begin();

  ui_init();
}

void loop()
{
  lv_timer_handler();
  delay(5);
}
