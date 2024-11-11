# Waveshare ESP32-S3-Touch-LCD-7 遥控器


Check the profile in platformio.ini:

```
[env:esp32s3box]
platform = espressif32
board = esp32s3box
framework = arduino
monitor_speed = 115200
board_upload.flash_size = 8MB
build_flags = 
	-D BOARD_HAS_PSRAM
	-D LV_CONF_INCLUDE_SIMPLE
	-I lib
board_build.arduino.memory_type = qio_opi
board_build.f_flash = 80000000L
board_build.flash_mode = qio
lib_deps = 
	lvgl/lvgl@8.3.8
```

