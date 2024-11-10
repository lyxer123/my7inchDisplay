# Arduino project for Arduino GIGA R1 Wifi and GIGA Display Shield 3.97" Display with 480x800 RGB resolution

## Prerequisites

1. Arduino-IDE should be installed on your system. (The build and board was tested with v2.3.2)
2. In `Tools`/`Boards Manager` select and install `Arduino Mbed OS GIGA Boards` package (contains the Ardunino GIGA R1 toolchains, the board was tested with version 4.1.5 of Arduino Mbed OS GIGA Boards package.)
3. (The pre-set essence of `Arduino_H7_Video V1.0.0` & `Arduino_GigaDisplayTouch V1.0.1` with the display and touch driver is included in the local `libraries` folder of this board-template.)

## Board settings (retained between Arduino-IDE sessions)

1. In `Tools` select `Arduino Mbed OS GIGA Boards` / `Arduino Giga R1` as board

## Compilation

1. Open the `.ino` file in Arduino
2. Go to File/Preferences and set Sketchbook location to the path of your UI project (where this README is located). The setting is retained between Arduino-IDE sessions.
3. Build the project

## Burning

The built-in Arduino-IDE chip-programmer, press the 'Upload' (arrow) button in the toolbar after selecting the port. (`COMxx (Arduino Giga R1)` Board selected in Tools/Port menu)

## For more information:

- https://docs.arduino.cc/tutorials/giga-display-shield/lvgl-guide/



2024 SquareLine
