#include "Arduino.h"

extern "C" void arduino_main( void )
{
    Serial.begin(115200);
    Serial.print("Arduino start\r\n");
}
