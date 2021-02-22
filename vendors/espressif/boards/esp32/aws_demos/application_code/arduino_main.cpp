#include "Arduino.h"

#include "FreematicsPlus.h"

FreematicsESP32 sys;

#ifdef ARDUINO_EXAMPLE

COBD obd;
bool connected = false;
unsigned long count = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, HIGH);
  delay(1000);
  digitalWrite(PIN_LED, LOW);
  Serial.begin(115200);

  // initializations
  while (!sys.begin());
  sys.gpsBegin(115200);
}

GPS_DATA* gd = 0;
char isoTime[26] = {0};

void loop() {
    sys.gpsGetData(&gd);
    
  float kph = (float)((int)(gd->speed * 1.852f * 10)) / 10;

    char *p = isoTime + sprintf(isoTime, "%04u-%02u-%02uT%02u:%02u:%02u",
        (unsigned int)(gd->date % 100) + 2000, (unsigned int)(gd->date / 100) % 100, (unsigned int)(gd->date / 10000),
        (unsigned int)(gd->time / 1000000), (unsigned int)(gd->time % 1000000) / 10000, (unsigned int)(gd->time % 10000) / 100);
    unsigned char tenth = (gd->time % 100) / 10;
    if (tenth) p += sprintf(p, ".%c00", '0' + tenth);
    *p = 'Z';
    *(p + 1) = 0;
  
    Serial.print("[GPS] ");
    Serial.print(gd->lat, 6);
    Serial.print(' ');
    Serial.print(gd->lng, 6);
    Serial.print(' ');
    Serial.print((int)kph);
    Serial.print("km/h");
    if (gd->sat) {
    Serial.print(" SATS:");
    Serial.print(gd->sat);
    }
    Serial.print(" Course:");
    Serial.print(gd->heading);

    Serial.print(' ');
    Serial.println(isoTime);
    vTaskDelay( pdMS_TO_TICKS( 1000 ));
}


extern "C" void arduino_main( void )
{
    Serial.begin(115200);
    Serial.print("Arduino start\r\n");
    
    setup();
    while(1)
    {
        loop();
    }
}

#else

#define FREEMATIC_SYS_INIT_MAX_TRIES        ( 3U )

extern "C" void arduino_main( void )
{
    int i = 0;

    /* Setup the system indication and logging. */
    Serial.begin(115200);   /* Serial device is initialized by FreeRTOS AWS. */

    /* Initialize system device. */
    for( i = 0; i < FREEMATIC_SYS_INIT_MAX_TRIES; i++ )
    {
        if( sys.begin() == true )
        {
            break;
        }
        else
        {
            sys.end();
        }
    }
}

#endif
