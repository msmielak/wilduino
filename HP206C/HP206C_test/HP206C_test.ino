/*
 * Demo name   : HP206C_test
 * Usage       : I2C PRECISION BAROMETER AND ALTIMETER [HP206C hopeRF]
 * Author      : MŚ based on HP20X_dev library by Oliver Wang from Seeed Studio
 * Version     : V0.1
 * Change log  : I got rid of the Kalman filter to simplify the code
*/

#include <HP20x_dev.h>
#include "Arduino.h"
#include "Wire.h"
#include "RTClib.h"


/* Using external RTC for timekeeping */
RTC_DS1307 rtc;
unsigned char ret = 0;




void setup()
{
  Serial.begin(9600);        // start serial for output

  Serial.println("****HP206C test procedure \n");
  Serial.println("Calculation formula: H = [8.5(101325-P)]/100 \n");
  /* Power up,delay 150ms,until voltage is stable */
  delay(150);
  /* Reset HP20x_dev */
  HP20x.begin();
  delay(100);

  /* Determine HP20x_dev is available or not */
  ret = HP20x.isAvailable();
  if(OK_HP20X_DEV == ret)
  {
    Serial.println("HP20x_dev is available.\n");
  }
  else
  {
    Serial.println("HP20x_dev isn't available.\n");
  }

}


void loop()
{

 /* Reading time from the RTC and outputing it */
  DateTime now = rtc.now();

    Serial.print(now.year(), DEC);
    Serial.print('/');
    Serial.print(now.month(), DEC);
    Serial.print('/');
    Serial.print(now.day(), DEC);
    Serial.print(' ');
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    Serial.print(now.second(), DEC);


    char display[40];

   /* Reading temperature, pressure and altitude and displaying it - once every second (note that measurement can take up to 0.13s so it is probably going to be slightly more than 1s) */

    if(OK_HP20X_DEV == ret)

{
	  long Temper = HP20x.ReadTemperature();
	  Serial.print("Temper:");
	  t = Temper/100.0;
	  Serial.print(t);
	  Serial.print("C.; ");

      long Pressure = HP20x.ReadPressure();
	  Serial.print("Pressure:");
	  p = Pressure/100.0;
	  Serial.print(p);
	  Serial.print("hPa.; ");

	  long Altitude = HP20x.ReadAltitude();
	  Serial.print("Altitude:");
	  a = Altitude/100.0;
	  Serial.print(a);
	  Serial.println("m.; ");




      delay(1000);
    }
}
