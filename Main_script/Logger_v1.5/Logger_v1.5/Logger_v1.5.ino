
/*
 * Logger_v1.4.ino
 * Robert Farrell 22/5/2017
 * GPS configuration (configGPS(), sendUBX() and getACK_UBX())
 * based upon https://ukhas.org.uk/guides:ublox6 calcChecksum()
 * from https://playground.arduino.cc/UBlox/GPS
 *
 * Configures u-blox GPS, starts HP206c Barometric sensor, and
 * sets RTC to AEST time from GPS. It then samples data at defined,
 * configurable, intervals. HP206c provides temperature, pressure
 * and altitude, with time of sampling. GPS sampling provides
 * position data. Sleep functions have been implemented that turn
 * off GPS and put anarduino to sleep. These sleep functions are
 * for long sleep, during day, plus short sleep, between measurements
 *
 * Data is output to serial monitor, as well as saved to SD card.
 *
 */

#include <SD.h>
#include <SPI.h>
#include <Narcoleptic.h>
#include <avr/sleep.h>
#include <HP20x_dev.h>
#include <Wire.h>
#include <MCP7940RTC.h>
#include <Time.h>
#include <TimeLib.h>
#include <SoftwareSerial.h>

//Libraries to reduce power consumption
#include <RH_RF95.h>
#include <SPIFlash.h>
RH_RF95 rf95;
SPIFlash flash(5, 0);  //Anarduino
//SPIFlash flash(8, 0xEF30); // flash(SPI_CS, MANUFACTURER_ID)


#define TIMEOFSLEEP 19					  //hour (24) when GPS and anarduino go into power saving
#define SLEEPFOR 3600000				  //Time (ms) to be in power saving (currently 10 hours).
#define HP20LOG 60						  //Logging schedule for barometric data, in seconds
#define GPSLOG 120						  //Logging schedule for GPS position data, in seconds.
										  //The assumption is that the barometric data is required
										  //more frequently than the GPS data. See below, schdCount
										  //variable.

SoftwareSerial gpsSerial(A2, A3);
MCP7940RTC *pRTC;
uint8_t ret = 0, schdCount = GPSLOG / HP20LOG;

void setup()
{
	pinMode(3, INPUT);
	pinMode(9, OUTPUT);


	Serial.begin(9600);        //start serial for output
	delay(2000);
	Serial.println(F("Initialising...."));
	//Start gps softwareSerial port
	gpsSerial.begin(9600);
	delay(2000);

//Turning off flash memory and radio module
	rf95.init();
	rf95.sleep();
	flash.sleep();
Serial.println(F("Flash memory and RF95 module off"));

	//Reset HP20x_dev
	HP20x.begin();
	delay(100);

	//Determine HP20x_dev is available or not
	ret = HP20x.isAvailable();
	if(OK_HP20X_DEV == ret)
	{
		Serial.println(F("HP20x_dev is available.\n"));
	}
	else
	{
		Serial.println(F("HP20x_dev isn't available.\n"));
	}

	configGPS();

	//Wait for GPS warm ups to finish
	delay(2000);

	//Use this delay, instead of the one above, if starting from a cold
	//start. Half hour delay so that GPS obtains a fix.
	//delay(1800000);

	//Flush the gps serial port
	while(gpsSerial.available())
	{
		gpsSerial.read();
	}

	//Start RTC
	pRTC = new MCP7940RTC();

	//Set RTC via GPS
	setNewTimeRTC();
	//initialise sd card. Placed here after the more memory intensive
	//functions have exited.
	SD.begin(8);
}

void loop()
{
	//check whether time to go to sleep; sleep if yes
	sleepLong();

	//Time elapsed exceeds barometric sensor schedule: get data
	getHP20Data();
	Serial.println();
	delay(500);

	//sleep between measurements
	sleepTimer(HP20LOG * 1000L);
	delay(500);
}

/*
 * Configures the u-blox GPS. The GPS is first set back to factory
 * defaults, just to make sure that no errant settings are still present.
 * Navigation mode is then set to pedestrian; NMEA messages - GLL, GSA,
 * GSV, RMC, VTG, GGA, TXT - are turned off; and the current settings are saved.
 * Ten SUCCESS! statements should be received. If any configuration fails,
 * then the configuration is retried.
 */
void configGPS()
{
	boolean gps_set_success = false;

	//Making sure settings are at default
	Serial.println(F("Setting ublox to defaults"));
	uint8_t defConf[] = {0xB5, 0X62, 0x06, 0x09, 0x0D, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x01, 0x19, 0x98};

	//Retries until successful configuration
	while(!gps_set_success)
	{
		sendUBX(defConf, sizeof(defConf)/sizeof(uint8_t));
		gps_set_success = getUBX_ACK(defConf);
	}
	gps_set_success = false;

	//Set to pedestrian mode
	Serial.println(F("Setting uBlox nav mode: "));
	uint8_t setNav[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x03, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x13, 0x76};

	while(!gps_set_success)
	{
		sendUBX(setNav, sizeof(setNav)/sizeof(uint8_t));
		gps_set_success = getUBX_ACK(setNav);
	}
	gps_set_success = false;

	//Switching off NMEA GLL
	Serial.println(F("Switching off NMEA GLL: "));
	uint8_t setGLL[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x2B};

	while(!gps_set_success)
	{
		sendUBX(setGLL, sizeof(setGLL)/sizeof(uint8_t));
		gps_set_success = getUBX_ACK(setGLL);
	}
	gps_set_success = false;

	//Switching off NMEA GSA
	Serial.println(F("Switching off NMEA GSA: "));
	uint8_t setGSA[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x32};

	while(!gps_set_success)
	{
		sendUBX(setGSA, sizeof(setGSA)/sizeof(uint8_t));
		gps_set_success = getUBX_ACK(setGSA);
	}
	gps_set_success = false;

	//Switching off NMEA GSV
	Serial.println(F("Switching off NMEA GSV: "));
	uint8_t setGSV[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x39};

	while(!gps_set_success)
	{
		sendUBX(setGSV, sizeof(setGSV)/sizeof(uint8_t));
		gps_set_success = getUBX_ACK(setGSV);
	}
	gps_set_success = false;

	//Switching off NMEA RMC
	Serial.print(F("Switching off NMEA RMC: "));
	uint8_t setRMC[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x04, 0x40};

	while(!gps_set_success)
	{
		sendUBX(setRMC, sizeof(setRMC)/sizeof(uint8_t));
		gps_set_success = getUBX_ACK(setRMC);
	}
	gps_set_success = false;

	//Switching off NMEA VTG
	Serial.print(F("Switching off NMEA VTG: "));
	uint8_t setVTG[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x46};

	while(!gps_set_success)
	{
		sendUBX(setVTG, sizeof(setVTG)/sizeof(uint8_t));
		gps_set_success = getUBX_ACK(setVTG);
	}
	gps_set_success = false;

	//Switching off NMEA GGA
	Serial.print(F("Switching off NMEA GGA: "));
	uint8_t setGGA[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x23};

	while(!gps_set_success)
	{
		sendUBX(setGGA, sizeof(setGGA)/sizeof(uint8_t));
		gps_set_success = getUBX_ACK(setGGA);
	}
	gps_set_success = false;

	//Switching off NMEA TXT
	Serial.print(F("Switching off NMEA TXT: "));
	uint8_t setTXT[] = {0xB5, 0x62, 0x06, 0x02, 0x0A, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x13, 0xF0};

	while(!gps_set_success)
	{
		sendUBX(setTXT, sizeof(setTXT)/sizeof(uint8_t));
		gps_set_success = getUBX_ACK(setTXT);
	}
	gps_set_success = false;

	//Save current configurations
	Serial.println(F("Saving ublox configuration"));
	uint8_t newConf[] = {0xB5, 0X62, 0x06, 0x09, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x1B, 0xA9};

	while(!gps_set_success)
	{
		sendUBX(newConf, sizeof(newConf)/sizeof(uint8_t));
		gps_set_success = getUBX_ACK(newConf);
	}
}

/*
 *  Send a byte array of UBX protocol to the GPS
 */
void sendUBX(uint8_t *MSG, uint8_t len)
{
	for(int i = 0; i < len; i++)
	{
		gpsSerial.write(MSG[i]);
		Serial.print(MSG[i], HEX);
	}
	gpsSerial.println();

//blink LED to confirm GPS configuration (2 times)
digitalWrite(9, HIGH);
delay(150);
digitalWrite(9, LOW);
delay(150);
digitalWrite(9, HIGH);
delay(150);
digitalWrite(9, LOW);

}

/*
 * Calculate expected UBX ACK packet and parse UBX response from GPS.
 * Returns true on success and false on failure.
 */
boolean getUBX_ACK(uint8_t *MSG)
{
	uint8_t b;
	uint8_t ackByteID = 0;
	uint8_t ackPacket[10];
	unsigned long startTime = millis();
	Serial.print(F(" * Reading ACK response: "));

	//Construct the expected ACK packet
	ackPacket[0] = 0xB5;  // header
	ackPacket[1] = 0x62;  // header
	ackPacket[2] = 0x05;  // class
	ackPacket[3] = 0x01;  // id
	ackPacket[4] = 0x02;  // length
	ackPacket[5] = 0x00;
	ackPacket[6] = MSG[2];  // ACK class
	ackPacket[7] = MSG[3];  // ACK id
	ackPacket[8] = 0;   // CK_A
	ackPacket[9] = 0;   // CK_B

	//Calculate the checksums
	for(uint8_t i=2; i<8; i++)
	{
		ackPacket[8] = ackPacket[8] + ackPacket[i];
		ackPacket[9] = ackPacket[9] + ackPacket[8];
	}

	while(true)
	{

		// Test for success
		if(ackByteID > 9)
		{
			// All packets in order!
			Serial.println(F(" (SUCCESS!)"));
			return true;
		}

		// Timeout if no valid response in 3 seconds
		if(millis() - startTime > 3000)
		{
			Serial.println(F(" (FAILED!)"));
			while(gpsSerial.available())
			{
				gpsSerial.read();
			}
			return false;
		}

		// Make sure data is available to read
		if(gpsSerial.available())
		{
			b = gpsSerial.read();

			// Check that bytes arrive in sequence as per expected ACK packet
			if(b == ackPacket[ackByteID])
			{
				ackByteID++;
				Serial.print(b, HEX);
			}
			else
			{
				ackByteID = 0;  // Reset and look again, invalid order
			}
		}
	}
}

/*
 * Sets the RTC time from GPS datetime data and converts it to AEST.
 */
void setNewTimeRTC()
{
	uint8_t dateTime[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	uint8_t count = 0;
	char input;
	boolean complete = false;

	//Get GPS date/time data
	gpsSerial.println(F("$PUBX,04*37"));

	while(!gpsSerial.available())
	{}

	//Get raw GPS data
	while(!complete)
	{
		input = (char)gpsSerial.read();
		if(count < 16)
		{
			if(isdigit(input))
			{
				input -= '0';
				dateTime[count] = input;
				count++;
			}
		}
		if(input == '\n')
		{
			complete = true;
		}
	}
	//Adjust for year since 1970
	dateTime[14] += 3;

	//Convert data to a form that can be saved to tmElements_t struct
	for(int i = 0, j = 0; i < (sizeof dateTime / sizeof(uint8_t)); i = i + 2, j++)
	{
		if(dateTime[i] == 0)
		{
			dateTime[j] = dateTime[i + 1];
		}
		else
		{
			dateTime[j] = dateTime[i] * 10 + dateTime[i + 1];
		}
	}

	//Change time from UTC to AEST, checking for month, year and leap year
	if(dateTime[1] < 14)
	{
		dateTime[1] += 10;
	}
	else
	{
		dateTime[1] -= 14;
		if(dateTime[5] == 30 && (dateTime[6] == 4 || dateTime[6] == 6 || dateTime[6] == 9 || dateTime[6] == 11))
		{
			dateTime[5] = 1;
			dateTime[6]++;
		}
		else if(dateTime[5] == 31 && (dateTime[6] == 1 || dateTime[6] == 3 || dateTime[6] == 5 || dateTime[6] == 7 || dateTime[6] == 8 || dateTime[6] == 10))
		{
			dateTime[5] = 1;
			dateTime[6]++;
		}
		else if(dateTime[5] == 31 && dateTime[6] == 12)
		{
			dateTime[5] = 1;
			dateTime[6] = 1;
			dateTime[7]++;
		}
		else if(dateTime[5] == 28 && dateTime[6] == 2 && dateTime[7] % 4 != 0)
		{
			dateTime[5] = 1;
			dateTime[6]++;
		}
		else if(dateTime[5] == 29 && dateTime[6] == 2)
		{
			dateTime[5] = 1;
			dateTime[6]++;
		}
		else
			dateTime[5]++;
	}

	//Save the data to the RTC
	tmElements_t tm1;
	memcpy(&tm1.Hour, (dateTime + 1), 1);
	memcpy(&tm1.Minute, (dateTime + 2), 1);
	memcpy(&tm1.Second, (dateTime + 3), 1);
	memcpy(&tm1.Day, (dateTime + 5), 1);
	memcpy(&tm1.Month, (dateTime + 6), 1);
	memcpy(&tm1.Year, (dateTime + 7), 1);
	time_t t = makeTime(tm1);
	pRTC->setTimeRTC(t);

	delay(1000);
}

void sleepTimer(long duration)
{
	//Turn off as many power sources as possible
	Narcoleptic.disableTimer1();
	Narcoleptic.disableTimer2();
	Narcoleptic.disableSerial();
	Narcoleptic.disableADC();
	Narcoleptic.disableWire();
	Narcoleptic.disableSPI();

	//go to sleep for parameter passed time
	Narcoleptic.delay(duration);

	//Turn on the peripherals
	Narcoleptic.enableTimer1();
	Narcoleptic.enableTimer2();
	Narcoleptic.enableSerial();
	Narcoleptic.enableADC();
	Narcoleptic.enableWire();
	Narcoleptic.enableSPI();
}
/*
 * Gets the date and time as comma separated data and saves to file.
 *
 */
void getTime(File &data)
{
	uint8_t dt = pRTC->getDay();
	data.print(dt);
	data.print(F(","));

	dt = pRTC->getMonth();
	data.print(dt);
	data.print(F(","));

	dt = pRTC->getYear();
	data.print(dt);
	data.print(F(","));

	dt = pRTC->getHour();
	data.print(dt);
	data.print(F(","));

	dt = pRTC->getMinute();
	data.print(dt);
	data.print(F(","));

	dt = pRTC->getSecond();
	data.print(dt);
	data.print(F(","));
}

/*
 * Opens data file and calls getTime(). It then reads temperature, pressure,
 * altitude, from HP206c. If enough time has elapsed, then it gets GPS data
 */
void getHP20Data()
{
	if(OK_HP20X_DEV == ret)
	{
		File file = SD.open(F("data.txt"), FILE_WRITE);
		if(file)
		{
			getTime(file);
			long tmp = HP20x.ReadTemperature();
			float t = tmp / 100.0;
			Serial.print(t);
			file.print(t);
			Serial.print(F(","));
			file.print(F(","));

			tmp = HP20x.ReadPressure();
			t = tmp / 100.0;
			Serial.print(t);
			file.print(t);
			Serial.print(F(","));
			file.print(F(","));

			tmp = HP20x.ReadAltitude();
			t = tmp / 100.0;

			if(schdCount < GPSLOG / HP20LOG)
			{
				Serial.println(t);
				file.println(t);
			}
			else
			{
				Serial.print(t);
				file.print(t);
				Serial.print(F(","));
				file.print(F(","));
				getGPSData(file);
				schdCount = 0;
			}
			schdCount++;
			file.close();

//Blink to confirm recording altitude data (3 times)
digitalWrite(9, HIGH);
delay(150);
digitalWrite(9, LOW);
delay(150);
digitalWrite(9, HIGH);
delay(150);
digitalWrite(9, LOW);
delay(150);
digitalWrite(9, HIGH);
delay(150);
digitalWrite(9, LOW);

		}
		else
		{
			Serial.println(F("Error opening data.txt"));
		}
	}
	delay(200);
}

/*
 * Gets GPS data, cleaning it of extraneous junk
 */
void getGPSData(File &data)
{
	boolean stringComplete = false;
	gpsSerial.println(F("$PUBX,00*33"));
	while(!gpsSerial.available())
	{}
	while(!stringComplete)
	{
		char input = (char)gpsSerial.read();
		if(isalpha(input) || isdigit(input) || input == ',' || input == '.' || input == '$' || input == '*' || input == '\r' || input == '\n')
		{
			Serial.write(input);
			data.print(input);
		}

		if(input == '\n')
		{
			stringComplete = true;
		}
	}
	delay(200);
}
/*
 * Sets up UBX message for turning off GPS by calculating hex value of
 * SLEEPFOR and inserting it into setSleep array. Checksum is then
 * calculated and appended. UBX message is then sent to GPS.
 */
void GPSSleep()
{
	union long2Hex
	{
		unsigned long l;
		uint8_t hex[4];
	}conv;

	conv.l = SLEEPFOR;
	uint8_t setSleep[] = {0xB5, 0x62, 0x02, 0x41, 0x08, 0x00, conv.hex[0], conv.hex[1], conv.hex[2], conv.hex[3], 0x02, 0x00, 0x00, 0x00, 0x00, 0x00};
	calcChecksum(&setSleep[2], sizeof(setSleep) - 4);
	sendUBX(setSleep, sizeof(setSleep)/sizeof(uint8_t));
}

/*
 * Gets the hour of the day from the RTC and checks to see whether
 * this corresponds to defined TIMEOFSLEEP. It then turns the GPS off
 * and puts the anarduino to sleep. If it isn't the specified hour of
 * the day, then it simply exits.
 */
void sleepLong()
{
	uint8_t hrOfDay = pRTC->getHour();
	uint8_t minutes = pRTC->getMinute();

	if(hrOfDay == TIMEOFSLEEP && minutes == 0)
	{
		Serial.println(F("Entering sleep mode ... "));
		GPSSleep();
		delay(500);

		sleepTimer(SLEEPFOR);
		//Make sure that GPS has woken up before proceeding.
		delay(1000);

		//Use this delay, instead of the one above, if you want the GPS
		//to reobtain a fix before logging data.
		//delay(1800000);
		Serial.println();
		Serial.println(F("leaving sleep mode ... "));
	}
}
/*
 * Calculates the checksum for a UBX packet
 */
void calcChecksum(uint8_t *checksum, uint8_t sizeOf)
{
  uint8_t CK_A = 0, CK_B = 0;
  for (uint8_t i = 0; i < sizeOf ;i++)
  {
    CK_A = CK_A + *checksum;
    CK_B = CK_B + CK_A;
    checksum++;
  }
  *checksum = CK_A;
  checksum++;
  *checksum = CK_B;
}
