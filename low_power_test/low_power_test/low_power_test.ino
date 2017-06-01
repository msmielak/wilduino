

//Test sketch for to measure power consumption using Narcoleptic library

#include <Narcoleptic.h>
#include <RH_RF69.h>    
#include <SPI.h>    
#include <SPIFlash.h>    
#include <Wire.h>
#include <HP20x_dev.h>

//Apparently these are SDFat libraries, probably not all of them needed
#include <BlockDriver.h>
#include <FreeStack.h>
#include <MinimumSerial.h>
#include <SdFat.h>
#include <SdFatConfig.h>
#include <SysCall.h>


SdFat SD;
#define SD_CS_PIN 8
File myFile;
 
RH_RF69 rf69;    
      
SPIFlash flash(5, 0);  //Anarduino  

//Narcoleptic sleep time (30s)
#define SLEEPFOR 30000
uint8_t ret = 0;




void setup() {
Serial.begin(9600); 

//Initialising SD card, writing to a file and closing it to put card into sleep.
 Serial.print("Initializing SD card...");

  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  myFile = SD.open("test.txt", FILE_WRITE);

  // if the file opened okay, write to it:
  if (myFile) {
    Serial.print("Writing to test.txt...");
    myFile.println("testing 1, 2, 3.");
    // close the file:
    myFile.close();
    Serial.println("done.");
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }

  // re-open the file for reading:
  myFile = SD.open("test.txt");
  if (myFile) {
    Serial.println("test.txt:");

    // read from the file until there's nothing else in it:
    while (myFile.available()) {
      Serial.write(myFile.read());
    }
    // close the file:
    myFile.close();
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }



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


//Turning radio module and flash memory to sleep  
  pinMode(9, OUTPUT);
  rf69.init();    
  rf69.sleep();    
  flash.sleep();    

//Putting MPU9255 into sleep mode
 Wire.begin(9600);

 Wire.beginTransmission(0x68);
 Wire.write(0x6B);
 Wire.write(0x40);
 Wire.endTransmission();
 delay(200);

}


void loop() {
  // first blinking LED 3 times for 3s each

digitalWrite(9, HIGH);
delay(3000);
digitalWrite(9, LOW);
delay(3000);
digitalWrite(9, HIGH);
delay(3000);
digitalWrite(9, LOW);
delay(3000);
digitalWrite(9, HIGH);
delay(3000);
digitalWrite(9, LOW);
delay(3000);




//sleep

sleepTimer(SLEEPFOR);
  delay(500);


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

