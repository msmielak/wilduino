/*
 * 01_flash_probe.ino  --  Wilduino bench test 1
 *
 * PURPOSE
 *   Determine which Arduino pin drives the chip-select of the S25FL127S
 *   SPI NOR flash on the Anarduino MiniWireless, and confirm it does not
 *   collide with the microSD chip-select (D8) used by the Wilduino shield.
 *
 *   This is a blocking question for the storage architecture: if the flash
 *   CS and the SD CS are the same pin, the two devices cannot coexist and
 *   the flash-buffered logging design has to be abandoned.
 *
 * METHOD
 *   Walk a candidate CS pin across the pins left free by the shield,
 *   asserting each in turn and issuing the JEDEC RDID opcode (0x9F).
 *   A responding SPI NOR flash returns three bytes:
 *
 *       manufacturer ID, memory type, capacity
 *
 *   For the Spansion/Cypress S25FL127S the expected reply is:
 *
 *       0x01 0x20 0x18      (Spansion, FL-S family, 128 Mbit / 16 MB)
 *
 *   Any other three-byte pattern that is neither all-0x00 nor all-0xFF is
 *   still interesting -- report it. All-0x00 or all-0xFF means nothing is
 *   listening on that pin.
 *
 * SAFETY
 *   All candidate CS pins, the radio CS (D10) and the SD CS (D8) are driven
 *   HIGH before probing so that only one device is ever selected. RDID is a
 *   read-only opcode, so an accidental assertion of the radio CS is harmless.
 *
 * WIRING
 *   None. Board only. Serial monitor at 9600 baud.
 *
 * EXPECTED RUNTIME
 *   A couple of seconds. Output is printed once, then the sketch idles.
 *
 * Wilduino -- GPL-3.0
 */

#include <SPI.h>

/* ------------------------------------------------------------------ */
/* Pin map                                                             */
/*                                                                     */
/* Occupied by the Wilduino shield (from the KiCad netlist):           */
/*   A0  V_SENSE battery divider                                       */
/*   A1  MCP7940 MFP / alarm                                           */
/*   A2  GPS TX  (or D0 via jumper J11)                                */
/*   A3  GPS RX  (or D1 via jumper J12)                                */
/*   A4  SDA     A5  SCL                                               */
/*   D2  HP206C INT (J13) / TSL2591 INT (J14) / radio DIO0             */
/*   D3  MPU-9255 INT                                                  */
/*   D8  microSD CS                                                    */
/*   D11 MOSI   D12 MISO   D13 SCK                                     */
/*                                                                     */
/* Left free, therefore candidates for the onboard flash CS:           */
/*   D4, D5, D6, D7, D9                                                */
/*                                                                     */
/* D8 is probed as well, deliberately: if the flash answers on D8 then */
/* we have found the collision we are looking for.                     */
/* ------------------------------------------------------------------ */

static const uint8_t CANDIDATE_CS[] = { 4, 5, 6, 7, 9, 8 };
static const uint8_t N_CANDIDATES   = sizeof(CANDIDATE_CS) / sizeof(CANDIDATE_CS[0]);

static const uint8_t RADIO_CS = 10;   /* MiniWireless RFM69/RFM9x select */
static const uint8_t SD_CS    = 8;    /* Wilduino shield microSD select  */

static const uint8_t CMD_RDID = 0x9F; /* JEDEC read identification       */

/* Expected reply from the S25FL127S */
static const uint8_t S25FL127S_ID[3] = { 0x01, 0x20, 0x18 };


/* Drive every chip-select we know about HIGH (inactive). */
static void deselectAll()
{
  pinMode(RADIO_CS, OUTPUT);
  digitalWrite(RADIO_CS, HIGH);

  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH);

  for (uint8_t i = 0; i < N_CANDIDATES; i++) {
    pinMode(CANDIDATE_CS[i], OUTPUT);
    digitalWrite(CANDIDATE_CS[i], HIGH);
  }
}


/* Issue RDID on the given CS pin and store the three reply bytes. */
static void readJedecId(uint8_t csPin, uint8_t *id)
{
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
  digitalWrite(csPin, LOW);

  SPI.transfer(CMD_RDID);
  id[0] = SPI.transfer(0x00);
  id[1] = SPI.transfer(0x00);
  id[2] = SPI.transfer(0x00);

  digitalWrite(csPin, HIGH);
  SPI.endTransaction();
}


/* A reply of all-0x00 or all-0xFF means the bus was floating: no device. */
static bool isPlausibleId(const uint8_t *id)
{
  bool allZero = (id[0] == 0x00 && id[1] == 0x00 && id[2] == 0x00);
  bool allOnes = (id[0] == 0xFF && id[1] == 0xFF && id[2] == 0xFF);
  return !(allZero || allOnes);
}


static void printHexByte(uint8_t b)
{
  Serial.print(F("0x"));
  if (b < 0x10) Serial.print('0');
  Serial.print(b, HEX);
}


void setup()
{
  Serial.begin(9600);
  while (!Serial) { ; }

  Serial.println();
  Serial.println(F("=== Wilduino flash CS probe ==="));
  Serial.println(F("Looking for S25FL127S: expect 0x01 0x20 0x18"));
  Serial.println();

  deselectAll();
  SPI.begin();
  delay(10);   /* let the flash finish its own power-up */

  uint8_t found = 0;

  for (uint8_t i = 0; i < N_CANDIDATES; i++) {
    uint8_t cs = CANDIDATE_CS[i];
    uint8_t id[3];

    readJedecId(cs, id);

    Serial.print(F("D"));
    Serial.print(cs);
    if (cs == SD_CS) Serial.print(F(" (shield SD CS!)"));
    Serial.print(F("  ->  "));
    printHexByte(id[0]); Serial.print(' ');
    printHexByte(id[1]); Serial.print(' ');
    printHexByte(id[2]);

    if (!isPlausibleId(id)) {
      Serial.println(F("   -- nothing here"));
      continue;
    }

    found++;

    if (id[0] == S25FL127S_ID[0] &&
        id[1] == S25FL127S_ID[1] &&
        id[2] == S25FL127S_ID[2]) {
      Serial.println(F("   ** S25FL127S, 16 MB **"));
      if (cs == SD_CS) {
        Serial.println(F("      !! COLLISION with shield microSD CS !!"));
      }
    } else {
      Serial.println(F("   ?? something responded, ID not recognised"));
    }
  }

  Serial.println();
  if (found == 0) {
    Serial.println(F("No SPI device answered RDID on any candidate pin."));
    Serial.println(F("Check: board powered? SPI pins correct? flash fitted?"));
  } else {
    Serial.print(F("Devices responding: "));
    Serial.println(found);
  }
  Serial.println(F("=== done ==="));
}


void loop()
{
  /* nothing -- results are printed once in setup() */
}
