/*
 * 02_tsl2591_autorange.ino  --  Wilduino bench test 2
 *
 * PURPOSE
 *   Read the TSL2591 in a form suitable for RELATIVE low-light photometry
 *   (moonlight), rather than the absolute-lux form the vendor libraries
 *   provide. Written as a direct register driver, with no external library,
 *   so that flash and RAM cost is known and the behaviour is explicit.
 *
 * DESIGN DECISIONS, AND WHY
 *
 *   1. Raw channels only, never computed lux.
 *      The lux equations in the common libraries are empirical fits derived
 *      for indoor illumination and are not trustworthy at moonlight levels.
 *      We log CH0 (full spectrum) and CH1 (infrared) as raw ADC counts, plus
 *      the gain and integration setting in force. Conversion happens in R,
 *      where it can be revisited without reflashing anything.
 *      The CH0/CH1 ratio also carries spectral information that computing
 *      lux on-device would discard -- moonlight is spectrally distinct from
 *      twilight, and that may be worth having later.
 *
 *   2. Auto-ranging, with the range recorded in every sample.
 *      Dusk to a moonless night spans five or six orders of magnitude. No
 *      single gain/integration setting covers it. A reading is meaningless
 *      without knowing the range it was taken in.
 *
 *   3. Bracketed readings at every range change.
 *      This is the important one for relative work. The nominal gain steps
 *      (1x / 25x / 428x / 9876x) are typical values; real parts deviate by
 *      several percent. Uncorrected, every range change injects a step into
 *      the time series -- and range changes correlate with moonrise and
 *      cloud, i.e. with the signal of interest. So whenever the range
 *      changes we take one extra reading in the OLD range as well, and emit
 *      both. Post hoc in R those bracketed pairs give the empirical gain
 *      ratio for this individual sensor.
 *
 *   4. Dark reference is NOT taken here.
 *      It cannot be, without a shutter. Characterise dark current against
 *      temperature on the bench in a dark box before deployment, and correct
 *      using the HP206C temperature channel. At 0.001 lux the dark offset is
 *      comparable to the signal, so this step is not optional.
 *
 * OUTPUT
 *   CSV to serial, one line per reading:
 *     millis,gain_code,atime_code,ch0,ch1,status
 *   where status is 'O' (ok), 'S' (saturated), 'U' (underflow),
 *   'B' (bracket reading, taken in the previous range for ratio recovery).
 *
 * WIRING
 *   TSL2591 on I2C via shield header P2: SDA = A4, SCL = A5, 3V3, GND.
 *   The TSL2591 interrupt line (P2 pin 8) is deliberately NOT used -- on
 *   this shield it lands on D2, which is shared with the HP206C interrupt
 *   (J14/J13) and the radio DIO0. We poll with a delay instead.
 *
 * NOTE
 *   Register addresses and gain codes below are from the AMS TSL2591
 *   datasheet. Worth verifying against your copy before trusting the
 *   numbers -- a transposed register here is a silent wrong answer.
 *
 * Wilduino -- GPL-3.0
 */

#include <Wire.h>

/* ---------------- TSL2591 register map ---------------- */

static const uint8_t TSL_ADDR       = 0x29;

static const uint8_t TSL_CMD_NORMAL = 0xA0;  /* command bit | normal op    */

static const uint8_t REG_ENABLE     = 0x00;
static const uint8_t REG_CONFIG     = 0x01;
static const uint8_t REG_ID         = 0x12;
static const uint8_t REG_STATUS     = 0x13;
static const uint8_t REG_C0DATAL    = 0x14;  /* CH0 low; reads auto-increment */

static const uint8_t ENABLE_POWERON = 0x01;
static const uint8_t ENABLE_AEN     = 0x02;  /* ALS enable */
static const uint8_t ENABLE_POWEROFF= 0x00;

static const uint8_t TSL_DEVICE_ID  = 0x50;

/* Gain codes, as written into CONFIG bits 5:4 */
static const uint8_t GAIN_LOW  = 0x00;  /* nominal    1x */
static const uint8_t GAIN_MED  = 0x10;  /* nominal   25x */
static const uint8_t GAIN_HIGH = 0x20;  /* nominal  428x */
static const uint8_t GAIN_MAX  = 0x30;  /* nominal 9876x */

static const uint8_t GAIN_STEPS[4] = { GAIN_LOW, GAIN_MED, GAIN_HIGH, GAIN_MAX };
static const uint8_t N_GAIN = 4;

/* Integration time codes, CONFIG bits 2:0. 0 = 100 ms ... 5 = 600 ms */
static const uint8_t ATIME_100MS = 0x00;
static const uint8_t ATIME_600MS = 0x05;

/* Auto-ranging thresholds, as a fraction of the 16-bit full scale.
 * Hysteresis is deliberately wide to stop the range hunting back and
 * forth around a threshold, which would litter the record with brackets. */
static const uint16_t COUNT_SATURATED = 60000;  /* step gain DOWN above this */
static const uint16_t COUNT_TOO_LOW   =   500;  /* step gain UP   below this */

/* Current range state */
static uint8_t gainIndex = 3;                 /* start at max gain (night) */
static uint8_t atimeCode = ATIME_600MS;


/* ---------------- low level I2C ---------------- */

static void tslWrite8(uint8_t reg, uint8_t value)
{
  Wire.beginTransmission(TSL_ADDR);
  Wire.write(TSL_CMD_NORMAL | reg);
  Wire.write(value);
  Wire.endTransmission();
}

static uint8_t tslRead8(uint8_t reg)
{
  Wire.beginTransmission(TSL_ADDR);
  Wire.write(TSL_CMD_NORMAL | reg);
  Wire.endTransmission();

  Wire.requestFrom(TSL_ADDR, (uint8_t)1);
  if (Wire.available()) return Wire.read();
  return 0;
}

/* Read CH0 and CH1 together. The data registers auto-increment, so a
 * single burst read keeps both channels from the same integration cycle --
 * important, because separate reads could straddle a conversion boundary. */
static bool tslReadChannels(uint16_t *ch0, uint16_t *ch1)
{
  Wire.beginTransmission(TSL_ADDR);
  Wire.write(TSL_CMD_NORMAL | REG_C0DATAL);
  Wire.endTransmission();

  Wire.requestFrom(TSL_ADDR, (uint8_t)4);
  if (Wire.available() < 4) return false;

  uint8_t c0l = Wire.read();
  uint8_t c0h = Wire.read();
  uint8_t c1l = Wire.read();
  uint8_t c1h = Wire.read();

  *ch0 = ((uint16_t)c0h << 8) | c0l;
  *ch1 = ((uint16_t)c1h << 8) | c1l;
  return true;
}


/* ---------------- device control ---------------- */

static void tslApplyRange()
{
  tslWrite8(REG_CONFIG, GAIN_STEPS[gainIndex] | atimeCode);
}

static void tslEnable()
{
  tslWrite8(REG_ENABLE, ENABLE_POWERON | ENABLE_AEN);
}

static void tslDisable()
{
  tslWrite8(REG_ENABLE, ENABLE_POWEROFF);
}

/* Integration time in ms for the current atime code. */
static uint16_t integrationMs()
{
  return (uint16_t)(atimeCode + 1) * 100;
}

/* Power up, integrate, read, power down.
 *
 * Powering down between samples matters: the TSL2591 draws a few hundred
 * microamps while integrating and a few microamps in standby. On the
 * stationary reference station that is irrelevant, but the same routine
 * will be reused on anything battery powered, so do it properly here.
 *
 * We wait integration time + 20% margin. The datasheet AVALID bit in
 * STATUS could be polled instead; the fixed wait is simpler and the
 * margin covers oscillator tolerance. */
static bool tslSample(uint16_t *ch0, uint16_t *ch1)
{
  tslApplyRange();
  tslEnable();
  delay(integrationMs() + integrationMs() / 5 + 10);
  bool ok = tslReadChannels(ch0, ch1);
  tslDisable();
  return ok;
}


/* ---------------- output ---------------- */

static void emit(uint16_t ch0, uint16_t ch1, char status)
{
  Serial.print(millis());        Serial.print(',');
  Serial.print(gainIndex);       Serial.print(',');
  Serial.print(atimeCode);       Serial.print(',');
  Serial.print(ch0);             Serial.print(',');
  Serial.print(ch1);             Serial.print(',');
  Serial.println(status);
}


void setup()
{
  Serial.begin(9600);
  while (!Serial) { ; }

  Wire.begin();

  Serial.println();
  Serial.println(F("=== Wilduino TSL2591 autorange ==="));

  uint8_t id = tslRead8(REG_ID);
  if (id != TSL_DEVICE_ID) {
    Serial.print(F("TSL2591 not found. ID read back: 0x"));
    Serial.println(id, HEX);
    Serial.println(F("Check P2 wiring (SDA=A4, SCL=A5) and 3V3."));
    while (1) { ; }
  }
  Serial.println(F("TSL2591 present."));
  Serial.println(F("millis,gain_code,atime_code,ch0,ch1,status"));

  tslDisable();
}


void loop()
{
  uint16_t ch0, ch1;

  if (!tslSample(&ch0, &ch1)) {
    Serial.println(F("# I2C read failed"));
    delay(1000);
    return;
  }

  /* Decide whether the range needs to move. CH0 is the fuller channel,
   * so range on CH0 and let CH1 follow. */
  int8_t newIndex = gainIndex;
  char   status   = 'O';

  if (ch0 > COUNT_SATURATED && gainIndex > 0) {
    newIndex = gainIndex - 1;
    status   = 'S';
  } else if (ch0 < COUNT_TOO_LOW && gainIndex < (N_GAIN - 1)) {
    newIndex = gainIndex + 1;
    status   = 'U';
  }

  emit(ch0, ch1, status);

  if (newIndex != gainIndex) {
    /* Bracket: re-read immediately in the NEW range, so that this sample
     * and the next form a pair taken seconds apart under near-identical
     * illumination. In R, the ratio of the two recovers this device's
     * true gain step, which is what makes the series continuous across
     * the discontinuity. */
    uint8_t oldIndex = gainIndex;
    gainIndex = (uint8_t)newIndex;

    if (tslSample(&ch0, &ch1)) {
      emit(ch0, ch1, 'B');
    }

    Serial.print(F("# range "));
    Serial.print(oldIndex);
    Serial.print(F(" -> "));
    Serial.println(gainIndex);
  }

  /* Bench cadence. In deployment this becomes an RTC-alarm wake, and the
   * schedule comes from the SD config file (night windows precomputed in R
   * with suncalc / moonlit). */
  delay(2000);
}
