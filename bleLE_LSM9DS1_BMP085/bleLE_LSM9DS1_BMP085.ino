/*********************************************************************
 This is an example for our nRF51822 based Bluefruit LE modules

 Pick one up today in the adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "Adafruit_BLEBattery.h"

#include "BluefruitConfig.h"

#if SOFTWARE_SERIAL_AVAILABLE
  #include <SoftwareSerial.h>
#endif

/*=========================================================================
    APPLICATION SETTINGS

    FACTORYRESET_ENABLE       Perform a factory reset when running this sketch
   
                              Enabling this will put your Bluefruit LE module
                              in a 'known good' state and clear any config
                              data set in previous sketches or projects, so
                              running this at least once is a good idea.
   
                              When deploying your project, however, you will
                              want to disable factory reset by setting this
                              value to 0.  If you are making changes to your
                              Bluefruit LE device via AT commands, and those
                              changes aren't persisting across resets, this
                              is the reason why.  Factory reset will erase
                              the non-volatile memory where config data is
                              stored, setting it back to factory default
                              values.
       
                              Some sketches that require you to bond to a
                              central device (HID mouse, keyboard, etc.)
                              won't work at all with this feature enabled
                              since the factory reset will clear all of the
                              bonding data stored on the chip, meaning the
                              central device won't be able to reconnect.
    MINIMUM_FIRMWARE_VERSION  Minimum firmware version to have some new features
    MODE_LED_BEHAVIOUR        LED activity, valid options are
                              "DISABLE" or "MODE" or "BLEUART" or
                              "HWUART"  or "SPI"  or "MANUAL"
    -----------------------------------------------------------------------*/
    #define FACTORYRESET_ENABLE         1
    #define MINIMUM_FIRMWARE_VERSION    "0.6.6"
    #define MODE_LED_BEHAVIOUR          "MODE"
/*=========================================================================*/

// Create the bluefruit object, either software serial...uncomment these lines
/*
SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                      BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);
*/

/* ...or hardware serial, which does not need the RTS/CTS pins. Uncomment this line */
// Adafruit_BluefruitLE_UART ble(Serial1, BLUEFRUIT_UART_MODE_PIN);

/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

/* ...software SPI, using SCK/MOSI/MISO user-defined SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_SCK, BLUEFRUIT_SPI_MISO,
//                             BLUEFRUIT_SPI_MOSI, BLUEFRUIT_SPI_CS,
//                             BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);


// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}
//////////////////////////
// Battery Library Init //
//////////////////////////]
Adafruit_BLEBattery battery(ble);
#define VBATPIN A9
int Battery_update_intervals = 60000;


//////////////////////////
// LSM9DS1 Library Init //
//////////////////////////
// Use the LSM9DS1 class to create an object. [imu] can be
// named anything, we'll refer to that throught the sketch.
#include <SparkFunLSM9DS1.h>

#define PRINT_CALCULATED
LSM9DS1 imu;

// Earth's magnetic field varies by location. Add or subtract
// a declination to get a more accurate heading. Calculate
// your's here:
// http://www.ngdc.noaa.gov/geomag-web/#declination
#define DECLINATION -8.58 // Declination (degrees) in Boulder, CO.

//Function definitions
void printGyro();
void printAccel();
void printMag();
void printAttitude(float ax, float ay, float az, float mx, float my, float mz);

//////////////////////////
// BMP085  Library Init //
//////////////////////////
#include <Adafruit_BMP085.h>

Adafruit_BMP085 bmp;

void printTemp();
void printPrs();

////////////////////////////
// 　　　 計測時間測定 　　　 //
////////////////////////////
static unsigned long lastPrint = 0; // Keep track of print time
static unsigned long now = 0; // Keep track of print time
static unsigned long lastBattery_update_Print = 0;


void setup(void)
{

  Serial.begin(115200);
  Serial.println(F("Select mode"));
  Serial.println(F("---------------------------------------"));


 //以下bluefruit_LE用
/*=========================================================================*/
  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  
  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ) {
      error(F("Couldn't factory reset"));
    }
  }
  /* Disable command echo from Bluefruit */
  ble.echo(false);
  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
  }

  // Set module to DATA mode
  Serial.println( F("Switching to DATA mode!") );
  ble.setMode(BLUEFRUIT_MODE_DATA);

  Serial.println(F("******************************"));
/*=========================================================================*/

//  while (! select_mode()) {
//      delay(500);
//      Serial.println("select_mode");
//  }
  
  /*=========================================================================*/
  //Battery用設定
  /*=========================================================================*/
  battery.begin(true);
  
  /*=========================================================================*/
  //BMP085用設定
  /*=========================================================================*/
  bmp.begin();

  /*=========================================================================*/
  //LSM9DS1用設定
  /*=========================================================================*/
  Wire.begin();

  if (imu.begin() == false) // with no arguments, this uses default addresses (AG:0x6B, M:0x1E) and i2c port (Wire).
  {
    ble.println("Failed to communicate with LSM9DS1.");
    ble.println("Double-check wiring.");
    ble.println("Default settings in this sketch will " \
                   "work for an out of the box LSM9DS1 " \
                   "Breakout, but may need to be modified " \
                   "if the board jumpers are.");
    while (1);
  }
  //倍率の設定
  imu.setAccelScale(16);
  /*=========================================================================*/
}

uint8_t CASE = 0;

void loop(void)
{
  now = millis();

  if(now - lastBattery_update_Print > Battery_update_intervals )
  {
    float measuredvbat = analogRead(VBATPIN);
    measuredvbat *= 2;    // we divided by 2, so multiply back
    measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
    measuredvbat /= 1024; // convert to voltage
    battery.update(measuredvbat/4.2*100);
    lastBattery_update_Print = now;
  }

  if(ble.available() > 0)
  {
    select_mode();
  }
  switch(CASE) {
    case 0:
    ble.print("select_mode");
    delay(500);
    break;
    case 1:
    imu.readGyro();
    printGyro();
    break;
    case 2:
    imu.readAccel();
    printAccel();
    break;
    case 3:
    imu.readMag();
    printMag();
    break;
    case 4:
    printTemp();
    break;
    case 5:
    printPrs();
    break;
    default:
    break;
  }

    ble.print(",");
    ble.print((float)(now - lastPrint)/1000);
    ble.println();

    lastPrint = now; // Update lastPrint time
  
}


bool select_mode()
{
  char buf[10];
  uint8_t I_CASE = 0;
  ble.readline(buf,10,100,false);
  I_CASE = atoi(buf);
  Serial.println(buf);
  Serial.println(I_CASE);
  switch(I_CASE) {
    case 1:
    CASE = 1;
    break;
    case 2:
    CASE = 2;
    break;
    case 3:
    CASE = 3;
    break;
    case 4:
    CASE = 4;
    break;
    case 5:
    CASE = 5;
    break;
    default:
    return false;
  }
    return true;
}

void printGyro()
{
  // Now we can use the gx, gy, and gz variables as we please.
  // Either print them as raw ADC values, or calculated in DPS.
  ble.print("G: ");
#ifdef PRINT_CALCULATED
  // If you want to print calculated values, you can use the
  // calcGyro helper function to convert a raw ADC value to
  // DPS. Give the function the value that you want to convert.
  ble.print(imu.calcGyro(imu.gx), 6);
  ble.print(", ");
  ble.print(imu.calcGyro(imu.gy), 6);
  ble.print(", ");
  ble.print(imu.calcGyro(imu.gz), 6);
  ble.print(" deg/s");
#elif defined PRINT_RAW
  ble.print(imu.gx);
  ble.print(", ");
  ble.print(imu.gy);
  ble.print(", ");
  ble.print(imu.gz);
#endif
}

void printAccel()
{
  // Now we can use the ax, ay, and az variables as we please.
  // Either print them as raw ADC values, or calculated in g's.
  //ble.print("A: ");
#ifdef PRINT_CALCULATED
  // If you want to print calculated values, you can use the
  // calcAccel helper function to convert a raw ADC value to
  // g's. Give the function the value that you want to convert.
  ble.print(imu.calcAccel(imu.ax), 6);
  ble.print(",");
  ble.print(imu.calcAccel(imu.ay), 6);
  ble.print(",");
  ble.print(imu.calcAccel(imu.az), 6);
  ble.print(" g");
#elif defined PRINT_RAW
  ble.print(imu.ax);
  ble.print(" ");
  ble.print(imu.ay);
  ble.print(" ");
  ble.print(imu.az);
#endif

}

void printMag()
{
  // Now we can use the mx, my, and mz variables as we please.
  // Either print them as raw ADC values, or calculated in Gauss.
  ble.print("M: ");
#ifdef PRINT_CALCULATED
  // If you want to print calculated values, you can use the
  // calcMag helper function to convert a raw ADC value to
  // Gauss. Give the function the value that you want to convert.
  ble.print(imu.calcMag(imu.mx), 6);
  ble.print(", ");
  ble.print(imu.calcMag(imu.my), 6);
  ble.print(", ");
  ble.print(imu.calcMag(imu.mz), 6);
  ble.print(" gauss");
#elif defined PRINT_RAW
  ble.print(imu.mx);
  ble.print(", ");
  ble.print(imu.my);
  ble.print(", ");
  ble.println(imu.mz);
#endif
}

// Calculate pitch, roll, and heading.
// Pitch/roll calculations take from this app note:
// https://web.archive.org/web/20190824101042/http://cache.freescale.com/files/sensors/doc/app_note/AN3461.pdf
// Heading calculations taken from this app note:
// https://web.archive.org/web/20150513214706/http://www51.honeywell.com/aero/common/documents/myaerospacecatalog-documents/Defense_Brochures-documents/Magnetic__Literature_Application_notes-documents/AN203_Compass_Heading_Using_Magnetometers.pdf
void printAttitude(float ax, float ay, float az, float mx, float my, float mz)
{
  float roll = atan2(ay, az);
  float pitch = atan2(-ax, sqrt(ay * ay + az * az));

  float heading;
  if (my == 0)
    heading = (mx < 0) ? PI : 0;
  else
    heading = atan2(mx, my);

  heading -= DECLINATION * PI / 180;

  if (heading > PI) heading -= (2 * PI);
  else if (heading < -PI) heading += (2 * PI);

  // Convert everything from radians to degrees:
  heading *= 180.0 / PI;
  pitch *= 180.0 / PI;
  roll  *= 180.0 / PI;

  ble.print("Pitch, Roll: ");
  ble.print(pitch, 2);
  ble.print(", ");
  ble.println(roll, 2);
  ble.print("Heading: "); 
  ble.print(heading, 2);
}

void printTemp()
{
  ble.print(bmp.readTemperature());
  Serial.println(bmp.readTemperature());
  ble.print(" *C");
}

void printPrs()
{
  ble.print(double(bmp.readPressure()) / 100); //気圧(hpa)の表示
  Serial.println(double(bmp.readPressure()) / 100);
  ble.print(" hPa");
}
