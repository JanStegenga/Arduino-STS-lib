/*
 * V Example to read temperature from STS-30: https://create.arduino.cc/projecthub/Dcube/temperature-measurement-using-sts21-and-arduino-nano-a5f264
 * V Example to read temperature and humidity from SHT-30: https://developer.sensirion.com/platforms/arduino/
 * 
 * X Expand with adding more I2C lines to the Arduino by assigning pins: https://www.arduino.cc/en/Tutorial/SamdSercom
 * 
 */

#include <Wire.h>
#include <SHTSensor.h>
#include <STSSensor.h>
#include "wiring_private.h"           //pinPeripheral

//globals and constants:
#define STSaddr 0x4A                  //0x4A (ADDR_pin=LOW), 0x4B(ADDR_pin=HIGH)
#define SHTaddr 0x44                  //0x44 (ADDR_pin=LOW), 0x45(ADDR_pin=HIGH)
byte val = 0;                         //from example
SHTSensor sht;                        //To use a specific sensor instead of probing the bus use this command: SHTSensor sht(SHTSensor::SHT3X);
bool USE_SHT = false;
bool USE_STS  = true;
bool USE_STS2 = true;
TwoWire MyWire(&sercom2, 2, 3);     // Create the new wire instance assigning it to pin 2 and 3
                                    // Wire class instance is : TwoWire Wire()
STSSensor sts( Wire );            // adapted version of SHT library
STSSensor sts2( MyWire );           // REQUIRES TWO 10K RESISTORS TO 3V3


// Attach the interrupt handler to the SERCOM
/*
extern "C" {
  void SERCOM2_Handler(void);

  void SERCOM2_Handler(void) {
    MyWire.onService();
  }
}
*/

//-------------------------------------------------------------------------------
void setup()
{

 
  Wire.begin();                 // join i2c bus (address optional for master)
  delay(1000);
  SerialUSB.begin(115200);      // init serial port
  MyWire.begin();               // join i2c bus
  delay(1000);                  // give user time to open a terminal
  pinPeripheral(2, PIO_SERCOM_ALT); // Assign SERCOM function to pin 2
  pinPeripheral(3, PIO_SERCOM_ALT); // Assign SERCOM function to pin 3
  delay(1000);
  SerialUSB.print("\n\n------------------readSTS---------------------\n\n");
  /*
  if (sht.init() && USE_SHT) {
      SerialUSB.print("init(): SHT success\n");
      //sht.setAccuracy(SHTSensor::SHT_ACCURACY_HIGH); // only supported by SHT3x
  } else {
      SerialUSB.print("init(): SHT failed\n");
  }
  */

  if (USE_STS) {
    if( sts.init(Wire)  ) {
      SerialUSB.print("init(): STS success\n");
      //sts.setAccuracy(STSSensor::STS_ACCURACY_HIGH); // only supported by SHT3x
    } else {
      SerialUSB.print("init(): STS failed\n");
    }
  } else {
      SerialUSB.print("init(): STS failed\n");
  }
  
  if ( USE_STS2 ) {
    SerialUSB.print("init(): STS2 init attempt\n");
    //SerialUSB.print( sts2.mSensor );
    if ( sts2.init(MyWire) ) {
      SerialUSB.print("init(): STS2 success\n");
      //sts.setAccuracy(STSSensor::STS_ACCURACY_HIGH); // only supported by SHT3x
    } else {
      SerialUSB.print("init(): STS2 failed\n");
    }
  } else {
      SerialUSB.print("init(): STS2 failed\n");
  }
  
  
}


//-------------------------------------------------------------------------------
void loop()
{
  if (USE_SHT)
    {
    sht.readSample();
    SerialUSB.print("SHT:\n");
    SerialUSB.print("  RH: ");
    SerialUSB.print(sht.getHumidity(), 2);
    SerialUSB.print("\n");
    SerialUSB.print("  T:  ");
    SerialUSB.print(sht.getTemperature(), 2);
    SerialUSB.print("\n");
    }  
  if (USE_STS)
    {
    sts.readSample();
    SerialUSB.print("STS:\n");
    SerialUSB.print("  T:  ");
    SerialUSB.print(sts.getTemperature(), 2);
    SerialUSB.print("\n");
    }
   
  if (USE_STS2)
    {
    sts2.readSample();
    SerialUSB.print("STS2:\n");
    SerialUSB.print("  T:  ");
    SerialUSB.print(sts2.getTemperature(), 2);
    SerialUSB.print("\n");
    }
  
  SerialUSB.print("\n--------\n");
  delay(5000);
}


/*
 * 
  //test the crc8
  //byte testcode[] = {0xBEEF};
  //byte crc = CRC8( testcode, 1 );
  //SerialUSB.print( crc, HEX );
 * 
 * 
 * 
 * 
  unsigned int data[2];
  int rawtmp, value;
  double cTemp, fTemp;
  byte crc;
  Wire.beginTransmission(addr);   //S
  //Wire.write( 0x4A << 1 + 0x00 ); //start: 0x7F & ADDR or 0x4A << 1 + 0x00
  Wire.write( 0x24 );             //clock stretching disabled
  Wire.write( 0x00 );             //high repeatability
  Wire.write( crc );                  //crc
  Wire.endTransmission();         //P
  delay(1);

  Wire.requestFrom( addr, 3 );   //read: 0x7F & ADDR or 0x4A << 1 + 0x00
  if (Wire.available() == 3)  
  {    
    data[0] = Wire.read();    
    data[1] = Wire.read();  
    crc     = Wire.read();
  }
 */






/*
  // Start I2C Transmission  
  Wire.beginTransmission(addr);  
  // Select no hold master  
  Wire.write(0xF3);  
  // End I2C Transmission  
  Wire.endTransmission();  
  delay(300);
  // Request 2 bytes of data  
  Wire.requestFrom(addr, 2);
  // Read 2 bytes of data  
  if (Wire.available() == 2)  
  {    
    data[0] = Wire.read();    
    data[1] = Wire.read();  
  }
  // Convert the data  
  rawtmp = data[0] * 256 + data[1];  
  value = rawtmp & 0xFFFC;  
  cTemp = -46.85 + (175.72 * (value / 65536.0));  
  fTemp = cTemp * 1.8 + 32;
  // Output data to serial monitor  
  SerialUSB.print("Temperature in Celsius:  ");  
  SerialUSB.print(cTemp);  
  SerialUSB.println(" C");  
  SerialUSB.print("Temperature in Fahrenheit:  ");  
  SerialUSB.print(fTemp);  
  SerialUSB.println(" F");  
  delay(300)
  */
