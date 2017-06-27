/*
 * Example to read temperature from STS-30
 * Jan Stegenga, EVAbits, 2017
 */

#include <Wire.h>
#include <STSSensor.h>

//globals and constants:
#define STSaddr 0x4A                  //0x4A (ADDR_pin=LOW), 0x4B(ADDR_pin=HIGH)
STSSensor sts;                       

//-------------------------------------------------------------------------------
void setup()
{
  Wire.begin();                 // join i2c bus (address optional for master)
  SerialUSB.begin(115200);      // init serial port
  delay(3000);                  // give user time to open a terminal

  SerialUSB.print("\n\n------------------readSTS---------------------\n\n");
  if (sts.init()) {
      SerialUSB.print("init(): STS init success\n");
      //sts.setAccuracy(STSSensor::STS_ACCURACY_HIGH); // only supported by STS3x
  } else {
      SerialUSB.print("init(): STS init failed\n");
  }
  
}


//-------------------------------------------------------------------------------
void loop()
{
  sts.readSample();
  SerialUSB.print("STS:\n");
  SerialUSB.print("  T:  ");
  SerialUSB.print(sts.getTemperature(), 2);
  SerialUSB.print("\n");
  delay(3000);
  SerialUSB.print("\n--------\n");
}
