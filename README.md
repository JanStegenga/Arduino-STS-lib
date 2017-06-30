
# arduino-sts
Repository for Sensirion temperature sensor support on Arduino

Adapted from arduino-sht library  
Jan Stegenga,  
EVAbits,  
26-6-2017  

## Supported sensors:
- STS3x-DIS (I2C)

## Installation

Download arduino-sts either via git and place it in your Arduino/libraries directory. 
After restarting the Arduino IDE, you will see the new STSSensor menu items under libraries and examples.

## Integrating it into your sketch

Assuming you installed the library as described above, the following steps are
necessary:

1. Import the Wire library like this: From the menu bar, select Sketch > Import
   Library > Wire
1. Import the arduino-sht library: From the menu bar, select Sketch >
   Import Library > arduino-sts
1. Create an instance of the `STSSensor` class (`STSSensor sts;`)
2. In `setup()`, make sure to init the Wire library with `Wire.begin()`
3. If you want to use the serial console, remember to initialize the Serial
   library with `Serial.begin(9600)`
1. Call `sts.readSample()` in the `loop()` function, which reads a temperature sample from the sensor
2. Use `sts.getTemperature()` to get the values from the last sample

*Important:* `getTemperature()` do *not* read a new sample from the sensor, but return the values read last. To read a new sample, make
sure to call `readSample()`

### Sample code
```c++
#include <Wire.h>

#include <STSSensor.h>

SHTSensor sts;

void setup() {
  // put your setup code here, to run once:

  Wire.begin();
  Serial.begin(9600);
  sta.init();
}

void loop() {
  // put your main code here, to run repeatedly:

  sht.readSample();
  Serial.print("STS:\n");
  Serial.print("  T:  ");
  Serial.print(sts.getTemperature(), 2);
  Serial.print("\n");

  delay(1000);
}
```
