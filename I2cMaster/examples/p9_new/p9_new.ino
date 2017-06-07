/**
   ReadSHT1xValues

   Read temperature and humidity values from an SHT1x-series (SHT10,
   SHT11, SHT15) sensor.

   Copyright 2009 Jonathan Oxer <jon@oxer.com.au>
   www.practicalarduino.com
*/

#include <SHT1x.h>
#include <I2cMaster.h>

// select software or hardware i2c
#define USE_SOFT_I2C 1


#define SDA_PIN 4
#define SCL_PIN 5

// An instance of class for software master
SoftI2cMaster rtc(SDA_PIN, SCL_PIN);
// Specify data and clock connections and instantiate SHT1x object
#define dataPin  4
#define clockPin 5
SHT1x sht1x(dataPin, clockPin);

void setup()
{
  Serial.begin(9600); // Open serial connection to report values to host
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  Serial.println("Starting up");
}
int dev = 0xB4;
void loop()
{
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  digitalWrite(4, HIGH);
  digitalWrite(5, HIGH);
  delay(100);
  int data_low = 0;
  int data_high = 0;
  int pec = 0;
  byte x[4] = {};

  readDS1307(0x07, x, 3);
  double tempFactor = 0.02; // 0.02 degrees per LSB (measurement resolution of the MLX90614)
  double tempData = 0x0000; // zero out the data
  int frac; // data past the decimal point
  tempData = (double)(((x[1] & 0x007F) << 8) + x[0]);
  tempData = (tempData * tempFactor) - 0.01;

  float celcius = tempData - 273.15;

  Serial.print("celcius: ");
  Serial.println(celcius);

  delay(2000);
  float temp_c;
  float temp_f;
  float humidity;

  // Read values from the sensor
  temp_c = sht1x.readTemperatureC();
  temp_f = sht1x.readTemperatureF();
  humidity = sht1x.readHumidity();

  // Print the values to the serial port
  Serial.print("Temperature: ");
  Serial.print(temp_c, DEC);
  Serial.print("C / ");
  Serial.print(temp_f, DEC);
  Serial.print("F. Humidity: ");
  Serial.print(humidity);
  Serial.println("%");
  delay(2000);

}
uint8_t readDS1307(uint8_t address, uint8_t *buf, uint8_t count) {
  // issue a start condition, send device address and write direction bit
  if (!rtc.start(dev | I2C_WRITE)) return false;

  // send the DS1307 address
  if (!rtc.write(address)) return false;

  // issue a repeated start condition, send device address and read direction bit
  if (!rtc.restart(dev | I2C_READ))return false;

  // read data from the DS1307
  for (uint8_t i = 0; i < count; i++) {

    // send Ack until last byte then send Ack
    buf[i] = rtc.read(i == (count - 1));
  }

  // issue a stop condition
  rtc.stop();
  return true;
}
