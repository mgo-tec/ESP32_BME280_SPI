#include "ESP32_BME280_SPI.h"

const uint8_t SCLK_bme280 = 14;
const uint8_t MOSI_bme280 =13; //Master Output Slave Input ESP8266=Master,BME280=slave 
const uint8_t MISO_bme280 =12; //Master Input Slave Output
const uint8_t CS_bme280 = 26; //CS pin

ESP32_BME280_SPI bme280spi(SCLK_bme280, MOSI_bme280, MISO_bme280, CS_bme280, 10000000);

void setup(){
  Serial.begin(115200);
  delay(1000); //Take some time to open up the Serial Monitor

  uint8_t t_sb = 5; //stanby 1000ms
  uint8_t filter = 0; //filter O = off
  uint8_t osrs_t = 4; //OverSampling Temperature x4
  uint8_t osrs_p = 4; //OverSampling Pressure x4
  uint8_t osrs_h = 4; //OverSampling Humidity x4
  uint8_t Mode = 3; //Normal mode
 
  bme280spi.ESP32_BME280_SPI_Init(t_sb, filter, osrs_t, osrs_p, osrs_h, Mode);
  delay(1000);
}

void loop(){
  bme_get();
  delay(5000);
}

//************** BME280 測定 *************************
void bme_get(){ 
  byte temperature = (byte)round(bme280spi.Read_Temperature());
  byte humidity = (byte)round(bme280spi.Read_Humidity());
  uint16_t pressure = (uint16_t)round(bme280spi.Read_Pressure());

  char temp_c[10], hum_c[10], pres_c[10];
  sprintf(temp_c, "%2d ℃", temperature);
  sprintf(hum_c, "%2d ％", humidity);
  sprintf(pres_c, "%4d hPa", pressure);

  Serial.println("-----------------------");
  Serial.print("Temperature = "); Serial.println(temp_c);
  Serial.print("Humidity = "); Serial.println(hum_c);
  Serial.print("Pressure = "); Serial.println(pres_c);
  Serial.println("-----------------------");
  Serial.flush();
}
