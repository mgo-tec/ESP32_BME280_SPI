/*
  ESP32_BME280_SPI.cpp - for Arduino core for ESP32
  Beta version 1.0
  
License MIT [Modified person is Mgo-tec.]

Reference library:
https://github.com/adafruit/Adafruit_BME280_Library

Original License is BSD [Copyright (c) 2012, Adafruit Industries]

BSD License:
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.
3. Neither the name of the copyright holders nor the
names of its contributors may be used to endorse or promote products
derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "ESP32_BME280_SPI.h"

ESP32_BME280_SPI::ESP32_BME280_SPI(uint8_t sclk, uint8_t mosi, uint8_t miso, uint8_t cs, uint32_t freq)
  : _sclk(sclk), _mosi(mosi), _miso(miso), _cs(cs), _freq(freq)
{}

SPIClass _Spi(HSPI);

//****************ESP32_BME280_SPI初期化*************************************************
void ESP32_BME280_SPI::ESP32_BME280_SPI_Init(uint8_t Stanby_t, uint8_t filter, uint8_t overS_T, uint8_t overS_P, uint8_t overS_H, uint8_t mode){
	
	pinMode(_cs, OUTPUT);
	
	_Spi.begin(_sclk, _miso, _mosi, _cs);
  _Spi.setFrequency(_freq); //BME280 Max 10MHz
  _Spi.setDataMode(SPI_MODE3);
  //_Spi.setHwCs(false);

  digitalWrite(_cs, HIGH);
  
  uint8_t spi3or4 = 0; //SPI 3wire or 4wire, 0=4wire, 1=3wire
  
  uint8_t ctrl_meas = (overS_T << 5) | (overS_P << 2) | mode;
  uint8_t config    = (Stanby_t << 5) | (filter << 2) | spi3or4;
  uint8_t ctrl_hum  = overS_H;
  
  WriteRegister(0xE0, 0xB6); //reset
  delay(2000);
  WriteRegister(0xF2, ctrl_hum);
  WriteRegister(0xF4, ctrl_meas);
  WriteRegister(0xF5, config);
  
  ReadCalibration();
}
//***************BME280へ初期レジスタ書き込み関数****************************
void ESP32_BME280_SPI::WriteRegister(uint8_t reg_address, uint8_t data) {
  digitalWrite(_cs, LOW);
  _Spi.transfer(reg_address & B01111111); // write, bit 7 low
  _Spi.transfer(data);
  digitalWrite(_cs, HIGH);
}
//*****************初期値読み込み**************************************
void ESP32_BME280_SPI::ReadCalibration(void) {
	_dig_T1 = read16bit(0x88);
	_dig_T2 = (int16_t)read16bit(0x8A);
	_dig_T3 = (int16_t)read16bit(0x8C);

	_dig_P1 = read16bit(0x8E);
	_dig_P2 = (int16_t)read16bit(0x90);
	_dig_P3 = (int16_t)read16bit(0x92);
	_dig_P4 = (int16_t)read16bit(0x94);
	_dig_P5 = (int16_t)read16bit(0x96);
	_dig_P6 = (int16_t)read16bit(0x98);
	_dig_P7 = (int16_t)read16bit(0x9A);
	_dig_P8 = (int16_t)read16bit(0x9C);
	_dig_P9 = (int16_t)read16bit(0x9E);

	_dig_H1 = read8bit(0xA1);
	_dig_H2 = (int16_t)read16bit(0xE1);
	_dig_H3 = read8bit(0xE3);
	_dig_H4 = (int16_t)((read8bit(0xE4) << 4) | (read8bit(0xE5) & 0x0F));
	_dig_H5 = (int16_t)((read8bit(0xE6) << 4) | (read8bit(0xE5) >> 4));
	_dig_H6 = (int8_t)read8bit(0xE7);
}
//***************BME280からの温度データ読み込み関数****************************
double ESP32_BME280_SPI::Read_Temperature(){
	
	_Spi.setFrequency(_freq); //BME280 Max 10MHz
  _Spi.setDataMode(SPI_MODE3);
  //_Spi.setHwCs(false);
  pinMode(_cs, OUTPUT);
	
  uint32_t data[3];
  uint8_t i;
  
  digitalWrite(_cs, LOW);

  _Spi.transfer(0xFA | B10000000); //0xFA temperature msb read, bit 7 high

  for(i=0; i<3; i++){
		data[i] = _Spi.transfer(0x00);
  }

  digitalWrite(_cs, HIGH);
  
  uint32_t adc_T = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4); //0xFA, msb+lsb+xlsb=19bit
  return compensate_T((int32_t)adc_T) / 100.0;
}
//***************BME280からの気圧データ読み込み関数****************************
double ESP32_BME280_SPI::Read_Pressure(){
	
	_Spi.setFrequency(_freq); //BME280 Max 10MHz
  _Spi.setDataMode(SPI_MODE3);
  //_Spi.setHwCs(false);
  pinMode(_cs, OUTPUT);
	
  uint32_t data[3];
  uint8_t i;
  
  Read_Temperature(); //_t_fineに値を入れるために必ず気温を読み込む
  
  digitalWrite(_cs, LOW);
  _Spi.transfer(0xF7 | B10000000); //0xF7 puressure msb read, bit 7 high
  for(i=0; i<3; i++){
    data[i] = _Spi.transfer(0x00);
  }
  digitalWrite(_cs, HIGH);
  
  uint32_t adc_P = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4); //0xF7, msb+lsb+xlsb=19bit
  return compensate_P((int32_t)adc_P) / 100.0;
}
//***************BME280からの湿度データ読み込み関数****************************
double ESP32_BME280_SPI::Read_Humidity(){
	
	_Spi.setFrequency(_freq); //BME280 Max 10MHz
  _Spi.setDataMode(SPI_MODE3);
  //_Spi.setHwCs(false);
  pinMode(_cs, OUTPUT);
	
  uint32_t data[2];
  
  Read_Temperature(); //_t_fineに値を入れるために必ず気温を読み込む
  
  digitalWrite(_cs, LOW);
  _Spi.transfer(0xFD | B10000000); //0xFD Humidity msb read, bit 7 high
	data[0] = _Spi.transfer(0x00);
  data[1] = _Spi.transfer(0x00);
  digitalWrite(_cs, HIGH);
  
  uint32_t adc_H = (data[0] << 8) | data[1];  //0xFD, msb+lsb=19bit(16:0)
  return compensate_H((int32_t)adc_H) / 1024.0;
}
//***************温度キャリブレーション関数****************************
int32_t ESP32_BME280_SPI::compensate_T(int32_t adc_T) {
  int32_t var1, var2, T;

  var1 = ((((adc_T >> 3) - ((int32_t)_dig_T1<<1))) * ((int32_t)_dig_T2)) >> 11;
  var2 = (((((adc_T >> 4) - ((int32_t)_dig_T1)) * ((adc_T>>4) - ((int32_t)_dig_T1))) >> 12) * ((int32_t)_dig_T3)) >> 14;

  _t_fine = var1 + var2;
  T = (_t_fine * 5 + 128) >> 8;

  return T;
}
//***************気圧キャリブレーション関数****************************
uint32_t ESP32_BME280_SPI::compensate_P(int32_t adc_P) {
  int32_t var1, var2;
  uint32_t P;

  var1 = (((int32_t)_t_fine)>>1) - (int32_t)64000;
  var2 = (((var1>>2) * (var1>>2)) >> 11) * ((int32_t)_dig_P6);
  var2 = var2 + ((var1*((int32_t)_dig_P5))<<1);
  var2 = (var2>>2)+(((int32_t)_dig_P4)<<16);
  var1 = (((_dig_P3 * (((var1>>2)*(var1>>2)) >> 13)) >>3) + ((((int32_t)_dig_P2) * var1)>>1))>>18;
  var1 = ((((32768+var1))*((int32_t)_dig_P1))>>15);

  if (var1 == 0) {
    return 0;
  }

  P = (((uint32_t)(((int32_t)1048576)-adc_P)-(var2>>12)))*3125;

  if(P<0x80000000) {
    P = (P << 1) / ((uint32_t) var1);
  }else{
    P = (P / (uint32_t)var1) * 2;
  }

  var1 = (((int32_t)_dig_P9) * ((int32_t)(((P>>3) * (P>>3))>>13)))>>12;
  var2 = (((int32_t)(P>>2)) * ((int32_t)_dig_P8))>>13;
  P = (uint32_t)((int32_t)P + ((var1 + var2 + _dig_P7) >> 4));

  return P;
}
//***************湿度キャリブレーション関数****************************
uint32_t ESP32_BME280_SPI::compensate_H(int32_t adc_H) {
  int32_t v_x1;

  v_x1 = (_t_fine - ((int32_t)76800));
  v_x1 = (((((adc_H << 14) -(((int32_t)_dig_H4) << 20) - (((int32_t)_dig_H5) * v_x1)) + 
         ((int32_t)16384)) >> 15) * (((((((v_x1 * ((int32_t)_dig_H6)) >> 10) * 
         (((v_x1 * ((int32_t)_dig_H3)) >> 11) + ((int32_t) 32768))) >> 10) + ((int32_t)2097152)) * 
         ((int32_t) _dig_H2) + 8192) >> 14));
  v_x1 = (v_x1 - (((((v_x1 >> 15) * (v_x1 >> 15)) >> 7) * ((int32_t)_dig_H1)) >> 4));
  v_x1 = (v_x1 < 0 ? 0 : v_x1);
  v_x1 = (v_x1 > 419430400 ? 419430400 : v_x1);

  return (uint32_t)(v_x1 >> 12);
}
//***************標高計算関数****************************************************
double ESP32_BME280_SPI::ReadAltitude(double SeaLevel_Pres, double pressure) {
  double altitude = 44330.0 * (1.0 - pow(pressure / SeaLevel_Pres, (1.0/5.255)));  
  return altitude;
}
//***************BME280から16bitデータ読み込み関数****************************
uint16_t ESP32_BME280_SPI::read16bit(uint8_t reg) {
  uint16_t d1, d2;
  uint16_t data;
  digitalWrite(_cs, LOW);

	_Spi.transfer(reg | B10000000);
	d1 = _Spi.transfer(0x00);
  d2 = _Spi.transfer(0x00);
  data = (d2 << 8) | d1;

  digitalWrite(_cs, HIGH);
  return data;
}
//***************BME280から8bitデータ読み込み関数****************************
uint8_t ESP32_BME280_SPI::read8bit(uint8_t reg) {
  uint8_t data;

  digitalWrite(_cs, LOW);
  _Spi.transfer(reg | B10000000); // read, bit 7 high
	data = _Spi.transfer(0x00);
  digitalWrite(_cs, HIGH);
  return data;
}