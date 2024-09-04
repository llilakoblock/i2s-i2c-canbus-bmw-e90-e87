#include <Arduino.h>
#include <Wire.h>
#include <I2S.h>
#include <bitset>
#include <vector>
#include <numeric>
#include <random>

byte ADDRESS = 0x1c;
#define PIN_I2S_SCK 36
#define PIN_I2S_FS 39
#define PIN_I2S_SD 34

const int sampleRate = 44100; // sample rate in Hz
int count = 0;
uint16_t adcData1 [64];
uint16_t adcData2 [64];

  extern const uint8_t no_start[] asm("_binary_src_no_wav_start");
  extern const uint8_t no_end[] asm("_binary_src_no_wav_end");

void receiveEvent(int bytes)
{
  while(Wire.available() > 0) // loop through all but the last
  {
    byte c = Wire.read(); // receive byte as a character
    Serial.print(c, 16);         // print the character
    Serial.print(" ");
  }
}

void setup() 
{
Serial.begin(115200);

delay(1000);

if(!I2S.begin(I2S_LEFT_JUSTIFIED_MODE, sampleRate, 16) )
  Serial.println( "Error, init I2S");

if(!Wire.begin())
  Serial.println( "Error, init I2C");

//Wire.onReceive(receiveEvent); // register event
  
if( !Wire.setClock(400000))
   Serial.println( "Error, set I2C clock");
}

void writeI2S_sample()
{
  auto wav_file_start = no_start, wav_file_end = no_end;
  while(wav_file_start != wav_file_end)
  {
    auto bytes = I2S.write(*no_start);
    ++wav_file_start;
  }
}

uint16_t readReg16(uint8_t h_addr, uint8_t l_addr, uint8_t dev_addr)
{
  uint16_t res = 0;
  Wire.beginTransmission( dev_addr);
  Wire.write( h_addr); // MSB of Device ID Register
  Wire.write(l_addr);
  int error = Wire.endTransmission( false);  // no stop, to create a repeated start.
  if( error != 0)
    Serial.println( "Error, no device found at 0x1c");

  int n = Wire.requestFrom( dev_addr, 2);
  if( n != 2)
  {
    Serial.println( "requestFrom failed");
  }
  else
  {
    res = Wire.read();
    res = (res << 8) ;
    res = res | Wire.read();
  }

  error = Wire.endTransmission(true);
  if( error != 0)
    Serial.println( "Error, no device found at 0x1c");
    
  return res;
}

void writeReg16( uint8_t h_addr, uint8_t l_addr, uint8_t dev_addr, uint8_t* data, size_t data_len)
{
  Wire.beginTransmission( dev_addr);
  Wire.write(h_addr); // MSB of Device ID Register
  Wire.write(l_addr);

  for(int i = 0; i < data_len; ++i)
  {
    if(int bytes = Wire.write(data[i]); bytes != 1)
      Serial.println( "Failed to write byte to 0x1c");
  }

  int error = Wire.endTransmission(true);
  if( error != 0)
    Serial.println( "Error, no device found at 0x1c");
}

void printReg16(uint16_t status, char* reg_name)
{
  Serial.print("|");
  Serial.print(reg_name);
  Serial.print("| ");
  Serial.print("BIN: ");
  Serial.print(std::bitset<8>(uint8_t(status >> 8)).to_string().c_str());
  Serial.print(" ");
  Serial.print(std::bitset<8>(uint8_t(status << 8 >> 8)).to_string().c_str());
  Serial.print(" HEX: 0x");
  Serial.print(status >> 8, HEX);
  Serial.print(" 0x");
  Serial.print((uint8_t)(status << 8 >> 8), HEX);
  Serial.println();
}

void printAllRegisters()
{
printReg16(readReg16(0x0f, 0xff, ADDRESS), "DSP Control register 0x0FFF");
printReg16(readReg16(0x1f, 0xff, ADDRESS), "Program counter register DSP2 0x1FFF");
printReg16(readReg16(0x1f, 0xfe, ADDRESS), "Status register DSP2 0x1FFE");
printReg16(readReg16(0x1f, 0xfd, ADDRESS), "I/O configuration register DSP2 0x1FFD");
printReg16(readReg16(0x1f, 0xfc, ADDRESS), "Phone, navigation and audio register 0x1FFc");
printReg16(readReg16(0x1f, 0xfb, ADDRESS), "FM and RDS sensitivity register 0x1FFB");
printReg16(readReg16(0x1f, 0xfa, ADDRESS), "Clock coefficient register 0x1FFFA");
printReg16(readReg16(0x1f, 0xf9, ADDRESS), "Clock settings register 0x1FF9");
printReg16(readReg16(0x1f, 0xf8, ADDRESS), "IAC settings register 0x1FF8");
printReg16(readReg16(0x1f, 0xf7, ADDRESS), "Selector register 0x1FFF7");
printReg16(readReg16(0x1f, 0xf6, ADDRESS), "CL_GEN register 4 0x1FF6");
printReg16(readReg16(0x1f, 0xf5, ADDRESS), "CL_GEN register 3 0x1FF5");
printReg16(readReg16(0x1f, 0xf4, ADDRESS), "CL_GEN register 2 0x1FF4");
printReg16(readReg16(0x1f, 0xf3, ADDRESS), "CL_GEN register 1 0x1FF3");
printReg16(readReg16(0x1f, 0xf2, ADDRESS), "Unknown register 2 0x1FF2");
printReg16(readReg16(0x1f, 0xf1, ADDRESS), "Unknown register 1 0x1FF1");
printReg16(readReg16(0x1f, 0xf0, ADDRESS), "Evaluation register 0x1FFF0");
Serial.println();
delay(10000);
}

void maxAdcSignal(uint16_t& max, uint16_t& average)
{
  uint64_t sum1 = 0, sum2 = 0;
  for(int i = 0; i < 64; ++i)
  {
    adcData1[i] = analogRead(36);
    adcData2[i] = analogRead(39);
    if (adcData1[i] > max)
      max = adcData1[i];
    sum1 += adcData1[i];             
  }
  average = sum1 / 64;
}

std::vector<uint16_t> vec(0x4fff);
void randRegister()
{
  std::iota(begin(vec), end(vec), 0);
  std::mt19937 rng(std::random_device{}());
  std::shuffle(begin(vec), end(vec), rng);

  uint8_t data [] = {0x0, 0x0, 0x0};
  
  for(auto k : vec)
  {
    data[0] = k >> 8;
    data[1] = k << 8 >> 8;
    writeReg16(0x1f, 0xfc, ADDRESS, data, 3);
    printReg16(readReg16(0x1f, 0xfc, ADDRESS), "Phone, navigation and audio register");
      
      for(auto i : vec)
      {
        data[0] = i >> 8;
        data[1] = i << 8 >> 8;
        writeReg16(0x1f, 0xf7, ADDRESS, data, 3);
        printReg16(readReg16(0x1f, 0xf7, ADDRESS), "Selector register");

        for(auto j : vec)
        {
          data[0] = j >> 8;
          data[1] = j << 8 >> 8;
          writeReg16(0x1f, 0xf9, ADDRESS, data, 3);
          printReg16(readReg16(0x1f, 0xf9, ADDRESS), "Clock settings register");
          delay(500);
        }
      }
  }
}

void cycleRegistersAll()
{
  uint8_t data [] = {0x0, 0x0, 0x0};
  for(uint16_t i = 0; i <= 0x4FFF; ++i)
  {
    data[0] = i >> 8;
    data[1] = i << 8 >> 8;
    writeReg16(0x1f, 0xfc, ADDRESS, data, sizeof(3));
    printReg16(readReg16(0x1f, 0xfc, ADDRESS), "Phone, navigation and audio register");
    //writeReg16(0x1f, 0xf7, ADDRESS, data, sizeof(3));
    //printReg16(readReg16(0x1f, 0xf7, ADDRESS), "Selector register");
    delay(100);
    continue;


    uint16_t max, avarage;
    maxAdcSignal(max, avarage);
    if(max > 2000 && avarage > 2000)
    {
      Serial.println("Got signal with register state:");
      printReg16(readReg16(0x1f, 0xfc, ADDRESS), "Phone, navigation and audio register");
      //printReg16(readReg16(0x1f, 0xf7, ADDRESS), "Selector register");
    }
  }
}

void cycleRegistersBitPosition()
{
  uint8_t data [] = {0x0, 0x0, 0x0};
  uint16_t i = 1;
  while(i != 0xffff)
  {
    data[0] = i >> 8;
    data[1] = i << 8 >> 8;
    //writeReg16(0x1f, 0xfc, ADDRESS, data, sizeof(3));
    //printReg16(readReg16(0x1f, 0xfc, ADDRESS), "Phone, navigation and audio register");
    writeReg16(0x1f, 0xf7, ADDRESS, data, sizeof(3));
    printReg16(readReg16(0x1f, 0xf7, ADDRESS), "Selector register");
    delay(1000);
    i = i << 1;
    continue;


    uint16_t max, avarage;
    maxAdcSignal(max, avarage);
    if(max > 2000 && avarage > 2000)

    {
      Serial.println("Got signal with register state:");
      printReg16(readReg16(0x1f, 0xfc, ADDRESS), "Phone, navigation and audio register");
      //printReg16(readReg16(0x1f, 0xf7, ADDRESS), "Selector register");
    }
  }
}

void loop() 
{

writeI2S_sample();
  //uint8_t data [] = {0x43, 0x94, 0x0};
  //writeReg16(0x1f, 0xf7, ADDRESS, data, sizeof(data));
  //printAllRegisters();
  //cycleRegistersBitPosition();
  //randRegister();
  
//   return;
// Serial.println(analogRead(36));
// Serial.println(analogRead(39));
// return;
// uint16_t max, avarage;
// maxAdcSignal(max, avarage);
// Serial.print("Max: ");
// Serial.print(max);
// Serial.println();
// Serial.print("Avagare: ");
// Serial.print(avarage);
// Serial.println();
// delay(1000);
//cycleRegistersBitPosition();
//cycleRegistersAll();
//Serial.println(maxAdcSignal());
return;
Serial.println("BEFORE WRITE");
printAllRegisters();

//writeReg16(0x1f, 0xfc, ADDRESS, data, sizeof(data));
//writeReg16(0x1f, 0xf7, ADDRESS, data, sizeof(data));

//writeI2S_sample();
Serial.println("AFTER WRITE");
printAllRegisters();

}