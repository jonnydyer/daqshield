#include <DAQShield.h>
#include "ADS1248.h"

void setup()
{
  
  Serial.begin(115200);
  if(DAQShield.begin() == true)
  {
    Serial.print("DAQShield successfully configured!\n"); 
  }
  else
  {
    Serial.print("Problem configuring DAQShield!\n");
  }
  //DAQShield.enableIntRef();
  //DAQShield.selectRef(ADC_INTREF);
  //DAQShield.selectMuxCal(ADC_MUXCAL_TEMP);
}

void loop()
{
  /*bool drdy_status;
  drdy_status = digitalRead(ADS1248_DRDY);
  if(drdy_status == LOW)
  {
    Serial.print("DRDY, ");
    Serial.print("Data: ");
    Serial.print(DAQShield.readData(),16);
    Serial.print("\n");
    DAQShield.startSingle();
    delay(10);
  }*/
  Serial.print((DAQShield.sample(ADC_MUXCAL_TEMP,ADC_5_SPS)-0.118)/405e-6+25.0,6);
  Serial.print(",");
  Serial.print((DAQShield.sample(6,7,ADC_PGA_1,ADC_5_SPS,ADC_INTREF)-.6)*100,6);
  Serial.print("\r\n");
  delay(1000);
}

