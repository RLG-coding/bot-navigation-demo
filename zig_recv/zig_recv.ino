#include <ZigduinoRadio.h>

void errHandle(radio_error_t err)
{
  Serial.println();
  Serial.print("Error: ");
  Serial.print((uint8_t)err, 10);
  Serial.println();
}

void setup()
{
  ZigduinoRadio.begin(13);
  Serial.begin(9600);
  ZigduinoRadio.attachError(errHandle);
  Serial.println("Receiver online.");
}

void loop()
{
  if (Serial.available())
  {
    ZigduinoRadio.beginTransmission();
    int res = Serial.read();
     if (Serial.available())
  {
    Serial.println(res);
  }
    ZigduinoRadio.write(res);
    ZigduinoRadio.endTransmission();
  }

  while (ZigduinoRadio.available())
  {
    if(ZigduinoRadio.read() == 101)
      Serial.println("1");
  }
  
  delay(50);
}
