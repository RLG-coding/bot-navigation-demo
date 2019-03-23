#include <ZigduinoRadio.h>

#define PIR_MOTION_SENSOR 4
#define LED 2
int lock = 0;
int code;

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
  Serial.println("Transmitter online.");
  
  pinMode(PIR_MOTION_SENSOR, INPUT);
  pinMode(LED, OUTPUT);
}

void loop()
{  
  
    ZigduinoRadio.beginTransmission();
    
    if(lock == 0){
      if(digitalRead(PIR_MOTION_SENSOR))
      {
        digitalWrite(LED, HIGH);
        Serial.println("Intruder detected");
        ZigduinoRadio.write("alert");
        delay(2000);     
      } else {
        digitalWrite(LED, LOW);
      }
    }

    ZigduinoRadio.endTransmission();
  

  if(ZigduinoRadio.available()){
    code = ZigduinoRadio.read();
    if(code == 48) // '0' in ASCII
    { 
      Serial.println("LED is off.");
      digitalWrite(LED, LOW);
      lock = 0;
    } 
    else if(code == 49) // '1' in ASCII
    { 
      Serial.println("LED is on.");
      digitalWrite(LED, HIGH);
      lock = 1;
    }
  }
  
  delay(50);
}
