#include <SoftwareSerial.h>
    
//Define os pinos para a serial   
SoftwareSerial mySerial(10, 11); // RX, TX  
char command = ""; // Stores response of bluetooth device  
            // which simply allows n between each  
            // response.  
    
void setup()   
{  
  //Inicia a serial  
  Serial.begin(115200);  
  //Inicia a serial configurada nas portas 10 e 11
  mySerial.begin(38400);  
}  
    
void loop()  
{  
  // Read device output if available.  
  if (mySerial.available()>0) 
  {  
    command = mySerial.read();
    Serial.println(command);
  }
}
