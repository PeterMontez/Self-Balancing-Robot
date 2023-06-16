#include <SoftwareSerial.h>
      
SoftwareSerial mySerial(10, 11); 

bool paraFrente = false;
bool paraTras = false;
bool paraDireita = false;
bool paraEsquerda = false;

void setup()   
{    
  Serial.begin(115200);  
  mySerial.begin(38400);  
}  
    
void loop()  
{   
  char value;
 
  if (mySerial.available()>0) 
  {
    value = mySerial.read();
    Serial.println(value);
    switch (value)
    {
      case '1':  
        paraFrente = true;
        Serial.println("foi");
      break;
      
      case '2':
        paraTras = true;
        Serial.println("foi tras");
      break;
        
      case '3':
        paraDireita = true;
        Serial.println("foi dir");
      break;

      case '4':
        paraEsquerda = true;
        Serial.println("foi esq");
      break;
    }  
  }  
 }
