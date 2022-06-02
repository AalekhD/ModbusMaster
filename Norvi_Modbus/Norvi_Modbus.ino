#include <ModbusMaster.h>                             //Library for using ModbusMaster
#include <WiFiClient.h>
#include <string.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#define FC 4                                          //RE & DE are connected with the GPIO4 (for flow control) of the Norvi ESP32 Device.
ModbusMaster node;                                    //object node for class ModbusMaster

#include "SSD1306.h" // alias for `#include "SSD1306Wire.h"`

#include <SoftwareSerial.h>

const char* ssid     = "OnePlus 6"; //Wifi SSID
const char* password = "11143139"; //Wifi Password

String serverName = "http://128.199.24.255:5001/data?";

SSD1306  display(0x3C, 16, 17);
HTTPClient http;
void preTransmission()                                //Function for setting stste of Flow control pin of RS-485
{
  digitalWrite(FC, 1);
}

void postTransmission()
{
  digitalWrite(FC, 0);
}

void connect()
{
  WiFi.mode(WIFI_OFF);
  delay(1000);
  WiFi.mode(WIFI_STA);
   WiFi.begin(ssid, password);
  Serial.println("Connecting");
  while(WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("Connected to ");
  Serial.println(ssid);
  Serial.println("Ip Address :");
  Serial.println(WiFi.localIP());

  String serverPath = serverName + "temp=23&current=6&x=1&y=2&z=3";
   http.begin(serverPath.c_str());
  int httpCode = http.GET();
  String payload = http.getString();
  Serial.println("httpcode");
  Serial.println(httpCode);
  Serial.println("payload");
  Serial.println(payload);
  
  
}

void setup()
{
  Serial.begin(9600);
  Serial.println("started");
  pinMode(FC, OUTPUT);
  digitalWrite(FC, 0);
  Serial1.begin(9600);                                //Baud Rate as 9600
  pinMode(18, INPUT);                                //Initialize the inputs and outputs
  pinMode(39, INPUT);
  pinMode(34, INPUT);
  pinMode(35, INPUT);
  pinMode(19, INPUT);
  pinMode(21, INPUT);
  pinMode(22, INPUT);
  pinMode(23, INPUT);
  node.begin(14, Serial1);                              //Slave ID as 1
  node.preTransmission(preTransmission);              //Callback for configuring RS-485 Transreceiver correctly
  node.postTransmission(postTransmission);
  

  
  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_24);
  for(int i =0; i< 60;i++)
   {
    display.clear(); 
    display.drawString(0, i,   "Minsight ");
    display.drawString(0, i+25,"Solution ");
    display.display();
   
   }
    display.setFont(ArialMT_Plain_16);
  connect();
}

int counter = 0;
String Line[4];
uint32_t FFT_data[200];
String axisName[] = {"x-axis", "y-axis","z-axis"};
uint8_t currentAxis_FFT=0;
int x,y,z;
void loop()
{
   
  int rmsValues[3]={0,0,0};
  uint8_t result;
  
  uint16_t data[6];
  
  String serverPath ;
  if(counter <= 1){
        counter = 20;
        result = node.readHoldingRegisters(0x2710, 6);
        if (result == node.ku8MBSuccess)
        {
          UpdateScreen(0 );
          x = node.getResponseBuffer(1);
          y = node.getResponseBuffer(3);
          z = node.getResponseBuffer(5);
          serverPath = serverName + "temp=23&current=6&x="+String(x)+
          "&y="+ String(y) +"&z="+ String(z);
          
         http.begin(serverPath.c_str());
        int httpCode = http.GET();
        String payload = http.getString();
        Serial.println("httpcode");
        Serial.println(httpCode);
        Serial.println("payload");
        Serial.println(payload);
        
          
        }else
        {
          display.clear();
          display.drawString(0, 0,   "Read Fail ");
          display.display();
        }
        getFFT_data(currentAxis_FFT);
        initiate_FFT(currentAxis_FFT);
        currentAxis_FFT++;
        if(currentAxis_FFT >= 3){
          currentAxis_FFT = 0;
        }
    
  }else
  {
    counter--;
    UpdateScreen(counter); 
  }/* end of counter loop */

    delay(1000);
}

void UpdateScreen(int counter){
  display.clear();
  Line[0] = "RMS Value : "+String(counter) ;
  Line[1] = "x-axis: " + String(x*0.01) + " mm/s"; 
  Line[2] = "y-axis: " + String(y*0.01) + " mm/s"; 
  Line[3] = "z-axis: " + String(z*0.01) + " mm/s"; 
  display.drawString(0, 0,  Line[0]);
  display.drawString(0, 15, Line[1]);
  display.drawString(0, 30, Line[2]);
  display.drawString(0, 45, Line[3]);
  display.display();
}

#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2

void initiate_FFT(uint8_t axis){
  uint8_t result;
    Serial.println(axisName[axis]);
    if(axis == X_AXIS){
      node.writeSingleRegister(0x2730, 0x0100);
    }else if(axis == Y_AXIS){
      node.writeSingleRegister(0x2730, 0x0101);
    }else{
      node.writeSingleRegister(0x2730, 0x0103);      
    }
    if (result == node.ku8MBSuccess)
    {
      Serial.println("FFT initiated Successful");
    }else
    {
      Serial.println("FFT initiated ! Successful");
    }
} 

void getFFT_data(uint8_t axis){
  uint32_t Temp = 0; 
  uint8_t result;
  uint16_t fft_data_index=0;
  Serial.println("Getting FFT Data:");
  Serial.println(axisName[axis]);
  for(uint16_t addr=0; addr <= 200; addr+=50)
  {
     if(axis == X_AXIS){
      result = node.readHoldingRegisters(addr, 50);
    }else if(axis == Y_AXIS){
      result = node.readHoldingRegisters(addr,50);
    }else{
      result = node.readHoldingRegisters(addr, 50);      
    }
    
    Serial.println(result);
    Serial.println(addr);

    if (result == node.ku8MBSuccess)
    {
      for (uint16_t i=0; i<=50; i=i+2)
      {
        /*
        Serial.print(i);
        Serial.print(":");
        //Temp = (uint32_t)((node.getResponseBuffer(i) << 16) | node.getResponseBuffer(i+1));
        Serial.print(node.getResponseBuffer(i+1));
        Serial.print(":");
        Serial.println(node.getResponseBuffer(i));
        */
        //Serial.print(node.getResponseBuffer(i+1));
        FFT_data[fft_data_index] = node.getResponseBuffer(i+1);
        fft_data_index++;
      }
    }
    delay(100);
  }
  for(uint16_t i =0;i<200;i++){
        Serial.print(i);
        Serial.print(":");
        Serial.print(":");
        Serial.println(FFT_data[i]);
  }
  
   
}
