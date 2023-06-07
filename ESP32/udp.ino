#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"

const char* ssid = "Convergentes";
const char* password = "RedesConvergentes*#";

const char * udpAddress = "172.1.1.19";
const int udpPort = 3333;

WiFiUDP udp;

const byte MLX90640_address = 0x33; //Default 7-bit unshifted address of the MLX90640

#define TA_SHIFT 8 //Default shift for MLX90640 in open air

float mlx90640To[768];
paramsMLX90640 mlx90640;

const byte calcStart = 33; //Pin that goes high/low when calculations are complete
//This makes the timing visible on the logic analyzer

void setup()
{
  pinMode(calcStart, OUTPUT);

  Wire.begin();
  Wire.setClock(400000); //Increase I2C clock speed to 400kHz

  Serial.begin(115200); //Fast serial as possible
  connectToWiFi(ssid, password);

  if (isConnected() == false)
  {
    Serial.println("MLX90640 not detected at default I2C address. Please check wiring. Freezing.");
    while (1);
  }

  //Get device parameters - We only have to do this once
  int status;
  uint16_t eeMLX90640[832];
  status = MLX90640_DumpEE(MLX90640_address, eeMLX90640);
  if (status != 0)
    Serial.println("Failed to load system parameters");

  status = MLX90640_ExtractParameters(eeMLX90640, &mlx90640);
  if (status != 0)
    Serial.println("Parameter extraction failed");

  MLX90640_SetRefreshRate(MLX90640_address, 0x04); //Set rate to 4Hz effective - Works
  Wire.setClock(1000000); //Teensy will now run I2C at 800kHz (because of clock division)
}

void loop()
{
  long startTime = millis();
  for (byte x = 0 ; x < 2 ; x++)
  {
    uint16_t mlx90640Frame[834];
    int status = MLX90640_GetFrameData(MLX90640_address, mlx90640Frame);

    digitalWrite(calcStart, HIGH);
    float vdd = MLX90640_GetVdd(mlx90640Frame, &mlx90640);
    float Ta = MLX90640_GetTa(mlx90640Frame, &mlx90640);

    float tr = Ta - TA_SHIFT; //Reflected temperature based on the sensor ambient temperature
    float emissivity = 0.95;

    MLX90640_CalculateTo(mlx90640Frame, &mlx90640, emissivity, tr, mlx90640To);
    digitalWrite(calcStart, LOW);
    //Calculation time on a Teensy 3.5 is 71ms
  }
  long stopReadTime = millis();
  udp.beginPacket(udpAddress, udpPort);

  for (int x = 0 ; x < 768 ; x++)
  {
    udp.printf("%.2f", mlx90640To[x]);
    udp.print(",");
    Serial.print(mlx90640To[x], 2);
    Serial.print(",");
  }
  Serial.println("");
  udp.println("");
  udp.endPacket();
  
  long stopPrintTime = millis();

  Serial.print("Read rate: ");
  Serial.print( 1000.0 / (stopReadTime - startTime), 2);
  Serial.println(" Hz");
  Serial.print("Read plus print rate: ");
  Serial.print( 1000.0 / (stopPrintTime - startTime), 2);
  Serial.println(" Hz");
}

boolean isConnected()
{
  Wire.beginTransmission((uint8_t)MLX90640_address);
  if (Wire.endTransmission() != 0)
    return (false); //Sensor did not ACK
  return (true);
}

void connectToWiFi(const char * ssid, const char * pwd){
  Serial.println("Connecting to WiFi network: " + String(ssid));
  WiFi.disconnect(true);
  WiFi.onEvent(WiFiEvent);
  WiFi.begin(ssid, pwd);
  Serial.println("Waiting for WIFI connection...");
}

void WiFiEvent(WiFiEvent_t event){
    switch(event) {
      case SYSTEM_EVENT_STA_GOT_IP:
          Serial.print("WiFi connected! IP address: ");
          Serial.println(WiFi.localIP());  
          udp.begin(WiFi.localIP(),udpPort);
          break;
      case SYSTEM_EVENT_STA_DISCONNECTED:
          Serial.println("WiFi lost connection");
          break;
    }
}
