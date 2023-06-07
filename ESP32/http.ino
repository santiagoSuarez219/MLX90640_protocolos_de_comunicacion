#include <Wire.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <HTTPClient.h>

#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"

WiFiMulti wifiMulti;

const byte MLX90640_address = 0x33; //Default 7-bit unshifted address of the MLX90640

#define TA_SHIFT 8 //Default shift for MLX90640 in open air

float mlx90640To[768];
paramsMLX90640 mlx90640;

const byte calcStart = 33; //Pin that goes high/low when calculations are complete
//This makes the timing visible on the logic analyzer

void setup()
{
  Wire.begin();
  Wire.setClock(400000); //Increase I2C clock speed to 400kHz
  Serial.begin(115200); //Fast serial as possible
  wifiMulti.addAP("Convergentes", "RedesConvergentes*#");
  Serial.println("Conectando a Wifi");
  while(wifiMulti.run() != WL_CONNECTED){
    Serial.println(".");
  }
  Serial.println();
  Serial.println("Wifi Conectado");
  Serial.println("Direccion IP: ");
  Serial.println(WiFi.localIP());

  if (isConnected() == false)
  {
    Serial.println("MLX90640 not detected at default I2C address. Please check wiring. Freezing.");
    while (1);
  }

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
  for (byte x = 0 ; x < 2 ; x++)
  {
    uint16_t mlx90640Frame[834];
    int status = MLX90640_GetFrameData(MLX90640_address, mlx90640Frame);
    float vdd = MLX90640_GetVdd(mlx90640Frame, &mlx90640);
    float Ta = MLX90640_GetTa(mlx90640Frame, &mlx90640);
    float tr = Ta - TA_SHIFT; //Reflected temperature based on the sensor ambient temperature
    float emissivity = 0.95;
    MLX90640_CalculateTo(mlx90640Frame, &mlx90640, emissivity, tr, mlx90640To);
  }
  HTTPClient http;
  Serial.println("[HTTP] Iniciando ... ");
  http.begin("http://172.1.1.19:3000/api/v1/image/");
  http.addHeader("Content-Type", "application/json");
  Serial.println("[HTTP] POST...");

  String thermalImage = "";
  for (int x = 0 ; x < 768 ; x++)
  {
    thermalImage += String(mlx90640To[x], 2) + ",";

  }
  thermalImage.remove(thermalImage.length() - 1);
  String json = "{\"thermalImage\":[" + thermalImage + "]}";
  int httpCode = http.POST(json);
  String payload = http.getString();
  Serial.println(httpCode);
  Serial.println(payload);
  http.end();
  delay(1000);
}

//Returns true if the MLX90640 is detected on the I2C bus
boolean isConnected()
{
  Wire.beginTransmission((uint8_t)MLX90640_address);
  if (Wire.endTransmission() != 0)
    return (false); //Sensor did not ACK
  return (true);
}
