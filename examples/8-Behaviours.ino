#include <Wire.h>
#include <Adafruit_NeoPixel.h>

#include <WiFi.h>
#include <ESPAsyncWebSrv.h>

const char* ssid = "ESP32-Access-Point";
const char* password = "12345678";

AsyncWebServer server(80);
int modeFlag = 0;

//---I2C vars/defines
#define ESPEEDY_ADDR 0x01
int espeedySensorN = 0;

//---Neopixel (8 colour LEDs on front of robot)
Adafruit_NeoPixel espeedyPixel(8, 48, NEO_GRB + NEO_KHZ800);

//---Gyro
#define LSM6DSR_ADDRESS  0x6A  // Default I2C address for LSM6DSR
#define LSM6DSR_GYRO_OUT_X_L 0x22  // Start address for gyroscope output registers

// LSM6DSR Register Addresses
#define LSM6DSR_CTRL2_G 0x11  // Gyroscope control register

// Gyroscope ODR (Output Data Rate) Settings
#define GYRO_ODR_OFF   0b00000000
#define GYRO_ODR_12_5  0b00010000
#define GYRO_ODR_26    0b00100000
#define GYRO_ODR_52    0b00110000
#define GYRO_ODR_104   0b01000000
#define GYRO_ODR_208   0b01010000
#define GYRO_ODR_416   0b01100000
#define GYRO_ODR_833   0b01110000
#define GYRO_ODR_1660  0b10000000
#define GYRO_ODR_3330  0b10010000
#define GYRO_ODR_6660  0b10100000

// Gyroscope Full Scale Selection
#define GYRO_FS_250    0b00000000  // +/- 250 dps
#define GYRO_FS_500    0b00000100  // +/- 500 dps
#define GYRO_FS_1000   0b00001000  // +/- 1000 dps
#define GYRO_FS_2000   0b00001100  // +/- 2000 dps

struct GyroData {
  int16_t x;
  int16_t y;
  int16_t z;
};

//time
unsigned long time1;

//values for you to use
//

void control1(float values[8], float angle){
  
  //go forward
  espeedySendI2C(7,100);
  espeedySendI2C(8,100);

  if(time1<millis()){modeFlag=0;}
}

void control2(float values[8], float angle){

  float error=values[1]-values[4];
  error=error*25.0;
  
  espeedySendI2C(7,10+(int8_t)error);
  espeedySendI2C(8,10-(int8_t)error);

  //if(time1<millis()){modeFlag=0;}
}

void control3(float values[8], float angle){

  if(time1<millis()){modeFlag=0;}
}

void control4(float values[8], float angle){

  if(time1<millis()){modeFlag=0;}
}

void controlStop(){

  //send ZERO power to left and right motors
  espeedySendI2C(7,0);
  espeedySendI2C(8,0);
  
}

// Global variable to keep track of angles
float angleX = 0.0;
float angleY = 0.0;
float angleZ = 0.0;

// Assuming a sample rate of 100Hz (10ms sample time). Adjust based on your actual rate.
const float sampleTime = 0.013;

// Conversion factor. Depends on the scale you've set for the gyroscope.
// E.g., for a 2000 dps (degrees per second) scale, the factor is 2000/32768.
const float gyroScaleFactor = 2000.0 / 32768.0;

float setPoint;

const char htmlPage[] PROGMEM = R"=====(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>ESP32 Control Panel</title>
<script>
function sendMode(mode) {
  var xhttp = new XMLHttpRequest();
  xhttp.open("POST", "/mode", true);
  xhttp.setRequestHeader("Content-type", "application/x-www-form-urlencoded");
  xhttp.send("mode=" + mode);
}
</script>
</head>
<body>
<button onclick="sendMode('green')" style="background-color:green">1. Green</button>
<button onclick="sendMode('yellow')" style="background-color:yellow">2. Yellow</button>
<button onclick="sendMode('cyan')" style="background-color:cyan">3. Cyan</button>
<button onclick="sendMode('blue')" style="background-color:blue">4. Blue</button>
<button onclick="sendMode('stop')" style="background-color:red">STOP</button>
</body>
</html>
)=====";




void setup() {
  Serial.begin(115200);
  Wire.begin(11,10);

  //clear colour pixels
  espeedyPixel.begin();
  espeedyPixel.show();

  //wait for things to settle
  delay(1000);

  //config for IR sensors
  espeedySendI2C(1,0b00000010);
  espeedySensorN=8;
  Serial.print("Config: ");
  Serial.println(espeedyReadI2C(1));
  
  Serial.println(espeedyReadI2C(7));

  //gyro
  configureGyro();
  time1=millis();
  setPoint=0;

  //configure wi-fi  
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", htmlPage);
  });

  server.on("/mode", HTTP_POST, [](AsyncWebServerRequest *request){
    if(request->hasParam("mode", true)){
      String mode = request->getParam("mode", true)->value();
      if(mode == "green"){modeFlag = 1; time1=millis()+1000; Serial.println("green");}
      else if(mode == "yellow"){modeFlag = 2; time1=millis()+1000; Serial.println("yellow");}
      else if(mode == "cyan"){modeFlag = 3; time1=millis()+1000; Serial.println("cyan");}
      else if(mode == "blue"){modeFlag = 4; time1=millis()+1000; Serial.println("blue");}
      else if(mode == "stop"){modeFlag = 0; time1=millis()+1000; Serial.println("stop");}
    }
    request->send(200);
  });

  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);


  server.begin();
}


void loop() {

  //---declare new arrays
  uint16_t* values=new uint16_t[espeedySensorN];
  float* valuesF=new float[espeedySensorN];

  //---get sensor readings and convert to floats
  espeedyReceiveI2C(values);  
  for(int i=0;i<espeedySensorN;i++){
    valuesF[i]=((float)values[i])/4096.0;
  }

  //---set colour pixels at the front to represent intensity from sensors
  espeedyPixel.setPixelColor(0,espeedyPixel.Color(0,0,(int)(valuesF[0]*255.0)));
  espeedyPixel.setPixelColor(1,espeedyPixel.Color(0,0,(int)(valuesF[1]*255.0)));
  espeedyPixel.setPixelColor(2,espeedyPixel.Color(0,0,(int)(valuesF[2]*255.0)));
  espeedyPixel.setPixelColor(3,espeedyPixel.Color(0,0,(int)(valuesF[7]*255.0)));
  espeedyPixel.setPixelColor(4,espeedyPixel.Color(0,0,(int)(valuesF[6]*255.0)));
  espeedyPixel.setPixelColor(5,espeedyPixel.Color(0,0,(int)(valuesF[3]*255.0)));
  espeedyPixel.setPixelColor(6,espeedyPixel.Color(0,0,(int)(valuesF[4]*255.0)));
  espeedyPixel.setPixelColor(7,espeedyPixel.Color(0,0,(int)(valuesF[5]*255.0)));
  
  espeedyPixel.show();

  updateAngle();

  switch(modeFlag){
    case 1: control1(valuesF,angleZ); break;
    case 2: control2(valuesF,angleZ); break;
    case 3: control3(valuesF,angleZ); break;
    case 4: control4(valuesF,angleZ); break;
    case 0: controlStop(); break;
    default: break;
  }

  //Serial.print("Angle X: "); Serial.print(angleX); Serial.print(", ");
  //Serial.print("Angle Y: "); Serial.print(angleY); Serial.print(", ");
  //Serial.print("Angle Z: "); Serial.println(angleZ);

  //angleX=angleX/1.001;
  //angleX=angleY/1.001;
  //angleX=angleZ/1.001;

  //float setPoint;
  //if (Serial.available()){setPoint= Serial.parseFloat();}

  //setPoint=setPoint+constrain((valuesF[6]-valuesF[7])*25.0,-1.0,1.0);
  //setPoint=0;
  
  //int8_t error=(int8_t)constrain((angleZ-setPoint),-10,10);
  //espeedySendI2C(7,((uint8_t)-error));
  //espeedySendI2C(8,((uint8_t)error));
  
  //if (Serial.availableForWrite()){Serial.println(setPoint);}

  //if(time1<millis()){setPoint=setPoint+90;time1=millis()+3000;}
  delay(10);

}

void espeedySendI2C(uint8_t reg, uint8_t val){
  Wire.beginTransmission(ESPEEDY_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();  
}

uint8_t espeedyReadI2C(uint8_t reg){
  Wire.beginTransmission(ESPEEDY_ADDR);
  Wire.write(0);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(ESPEEDY_ADDR,1);
  return Wire.read();  
}

void espeedyReceiveI2C(uint16_t* values){
  Wire.requestFrom(ESPEEDY_ADDR,espeedySensorN*2);
  int i=0; uint8_t received[20]={0};
  while(Wire.available()){
    received[i++]=Wire.read();
  }
  for(int k=0;k<espeedySensorN;k++){
    values[k] = (uint16_t)received[k * 2 + 1] | ((uint16_t)received[k * 2] << 8);
  }
  
  //memcpy(values, received, espeedySensorN * 2);
}

GyroData readGyroData() {
  GyroData data;

  Wire.beginTransmission(LSM6DSR_ADDRESS);
  Wire.write(LSM6DSR_GYRO_OUT_X_L);  // Point to the start of the gyro data
  Wire.endTransmission(false);
  Wire.requestFrom(LSM6DSR_ADDRESS, 6);

  data.x = Wire.read() | (Wire.read() << 8);
  data.y = Wire.read() | (Wire.read() << 8);
  data.z = Wire.read() | (Wire.read() << 8);

  return data;
}

void updateAngle() {
  GyroData gyro = readGyroData();

  // Convert the raw data to dps (degrees per second)
  float dpsX = gyro.x * gyroScaleFactor;
  float dpsY = gyro.y * gyroScaleFactor;
  float dpsZ = gyro.z * gyroScaleFactor;

  // Integrate the angular velocities to get the angle
  angleX += dpsX * sampleTime;
  angleY += dpsY * sampleTime;
  angleZ += dpsZ * sampleTime;
}

void configureGyro() {
  Wire.beginTransmission(LSM6DSR_ADDRESS);
  Wire.write(LSM6DSR_CTRL2_G);

  // Setting ODR to 104 Hz and Full Scale to 2000 dps
  byte config = GYRO_ODR_104 | GYRO_FS_2000;
  Wire.write(config);

  Wire.endTransmission();
}

