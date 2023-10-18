// Include necessary libraries
#include <Wire.h>  // This library allows for I2C communication
#include <Adafruit_NeoPixel.h>  // Library for controlling NeoPixels

// Define constants
#define STM32_ADDRESS 0x01  // I2C address for the slave device
#define NUM_ADC_VALUES 8  // Number of ADC values we expect to handle

// Global variables
uint16_t adcValues[NUM_ADC_VALUES];  // Array to store ADC values
uint32_t color = 0;  // Variable to store color value for NeoPixels
bool isToggledOff = true;   // State variable to keep track if the functionality is toggled on or off
bool lastButtonState = LOW;  // Last observed button state

//enums
enum Motor{left, right};


// Create a NeoPixel strip object
// Arguments are: number of pixels, pin number, and pixel type
Adafruit_NeoPixel strip(8, 48, NEO_GRB + NEO_KHZ800);

void control(float leftSensor, float rightSensor) {
  float error = (leftSensor - rightSensor) * 25.0;

  
  motorSet(0.13 - constrain(error,-0.2,0.2), left);
  motorSet(0.13 + constrain(error,-0.2,0.2), right);

  if (Serial.availableForWrite() > 0) {
    Serial.println(error);
  }
}


void motorSet(float power, Motor motor){
  power=power*127.0;
  int8_t motorBits = (power>0)?map(power,0.0,127.0,5.0,127.0):map(power,-127.0,0.0,-127.0,-5.0);  //remove motor power deadzone (values too small don't move)
  if(motor==left){
    sendI2CData(8, motorBits);
  } else {
    sendI2CData(7, motorBits);
  } 
}

void setup() {
  // Initialize serial communication at 115200 baud
  Serial.begin(115200);
  
  // Initialize I2C communication
  // Here, we define SDA to be on pin 11 and SCL on pin 10
  Wire.begin(11, 10); 
  
  // Delay for a short period to allow the serial connection to establish
  delay(1000); 
  
  // Send a message to the serial monitor indicating the I2C master has initialized
  Serial.println("I2C Master initialized");

  // Initialize the NeoPixel strip
  strip.begin();  // This is required to setup the NeoPixel strip
  
  // Turn off all pixels on the strip as soon as possible
  strip.show();
  
  // Set the brightness level for the NeoPixels
  // The maximum brightness is 255, but here it's set to about 1/5 of that
  strip.setBrightness(30);

  // Send configuration data to the robot over I2C
  // The purpose here is to instruct the robot to start sending ADC data
  sendI2CData(1, 0b00000010);

  pinMode(0, INPUT_PULLUP);  // Set IO0 as input with pull-up
}

/* 
 * This Arduino loop performs the following tasks:
 * 1. Reads ADC values from sensors.
 * 2. Uses these ADC values to set the brightness of individual NeoPixels.
 * 3. Sends the ADC values to a computer using serial communication.
 */

void loop() {
  // 1. Retrieve ADC values from sensors.
  // This function collects data from connected sensors and stores them in the adcValues array.
  requestADCValues();  

  // 2. Set NeoPixel colors using the retrieved ADC values.
  // Each setPixelWithBrightness function call sets the brightness of a NeoPixel based on an ADC value.
  setPixelWithBrightness(0, adcValues[0]);
  setPixelWithBrightness(1, adcValues[1]);
  setPixelWithBrightness(2, adcValues[2]);
  setPixelWithBrightness(3, adcValues[7]);
  setPixelWithBrightness(4, adcValues[6]);
  setPixelWithBrightness(5, adcValues[3]);
  setPixelWithBrightness(6, adcValues[4]);
  setPixelWithBrightness(7, adcValues[5]);

  // After setting the brightness values for all NeoPixels, update them so the changes are displayed.
  strip.show();
/*
  // 3. Send the ADC values to the computer through serial communication.
  // First, check if the Serial port is ready for communication.
  if (Serial.availableForWrite() > 0) {
    // This for-loop iterates over each ADC value and sends it over the serial connection.
    for (int i = 0; i < NUM_ADC_VALUES; i++) {
      Serial.print(adcValues[i]);  // Send the ADC value.

      // If we're not at the last ADC value, send a comma to separate values.
      // Otherwise, send a newline to indicate the end of the data set.
      if (i < NUM_ADC_VALUES - 1) {
        Serial.print(", ");  
      } else {
        Serial.println();  
      }
    }
  }
*/
  bool currentButtonState = !digitalRead(0);  // True if the button is pressed

  // Detect a button press event (rising edge)
  if (currentButtonState && !lastButtonState) {
    isToggledOff = !isToggledOff; // Toggle the state
  }
  lastButtonState = currentButtonState;

  if (isToggledOff) {
    sendI2CData(8, 0);
    sendI2CData(7, 0);
  } else {
    //int8_t error = adcValues[6]-adcValues[7]; 

    //sendI2CData(8, 30-constrain(error,-20,20));
    //sendI2CData(7, 30+constrain(error,-20,20));
    control(((float)adcValues[6])/4096.0,((float)adcValues[7])/4096.0);
  }


  // Introduce a very short delay before the loop starts over. This can help prevent data flooding.
  delay(1);
}


// This function sets the brightness and color of a specific NeoPixel based on an ADC value.
void setPixelWithBrightness(int pixelIndex, int adcValue) {
    
    // Map the ADC value to a hue in the range 0-65535. 
    // This means an adcValue of 0 would be mapped to hue 0, and an adcValue of 4095 would be mapped to hue 65535.
    uint32_t hue = map(adcValue, 0, 4095, 0, 65535);
    
    // Similarly, map the ADC value to a brightness level between 0 and 255.
    uint8_t brightness = map(adcValue, 0, 1000, 0, 255);
    
    // Create a color value using the HSV format, assuming a constant saturation of 255.
    uint32_t color = strip.ColorHSV(hue, 255, brightness);
    
    // Set the computed color for the specified NeoPixel on the strip.
    // The gamma32 function adjusts the color to be more visually linear.
    strip.setPixelColor(pixelIndex, strip.gamma32(color));
}

// This function requests ADC values from the slave device over I2C.
void requestADCValues() {
    
    // Create an array to store the received bytes. Each ADC value will be represented by 2 bytes.
    byte receivedBytes[2 * NUM_ADC_VALUES];
    
    // Send an I2C request to the slave device to get 2 bytes for each ADC value.
    Wire.requestFrom(STM32_ADDRESS, 2 * NUM_ADC_VALUES);
    
    // As long as there's data available over I2C, read and store the bytes.
    int index = 0;
    while (Wire.available()) {
        receivedBytes[index++] = Wire.read();
    }
    
    // Convert the received byte pairs into uint16_t ADC values and store them.
    for (int i = 0; i < NUM_ADC_VALUES; i++) {
        adcValues[i] = (uint16_t)receivedBytes[i * 2 + 1] | ((uint16_t)receivedBytes[i * 2] << 8);
    }
}

// This function sends data over I2C to a specified register of the slave device.
void sendI2CData(byte reg, byte val) {
    
    // Begin an I2C transmission to the slave device with address 0x01.
    Wire.beginTransmission(0x01);
    
    // Send the specified register and value bytes.
    Wire.write(reg);
    Wire.write(val);
    
    // End the I2C transmission.
    Wire.endTransmission();
}
