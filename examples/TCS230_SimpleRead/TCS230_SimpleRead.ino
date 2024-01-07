#include <TCS230_ESP32.h>

// DEFINES
#define TCS230_S0_PIN  GPIO_NUM_25 // Output frequency scaling S0
#define TCS230_S1_PIN  GPIO_NUM_26 // Output frequency scaling S1
#define TCS230_S2_PIN  GPIO_NUM_27 // Filter selection S2
#define TCS230_S3_PIN  GPIO_NUM_14 // Filter selection S3
#define TCS230_OE_PIN  GPIO_NUM_12 // Output Enable Pin
#define TCS230_OUT_PIN GPIO_NUM_13 // Output Sensor

// Create a TCS230 object passing all pins
TCS230 tcs(
    TCS230_OUT_PIN, 
    TCS230_S2_PIN, 
    TCS230_S3_PIN, 
    TCS230_S0_PIN, 
    TCS230_S1_PIN, 
    TCS230_OE_PIN
);

void setup() {
    Serial.begin(115200);   // Initialize serial communication
    tcs.begin();            // Initialize TCS230 object
}

void loop() {
    tcs.read();  // Read sensor
    Serial.printf("RGB(%d, %d, %d) | COR: %s\n", tcs.getRed(), tcs.getGreen(), tcs.getBlue(), tcs.getColor());
    delay(2000); // Wait 2 seconds
}