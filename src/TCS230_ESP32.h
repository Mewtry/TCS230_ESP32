/**************************************************************************/
/**
 * @file    TCS230_ESP32.h
 * @author  Theo Pires
 * @date    08/10/2023
 * @see     www.linkedin.com/in/theo-pires-a34b33183/
 * @brief   Header file for TCS230_ESP32 library
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
*/
/**************************************************************************/
#ifndef _TCS230_ESP32_H
#define _TCS230_ESP32_H

#include <Arduino.h>

/********************* TCS230 Configurações **********************/    
// S0  |S1 |Output Freq Scaling | |S2  |S3 |Photodiode Selection  |
// :--:|--:|:-------------------| |:--:|--:|:---------------------|
// L   |L  |Power Down          | |L   |L  |Red                   |
// L   |H  |2%                  | |L   |H  |Blue                  |
// H   |L  |20%                 | |H   |L  |Clear (no filter)     |
// H   |H  |100%                | |H   |H  |Green                 |
/*****************************************************************/

// Frequency setting defines
enum id_freq {TCS230_FREQ_HI, TCS230_FREQ_MID, TCS230_FREQ_LO, TCS230_FREQ_OFF};
// Indices for RGB data and filter selection
enum id_rgb_colors {TCS230_RGB_R, TCS230_RGB_G, TCS230_RGB_B, TCS230_RGB_X};
// Indices for reading colors response
enum id_colors {RED, GREEN, BLUE, BLACK, WHITE, GRAY};
// States of class
enum id_states {TCS230_READY, TCS230_READING};

const uint8_t RGB_SIZE = 3;  // Limit index of RGB components
const uint8_t NO_PIN = 0xff; // Default value for pins that not defined

// Sensor data structure type. Contains the RGB raw data from sensor.
typedef struct {
    int32_t value[RGB_SIZE];  
} sensorData;

// RGB data structure type. Contains de RGB data from the sensor
typedef struct {
    uint8_t value[RGB_SIZE];
} colorData;

class TCS230 {
    private:
    uint8_t _S0;            // frequency scaler S0 pin
    uint8_t _S1;            // frequency scaler S1 pin
    uint8_t _S2;            // photodiode filter selection S2 pin
    uint8_t _S3;            // photodiode filter selection S3 pin
    uint8_t _OE;            // output enable pin
    gpio_num_t _OUT;        // output frequency pin
    uint32_t _readTime;     // time of sampling in ms
    uint8_t _freqSet;       // current frequency setting
    uint8_t _readState;     // state of class reader
    
    volatile uint32_t _pulseCounter; // pulse counter of frequency output from sensor
    
    uint8_t _ds = 150;      // Limite para entender como preto em comparação ao total entre os valores RGB
    uint16_t _ws = 500;     // Limite para entender como branco em comparação ao total entre os valores RGB

    sensorData _fd;         // dark calibration parameters raw data
    sensorData _fw;         // white calibration parameters raw data

    sensorData _fo;         // current raw data from sensor reading
    colorData _rgb;         // current rgb data from sensor reading
    uint8_t _color;         // current color ID from sensor reading

    void initialize(void);                     // initialize variables 
    void RGBTransformation(void);              // convert the raw data structure to rgb data structure
    void setFrequencyInternal(uint8_t f);      // internal function for frequency prescaler
    static void pulseCounterIntr(void * data); // interrupt handler for pulse counter
    uint8_t ColorID(void);                     // get the color ID from rgb data structure

    public:
    // Constructors
    TCS230(gpio_num_t out, uint8_t s2, uint8_t s3);
    TCS230(gpio_num_t out, uint8_t s2, uint8_t s3, uint8_t oe);
    TCS230(gpio_num_t out, uint8_t s2, uint8_t s3, uint8_t s0, uint8_t s1);
    TCS230(gpio_num_t out, uint8_t s2, uint8_t s3, uint8_t s0, uint8_t s1, uint8_t oe);

    void begin(void);                     // initialize the sensor    
    ~TCS230(void);                        // destructor

    // Methods for hardware and object control
    void setFilter(uint8_t f);            // set the photodiode filter
    void setFrequency(uint8_t f);         // set the frequency prescaler
    void setEnable(bool b);               // set the output enable
    void setSampling(uint32_t t);         // set the sampling time
    void setDarkCal(sensorData *d);       // set the dark calibration data
    void darkCalibration(sensorData *d);  // perform the dark calibration
    void setWhiteCal(sensorData *d);      // set the white calibration data
    void whiteCalibration(sensorData *d); // perform the white calibration
    void setDarkSensitive(uint8_t d);     // set the dark sensitive value
    void setWhiteSensitive(uint16_t d);   // set the white sensitive value

    // Methods for reading sensor data
    void getRGB(colorData *rgb);          // get the rgb value of the current reading
    void getRaw(sensorData *d);           // get the raw data of the current reading
    uint8_t getRed(void);                 // get the red value of the current reading
    uint8_t getGreen(void);               // get the green value of the current reading
    uint8_t getBlue(void);                // get the blue value of the current reading
    uint32_t getRawRed(void);             // get the raw red value of the current reading
    uint32_t getRawGreen(void);           // get the raw green value of the current reading
    uint32_t getRawBlue(void);            // get the raw blue value of the current reading
    uint8_t getColor(void);               // get the color ID of the current reading
    uint32_t readSingle(void);            // read of a single sensor value (ie, not rgb)
    char *  getColorToString(void);       // get the color name of the current reading
    void read(void);                      // read of sensor data
    bool available(void);                 // check if the sensor data is available
};

#endif