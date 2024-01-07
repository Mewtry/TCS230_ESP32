/**************************************************************************/
/**
 * @file    TCS230_ESP32.cpp
 * @author  Theo Pires
 * @date    08/10/2023
 * @see     www.linkedin.com/in/theo-pires-a34b33183/
 * @brief   Source file for TCS230_ESP32 library
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

#include <Arduino.h>
#include <TCS230_ESP32.h>

// Set to 1 to enable debug output
#define DEBUG_TCS230 0

#if  DEBUG_TCS230
#define DUMP(s, v)  { Serial.print(F("TCS230: ")); Serial.print(F(s)); Serial.print(v); }  // s - info | v - value
#define DUMPS(s)    { Serial.print(F("TCS230: ")); Serial.print(F(s)); }           
#else
#define DUMP(s, v)  
#define DUMPS(s)    
#endif

/**************************************************************************/
/**
 * @brief   ISR for pulse counter
 * @param   data    Pointer to class instance
*/
/**************************************************************************/
void IRAM_ATTR TCS230::pulseCounterIntr(void * data){
    TCS230* dt = (TCS230*)data;
    if(dt->_pulseCounter < 4000000000){
        dt->_pulseCounter += 1;
    }
} // end pulseCounterIntr

/**************************************************************************/
/**
 * @brief   Internal function to initialize class variables with default 
 *          values
*/
/**************************************************************************/
void TCS230::initialize(void) {
    _OE  = NO_PIN;
    _S0  = NO_PIN;
    _S1  = NO_PIN;
    _S2  = NO_PIN;
    _S3  = NO_PIN;
    _OUT = GPIO_NUM_NC;
    _readTime = 100;
    _pulseCounter = 0;
    _freqSet = TCS230_FREQ_HI;
    _readState = TCS230_READY;

    for (uint8_t i=0; i<RGB_SIZE; i++) {
        _fd.value[i] = 4000L;  // just typical values
        _fw.value[i] = 50000L; // just typical values
    }
} // end initialize

/**************************************************************************/
/**
 * @brief   Constructor for TCS230 class
 * @param   out     GPIO pin for output frequency
 * @param   s2      GPIO pin for S2 frequency scaler
 * @param   s3      GPIO pin for S3 frequency scaler
*/
/**************************************************************************/
TCS230::TCS230(gpio_num_t out, uint8_t s2, uint8_t s3) {
    initialize();
    _OUT = out;
    _S2  = s2;
    _S3  = s3;
} // end constructor

/**************************************************************************/
/**
 * @brief   Constructor for TCS230 class
 * @param   out     GPIO pin for output frequency
 * @param   s2      GPIO pin for S2 frequency scaler
 * @param   s3      GPIO pin for S3 frequency scaler
 * @param   oe      GPIO pin for output enable
*/
/**************************************************************************/
TCS230::TCS230(gpio_num_t out, uint8_t s2, uint8_t s3, uint8_t oe) {
    initialize();
    _OUT = out;
    _S2  = s2;
    _S3  = s3;
    _OE  = oe;
} // end constructor

/**************************************************************************/
/**
 * @brief   Constructor for TCS230 class
 * @param   out     GPIO pin for output frequency
 * @param   s2      GPIO pin for S2 frequency scaler
 * @param   s3      GPIO pin for S3 frequency scaler
 * @param   s0      GPIO pin for S0 frequency scaler
 * @param   s1      GPIO pin for S1 frequency scaler
*/
/**************************************************************************/
TCS230::TCS230(gpio_num_t out, uint8_t s2, uint8_t s3, uint8_t s0, uint8_t s1) {
    initialize();
    _OUT = out;
    _S0  = s0;
    _S1  = s1;
    _S2  = s2;
    _S3  = s3;
} // end constructor

/**************************************************************************/
/**
 * @brief   Constructor for TCS230 class
 * @param   out     GPIO pin for output frequency
 * @param   s2      GPIO pin for S2 frequency scaler
 * @param   s3      GPIO pin for S3 frequency scaler
 * @param   s0      GPIO pin for S0 frequency scaler
 * @param   s1      GPIO pin for S1 frequency scaler
 * @param   oe      GPIO pin for output enable
*/
/**************************************************************************/
TCS230::TCS230(gpio_num_t out, uint8_t s2, uint8_t s3, uint8_t s0, uint8_t s1, uint8_t oe) {
    initialize();
    _OUT = out;
    _S0  = s0;
    _S1  = s1;
    _S2  = s2;
    _S3  = s3;
    _OE  = oe;
} // end constructor

/**************************************************************************/
/**
 * @brief   Destructor for TCS230 class
*/
/**************************************************************************/
TCS230::~TCS230(void) {}

/**************************************************************************/
/**
 * @brief   Configure and initialize the IO's and ISR for the sensor
*/
/**************************************************************************/
void TCS230::begin() {
    if (_S0 != NO_PIN) pinMode(_S0, OUTPUT);
    if (_S1 != NO_PIN) pinMode(_S1, OUTPUT);
    if (_S2 != NO_PIN) pinMode(_S2, OUTPUT);
    if (_S3 != NO_PIN) pinMode(_S3, OUTPUT);
    if (_OE != NO_PIN) pinMode(_OE, OUTPUT);
    if (_OUT != GPIO_NUM_NC) {
        gpio_config_t io_conf {
            .pin_bit_mask  = 1ULL<<_OUT,
            .mode          = GPIO_MODE_INPUT,
            .pull_up_en    = GPIO_PULLUP_DISABLE,
            .pull_down_en  = GPIO_PULLDOWN_DISABLE,
            .intr_type     = GPIO_INTR_POSEDGE
        };
        ESP_ERROR_CHECK(gpio_config(&io_conf));
        //(void)gpio_install_isr_service(); // ignore errors as it could be already installed
        ESP_ERROR_CHECK(gpio_isr_handler_add(_OUT, pulseCounterIntr, this));
        gpio_intr_disable(_OUT);
    }

    setEnable(false);
    setFrequencyInternal(_freqSet);


    DUMPS("\nLibrary begin initialized");
} // end begin

/**************************************************************************/
/**
 * @brief   Set the photodiode filter
 * @param   f   Filter option
*/
/**************************************************************************/
void TCS230::setFilter(uint8_t f) {
    if ((_S2 == NO_PIN) || (_S3 == NO_PIN))
        return;

    DUMPS("\nsetFilter ");
    switch (f)
    {
        case TCS230_RGB_R:  DUMPS("R"); digitalWrite(_S2, LOW);   digitalWrite(_S3, LOW);   break;
        case TCS230_RGB_G:  DUMPS("G"); digitalWrite(_S2, HIGH);  digitalWrite(_S3, HIGH);  break;
        case TCS230_RGB_B:  DUMPS("B"); digitalWrite(_S2, LOW);   digitalWrite(_S3, HIGH);  break;
        case TCS230_RGB_X:  DUMPS("X"); digitalWrite(_S2, HIGH);  digitalWrite(_S3, LOW);   break;
        default:  DUMP("\nUnknown filter option ", f); break;
    }
} // end setFilter

/**************************************************************************/
/**
 * @brief   Internal function to set the frequency prescaler
 * @param   f   Frequency option
*/
/**************************************************************************/
void TCS230::setFrequencyInternal(uint8_t f) {
    if ((_S0 == NO_PIN) || (_S1 == NO_PIN)) {
        DUMPS("\nPins not set properly for this function");
        return;
    }

    DUMPS("\nsetFrequency ");
    switch (f)
    {
        case TCS230_FREQ_HI:  DUMPS("HI");  digitalWrite(_S0, HIGH);  digitalWrite(_S1, HIGH);  break;
        case TCS230_FREQ_MID: DUMPS("MID"); digitalWrite(_S0, HIGH);  digitalWrite(_S1, LOW);   break;
        case TCS230_FREQ_LO:  DUMPS("LO");  digitalWrite(_S0, LOW);   digitalWrite(_S1, HIGH);  break;
        case TCS230_FREQ_OFF: DUMPS("OFF"); digitalWrite(_S0, LOW);   digitalWrite(_S1, LOW);   break;
        default:  DUMP("\nUnknown freq option ", f);	break;
    }
} // end setFrequencyInternal

/**************************************************************************/
/**
 * @brief   Set the frequency prescaler
 * @param   f   Frequency option
*/
/**************************************************************************/
void TCS230::setFrequency(uint8_t f) {
    _freqSet = f;
    setFrequencyInternal(f);
} // end setFrequency

/**************************************************************************/
/**
 * @brief   Set the sampling time
 * @param   t   Sampling time in ms
*/
/**************************************************************************/
void TCS230::setSampling(uint32_t t) {
    if (_readTime > 0 && _readTime <= 1000)
        _readTime = t;
} // end setSampling

/**************************************************************************/
/**
 * @brief   Enable or disable the sensor
 * @param   b   Enable or disable
*/
/**************************************************************************/
void TCS230::setEnable(bool b) {
    if(b) {
        DUMPS("\nEnabling using ");
    }
    else {
        DUMPS("\nDisabling using ");
    }
    
    if (_OE != NO_PIN) {
        DUMPS("OE");
        digitalWrite(_OE, (b) ? LOW : HIGH);	// reverse logic
    }
    else {
        DUMPS("FREQ");
        setFrequencyInternal((b) ? _freqSet : TCS230_FREQ_OFF);
    }
} // end setEnable

/**************************************************************************/
/**
 * @brief   Set the dark calibration data
 * @param   d   Pointer to sensorData structure
*/
/**************************************************************************/
void TCS230::setDarkCal(sensorData *d) {
    if (d == NULL)
        return;

    DUMPS("\nsetDarkCal\n");
    for (uint8_t i=0; i<RGB_SIZE; i++) {
        DUMP(" ", d->value[i]);
        _fd.value[i] = d->value[i];
    }
} // end setDarkCal

/**************************************************************************/
/**
 * @brief   Perform the dark calibration, first reading the sensor and then
 *          setting the dark calibration data.
 * @param   d   Pointer to sensorData structure
*/
/**************************************************************************/
void TCS230::darkCalibration(sensorData *d) {
    DUMPS("\ndarkCalibration\n");
    read();
    setDarkCal(&_fo);
    for (uint8_t i=0; i<RGB_SIZE; i++) {
        DUMP(" ", _fo.value[i]);
        d->value[i] = _fo.value[i];
    }
} // end darkCalibration

/**************************************************************************/
/**
 * @brief   Set the white calibration data
 * @param   d   Pointer to sensorData structure
*/
/**************************************************************************/
void TCS230::setWhiteCal(sensorData *d) {
    if (d == NULL)
        return;

    DUMPS("\nsetWhiteCal\n");
        for (uint8_t i=0; i<RGB_SIZE; i++) {
        DUMP(" ", d->value[i]);
        _fw.value[i] = d->value[i];
    }
} // end setWhiteCal

/**************************************************************************/
/**
 * @brief   Perform the white calibration, first reading the sensor and then
 *          setting the white calibration data.
 * @param   d   Pointer to sensorData structure
*/
/**************************************************************************/
void TCS230::whiteCalibration(sensorData *d) {
    DUMPS("\nwhiteCalibration\n");
    read();
    setWhiteCal(&_fo);
    for (uint8_t i=0; i<RGB_SIZE; i++) {
        DUMP(" ", _fo.value[i]);
        d->value[i] = _fo.value[i];
    }
} // end whiteCalibration

/**************************************************************************/
/**
 * @brief   Get the current RGB data from the sensor
 * @param   rgb     Pointer to colorData structure
*/
/**************************************************************************/
void TCS230::getRGB(colorData *rgb) {
    if (rgb == NULL)
        return;

    DUMPS("\ngetRGB\n");
    for (uint8_t i=0; i<RGB_SIZE; i++) {
        rgb->value[i] = _rgb.value[i];
        DUMP(" ", rgb->value[i]);
    }
} // end getRGB

/**************************************************************************/
/**
 * @brief   Get the current raw data from the sensor
 * @param   d     Pointer to sensorData structure
 * @note    Useful to set dark and white calibration data
*/
/**************************************************************************/
void TCS230::getRaw(sensorData *d) {
    if (d == NULL)
        return;

    DUMPS("\ngetRAW\n");
    for (uint8_t i=0; i<RGB_SIZE; i++) {
        d->value[i] = _fo.value[i];
        DUMP(" ", d->value[i]);
    }
} // end getRaw

/**************************************************************************/
/**
 * @brief   Get the current red RGB value from the sensor
 * @return  Red RBG value
*/
/**************************************************************************/
uint8_t TCS230::getRed(void) {
    DUMP("\ngetRed ", _rgb.value[RED]);
    return _rgb.value[RED];
} // end getRed

/**************************************************************************/
/**
 * @brief   Get the current green RGB value from the sensor
 * @return  Green RBG value
*/
/**************************************************************************/
uint8_t TCS230::getGreen(void) {
    DUMP("\ngetGreen ", _rgb.value[GREEN]);
    return _rgb.value[GREEN];
} // end getGreen

/**************************************************************************/
/**
 * @brief   Get the current blue RGB value from the sensor
 * @return  Blue RBG value
*/
/**************************************************************************/
uint8_t TCS230::getBlue(void) {
    DUMP("\ngetBlue ", _rgb.value[BLUE]);
    return _rgb.value[BLUE];
} // end getBlue

/**************************************************************************/
/**
 * @brief   Get the current red RAW data from the sensor
 * @return  Red RAW value
*/
/**************************************************************************/
uint32_t TCS230::getRawRed(void) {
    DUMP("\ngetRawRed ", _fo.value[RED]);
    return _fo.value[RED];
} // end getRawRed

/**************************************************************************/
/**
 * @brief   Get the current green RAW data from the sensor
 * @return  Green RAW value
*/
/**************************************************************************/
uint32_t TCS230::getRawGreen(void) {
    DUMP("\ngetRawGreen ", _fo.value[GREEN]);
    return _fo.value[GREEN];
} // end getRawGreen

/**************************************************************************/
/**
 * @brief   Get the current blue RAW data from the sensor
 * @return  Blue RAW value
*/
/**************************************************************************/
uint32_t TCS230::getRawBlue(void) {
    DUMP("\ngetRawBlue ", _fo.value[BLUE]);
    return _fo.value[BLUE];
} // end getRawBlue

/**************************************************************************/
/**
 * @brief   Get the current color ID from the sensor
 * @return  Color ID
*/
/**************************************************************************/
uint8_t TCS230::getColor(void) {
    DUMP("\ngetColor ", _color);
    return _color;
} // end getColor

/**************************************************************************/
/**
 * @brief   Get the current color name from the sensor
 * @return  Color name (char *)
*/
/**************************************************************************/
char * TCS230::getColorToString(void) {
    DUMPS("\ngetColorToString ");
    switch (_color) {
        case BLACK: DUMPS("BLACK");   return "BLACK";
        case WHITE: DUMPS("WHITE");   return "WHITE";
        case RED:   DUMPS("RED");     return "RED";
        case GREEN: DUMPS("GREEN");   return "GREEN";
        case BLUE:  DUMPS("BLUE");    return "BLUE";
        case GRAY:  DUMPS("GRAY");    return "GRAY";
        default:    DUMPS("UNKNOWN"); return "UNKNOWN";
    }
} // end getColorToString

/**************************************************************************/
/**
 * @brief   Set the dark sensitive value
 * @param   d   Dark sensitive value
 * @note    The value must be between 0 and 255
*/
/**************************************************************************/
void TCS230::setDarkSensitive(uint8_t d) {
    DUMP("\nsetDarkSensitive ", d);
    if(d > 0 && d < 255)
        _ds = d;
} // end setDarkSensitive

/**************************************************************************/
/**
 * @brief   Set the white sensitive value
 * @param   d   White sensitive value
 * @note    The value must be between 0 and 765
*/
/**************************************************************************/
void TCS230::setWhiteSensitive(uint16_t d) {
    DUMP("\nsetWhiteSensitive ", d);
    if(d > 0 && d < 765)
        _ws = d;
} // end setWhiteSensitive

/**************************************************************************/
/**
 * @brief   Read a single RAW sensor value (ie, not rgb)
 * @return  Raw Sensor value
 * @note    This function is blocking
*/
/**************************************************************************/
uint32_t TCS230::readSingle(void) {
    DUMPS("\nreadSingle");
    _readState = TCS230_READING;
    _pulseCounter = 0;
    setEnable(true);
    gpio_intr_enable(_OUT);  // Habilita interrupção
    vTaskDelay(_readTime / portTICK_PERIOD_MS);
    setEnable(false);
    gpio_intr_disable(_OUT); // Desabilita interrupção
    _readState = TCS230_READY;
    return ( 1000 * _pulseCounter / _readTime);
} // end readSingle

/**************************************************************************/
/**
 * @brief   Read the RGB sensor data and storage in the class variables
 * @note    This function is blocking
*/
/**************************************************************************/
void TCS230::read(void) {
    DUMPS("\nread");
    _readState = TCS230_READING;
    for(uint8_t i=0; i<RGB_SIZE; i++) {
        _pulseCounter = 0;
        setFilter(i);
        setEnable(true);
        gpio_intr_enable(_OUT);
        vTaskDelay(_readTime / portTICK_PERIOD_MS);
        setEnable(false);
        gpio_intr_disable(_OUT);
        _fo.value[i] = 1000 * _pulseCounter / _readTime;
    }
    _readState = TCS230_READY;
    RGBTransformation();
    _color = ColorID();
} // end read

/**************************************************************************/
/**
 * @brief   Check if the sensor data is available
 * @return  True if the sensor data is available or false otherwise
*/
/**************************************************************************/
bool TCS230::available(void) {
    DUMP("\navailable ", _readState);
    return (_readState == 0);
} // end available

/**************************************************************************/
/**
 * @brief   Internal function to convert the raw data structure to rgb data 
 *          structure
*/
/**************************************************************************/
void TCS230::RGBTransformation(void) {
    int32_t x;

    for (uint8_t i=0; i<RGB_SIZE; i++) {
        // Famosa regra de 3
        x = (_fo.value[i] - _fd.value[i]) * 255;
        x /= (_fw.value[i] - _fd.value[i]);

        // copia o resultado para as estruturas globais
        if (x < 0) _rgb.value[i] = 0; 
        else if (x > 255) _rgb.value[i] = 255;
        else _rgb.value[i] = x;
    }
} // end RGBTransformation

/**************************************************************************/
/**
 * @brief   Internal function to get the color ID from rgb data structure
 * @return  Color ID
 * @note    The color ID is based on the RGB data structure and the dark and
 *          white sensitive values
*/
/**************************************************************************/
uint8_t TCS230::ColorID(void) {
    int total = _rgb.value[RED]+_rgb.value[GREEN]+_rgb.value[BLUE];

    if (total < _ds)
        return BLACK;
    else if (total > _ws)
        return WHITE;
    else if (_rgb.value[RED] > _rgb.value[GREEN] && _rgb.value[RED] > _rgb.value[BLUE])
        return RED;
    else if (_rgb.value[GREEN] > _rgb.value[RED] && _rgb.value[GREEN] > _rgb.value[BLUE])
        return GREEN;
    else if (_rgb.value[BLUE] > _rgb.value[RED] && _rgb.value[BLUE] > _rgb.value[GREEN])
        return BLUE;
    else
        return GRAY;
} // end ColorID
