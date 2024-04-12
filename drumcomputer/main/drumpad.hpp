#pragma once
#include "sdkconfig.h"

#include <ads111x.h>

#include "esp_log.h"

#define DEFAULT_DRUMPADS_STACK_SIZE (4096)
// I2C settings
#define DEFAULT_I2C_PORT        1
#define DEFAULT_I2C_ADDR        ADS111X_ADDR_GND
#define DEFAULT_I2C_SDA_PIN     18//CONFIG_EXAMPLE_I2C_MASTER_SDA
#define DEFAULT_I2C_SCL_PIN     5//CONFIG_EXAMPLE_I2C_MASTER_SCL
// Chip settings
#define DEFAULT_SAMPLE_MODE     ADS111X_MODE_SINGLE_SHOT
#define DEFAULT_DATA_RATE       ADS111X_DATA_RATE_860
#define DEFAULT_INPUT_MUX       ADS111X_MUX_0_GND
#define DEFAULT_GAIN            ADS111X_GAIN_4V096
// Drumpad settings
#define DRUMPAD_PRESS_THRESHOLD 5000
#define NUMBER_OF_DRUMPADS      4

class drumpad
{
private:
    const char *TAG = "drumpad";
    // Current state and value of the drumpad
    int16_t newRaw;
    bool newPressedState;
    bool prevPressedState;

    bool currentState;

    ads111x_mux_t channel;

    void checkIfPressed();

public:
    drumpad();
    ~drumpad();

    void setChannel(ads111x_mux_t channel);
    ads111x_mux_t getChannel(void);

    void putSample(int16_t raw);

    bool getState();

    uint16_t getRaw();
    // float getVoltage();
};



class drumpads
{
private:
    const char *TAG = "drumpads";
    i2c_dev_t device;
    size_t numberOfDrumpads;
    // Pointer to the padList
    drumpad* padList;

public:
    drumpads(/* args */);
    ~drumpads();

    void installDrumpads(drumpad* padList, size_t numberOfDrumpads);
    // void installQueue(QueueHandle_t *queue);
    void sampleADC();
    // void handle(void *arg);
};
