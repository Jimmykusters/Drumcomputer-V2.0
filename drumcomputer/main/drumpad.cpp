#include "drumpad.hpp"

void drumpad::checkIfPressed()
{
    // Copntrolleer of de drumpad niet al bediend is.
    if (this->currentState == false)
    {
        // Check if the new state is pressed.
        this->newPressedState = (this->newRaw <= DRUMPAD_PRESS_THRESHOLD) ? true : false;
        // Als de vorige en de nieuwe niet gelijk zijn is de status gewijzigd.
        if ((this->newPressedState != this->prevPressedState) && (this->newPressedState == true))
        {
            // De drumpad is bediend.
            this->currentState = true;
        }
        // Zet de vorige state gelijk aan de huidige voor de volgende run.
        this->prevPressedState = this->newPressedState;
    }
}

drumpad::drumpad(/* args */)
{
    // Set all to 0 or false
    this->newRaw = 0;
    this->newPressedState = false;
    this->prevPressedState = false;
    this->currentState = false;
    // Dont forget to set this for each channel using setChannel()
    this->channel = DEFAULT_INPUT_MUX;

    ESP_LOGI(this->TAG, "i'm initialized");
}

drumpad::~drumpad()
{
}

void drumpad::setChannel(ads111x_mux_t channel)
{
    this->channel = channel;
}

ads111x_mux_t drumpad::getChannel(void)
{
    return this->channel;
}

void drumpad::putSample(int16_t raw)
{
    this->newRaw = raw;
    checkIfPressed();
}

bool drumpad::getState()
{
    bool res = this->currentState;
    // The state is read so can now be reseted.
    this->currentState = false;
    return res;
}

uint16_t drumpad::getRaw()
{
    return this->newRaw;
}

// float drumpad::getVoltage()
// {
//     return voltage = gain_val / ADS111X_MAX_VALUE * raw;
// }

void drumpads::sampleADC()
{
    bool busy;
    int16_t rawData;

    for (size_t i = 0; i < this->numberOfDrumpads; i++)
    {
        // Set the correct channel
        ads111x_set_input_mux(&this->device,
                              this->padList[i].getChannel());
        // Start the sampling
        ads111x_start_conversion(&this->device);

        // Wait till adc is done
        do
        {
            ads111x_is_busy(&this->device, &busy);
        } while (busy);

        // Request the value
        if (ads111x_get_value(&this->device, &rawData) == ESP_OK)
        {
            this->padList[i].putSample(rawData);
        }
    }
}

drumpads::drumpads(/* args */)
{
    ESP_ERROR_CHECK(ads111x_init_desc(&this->device,
                                      (uint8_t)DEFAULT_I2C_ADDR,
                                      (i2c_port_t)DEFAULT_I2C_PORT,
                                      (gpio_num_t)DEFAULT_I2C_SDA_PIN,
                                      (gpio_num_t)DEFAULT_I2C_SCL_PIN));

    ESP_ERROR_CHECK(ads111x_set_mode(&this->device, DEFAULT_SAMPLE_MODE));
    ESP_ERROR_CHECK(ads111x_set_data_rate(&this->device, DEFAULT_DATA_RATE));
    ESP_ERROR_CHECK(ads111x_set_input_mux(&this->device, DEFAULT_INPUT_MUX));
    ESP_ERROR_CHECK(ads111x_set_gain(&this->device, DEFAULT_GAIN));
}

drumpads::~drumpads()
{
}

void drumpads::installDrumpads(drumpad *padList, size_t numberOfDrumpads)
{
    this->padList = padList;
    this->numberOfDrumpads = numberOfDrumpads;
}

// void drumpads::installQueue(QueueHandle_t *queue)
// {
//     this->queue = queue;
// }

// void drumpads::handle(void *arg)
// {
//     while (true)
//     {
//         this->sampleADC();

//         vTaskDelay(10 / portTICK_PERIOD_MS);
//     }
// }
