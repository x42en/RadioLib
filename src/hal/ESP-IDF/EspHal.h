// EspHal_spimaster.h
// ESP-IDF HAL for RadioLib using spi_master driver
// This avoids conflicts with other SPI devices on the same bus

#ifndef ESP_HAL_SPIMASTER_H
#define ESP_HAL_SPIMASTER_H

#include "RadioLib.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_rom_sys.h" // For esp_rom_delay_us

// Arduino-style macros for RadioLib compatibility
#define LOW (0x0)
#define HIGH (0x1)
#define INPUT (GPIO_MODE_INPUT)
#define OUTPUT (GPIO_MODE_OUTPUT)
#define RISING (GPIO_INTR_POSEDGE)
#define FALLING (GPIO_INTR_NEGEDGE)
#define NOP() asm volatile("nop")

/**
 * @brief ESP-IDF HAL for RadioLib using spi_master driver
 * This implementation properly handles SPI transactions and works with shared SPI buses
 */
class EspHal : public RadioLibHal
{
public:
    /**
     * @brief Constructor
     * @param sck SPI clock pin
     * @param miso SPI MISO pin
     * @param mosi SPI MOSI pin
     * @param host SPI host to use (SPI2_HOST or SPI3_HOST)
     */
    EspHal(int8_t sck, int8_t miso, int8_t mosi, spi_host_device_t host = SPI3_HOST)
        : RadioLibHal(INPUT, OUTPUT, LOW, HIGH, RISING, FALLING),
          spiSCK(sck), spiMISO(miso), spiMOSI(mosi), spiHost(host),
          spiDevice(nullptr), busInitialized(false), deviceAdded(false), halInitialized(false)
    {
    }

    virtual ~EspHal()
    {
        term();
    }

    void init() override
    {
        if (!halInitialized)
        {
            spiBegin();
            halInitialized = true;
        }
    }

    void term() override
    {
        if (halInitialized)
        {
            spiEnd();
            halInitialized = false;
        }
    }

    // GPIO operations
    void pinMode(uint32_t pin, uint32_t mode) override
    {
        if (pin == RADIOLIB_NC)
        {
            return;
        }

        gpio_config_t conf = {
            .pin_bit_mask = (1ULL << pin),
            .mode = (mode == OUTPUT) ? GPIO_MODE_OUTPUT : GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&conf);
    }

    void digitalWrite(uint32_t pin, uint32_t value) override
    {
        if (pin == RADIOLIB_NC)
        {
            return;
        }
        gpio_set_level((gpio_num_t)pin, value);
    }

    uint32_t digitalRead(uint32_t pin) override
    {
        if (pin == RADIOLIB_NC)
        {
            return 0;
        }
        return gpio_get_level((gpio_num_t)pin);
    }

    void attachInterrupt(uint32_t interruptNum, void (*interruptCb)(void), uint32_t mode) override
    {
        if (interruptNum == RADIOLIB_NC)
        {
            return;
        }

        gpio_set_intr_type((gpio_num_t)interruptNum, (gpio_int_type_t)mode);
        gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
        gpio_isr_handler_add((gpio_num_t)interruptNum, (gpio_isr_t)interruptCb, nullptr);
    }

    void detachInterrupt(uint32_t interruptNum) override
    {
        if (interruptNum == RADIOLIB_NC)
        {
            return;
        }
        gpio_isr_handler_remove((gpio_num_t)interruptNum);
    }

    // Timing functions
    void delay(unsigned long ms) override
    {
        vTaskDelay(pdMS_TO_TICKS(ms));
    }

    void delayMicroseconds(unsigned long us) override
    {
        esp_rom_delay_us(us);
    }

    unsigned long millis() override
    {
        return (unsigned long)(esp_timer_get_time() / 1000ULL);
    }

    unsigned long micros() override
    {
        return (unsigned long)esp_timer_get_time();
    }

    long pulseIn(uint32_t pin, uint32_t state, unsigned long timeout) override
    {
        if (pin == RADIOLIB_NC)
        {
            return 0;
        }

        unsigned long startMicros = micros();
        unsigned long currentMicros;

        // Wait for pulse to start
        while (digitalRead(pin) != state)
        {
            currentMicros = micros();
            if (currentMicros - startMicros >= timeout)
            {
                return 0;
            }
        }

        // Measure pulse duration
        unsigned long pulseStart = micros();
        while (digitalRead(pin) == state)
        {
            currentMicros = micros();
            if (currentMicros - pulseStart >= timeout)
            {
                return 0;
            }
        }

        return micros() - pulseStart;
    }

    // SPI operations using spi_master driver
    void spiBegin()
    {
        ESP_LOGD("EspHal", "Initializing SPI on host %d (SCK=%d, MISO=%d, MOSI=%d)",
                 spiHost, spiSCK, spiMISO, spiMOSI);

        // Try to initialize the bus - it may already be initialized by another component
        spi_bus_config_t bus_config = {};
        bus_config.mosi_io_num = spiMOSI;
        bus_config.miso_io_num = spiMISO;
        bus_config.sclk_io_num = spiSCK;
        bus_config.quadwp_io_num = -1;
        bus_config.quadhd_io_num = -1;
        bus_config.data4_io_num = -1;
        bus_config.data5_io_num = -1;
        bus_config.data6_io_num = -1;
        bus_config.data7_io_num = -1;
        bus_config.max_transfer_sz = 4096;
        bus_config.flags = 0;
        bus_config.isr_cpu_id = ESP_INTR_CPU_AFFINITY_AUTO;
        bus_config.intr_flags = 0;

        esp_err_t ret = spi_bus_initialize(spiHost, &bus_config, SPI_DMA_CH_AUTO);
        if (ret == ESP_OK)
        {
            busInitialized = true;
            ESP_LOGD("EspHal", "SPI bus initialized successfully");
        }
        else if (ret == ESP_ERR_INVALID_STATE)
        {
            // Bus already initialized - this is OK for shared bus
            busInitialized = false; // We didn't init it, so we won't free it
            ESP_LOGD("EspHal", "SPI bus already initialized (shared bus)");
        }
        else
        {
            ESP_LOGE("EspHal", "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        }
    }

    void spiBeginTransaction()
    {
        // Acquire the SPI bus for this device
        if (spiDevice != nullptr)
        {
            spi_device_acquire_bus(spiDevice, portMAX_DELAY);
        }
    }

    uint8_t spiTransferByte(uint8_t b)
    {
        if (spiDevice == nullptr)
        {
            ESP_LOGE("EspHal", "SPI device not initialized!");
            return 0xFF;
        }

        spi_transaction_t trans = {};
        trans.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
        trans.length = 8; // 8 bits
        trans.tx_data[0] = b;

        esp_err_t ret = spi_device_polling_transmit(spiDevice, &trans);
        if (ret != ESP_OK)
        {
            ESP_LOGE("EspHal", "SPI transfer failed: %s", esp_err_to_name(ret));
            return 0xFF;
        }

        uint8_t received = trans.rx_data[0];

        // Debug log for first few transfers
        static int transfer_count = 0;
        if (transfer_count < 20)
        {
            ESP_LOGD("EspHal", "SPI: TX=0x%02X RX=0x%02X", b, received);
            transfer_count++;
        }

        return received;
    }

    void spiTransfer(uint8_t *out, size_t len, uint8_t *in)
    {
        if (spiDevice == nullptr)
        {
            ESP_LOGE("EspHal", "SPI device not initialized!");
            return;
        }

        if (len == 0)
        {
            return;
        }

        spi_transaction_t trans = {};
        trans.length = len * 8; // length in bits
        trans.tx_buffer = out;
        trans.rx_buffer = in;

        esp_err_t ret = spi_device_polling_transmit(spiDevice, &trans);
        if (ret != ESP_OK)
        {
            ESP_LOGE("EspHal", "SPI transfer failed: %s", esp_err_to_name(ret));
        }
    }

    void spiEndTransaction()
    {
        // Release the SPI bus
        if (spiDevice != nullptr)
        {
            spi_device_release_bus(spiDevice);
        }
    }

    void spiEnd()
    {
        // Remove device from bus
        if (deviceAdded && spiDevice != nullptr)
        {
            spi_bus_remove_device(spiDevice);
            spiDevice = nullptr;
            deviceAdded = false;
            ESP_LOGD("EspHal", "SPI device removed");
        }

        // Free the bus only if we initialized it
        if (busInitialized)
        {
            spi_bus_free(spiHost);
            busInitialized = false;
            ESP_LOGD("EspHal", "SPI bus freed");
        }
    }

    /**
     * @brief Add SPI device with CS pin
     * Must be called after init() and before using SPI
     * @param csPin Chip select pin
     * @param clockHz SPI clock frequency in Hz
     */
    bool addSpiDevice(int8_t csPin, uint32_t clockHz = 500000)
    {
        if (csPin == RADIOLIB_NC)
        {
            ESP_LOGE("EspHal", "Invalid CS pin");
            return false;
        }

        spi_device_interface_config_t dev_config = {};
        dev_config.command_bits = 0;
        dev_config.address_bits = 0;
        dev_config.dummy_bits = 0;
        dev_config.mode = 0; // SPI mode 0 (CPOL=0, CPHA=0)
        dev_config.clock_source = SPI_CLK_SRC_DEFAULT;
        dev_config.duty_cycle_pos = 128; // 50% duty cycle
        dev_config.cs_ena_pretrans = 0;
        dev_config.cs_ena_posttrans = 0;
        dev_config.clock_speed_hz = (int)clockHz;
        dev_config.input_delay_ns = 0;
        dev_config.spics_io_num = csPin;
        dev_config.flags = 0;
        dev_config.queue_size = 1;
        dev_config.pre_cb = nullptr;
        dev_config.post_cb = nullptr;

        esp_err_t ret = spi_bus_add_device(spiHost, &dev_config, &spiDevice);
        if (ret != ESP_OK)
        {
            ESP_LOGE("EspHal", "Failed to add SPI device: %s", esp_err_to_name(ret));
            return false;
        }

        deviceAdded = true;
        ESP_LOGD("EspHal", "SPI device added (CS=%d, clock=%u Hz)", csPin, clockHz);
        return true;
    }

private:
    int8_t spiSCK;
    int8_t spiMISO;
    int8_t spiMOSI;
    spi_host_device_t spiHost;
    spi_device_handle_t spiDevice;
    bool busInitialized;
    bool deviceAdded;
    bool halInitialized;
};

#endif // ESP_HAL_SPIMASTER_H
