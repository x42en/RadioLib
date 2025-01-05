// EspHal.hpp
#pragma once

/**
 * @file EspHal.hpp
 * @author 0x42en (0x42en@gmail.com)
 * @date 2025-01-05
 * @copyright Copyright (c) 2025
 *
 * @brief Provides Hardware Abastract Layer (HAL) interface for ESP32 boards
 *
 */

#ifndef ESP_HAL_H
#define ESP_HAL_H

#include "RadioLib.h"

// include all the dependencies
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/rtc.h"
#include "soc/spi_reg.h"
#include "soc/spi_struct.h"
#include "hal/gpio_hal.h"
#include "hal/clk_gate_ll.h"
#include "driver/gpio.h"
#include "soc/gpio_sig_map.h"

#include "esp_timer.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_intr_alloc.h"

#if CONFIG_IDF_TARGET_ESP32 // ESP32/PICO-D4
#include "soc/dport_reg.h"
#include "esp32/rom/ets_sys.h"
#include "esp32/rom/gpio.h"
#elif CONFIG_IDF_TARGET_ESP32S2
#include "soc/dport_reg.h"
#include "esp32s2/rom/ets_sys.h"
#include "esp32s2/rom/gpio.h"
#elif CONFIG_IDF_TARGET_ESP32S3
#include "soc/dport_reg.h"
#include "esp32s3/rom/ets_sys.h"
#include "esp32s3/rom/gpio.h"
#elif CONFIG_IDF_TARGET_ESP32C2
#include "esp32c2/rom/ets_sys.h"
#include "esp32c2/rom/gpio.h"
#elif CONFIG_IDF_TARGET_ESP32C3
#include "esp32c3/rom/ets_sys.h"
#include "esp32c3/rom/gpio.h"
#elif CONFIG_IDF_TARGET_ESP32C6
#include "esp32c6/rom/ets_sys.h"
#include "esp32c6/rom/gpio.h"
#elif CONFIG_IDF_TARGET_ESP32H2
#include "esp32h2/rom/ets_sys.h"
#include "esp32h2/rom/gpio.h"
#elif CONFIG_IDF_TARGET_ESP32P4
#include "esp32p4/rom/ets_sys.h"
#include "esp32p4/rom/gpio.h"
#else
#error Target CONFIG_IDF_TARGET is not supported
#endif

#if CONFIG_IDF_TARGET_ESP32S2
#define usr_mosi_dbitlen usr_mosi_bit_len
#define usr_miso_dbitlen usr_miso_bit_len
#elif !defined(CONFIG_IDF_TARGET_ESP32)
#define usr_mosi_dbitlen ms_data_bitlen
#define usr_miso_dbitlen ms_data_bitlen
#define mosi_dlen ms_dlen
#define miso_dlen ms_dlen
#endif

// define Arduino-style macros
#define LOW (0x0)
#define HIGH (0x1)
#define INPUT (0x01)
#define OUTPUT (0x03)
#define RISING (0x01)
#define FALLING (0x02)
#define NOP() asm volatile("nop")

// Should be included with soc/gpio_sig_map.h
// ... but it's not!
#define HSPICLK_OUT_IDX 8
#define HSPIQ_OUT_IDX 9
#define HSPID_IN_IDX 10

#define MATRIX_DETACH_OUT_SIG (0x100)
#define MATRIX_DETACH_IN_LOW_PIN (0x30)

// all of the following is needed to calculate SPI clock divider
#define ClkRegToFreq(reg) (apb_freq / (((reg)->clkdiv_pre + 1) * ((reg)->clkcnt_n + 1)))

typedef union
{
    uint32_t value;
    struct
    {
        uint32_t clkcnt_l : 6;
        uint32_t clkcnt_h : 6;
        uint32_t clkcnt_n : 6;
        uint32_t clkdiv_pre : 13;
        uint32_t clk_equ_sysclk : 1;
    };
} spiClk_t;

uint32_t getApbFrequency()
{
    rtc_cpu_freq_config_t conf;
    rtc_clk_cpu_freq_get_config(&conf);

    if (conf.freq_mhz >= 80)
    {
        return (80 * MHZ);
    }

    return ((conf.source_freq_mhz * MHZ) / conf.div);
}

uint32_t spiFrequencyToClockDiv(uint32_t freq)
{
    uint32_t apb_freq = getApbFrequency();
    if (freq >= apb_freq)
    {
        return SPI_CLK_EQU_SYSCLK;
    }

    const spiClk_t minFreqReg = {0x7FFFF000};
    uint32_t minFreq = ClkRegToFreq((spiClk_t *)&minFreqReg);
    if (freq < minFreq)
    {
        return minFreqReg.value;
    }

    uint8_t calN = 1;
    spiClk_t bestReg = {0};
    int32_t bestFreq = 0;
    while (calN <= 0x3F)
    {
        spiClk_t reg = {0};
        int32_t calFreq;
        int32_t calPre;
        int8_t calPreVari = -2;

        reg.clkcnt_n = calN;

        while (calPreVari++ <= 1)
        {
            calPre = (((apb_freq / (reg.clkcnt_n + 1)) / freq) - 1) + calPreVari;
            if (calPre > 0x1FFF)
            {
                reg.clkdiv_pre = 0x1FFF;
            }
            else if (calPre <= 0)
            {
                reg.clkdiv_pre = 0;
            }
            else
            {
                reg.clkdiv_pre = calPre;
            }
            reg.clkcnt_l = ((reg.clkcnt_n + 1) / 2);
            calFreq = ClkRegToFreq(&reg);
            if (calFreq == (int32_t)freq)
            {
                memcpy(&bestReg, &reg, sizeof(bestReg));
                break;
            }
            else if (calFreq < (int32_t)freq)
            {
                if (RADIOLIB_ABS(freq - calFreq) < RADIOLIB_ABS(freq - bestFreq))
                {
                    bestFreq = calFreq;
                    memcpy(&bestReg, &reg, sizeof(bestReg));
                }
            }
        }
        if (calFreq == (int32_t)freq)
        {
            break;
        }
        calN++;
    }
    return (bestReg.value);
}

// create a new ESP-IDF hardware abstraction layer
// the HAL must inherit from the base RadioLibHal class
// and implement all of its virtual methods
// this is pretty much just copied from Arduino ESP32 core
class EspHal : public RadioLibHal
{
public:
    // default constructor - initializes the base HAL and any needed private members
    EspHal(int8_t sck, int8_t miso, int8_t mosi)
        : RadioLibHal(INPUT, OUTPUT, LOW, HIGH, RISING, FALLING),
          spiSCK(sck), spiMISO(miso), spiMOSI(mosi)
    {
    }

    void init() override
    {
        // we only need to init the SPI here
        spiBegin();
    }

    void term() override
    {
        // we only need to stop the SPI here
        spiEnd();
    }

    // GPIO-related methods (pinMode, digitalWrite etc.) should check
    // RADIOLIB_NC as an alias for non-connected pins
    void pinMode(uint32_t pin, uint32_t mode) override
    {
        if (pin == RADIOLIB_NC)
        {
            return;
        }

        gpio_hal_context_t gpiohal;
        gpiohal.dev = GPIO_LL_GET_HW(GPIO_PORT_0);

        gpio_config_t conf = {
            .pin_bit_mask = (1ULL << pin),
            .mode = (gpio_mode_t)mode,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = (gpio_int_type_t)gpiohal.dev->pin[pin].int_type,
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
            return (0);
        }

        return (gpio_get_level((gpio_num_t)pin));
    }

    void attachInterrupt(uint32_t interruptNum, void (*interruptCb)(void), uint32_t mode) override
    {
        if (interruptNum == RADIOLIB_NC)
        {
            return;
        }

        gpio_install_isr_service((int)ESP_INTR_FLAG_IRAM);
        gpio_set_intr_type((gpio_num_t)interruptNum, (gpio_int_type_t)(mode & 0x7));

        // this uses function typecasting, which is not defined when the functions have different signatures
        // untested and might not work
        gpio_isr_handler_add((gpio_num_t)interruptNum, (void (*)(void *))interruptCb, NULL);
    }

    void detachInterrupt(uint32_t interruptNum) override
    {
        if (interruptNum == RADIOLIB_NC)
        {
            return;
        }

        gpio_isr_handler_remove((gpio_num_t)interruptNum);
        gpio_wakeup_disable((gpio_num_t)interruptNum);
        gpio_set_intr_type((gpio_num_t)interruptNum, GPIO_INTR_DISABLE);
    }

    void delay(unsigned long ms) override
    {
        vTaskDelay(ms / portTICK_PERIOD_MS);
    }

    void delayMicroseconds(unsigned long us) override
    {
        uint64_t m = (uint64_t)esp_timer_get_time();
        if (us)
        {
            uint64_t e = (m + us);
            if (m > e)
            { // overflow
                while ((uint64_t)esp_timer_get_time() > e)
                {
                    NOP();
                }
            }
            while ((uint64_t)esp_timer_get_time() < e)
            {
                NOP();
            }
        }
    }

    unsigned long millis() override
    {
        return ((unsigned long)(esp_timer_get_time() / 1000ULL));
    }

    unsigned long micros() override
    {
        return ((unsigned long)(esp_timer_get_time()));
    }

    long pulseIn(uint32_t pin, uint32_t state, unsigned long timeout) override
    {
        if (pin == RADIOLIB_NC)
        {
            return (0);
        }

        this->pinMode(pin, INPUT);
        uint32_t start = this->micros();
        uint32_t curtick = this->micros();

        while (this->digitalRead(pin) == state)
        {
            if ((this->micros() - curtick) > timeout)
            {
                return (0);
            }
        }

        return (this->micros() - start);
    }

    void spiBegin()
    {
        // enable peripheral
#if CONFIG_IDF_TARGET_ESP32S2
        DPORT_SET_PERI_REG_MASK(DPORT_PERIP_CLK_EN_REG, DPORT_SPI2_CLK_EN);
        DPORT_CLEAR_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_SPI2_RST);
#else
        periph_ll_reset(PERIPH_SPI2_MODULE);
        periph_ll_enable_clk_clear_rst(PERIPH_SPI2_MODULE);
#endif

        // reset the control struct
#if CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32
        this->spi->slave.trans_done = 0;
#endif
        this->spi->slave.val = 0;
#if CONFIG_IDF_TARGET_ESP32
        this->spi->pin.val = 0;
#else
        this->spi->misc.val = 0;
#endif
        this->spi->user.val = 0;
        this->spi->user1.val = 0;
        this->spi->ctrl.val = 0;
#if CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32
        this->spi->ctrl1.val = 0;
        this->spi->ctrl2.val = 0;
#else
        this->spi->clk_gate.val = 0;
        this->spi->dma_conf.val = 0;
        this->spi->dma_conf.rx_afifo_rst = 1;
        this->spi->dma_conf.buf_afifo_rst = 1;
#endif

        this->spi->slave.val = 0;
        this->spi->user.val = 0;
        this->spi->user1.val = 0;
        this->spi->ctrl.val = 0;
        this->spi->clock.val = 0;
        this->spi->user.usr_mosi = 1;
        this->spi->user.usr_miso = 1;
        this->spi->user.doutdin = 1;
        for (uint8_t i = 0; i < 16; i++)
        {
#if CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32
            this->spi->data_buf[i] = 0x00000000;
#else
            this->spi->data_buf[i].val = 0x00000000;
#endif
        }

        // set SPI mode 0
#if CONFIG_IDF_TARGET_ESP32
        this->spi->pin.ck_idle_edge = 0;
#else
        this->spi->misc.ck_idle_edge = 0;
#endif

        this->spi->user.ck_out_edge = 0;

        // set bit order to MSB first
        this->spi->ctrl.wr_bit_order = 0;
        this->spi->ctrl.rd_bit_order = 0;

        // set the clock
        this->spi->clock.val = spiFrequencyToClockDiv(2000000);

        // initialize pins
        this->pinMode(this->spiSCK, OUTPUT);
        this->pinMode(this->spiMISO, INPUT);
        this->pinMode(this->spiMOSI, OUTPUT);
        gpio_matrix_out(this->spiSCK, HSPICLK_OUT_IDX, false, false);
        gpio_matrix_in(this->spiMISO, HSPIQ_OUT_IDX, false);
        gpio_matrix_out(this->spiMOSI, HSPID_IN_IDX, false, false);
    }

    void spiBeginTransaction()
    {
        // not needed - in ESP32 Arduino core, this function
        // repeats clock div, mode and bit order configuration
    }

    uint8_t spiTransferByte(uint8_t b)
    {
        this->spi->mosi_dlen.usr_mosi_dbitlen = 7;
        this->spi->miso_dlen.usr_miso_dbitlen = 7;
#if CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32
        this->spi->data_buf[0] = b;
#else
        this->spi->data_buf[0].val = b;
#endif
        this->spi->cmd.usr = 1;
        while (this->spi->cmd.usr)
            ;
#if CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32
        return (this->spi->data_buf[0] & 0xFF);
#else
        return (this->spi->data_buf[0].val & 0xFF);
#endif
    }

    void spiTransfer(uint8_t *out, size_t len, uint8_t *in)
    {
        for (size_t i = 0; i < len; i++)
        {
            in[i] = this->spiTransferByte(out[i]);
        }
    }

    void spiEndTransaction()
    {
        // nothing needs to be done here
    }

    void spiEnd()
    {
        // detach pins
        gpio_matrix_out(this->spiSCK, MATRIX_DETACH_OUT_SIG, false, false);
        gpio_matrix_in(this->spiMISO, MATRIX_DETACH_IN_LOW_PIN, false);
        gpio_matrix_out(this->spiMOSI, MATRIX_DETACH_OUT_SIG, false, false);
    }

private:
    // the HAL can contain any additional private members
    int8_t spiSCK;
    int8_t spiMISO;
    int8_t spiMOSI;
    spi_dev_t *spi = (spi_dev_t *)(DR_REG_SPI2_BASE);
};

#endif
