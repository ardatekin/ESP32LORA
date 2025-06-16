/*!
 *  @file RFM95Lora.cpp
 *
 *  @brief Source file for the RFM95 LoRa Module
 *
 *  @author Arda Tekin (https://www.linkedin.com/in/ardatekin)
 *
 *  This file is part of ESP32LORA project.
 *  
 *  ESP32LORA is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as published by
 *  the Free Software Foundation, either version 2.1 of the License, or
 *  (at your option) any later version.
 *
 *  ESP32LORA is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with ESP32LORA. If not, see <https://www.gnu.org/licenses/>.
 */

#include "RFM95Lora.h"
#include "Config.h"

// Tag used for ESP logging
static const char* TAG = "LoRaCPP";

uint8_t LoRa::debugReadRegister(uint8_t reg) 
{
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    uint8_t tx_data[2] = { static_cast<uint8_t>(reg & 0x7F), 0x00 };
    uint8_t rx_data[2] = { 0 };
    t.length = 16; // 2 bytes
    t.tx_buffer = tx_data;
    t.rx_buffer = rx_data;
    esp_err_t ret = spi_device_transmit(spi, &t);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI debugReadRegister failed: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "debugReadRegister(0x%02X) = 0x%02X", reg, rx_data[1]);
    }
    return rx_data[1];
}

void LoRa::setFrequency(double freq_hz) 
{
    uint32_t frf = LORA_FREQ; // Use the frequency defined in Config.h
    ESP_LOGI(TAG, "Calculated FRF = 0x%06X for %.2f Hz", (unsigned int)frf, freq_hz);
    writeReg(0x06, (uint8_t)(frf >> 16));       // MSB
    writeReg(0x07, (uint8_t)(frf >> 8));        // MID    
    writeReg(0x08, (uint8_t)(frf & 0xFF));      // LSB
}

esp_err_t LoRa::writeReg(uint8_t reg, uint8_t value) 
{
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    uint8_t data[2];
    // MSB set to 1 for write operation
    data[0] = reg | 0x80;
    data[1] = value;
    t.length = 16; // 2 bytes (16 bits)
    t.tx_buffer = data;
    ret = spi_device_transmit(spi, &t);
    return ret;
}

uint8_t LoRa::readReg(uint8_t reg) 
{
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    uint8_t tx_data[2] = { static_cast<uint8_t>(reg & 0x7F), 0x00 };
    uint8_t rx_data[2] = { 0 };
    t.length = 16; // 2 bytes (16 bits)
    t.tx_buffer = tx_data;
    t.rx_buffer = rx_data;
    
    esp_err_t ret = spi_device_transmit(spi, &t);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "spi_device_transmit failed: %s", esp_err_to_name(ret));
    }
    return rx_data[1];
}

void LoRa::reset() 
{
    gpio_set_direction((gpio_num_t)LORA_PIN_NUM_RST, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)LORA_PIN_NUM_RST, 0);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    gpio_set_level((gpio_num_t)LORA_PIN_NUM_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(200));
}

esp_err_t LoRa::init() 
{
    esp_err_t ret;
    // Configure the SPI bus
    spi_bus_config_t buscfg = {
        .mosi_io_num = LORA_PIN_NUM_MOSI,
        .miso_io_num = LORA_PIN_NUM_MISO,
        .sclk_io_num = LORA_PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0,
        .flags = SPICOMMON_BUSFLAG_MASTER
    };
    

    // Configure the SPI device for the LoRa module
    spi_device_interface_config_t devcfg = { 
        0,                      // command_bits
        0,                      // address_bits
        0,                      // dummy_bits
        0,                      // mode (SPI mode 0)
        SPI_CLK_SRC_DEFAULT,    // clock_source (default clock source)
        0,                      // duty_cycle_pos (0 means not set, equivalent to 128)
        0,                      // cs_ena_pretrans
        0,                      // cs_ena_posttrans
        1000000,                 // clock_speed_hz (500Khz/1MHz/2MHz/4Mhz)
        0,                      // input_delay_ns
        LORA_PIN_NUM_CS,             // spics_io_num (chip select GPIO)
        0,                      // flags (no special flags)
        7,                      // queue_size (transaction queue size)
        nullptr,                // pre_cb (no pre-transaction callback)
        nullptr                 // post_cb (no post-transaction callback)
    };

    // Initialize the SPI bus on HSPI_HOST (you can change to VSPI_HOST if needed)
    ret = spi_bus_initialize(HSPI_HOST, &buscfg, 1);
    if(ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus");
        return ret;
    }

    ESP_LOGI(TAG, "SPI pins: CLK=%d, MISO=%d, MOSI=%d, CS=%d",
        LORA_PIN_NUM_CLK, LORA_PIN_NUM_MISO, LORA_PIN_NUM_MOSI, LORA_PIN_NUM_CS);

    ret = spi_bus_add_device(HSPI_HOST, &devcfg, &spi);
    if(ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add SPI device");
        return ret;
    }

    // Reset the LoRa module
    reset();
    vTaskDelay(pdMS_TO_TICKS(100));  // Wait for chip to wake up

    // Unmask all interrupts (TxDone, RxDone, etc.)
    writeReg(0x11, 0x00);  // RegIrqFlagsMask

    // Set default DIO mapping: DIO0=TxDone (in TX), DIO0=RxDone (in RX)
    writeReg(0x40, 0x00);  // RegDioMapping1

    // Set DIO0 as input (only affects ESP32 pin mode)
    gpio_set_direction((gpio_num_t)LORA_PIN_NUM_DIO0, GPIO_MODE_INPUT);

    // Debug log
    esp_log_level_set(TAG, ESP_LOG_DEBUG);
    ESP_LOGI(TAG, "About to read RegVersion (0x42)...");
    uint8_t version = readReg(0x42);
    ESP_LOGI(TAG, "SX1276 Version: 0x%02X", version);


    // Put module into sleep mode in LoRa mode
    writeReg(0x01, 0x80);  // RegOpMode: sleep mode + LoRa mode
    vTaskDelay(pdMS_TO_TICKS(10));

    setFrequency(868000000.0); // Set the RF Freq: 868 MHz

    // Configure modem settings:
    // RegModemConfig1: BW = 125 kHz, CR = 4/5, explicit header mode
    // RegModemConfig2: SF = 7, enable CRC
    writeReg(0x1D, 0x72);
    writeReg(0x1E, 0x74);

    // Set preamble length (example: 8 symbols)
    writeReg(0x20, 0x00);
    writeReg(0x21, 0x08);

    // === NEW CONFIG: Enable PA_BOOST + TxDone unmask + clear IRQs ===

    // Unmask TxDone interrupt only (bit 3 = 0)
    writeReg(0x11, 0xF7);  // RegIrqFlagsMask

    // Clear all pending IRQ flags
    writeReg(0x12, 0xFF);  // RegIrqFlags

    // Enable PA_BOOST and set power to max (0x0F → ~+17 dBm)
    writeReg(0x09, 0x8F);  // RegPaConfig

    // Put module into standby mode
    writeReg(0x01, 0x81);  // Standby mode
    vTaskDelay(pdMS_TO_TICKS(10));

    ESP_LOGI(TAG, "LoRa Initialized");
    return ESP_OK;
}

esp_err_t LoRa::send(const uint8_t* data, int length) 
{
    // Switch to standby mode before transmission
    writeReg(0x01, 0x81);  // RegOpMode: Standby
    vTaskDelay(pdMS_TO_TICKS(10));

    // Clear all IRQ flags before starting transmission
    writeReg(0x12, 0xFF);  // RegIrqFlags: clear all flags

    // Set FIFO TX base address to 0x00
    writeReg(0x0E, 0x00);  // RegFifoTxBaseAddr = 0x00
    // Set FIFO address pointer to base (required before writing payload)
    writeReg(0x0D, 0x00);  // RegFifoAddrPtr = 0x00

    // Write payload to FIFO
    for (int i = 0; i < length; i++) {
        writeReg(0x00, data[i]);  // RegFifo
    }

    // Set payload length
    writeReg(0x22, length);  // RegPayloadLength

    // Set DIO0 to map to TxDone
    writeReg(0x40, 0x40);  // RegDioMapping1: DIO0 = 01 (TxDone)
    // Optional: mask other IRQs, if needed
    writeReg(0x11, 0xF7);  // RegIrqFlagsMask: unmask TxDone

    // Begin transmission
    writeReg(0x01, 0x83);  // RegOpMode: TX mode
    ESP_LOGI(TAG, "Sending packet...");

    // Wait for TxDone IRQ flag with a timeout
    uint8_t flags = 0;
    int timeoutCounter = 0;
    const int timeoutLimit = 50; // Timeout after 50 iterations (5 seconds)
    while ((flags = readReg(0x12) & 0x08) == 0) {
        ESP_LOGI(TAG, "TX IRQ Flags: 0x%02X", flags);
        vTaskDelay(pdMS_TO_TICKS(100));
        if (++timeoutCounter >= timeoutLimit) {
            ESP_LOGE(TAG, "Packet not sent: Timeout waiting for TxDone flag");

            // Clear TxDone flag to reset the state
            writeReg(0x12, 0x08);  // RegIrqFlags: clear TxDone

            return ESP_FAIL; // Return failure if timeout occurs
        }
    }

    ESP_LOGI(TAG, "TX IRQ Flags (After send): 0x%02X", flags);

    // Clear TxDone flag
    writeReg(0x12, 0x08);  // RegIrqFlags: clear TxDone

    ESP_LOGI(TAG, "Packet sent");
    return ESP_OK;
}

int LoRa::receive(uint8_t* buffer, int bufferSize) 
{
    // 1. Clear all IRQ flags
    writeReg(0x12, 0xFF);  // Clear all interrupts

    // 2. Set FIFO base and pointer to RX base address
    writeReg(0x0F, 0x00);  // RegFifoRxBaseAddr
    writeReg(0x0D, 0x00);  // RegFifoAddrPtr

    // 3. Unmask interrupts (RegIrqFlagsMask)
    writeReg(0x11, 0x00);

    // 4. Ensure DIO0 is mapped to RxDone
    writeReg(0x40, 0x00);  // RegDioMapping1 = 0x00

    // 5. Enter continuous receive mode
    writeReg(0x01, 0x85);  // RegOpMode = LoRa | RXCONTINUOUS
    ESP_LOGI(TAG, "RegOpMode after RX set: 0x%02X", readReg(0x01));

    // 6. Optional small delay for RX settling
    vTaskDelay(pdMS_TO_TICKS(10));

    // 7. Wait until RxDone flag (bit 6) is set
    uint8_t flags = 0;
    int counter = 0;
    while ((flags = readReg(0x12) & 0x40) == 0) {
        if ((counter++ % 5) == 0) {
            ESP_LOGI(TAG, "[RX] No packet yet (IRQ Flags: 0x%02X)", readReg(0x12));
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // 8. Clear RxDone flag
    writeReg(0x12, 0x40);

    // 9. Read current FIFO address
    uint8_t fifo_rx_current_addr = readReg(0x10);
    writeReg(0x0D, fifo_rx_current_addr);  // RegFifoAddrPtr

    // 10. Read packet length
    int packet_length = readReg(0x13);
    if (packet_length > bufferSize) {
        packet_length = bufferSize;
    }

    // 11. Read from FIFO
    for (int i = 0; i < packet_length; i++) {
        buffer[i] = readReg(0x00);
    }

    // 12. Optional: log RSSI & SNR
    uint8_t rssi = readReg(0x1A);
    uint8_t snr = readReg(0x19);
    ESP_LOGI(TAG, "Received packet with %d bytes (RSSI: %d, SNR: %d)", packet_length, rssi, snr);

    return packet_length;
}
