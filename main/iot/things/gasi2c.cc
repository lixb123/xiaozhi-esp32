#include "iot/thing.h"
#include "board.h"
#include "audio_codec.h"
#include "i2c_device.h"
#include <driver/i2c_master.h>
#include "driver/gpio.h"
#include <string.h>
#include <math.h>
#include "esp_rom_sys.h"

#include <esp_log.h>
#include <string>

#define TAG "gasi2c"

namespace iot {

#define GM_VERF 5 // AVR verf 5V
#define GM_RESOLUTION 1023
//command
#define GM_102B 0x01 //NO2
uint8_t gm_102b_cmd = GM_102B;
#define GM_302B 0x03 //C2H5OH
uint8_t gm_302b_cmd = GM_302B;
#define GM_502B 0x05 //VOC
uint8_t gm_502b_cmd = GM_502B;
#define GM_702B 0x07 //CO
uint8_t gm_702b_cmd = GM_702B;
#define CHANGE_I2C_ADDR 0x55
#define WARMING_UP 0xFE
uint8_t warming_up_cmd = static_cast<uint8_t>(WARMING_UP);
#define WARMING_DOWN  0xFF
uint8_t warming_down_cmd = static_cast<uint8_t>(WARMING_DOWN);
bool isPreheated = true; // Default to true

float no2ppm_ = 0.00;
float c2h5ohppm_ = 0.06;
float vocppm_ = 0.06;
float coppm_ = 0.01;


float calcVol(uint32_t adc, float verf = GM_VERF, int resolution = GM_RESOLUTION) {
    return (adc * verf) / (resolution * 1.0);
};
float no2ppm(float voltage) {
    return (voltage - 0)*10/4.5; // 0V = 0ppm, 4.5V = 10ppm
};
float c2h5ohppm(float voltage) {
    return expf((voltage - 1.5)*1.852); // 1.5V = 0ppm, 5V = 500ppm
};
float vocppm(float voltage) {
    return expf((voltage - 1.5)*1.852); // 1.5V = 0ppm, 5V = 500ppm
};
float coppm(float voltage) {
    return expf((voltage - 2.5)*1.852); // 2.5V = 0ppm, 5V = 100ppm
};


// 这里仅定义 gasi2c 的属性和方法
class gasi2c : public Thing {
private:
    // 这里可以添加 gasi2c 的私有成员变量和方法
    const char* test_str = "No data received";
    char* no2ppm_str = new char[100];
    char* c2h5ohppm_str = new char[100];    
    char* vocppm_str = new char[100];
    char* coppm_str = new char[100];
    uint32_t adc_value_1;
    uint32_t adc_value_3;
    uint32_t adc_value_5;
    uint32_t adc_value_7;
    float voltage_1;
    float voltage_3;
    float voltage_5;
    float voltage_7;
    i2c_master_bus_handle_t i2c_bus_2;
    i2c_master_dev_handle_t gassensor_handle_;
    void InitializeI2c() {
        //Initialize I2C peripheral
        i2c_master_bus_config_t i2c_bus_cfg_2 = {
            .i2c_port = (i2c_port_t)0,
            .sda_io_num = GPIO_NUM_10, // SDA pin
            .scl_io_num = GPIO_NUM_11, // SCL pin
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {
                .enable_internal_pullup = 1,
            },
        };
        
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg_2, &i2c_bus_2));

        // ESP_ERROR_CHECK(i2c_master_get_bus_handle(1, &i2c_bus_2));
        // ESP_LOGI(TAG, "I2C bus handle: %p", i2c_bus_2); // Log the I2C bus handle
        // Initialize I2C device
        i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = 0x08,
            .scl_speed_hz = 100000,
        };
        ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus_2, &dev_cfg, &gassensor_handle_));
        //ESP_ERROR_CHECK(i2c_set_device_timeout(gassensor_handle_, 1000)); // Set timeout to 1 second
    }

    void preheated() {
        ESP_ERROR_CHECK(i2c_master_transmit(gassensor_handle_, &warming_up_cmd, sizeof(warming_up_cmd), -1));
        isPreheated = true;
    }

    void unPreheated() {
        ESP_ERROR_CHECK(i2c_master_transmit(gassensor_handle_, &warming_up_cmd, sizeof(warming_up_cmd), -1));
        isPreheated = false;
    }

    uint32_t GMXXXRead32() {
        uint8_t byte = 0;
        uint8_t index = 0;
        uint32_t value = 0;
        for(int i = 0; i < 4; i++) {
            ESP_ERROR_CHECK(i2c_master_transmit(gassensor_handle_, &byte, sizeof(byte), -1));
            value += byte << (8 * index);
            index++;
        }
        ESP_LOGI(TAG, "GMXXXRead32: %ld\n", value); // Log the read value
        if (value == 0xFFFFFFFF) {
            ESP_LOGE(TAG, "GMXXXRead32: Error reading data");
            return 0;
        }
        return value;
    }

    int32_t getGM102B() {
        if (!isPreheated) {
            preheated();
        }
        ESP_ERROR_CHECK(i2c_master_transmit(gassensor_handle_, &gm_102b_cmd, sizeof(gm_102b_cmd), -1));
        return GMXXXRead32();
    }

    int32_t getGM302B() {
        if (!isPreheated) {
            preheated();
        }
        ESP_ERROR_CHECK(i2c_master_transmit(gassensor_handle_, &gm_302b_cmd, sizeof(gm_302b_cmd), -1));
        return GMXXXRead32();
    }

    int32_t getGM502B() {
        if (!isPreheated) {
            preheated();
        }
        ESP_ERROR_CHECK(i2c_master_transmit(gassensor_handle_, &gm_502b_cmd, sizeof(gm_502b_cmd), -1));
        return GMXXXRead32();
    }

    int32_t getGM702B() {
        if (!isPreheated) {
            preheated();
        }
        ESP_ERROR_CHECK(i2c_master_transmit(gassensor_handle_, &gm_702b_cmd, sizeof(gm_702b_cmd), -1));
        return GMXXXRead32();
    }

public:
    gasi2c() : Thing("gasi2c", "NO2,酒精,CO和VOC气体传感器") {
        InitializeI2c();

        // 定义设备的属性
        properties_.AddStringProperty("NO2", "二氧化氮浓度值ppm", [this]() -> std::string {
            adc_value_1 = getGM102B();
            voltage_1 = calcVol(adc_value_1, GM_VERF, GM_RESOLUTION);
            no2ppm_ = no2ppm(voltage_1);
            snprintf(no2ppm_str, 100, "%.2f ppm", no2ppm_);
            ESP_LOGI(TAG, "NO2: %s", no2ppm_str); // Log the NO2 value
            return no2ppm_str;
        });

        properties_.AddStringProperty("C2H5CH", "酒精浓度值ppm", [this]() -> std::string {
            adc_value_3 = getGM302B();
            voltage_3 = calcVol(adc_value_3, GM_VERF, GM_RESOLUTION);
            c2h5ohppm_ = c2h5ohppm(voltage_3);
            snprintf(c2h5ohppm_str, 100, "%.2f ppm", c2h5ohppm_);
            ESP_LOGI(TAG, "C2H5CH: %s", c2h5ohppm_str); // Log the C2H5CH value
            return c2h5ohppm_str;
        });

        properties_.AddStringProperty("VOC", "VOC气体浓度值ppm", [this]() -> std::string {
            adc_value_5 = getGM502B();
            voltage_5 = calcVol(adc_value_5, GM_VERF, GM_RESOLUTION);
            vocppm_ = vocppm(voltage_5);
            snprintf(vocppm_str, 100, "%.2f ppm", vocppm_);
            ESP_LOGI(TAG, "VOC: %s", vocppm_str); // Log the VOC value
            return vocppm_str;
        });

        properties_.AddStringProperty("CO", "一氧化碳浓度值ppm", [this]() -> std::string {
            adc_value_7 = getGM702B();
            voltage_7 = calcVol(adc_value_7, GM_VERF, GM_RESOLUTION);
            coppm_ = coppm(voltage_7);
            snprintf(coppm_str, 100, "%.2f ppm", coppm_);
            ESP_LOGI(TAG, "CO: %s", coppm_str); // Log the CO value
            return coppm_str;
        });

         // 定义设备可以被远程执行的指令
         methods_.AddMethod("sniff", "闻一下当前所有气体浓度", ParameterList(), [this](const ParameterList& parameters) {
            adc_value_1 = getGM102B();
            voltage_1 = calcVol(adc_value_1, GM_VERF, GM_RESOLUTION);
            no2ppm_ = no2ppm(voltage_1);
            adc_value_3 = getGM302B();
            voltage_3 = calcVol(adc_value_3, GM_VERF, GM_RESOLUTION);
            c2h5ohppm_ = c2h5ohppm(voltage_3);
            adc_value_5 = getGM502B();
            voltage_5 = calcVol(adc_value_5, GM_VERF, GM_RESOLUTION);
            vocppm_ = vocppm(voltage_5);
            adc_value_7 = getGM702B();
            voltage_7 = calcVol(adc_value_7, GM_VERF, GM_RESOLUTION);
            coppm_ = coppm(voltage_7);
            //esp_rom_delay_us(500); // Delay for 500ms
        });
        
    }
};

} // namespace iot

DECLARE_THING(gasi2c);