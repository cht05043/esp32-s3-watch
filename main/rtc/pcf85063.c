#include <stdio.h>
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_err.h"

#include "pcf85063.h"

extern SemaphoreHandle_t i2c_mux;

/*
* rtc using step
* 1. initialize rtc (set time)
* 2. gettime
*/

const char *TAGPCF = "PCF85063";


/**
*@brief Read RTC register function
**/
esp_err_t rtc_read_reg(uint8_t reg_addr, uint8_t *data, size_t len) {
    esp_err_t ret = ESP_FAIL;
    if (xSemaphoreTake(i2c_mux, 100) == pdTRUE){
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (PCF85063_ADDR << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, reg_addr, true);
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (PCF85063_ADDR << 1) | I2C_MASTER_READ, true);
        i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);
        i2c_master_stop(cmd);
        ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
        if (ret != ESP_OK) {
            ESP_LOGE(TAGPCF, "I2C read error (reg 0x%02x, len %d): %s", reg_addr, len, esp_err_to_name(ret));
        }
        xSemaphoreGive(i2c_mux);
    }else{
        ESP_LOGE(TAGPCF, "Failed to acquire I2C mutex for read");
    }
    
    return ret;
}

/**
*@brief Write RTC register function
**/
esp_err_t rtc_write_reg(uint8_t reg_addr, uint8_t *data, size_t len) {
    esp_err_t ret = ESP_FAIL;
    if (xSemaphoreTake(i2c_mux, 100) == pdTRUE) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (PCF85063_ADDR << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, reg_addr, true);
        i2c_master_write(cmd, data, len, true);
        i2c_master_stop(cmd);
        ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
        xSemaphoreGive(i2c_mux);
    }else{
        ESP_LOGE(TAGPCF, "Failed to acquire I2C mutex for write");
    }
    return ret;
}

/**
*@brief BCD to decimal conversion
**/
uint8_t bcd_to_dec(uint8_t val) {
    return (val >> 4) * 10 + (val & 0x0F);
}

/**
*@brief Decimal to BCD conversion
**/
uint8_t dec_to_bcd(uint8_t val) {
    return ((val / 10) << 4) | (val % 10);
}

/**
*@brief Function to get current time from the RTC
**/
esp_err_t rtc_get_time(rtc_time *time_out) {
    uint8_t data[7];
    esp_err_t ret = rtc_read_reg(0x04, data, 7); // PCF85063 time registers start from 0x04
    if (ret != ESP_OK) {
        ESP_LOGE(TAGPCF, "Failed to read time registers");
        return ret;
    }

    time_out->seconds = bcd_to_dec(data[0] & 0x7F); // Masking out the stop bit
    time_out->minutes = bcd_to_dec(data[1]);
    time_out->hours = bcd_to_dec(data[2]);
    time_out->day = bcd_to_dec(data[3]);
    time_out->weekday = bcd_to_dec(data[4]);
    time_out->month = bcd_to_dec(data[5] & 0x1F); // Masking out the century bit
    time_out->year = bcd_to_dec(data[6]);

    // ESP_LOGI(TAGPCF, "Current Time: %02d:%02d:%02d %02d/%02d/%02d/%04d", time_out->hours, time_out->minutes, time_out->seconds, time_out->weekday+1,time_out->day, time_out->month, 2000 + time_out->year);
    return ESP_OK;
}

/**
*@brief Function to set time on the RTC
**/
esp_err_t rtc_set_time(rtc_time *time_out) {
    uint8_t data[7] = {
        dec_to_bcd(time_out->seconds),
        dec_to_bcd(time_out->minutes),
        dec_to_bcd(time_out->hours),
        dec_to_bcd(time_out->day),
        dec_to_bcd(time_out->weekday), // Weekday, you can set this if needed
        dec_to_bcd(time_out->month),
        dec_to_bcd(time_out->year)
    };
    return rtc_write_reg(0x04, data, 7);
}

/**
*@brief init rtc
**/
esp_err_t rtc_pcf85063_init() {
  
	// Initiate Normal Mode, RTC Run, NO reset, No correction, , 24hr format, Internal load capacitane 12.5pf

	uint8_t PCFINIT_ = 0x49;
    return rtc_write_reg(I2C_MASTER_NUM,&PCFINIT_,1);
}

/**
*@brief set alarm
**/
void rtc_set_alarm(rtc_time *time) {

    uint8_t alarm_data[5];
    alarm_data[0] = (time->seconds&0x80)?0x80:time->seconds; //second alarm
    alarm_data[1] = (time->minutes&0x80)?0x80:time->minutes; // minute alarm
    alarm_data[2] = (time->hours&0x80)?0x80:time->hours; // HOUR_ALARM masked (bit7=1, 不比較小時)
    alarm_data[3] = (time->day&0x80)?0x80:time->day; // DAY_ALARM masked
    alarm_data[4] = (time->weekday&0x80)?0x80:time->weekday; // WEEKDAY_ALARM masked

    rtc_write_reg(0x0B, alarm_data, 5);

    // Enable Alarm interrupt
    uint8_t ctrl2[] = {0x80}; // CTRL2: AF=0, AIE=1
    rtc_write_reg(0x01, ctrl2, 1);
}