#include <stdio.h>
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_err.h"
#include <math.h>


#include "qmi8658.h"

extern SemaphoreHandle_t i2c_mux;		//i2c semaphore
static uint32_t last_step_time = 0;
static int step_count = 0;
float alpha = 0.2; 		// between 0.0~1.0
float acc_filtered_x = 0, acc_filtered_y = 0, acc_filtered_z = 0;

const char *TAGQMI = "QMI8658";
/**
*@brief Read qmi8658 register function
**/
esp_err_t qmi8658_read_reg(uint8_t reg_addr, uint8_t *data, size_t len) {
    esp_err_t ret = ESP_FAIL;
	
	// try to get I2C Mutex，to set a ideal time
    TickType_t mutex_timeout = pdMS_TO_TICKS(100); // 100ms
	if (xSemaphoreTake(i2c_mux, mutex_timeout) == pdTRUE){
		i2c_cmd_handle_t cmd = i2c_cmd_link_create();
		i2c_master_start(cmd);
		i2c_master_write_byte(cmd, (QMI8658_ADDR << 1) | I2C_MASTER_WRITE, true);
		i2c_master_write_byte(cmd, reg_addr, true);
		i2c_master_start(cmd);
		i2c_master_write_byte(cmd, (QMI8658_ADDR << 1) | I2C_MASTER_READ, true);
		i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);
		i2c_master_stop(cmd);
		ret = i2c_master_cmd_begin(I2C_MASTER_NUM_QMI, cmd, I2C_TIMEOUT_MS_QMI / portTICK_PERIOD_MS);
		i2c_cmd_link_delete(cmd);
		xSemaphoreGive(i2c_mux);
    }else{
        ESP_LOGE(TAGQMI, "Failed to acquire I2C mutex for read");
		ret = ESP_ERR_TIMEOUT; // set ret acquire timeout
    }
    return ret;
}

/**
*@brief write qmi8658 register function
**/
esp_err_t qmi8658_write_reg(uint8_t reg_addr, uint8_t *data, size_t len) {
	esp_err_t ret = ESP_FAIL;
	
	// try to get I2C Mutex，to set a ideal time
    TickType_t mutex_timeout = pdMS_TO_TICKS(100); // 100ms
	if (xSemaphoreTake(i2c_mux, mutex_timeout) == pdTRUE){
		i2c_cmd_handle_t cmd = i2c_cmd_link_create();
		i2c_master_start(cmd);
		i2c_master_write_byte(cmd, (QMI8658_ADDR << 1) | I2C_MASTER_WRITE, true);
		i2c_master_write_byte(cmd, reg_addr, true);
		i2c_master_write(cmd, data, len, true);
		i2c_master_stop(cmd);
		ret = i2c_master_cmd_begin(I2C_MASTER_NUM_QMI, cmd, I2C_TIMEOUT_MS_QMI / portTICK_PERIOD_MS);
		i2c_cmd_link_delete(cmd);
		xSemaphoreGive(i2c_mux);
    }else{
        ESP_LOGE(TAGQMI, "Failed to acquire I2C mutex for read");
		ret = ESP_ERR_TIMEOUT; // set ret acquire timeout
    }
    return ret;
}

/**
*@brief qmi8658 init function
**/
esp_err_t qmi8658_init(){
	uint8_t chipid = 0x00;
	uint8_t rev_id = 0x00;
	uint8_t errCnt = 0;
	while((chipid == 0x00) && errCnt < 5){
		qmi8658_read_reg(WHO_AM_I,&chipid,1);
		if(chipid == 0x05){
			break;
		}
		errCnt++;
		ESP_LOGE(TAGQMI, "Failed to read chipid. chipid now = %x, retyr time = %d", chipid,errCnt);
		vTaskDelay(10 / portTICK_PERIOD_MS);  
	}
	
	qmi8658_read_reg(REVISION_ID,&rev_id,1);
	
	if(chipid == 0x05){
		ESP_LOGI(TAGQMI, "[QMI8658] address 0x%x, WHO_AM_I 0x%x, Revision_ID 0x%x.", QMI8658_ADDR, chipid,rev_id);
		uint8_t ctrl1DATA = 0x60;
		qmi8658_write_reg(CTRL1,&ctrl1DATA,1);
		uint8_t ctrl7DATA = 0x03;
		qmi8658_write_reg(CTRL7,&ctrl7DATA,1);
		uint8_t ctrl2DATA = 0x84;		//enable selftest  acc range +-2g  odr 500hz
		qmi8658_write_reg(CTRL2,&ctrl2DATA,1);
		uint8_t ctrl3DATA = 0xc4;		//enable selftest  gry range 256dps  odr 500hz
		qmi8658_write_reg(CTRL3,&ctrl3DATA,1);
		uint8_t ctrl5DATA = 0x55;		//5.32% acc/gry low pass filter enable
		qmi8658_write_reg(CTRL5,&ctrl5DATA,1);
		
		uint8_t read_data = 0x00;
		
		qmi8658_read_reg(CTRL1,&read_data,1);
		ESP_LOGI(TAGQMI, "QMI8658 CTRL1 = 0x%x", read_data);
		qmi8658_read_reg(CTRL2,&read_data,1);
		ESP_LOGI(TAGQMI, "QMI8658 CTRL2 = 0x%x", read_data);
		qmi8658_read_reg(CTRL3,&read_data,1);
		ESP_LOGI(TAGQMI, "QMI8658 CTRL3 = 0x%x", read_data);
		qmi8658_read_reg(CTRL4,&read_data,1);
		ESP_LOGI(TAGQMI, "QMI8658 CTRL4 = 0x%x", read_data);
		qmi8658_read_reg(CTRL5,&read_data,1);
		ESP_LOGI(TAGQMI, "QMI8658 CTRL5 = 0x%x", read_data);
		qmi8658_read_reg(CTRL6,&read_data,1);
		ESP_LOGI(TAGQMI, "QMI8658 CTRL6 = 0x%x", read_data);
		qmi8658_read_reg(CTRL7,&read_data,1);
		ESP_LOGI(TAGQMI, "QMI8658 CTRL7 = 0x%x", read_data);
		return ESP_OK;
		
	}else{
		ESP_LOGE(TAGQMI, "QMI8658 INIT FAIL!!!");
		return ESP_FAIL;
	}
	return ESP_OK;
}

/**
*@brief get qmi8658 acc and gyro value function
**/
esp_err_t qmi8658_accAndgry(motion_data_t *data){
	esp_err_t ret = ESP_FAIL;
	uint8_t buf_reg[12];
	short raw_acc_xyz[3];
	short raw_gyro_xyz[3];
	float temp[6];
	
	ret = qmi8658_read_reg(AX_L,buf_reg,12);
	raw_acc_xyz[0] = (short)((unsigned short)(buf_reg[1]<<8) | (buf_reg[0]));
	raw_acc_xyz[1] = (short)((unsigned short)(buf_reg[3]<<8) | (buf_reg[2]));
	raw_acc_xyz[2] = (short)((unsigned short)(buf_reg[5]<<8) | (buf_reg[4]));
	
	raw_gyro_xyz[0] = (short)((unsigned short)(buf_reg[7]<<8) | (buf_reg[6]));
	raw_gyro_xyz[1] = (short)((unsigned short)(buf_reg[9]<<8) | (buf_reg[8]));
	raw_gyro_xyz[2] = (short)((unsigned short)(buf_reg[11]<<8) | (buf_reg[10]));
	
	float acc_scale = 2.0f / 32768.0f;		//dps/16bit(signed bit)
	// ESP_LOGI(TAGQMI, "%d %d %d %d %d %d", raw_acc_xyz[0],raw_acc_xyz[1],raw_acc_xyz[2],raw_gyro_xyz[0],raw_gyro_xyz[1],raw_gyro_xyz[2]);
	//角度值
	// temp[0] = (float)raw_acc_xyz[0] / sqrt((float)raw_acc_xyz[1]*(float)raw_acc_xyz[1] + (float)raw_acc_xyz[2]*(float)raw_acc_xyz[2]);
	// temp[0] = atan(temp[0])*57.29578f;
	
	temp[0] = raw_acc_xyz[0] * acc_scale;
	data->acc[0] = temp[0];
	
	// temp[1] = (float)raw_acc_xyz[1] / sqrt((float)raw_acc_xyz[0]*(float)raw_acc_xyz[0] + (float)raw_acc_xyz[2]*(float)raw_acc_xyz[2]);
	// temp[1] = atan(temp[1])*57.29578f;
	temp[1] = raw_acc_xyz[1] * acc_scale;
	data->acc[1] = temp[1];
	
	// temp[2] = sqrt((float)raw_acc_xyz[0]*(float)raw_acc_xyz[0] + (float)raw_acc_xyz[1]*(float)raw_acc_xyz[1])/(float)raw_acc_xyz[2];
	// temp[2] = atan(temp[2])*57.29578f;
	temp[2] = raw_acc_xyz[1] * acc_scale;
	data->acc[2] = temp[2];
	
	float scale = 256.0f / 32768.0f;		//dps/16bit(signed bit)
	temp[3] = raw_gyro_xyz[0] * scale;
	data->gyro[0] = temp[3];
	temp[4] = raw_gyro_xyz[1] * scale;
	data->gyro[1] = temp[4];
	temp[5] = raw_gyro_xyz[2] * scale;
	data->gyro[2] = temp[5];
	// ESP_LOGI(TAGQMI, "angle x %f angle y %f angle z %f " ,temp[0],temp[1],temp[2] );
	return ret;
}

/**
 * @brief input three axis acc value, update step
 */
uint16_t update_step_count(motion_data_t *data)
{
    static float prev_magnitude = 0.0f;

    // calculate magnitude
    float acc_magnitude = sqrtf(data->acc[0] * data->acc[0]+ data->acc[1] * data->acc[1] + data->acc[2] * data->acc[2]);
    float gyro_magnitude = sqrtf(data->gyro[0] * data->gyro[0]+ data->gyro[1] * data->gyro[1] + data->gyro[2] * data->gyro[2]);
	// ESP_LOGI(TAGQMI, " acc magnitude = %f  gyro magnitude = %f",acc_magnitude,gyro_magnitude );
    uint32_t now_ms = esp_timer_get_time() / 1000;
    // uint32_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

    // check if larger than threshold and time is big enough
    if ((acc_magnitude > STEP_THRESHOLD) && (prev_magnitude <= STEP_THRESHOLD) && gyro_magnitude > 5.0f) {
        if ((now_ms - last_step_time) > MIN_STEP_INTERVAL_MS) {
            step_count++;
            last_step_time = now_ms;
            ESP_LOGI(TAGQMI,"步數：%d\n", step_count);
        }
    }

    prev_magnitude = acc_magnitude;
	return step_count;
}

/**
 * @brief get current step
 */
int get_step_count()
{
    return step_count;
}

/**
 * @brief reset step
 */
void reset_step_count()
{
    step_count = 0;
}

/**
 * @brief moving average filter acc
 */
void filter_acc_ema(motion_data_t *data) {
    acc_filtered_x = alpha * data->acc[0] + (1 - alpha) * acc_filtered_x;
    acc_filtered_y = alpha * data->acc[1] + (1 - alpha) * acc_filtered_y;
    acc_filtered_z = alpha * data->acc[2] + (1 - alpha) * acc_filtered_z;

    data->acc[0] = acc_filtered_x;
    data->acc[1] = acc_filtered_y;
    data->acc[2] = acc_filtered_z;
}


