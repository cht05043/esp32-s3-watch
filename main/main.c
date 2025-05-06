#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "driver/spi_master.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"
#include "lvgl.h"
#include "esp_task_wdt.h"
#include "esp_sleep.h"
#include "freertos/semphr.h"
#include "time.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/ledc.h"
#include "esp_pm.h"
#include "nvs_flash.h"
#include "esp_spiffs.h"


#include "buzzer/buzzer.h"
#include "esp_lcd_ili9341.h"
#include "esp_lcd_touch_cst816s.h"
#include "ui/ui.h"
#include "rtc/pcf85063.h"
#include "qmi8658/qmi8658.h"
#include "mywifi/mywifi.h"


#define EXAMPLE_LCD_H_RES              240
#define EXAMPLE_LCD_V_RES              280

#define GPIO_BL 15

#define LCD_HOST  SPI2_HOST
#define EXAMPLE_PIN_NUM_SCLK           6
#define EXAMPLE_PIN_NUM_DATA0          7
#define EXAMPLE_PIN_NUM_LCD_DC         4
#define EXAMPLE_PIN_NUM_LCD_RST        8
#define EXAMPLE_PIN_NUM_LCD_CS         5

#define I2C_MASTER_NUM                   0               /* I2C master i2c port number */
#define EXAMPLE_PIN_NUM_TOUCH_SCL             10              /* GPIO number used for I2C master clock */
#define EXAMPLE_PIN_NUM_TOUCH_SDA             11              /* GPIO number used for I2C master data  */
#define EXAMPLE_PIN_NUM_TOUCH_RST             13              /* GPIO number used for touch pad reset pin */
#define EXAMPLE_PIN_NUM_TOUCH_INT             14              /* GPIO number used for touch pad interrupt pin */

#define EXAMPLE_LVGL_TICK_PERIOD_MS     2               /*!< LVGL tick period in ms */
#define EXAMPLE_LVGL_TASK_MAX_DELAY_MS  1000
#define EXAMPLE_LVGL_TASK_MIN_DELAY_MS  1
#define EXAMPLE_LVGL_TASK_STACK_SIZE    (4 * 1024)
#define EXAMPLE_LVGL_TASK_PRIORITY      2

#define RTC_TASK_STACK_SIZE (3*1024)
#define QMI8658_TASK_STACK_SIZE (3*1024)
#define NORMAL_TASK_PRIORITY 1
#define TIMEOUT 5
#define INACTIVITY_TIMEOUT_US ((TIMEOUT) * 1000 * 1000) 

//#define NOTIFY_ENTER_LVGL_BLOCKING (1U << 0) // 0x01
//#define NOTIFY_EXIT_LVGL_BLOCKING  (1U << 1) // 0x02
#define EVENT_ENTER_SLEEP_REQUEST  (1U << 2) // 0x04

extern lv_obj_t * ui_MinutePng;
extern lv_obj_t * ui_HourPng;
extern lv_obj_t * ui_secondPng;
extern lv_obj_t * ui_weekday;
extern lv_obj_t * ui_date;
extern lv_obj_t * ui_TimeHour;
extern lv_obj_t * ui_TimeMinute;
extern lv_obj_t * ui_startbutton;
extern lv_obj_t * ui_stopbutton;
extern lv_obj_t * ui_clearbutton;
extern lv_obj_t * ui_timerhour;
extern lv_obj_t * ui_timerminute;
extern lv_obj_t * ui_timersecond;
extern lv_obj_t * ui_timermilisec;
extern lv_obj_t * ui_startbutton2;
extern lv_obj_t * ui_yearset;
extern lv_obj_t * ui_monthset;
extern lv_obj_t * ui_dayset;
extern lv_obj_t * ui_weekdayset;
extern lv_obj_t * ui_hourset;
extern lv_obj_t * ui_minset;
extern lv_obj_t * ui_stepCount;
extern lv_obj_t * ui_Bar1;
extern lv_obj_t * ui_loading;
extern lv_obj_t * ui_wakeupSwitch;

static const char *TAG = "main";

static SemaphoreHandle_t    touch_mux  			= NULL;
static SemaphoreHandle_t    lvgl_mux    		= NULL;
SemaphoreHandle_t    		i2c_mux 			= NULL;
esp_timer_handle_t 			inactivity_timer 	= NULL;
esp_lcd_touch_handle_t    	tp              	= NULL;
adc_oneshot_unit_handle_t adc1_handle           = NULL;

esp_lcd_panel_handle_t panel_handle = NULL; // 需要將 panel_handle 設為全域或可在 enter_light_sleep 中訪問
esp_timer_handle_t lvgl_tick_timer = NULL; 	// 需要將 lvgl_tick_timer 設為全域或可在 enter_light_sleep 中訪問
TaskHandle_t lvgl_task_handle = NULL; 		// 需要保存 LVGL 任務句柄
static volatile bool s_system_is_sleeping 		= false;
static SemaphoreHandle_t s_sleep_state_mutex 	= NULL;
static esp_timer_handle_t clock_timer;      //時鐘
static esp_timer_handle_t time_counter;     //計時器
static bool timer_running = false;
static uint32_t elapsed_time_ms = 0;
volatile bool rtc_ready = false;  


void enter_light_sleep(void);
static void example_increase_lvgl_tick(void *arg);

static void IRAM_ATTR isr_handler(){
    ESP_LOGI(TAG,"interrupt fired");
}

static void setTime_button_cb(lv_event_t *event){
    uint16_t yearS = lv_dropdown_get_selected(ui_yearset);
    uint16_t monthS = lv_dropdown_get_selected(ui_monthset)?lv_dropdown_get_selected(ui_monthset):1;
    uint16_t dayS = lv_dropdown_get_selected(ui_dayset)?lv_dropdown_get_selected(ui_dayset):1;
    uint16_t weekdayS = lv_dropdown_get_selected(ui_weekdayset)?lv_dropdown_get_selected(ui_weekdayset):1;
    uint16_t hourS = lv_dropdown_get_selected(ui_hourset)?lv_dropdown_get_selected(ui_hourset):1;
    uint16_t minuteS = lv_dropdown_get_selected(ui_minset)?lv_dropdown_get_selected(ui_minset):1;
    
    switch (yearS)
    {
    case 0:
        yearS = 2025;
        break;
    case 1:
        yearS = 2025;
        break;
    case 2:
        yearS = 2026;
        break;
    case 3:
        yearS = 2027;
        break;
    default:
        yearS = 2025;
        break;
    }
    ESP_LOGI(TAG, "SET TIME INFO");
    ESP_LOGI(TAG, "YEARS %d MONTH %d DAT %d",yearS, monthS,dayS);
    ESP_LOGI(TAG, "WEEKDAY %d HOUR %d MINUTE %d",weekdayS, hourS,minuteS);
    rtc_time SetTime = {
        .year = yearS,
        .month = monthS,
        .day = dayS, 
        .weekday = weekdayS - 1,
        .hours = hourS - 1,
        .minutes = minuteS - 1,
        .seconds = 0
    };
    esp_err_t ret =rtc_set_time(&SetTime);
    if(ret !=ESP_OK){
        ESP_LOGE(TAG, "Failed to SET TIME: %s", esp_err_to_name(ret));
    }
    lv_dropdown_set_selected(ui_yearset, 0);
    lv_dropdown_set_selected(ui_monthset, 0);
    lv_dropdown_set_selected(ui_dayset, 0);
    lv_dropdown_set_selected(ui_weekdayset, 0);
    lv_dropdown_set_selected(ui_hourset, 0);
    lv_dropdown_set_selected(ui_minset, 0);
    return;
}

/**
*@brief time counter call back
**/
static void time_counter_cb(void *arg) {
    if (timer_running) {
        elapsed_time_ms += 10; // 假設定時器週期是 10ms (可以根據需要調整)

        uint32_t milliseconds = elapsed_time_ms % 1000;
        uint32_t seconds = (elapsed_time_ms / 1000) % 60;
        uint32_t minutes = (elapsed_time_ms / (1000 * 60)) % 60;
        uint32_t hours = elapsed_time_ms / (1000 * 60 * 60);

        char time_str[8];
        sprintf(time_str, "%02d", (int)hours);
        lv_label_set_text(ui_timerhour, time_str);
        sprintf(time_str, "%02d", (int)minutes);
        lv_label_set_text(ui_timerminute, time_str);
        sprintf(time_str, "%02d",(int)seconds);
        lv_label_set_text(ui_timersecond, time_str);
        sprintf(time_str, "%03d", (int)milliseconds);
        lv_label_set_text(ui_timermilisec, time_str);
        
    }
}

/**
*@brief time counter start
**/
static void start_button_cb(lv_event_t *event) {
    if (!timer_running) {
        timer_running = true;
        ESP_ERROR_CHECK(esp_timer_start_periodic(time_counter, 10000)); // 10ms 週期
    }
}

/**
*@brief time counter stop
**/
static void stop_button_cb(lv_event_t *event) {
    if (timer_running) {
        timer_running = false;
        ESP_ERROR_CHECK(esp_timer_stop(time_counter));
    }
}

/**
*@brief time counter clear
**/
static void clear_button_cb(lv_event_t *event) {
    timer_running = false;
    elapsed_time_ms = 0;
    char time_str[12];
    sprintf(time_str, "00");
    lv_label_set_text(ui_timerhour, time_str);
    sprintf(time_str, "00");
    lv_label_set_text(ui_timerminute, time_str);
    sprintf(time_str, "00");
    lv_label_set_text(ui_timersecond, time_str);
    sprintf(time_str, "000");
    lv_label_set_text(ui_timermilisec, time_str);
    if(timer_running){
        esp_err_t err = esp_timer_stop(time_counter);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to stop timer: %s", esp_err_to_name(err));
        }
        timer_running = false;
    }
}

/**
*@brief clock call back
**/
static void update_clock(void *param) {
    rtc_time  now;
    if (rtc_get_time(&now) == ESP_OK){
        // calculate clock angle
        float hour_angle = fmodf((now.hours % 12 + now.minutes / 60.0), 12) * 30.0;
        float minute_angle = now.minutes * 6.0;
        float second_angle = now.seconds * 6.0;
        lv_img_set_angle(ui_HourPng, hour_angle * 10);   // because of set angle unit is 0.1 deg，we need to  multiple 10
        lv_img_set_angle(ui_MinutePng, minute_angle * 10);
        lv_img_set_angle(ui_secondPng, second_angle * 10);
        char time_str[8];
        sprintf(time_str, "%02d", now.hours);
        lv_label_set_text(ui_TimeHour, time_str);
        sprintf(time_str, "%02d", now.minutes);
        lv_label_set_text(ui_TimeMinute, time_str);

        sprintf(time_str, "%02d/%02d", now.month,now.day);
        lv_label_set_text(ui_date, time_str);
        switch (now.weekday)
        {
        case 0:
            sprintf(time_str, "SUN.");
            break;
        case 1:
            sprintf(time_str, "MON.");
            break;
        case 2:
            sprintf(time_str, "TUE.");
            break;
        case 3:
            sprintf(time_str, "WED.");
            break;
        case 4:
            sprintf(time_str, "THU.");
            break;
        case 5:
            sprintf(time_str, "FRI.");
            break;
        case 6:
            sprintf(time_str, "SAT.");
            break;
        default:
            break;
        }
        lv_label_set_text(ui_weekday, time_str);

        // lv_obj_invalidate(lv_disp_get_scr_act(NULL));
        lv_obj_invalidate(ui_Container1);
    }else {
        ESP_LOGE(TAG, "Failed to get RTC time for clock update");
    }
}

/**
*@brief start clock timer
**/
void start_clock_timer() {
    const esp_timer_create_args_t timer_args = {
        .callback = &update_clock,
        .name = "clock_update"
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &clock_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(clock_timer, 1000000)); // 1 sec a time
}

/**
*@brief inactivity timeout callback
**/
static void inactivity_timer_cb(void* arg)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    ESP_LOGI(TAG, "Inactivity timer expired, requesting Light Sleep...");
    // send notification to lvgl task in order to enter sleep
    if (lvgl_task_handle != NULL) { // ensure handle is valid
        xTaskNotifyFromISR(lvgl_task_handle, EVENT_ENTER_SLEEP_REQUEST, eSetBits, &xHigherPriorityTaskWoken);
    } else {
         ESP_LOGE(TAG, "LVGL task handle is NULL, cannot send sleep request notification!");
    }
	// if send notification to higher level task , need context switch
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
*@brief start or reset inactivity timer
**/
void reset_inactivity_timer()
{
    if (inactivity_timer) {
        esp_timer_stop(inactivity_timer); // stop timer
        esp_timer_start_once(inactivity_timer, INACTIVITY_TIMEOUT_US); // restart
        ESP_LOGD(TAG, "Inactivity timer reset.");
        
        esp_lcd_panel_disp_on_off(panel_handle, true);
        gpio_set_level(GPIO_BL, 1);
    }
}

/**
*@brief check if activity or not(call in example_lvgl_touch_cb)
**/
void handle_user_activity()
{

	bool currently_sleeping = false;
	if(xSemaphoreTake(s_sleep_state_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
         currently_sleeping = s_system_is_sleeping;
         xSemaphoreGive(s_sleep_state_mutex);
    } else {
        ESP_LOGE(TAG, "Failed to take sleep state mutex in handle_user_activity!");
         // 無法獲取鎖定時，採取一個安全默認行為，例如假設不在睡眠，繼續重置計時器
         reset_inactivity_timer();
        return;
        }
     if (currently_sleeping) {
         ESP_LOGI(TAG, "Activity detected while sleeping, wakeup in progress.");

    } else {
		ESP_LOGD(TAG, "Activity detected while awake, resetting timer.");
         reset_inactivity_timer();
    }
}

/**
*@brief enter light sleep mode
**/
void enter_light_sleep()
{
    if (xSemaphoreTake(s_sleep_state_mutex, portMAX_DELAY) == pdTRUE) {
        if (s_system_is_sleeping) {
             // already sleeping
             xSemaphoreGive(s_sleep_state_mutex);
         return;
         }
         s_system_is_sleeping = true;
        // release，esp_light_sleep_start will block task
         xSemaphoreGive(s_sleep_state_mutex);
    } else {
         ESP_LOGE(TAG, "Failed to take sleep state mutex for entry!");
         return;
    }

    ESP_LOGI(TAG, "Entering Light Sleep...");

    // 1. stop LVGL tick timer
     if (lvgl_tick_timer) {
         esp_timer_stop(lvgl_tick_timer);
    }

    // 2. tell lvgl to block(no need)
     

    // 3. close backlight and panel
	if(panel_handle){
		esp_lcd_panel_disp_on_off(panel_handle, false); 
	}
    vTaskDelay(20 / portTICK_PERIOD_MS);
    gpio_set_level(GPIO_BL, 0); 

    // 4. wake up setting
    // int gpio 、 wakeup mode

    const int wakeup_gpio = EXAMPLE_PIN_NUM_TOUCH_INT;

    ESP_LOGI(TAG, "Wakeup GPIO is RTC-capable: %s", rtc_gpio_is_valid_gpio(wakeup_gpio) ? "Yes" : "No");
    
    ESP_LOGI(TAG, "Wakeup GPIO is wakeup gpio: %s", esp_sleep_is_valid_wakeup_gpio(wakeup_gpio) ? "Yes" : "No");
    const esp_sleep_ext1_wakeup_mode_t wakeup_mode = ESP_EXT1_WAKEUP_ANY_LOW; // any gpio low level wake up
    const uint64_t wakeup_pin_mask = (1ULL << wakeup_gpio); // set bit mask
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);    

    // set wake up source

    // gpio_wakeup_enable(wakeup_gpio, GPIO_INTR_LOW_LEVEL);
    // esp_err_t ret=esp_sleep_enable_gpio_wakeup();
    //  esp_err_t ret =  esp_sleep_enable_ext0_wakeup(wakeup_gpio,0);
     esp_err_t ret = esp_sleep_enable_ext1_wakeup(wakeup_pin_mask, wakeup_mode);
    
    vTaskDelay(200 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure external wakeup: %s", esp_err_to_name(ret));
		//if set error, recover to old state and set sleeping flag false
         if (xSemaphoreTake(s_sleep_state_mutex, portMAX_DELAY) == pdTRUE) {
             s_system_is_sleeping = false;
            xSemaphoreGive(s_sleep_state_mutex);
        }
		//resume display
        if(panel_handle){
			esp_lcd_panel_disp_on_off(panel_handle, true); 
		}
        vTaskDelay(20 / portTICK_PERIOD_MS);
		gpio_set_level(GPIO_BL, 1); 
		
		//resume lvgl tick timer
        if (lvgl_tick_timer) {
            esp_timer_start_periodic(lvgl_tick_timer, EXAMPLE_LVGL_TICK_PERIOD_MS * 1000);
        }
		//resume lvgl task
		return;
    }
    // 5. enter light sleep
    // *** important：make sure there is no lock before calling esp_light_sleep_start() ***
    ESP_LOGI(TAG, "Starting esp_light_sleep_start(). Task will block here.");
    int gpio_level = gpio_get_level(wakeup_gpio);
    ESP_LOGI(TAG, "GPIO%d level before sleep: %d", wakeup_gpio, gpio_level);
    esp_light_sleep_start(); // task block here , cpu enter sleep
    // *** program run here after wake up ***
    ESP_LOGI(TAG, "Wakeup from Light Sleep. Resuming normal operation.");

    // 6. check wake up reason (choosable，can differ from different logic)
    esp_sleep_wakeup_cause_t wakeup_cause = esp_sleep_get_wakeup_cause();
    if (wakeup_cause == ESP_SLEEP_WAKEUP_EXT1) {
        ESP_LOGI(TAG, "Wakeup cause: EXT1 (GPIO)");
    } else {
        ESP_LOGI(TAG, "Wakeup cause: Other (%d)", wakeup_cause);
    }

    // clear waka up state for next use
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_EXT1); 

    // 7. resume normal mode
    // restart display
    esp_lcd_panel_disp_on_off(panel_handle, true);
    vTaskDelay(20 / portTICK_PERIOD_MS);
    gpio_set_level(GPIO_BL, 1); 
    // restart LVGL tick timer
    if (lvgl_tick_timer) {
        // 如果計時器在睡眠前被停止，這裡重新啟動它
        esp_timer_start_periodic(lvgl_tick_timer, EXAMPLE_LVGL_TICK_PERIOD_MS * 1000);
    }

	//resume lvgl task

    // reset inactivity timer, start new cycle
    reset_inactivity_timer();

    // set sleeping flag false，using mutex to protect
    if (xSemaphoreTake(s_sleep_state_mutex, portMAX_DELAY) == pdTRUE) {
        s_system_is_sleeping = false;
        xSemaphoreGive(s_sleep_state_mutex);
    } else {
        ESP_LOGE(TAG, "Failed to take sleep state mutex for exit!");
    }

    ESP_LOGI(TAG, "System fully resumed.");
}

/**
*@brief batery task
**/
static void battery_task(void *arg){
	ESP_LOGI(TAG, "Starting battery task");
	esp_err_t err = ESP_OK;
    int raw_value;
    // choose battery read gpio num
    adc_channel_t channel = GPIO_NUM_1; 
    while(1){
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, channel, &raw_value));
        // ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Value: %d", ADC_UNIT_1 + 1, channel, raw_value);
    
        // 将原始 ADC 值转换为电压 (需要根据您的 ESP32 和 ADC 特性进行校准)
        // 假设 ESP32 ADC 的参考电压约为 3.3V，采样位数为 12 位 (默认)
        // 则电压 = raw_value * (3.3V / 4095)
        float voltage = raw_value * (3.3f / 4095.0f);
        float percentage = voltage *100 / 3.3f;
        //  ESP_LOGI(TAG, "percent : %.2f V", percentage);
    
        vTaskDelay(pdMS_TO_TICKS(1000)); // 每秒读取一次
    }
}

/**
*@brief qmi8658 task
**/
static void qmi8658_task(void *arg){
	ESP_LOGI(TAG, "Starting QMI8658 task");
	esp_err_t err = ESP_OK;
	err = qmi8658_init();		//init qmi8658
	motion_data_t curData;
    uint16_t curstep = 0;
	char step[8];
	while (1){
		err = qmi8658_accAndgry(&curData);		//qmi8658 get value
		if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to get acc/gyro!!");
        }
		filter_acc_ema(&curData);
		curstep = update_step_count(&curData);
        sprintf(step, "%d",curstep);
        lv_label_set_text(ui_stepCount, step);
        vTaskDelay(40 / portTICK_PERIOD_MS);		//25hz
	}
}

/**
*@brief rtc pcf85063 task
**/
static void rtc_pcf85063_task(void *arg)
{
    ESP_LOGI(TAG, "Starting RTC task");
	esp_err_t err = ESP_OK;
	 rtc_pcf85063_init();
	// err = rtc_set_time(12, 30, 45, 0,27, 9, 24); // Set the time
    
	// if (err == ESP_OK) {
    //     ESP_LOGI(TAG, "Time set successfully");
    // } else {
    //     ESP_LOGE(TAG, "Failed to set time");
    // }
	rtc_time now;
    while (1) {
        // Get and log the current time
        // UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
        // ESP_LOGI(TAG, "RTC task stack high water mark: %u bytes", uxHighWaterMark * configMINIMAL_STACK_SIZE);
        // ESP_LOGI(TAG, "RTC task is running, about to get time");
        
        err = rtc_get_time(&now);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to get time");
        }
        
        
		//delay for 1 second
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

/**
*@brief lcd init flow (commands and sleep sec)
**/
static const ili9341_lcd_init_cmd_t lcd_init_cmds[] = {
    /* {cmd, { data }, data_size, delay_ms} */ 
    {0x11, (uint8_t []){0x00}, 1, 120},
    {0xB2, (uint8_t []){0x0B, 0x0B, 0x00, 0x33, 0x33}, 5, 0},
    {0xB7, (uint8_t []){0x11}, 1, 0},
    {0xBB, (uint8_t []){0x2F}, 1, 0},
    {0xC0, (uint8_t []){0x2C}, 1, 0},
    {0xC2, (uint8_t []){0x01}, 1, 0},
    {0xC3, (uint8_t []){0x0D}, 1, 0},
    {0xC4, (uint8_t []){0x20}, 1, 0},
    {0xC6, (uint8_t []){0x13}, 1, 0},
    {0xD0, (uint8_t []){0xA4, 0xA1}, 2, 0},
    {0xD6, (uint8_t []){0xA1}, 1, 0},
    {0xE0, (uint8_t []){0xF0, 0x04, 0x07, 0x09, 0x07, 0x13, 0x25, 0x33, 0x3C, 0x34, 0x10, 0x10, 0x29, 0x32}, 14, 0},
    {0xE1, (uint8_t []){0xF0, 0x05, 0x08, 0x0A, 0x09, 0x05, 0x25, 0x32, 0x3B, 0x3B, 0x17, 0x18, 0x2E, 0x37}, 14, 0},
    {0xE4, (uint8_t []){0x25, 0x00, 0x00}, 3, 0},
    {0x21, (uint8_t []){0x00}, 1, 0},
    {0x29, (uint8_t []){0x00}, 1, 0},
    {0x2A, (uint8_t []){0x00, 0x00, 0x00, 0xEF}, 4, 0},
    {0x2B, (uint8_t []){0x00, 0x14, 0x01, 0x2B}, 4, 0},
    {0x2C, (uint8_t []){0x00}, 1, 0},
};

/**
*@brief lvgl disp flush ready
**/
static bool example_notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    lv_disp_drv_t  *disp = (lv_disp_drv_t  *)user_ctx;
    lv_disp_flush_ready(disp);
    return false;
}

/**
*@brief when lcd has been touched will trigger a signal send to touch mux
**/
static void touch_callback(esp_lcd_touch_handle_t tp)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(touch_mux, &xHigherPriorityTaskWoken);

    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

/**
*@brief lvgl flush call back
**/
static void example_lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    esp_lcd_panel_handle_t panel_handle_local = (esp_lcd_panel_handle_t) drv->user_data;
    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;
    /* Copy a buffer's content to a specific area of the display */
    // esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
    // uint64_t start_time = esp_timer_get_time();
    /* Copy a buffer's content to a specific area of the display */
    esp_lcd_panel_draw_bitmap(panel_handle_local, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
    // uint64_t end_time = esp_timer_get_time();
    // ESP_LOGI(TAG, "esp_lcd_panel_draw_bitmap took %llu us", (end_time - start_time));

}

/**
*@brief lvgl touch call back and use handle_user_activity to make sure lcd isn't sleeping
**/
static void example_lvgl_touch_cb(lv_indev_drv_t * drv, lv_indev_data_t * data)
{
    esp_lcd_touch_handle_t tp_local = (esp_lcd_touch_handle_t)drv->user_data;
    assert(tp_local);

    uint16_t tp_x;
    uint16_t tp_y;
    uint8_t tp_cnt = 0;
    /* Read data from touch controller into memory */
    if (xSemaphoreTake(touch_mux, 0) == pdTRUE) {
        // ESP_LOGI(TAG, "In call back func. GPIO%d = %d", EXAMPLE_PIN_NUM_TOUCH_INT, gpio_get_level(EXAMPLE_PIN_NUM_TOUCH_INT));
        if (xSemaphoreTake(i2c_mux, 10) == pdTRUE){
            esp_lcd_touch_read_data(tp_local);
            xSemaphoreGive(i2c_mux);
        }else{
            ESP_LOGI(TAG, "I2c busy! Touch read fail!");
        }
    }
    
    /* Read data from touch controller */
    bool tp_pressed = esp_lcd_touch_get_coordinates(tp_local, &tp_x, &tp_y, NULL, &tp_cnt, 1);
    if (tp_pressed && tp_cnt > 0)
    {
        data->point.x = tp_x;
        data->point.y = tp_y;
        data->state = LV_INDEV_STATE_PRESSED;
        ESP_LOGD(TAG, "Touch position: %d,%d", tp_x, tp_y);
        // ESP_LOGI(TAG, "In call back func. GPIO%d = %d", EXAMPLE_PIN_NUM_TOUCH_INT, gpio_get_level(EXAMPLE_PIN_NUM_TOUCH_INT));
		
        handle_user_activity();
    }
    else
    {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

/**
*@brief lvgl tick timer increase
**/
static void example_increase_lvgl_tick(void *arg)
{
    /* Tell LVGL how many milliseconds has elapsed */
    lv_tick_inc(EXAMPLE_LVGL_TICK_PERIOD_MS);

}

/**
*@brief lvgl semaphore lock
**/
static bool example_lvgl_lock(int timeout_ms)
{
    assert(lvgl_mux && "bsp_display_start must be called first");

    const TickType_t timeout_ticks = (timeout_ms == -1) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return xSemaphoreTake(lvgl_mux, timeout_ticks) == pdTRUE;
}

/**
*@brief lvgl semaphore unlock
**/
static void example_lvgl_unlock(void)
{
    assert(lvgl_mux && "bsp_display_start must be called first");
    xSemaphoreGive(lvgl_mux);
}

void wifi_ap_task(void *pvParameter) {
    start_wifi_ap();  // 原本的 Wi-Fi 初始化流程放這裡
    start_web_server();
    vTaskDelete(NULL);
}

/**
*@brief test switch wifi handler
**/
void wifi_switch_event_handler(lv_event_t *e)
{
    lv_obj_t * sw = lv_event_get_target(e);
    bool is_checked = lv_obj_has_state(sw, LV_STATE_CHECKED);

    if (is_checked) {
            printf("Wi-Fi Switch 開啟\n");
            xTaskCreate(wifi_ap_task, "wifi_ap_task", 4096, NULL, 5, NULL);
        } else {
            printf("Wi-Fi Switch 關閉\n");
            stop_wifi_ap();      // 你可以自己實作一個關閉 Wi-Fi 的函式
            stop_web_server();   // 如果需要關閉 Web Server
        
    }
}

/**
*@brief lvgl task
**/
static void example_lvgl_port_task(void *arg)
{

    ESP_LOGI(TAG, "Starting LVGL task");
    ESP_LOGI(TAG, "Display LVGL UI");
    
	//get lvgl task handle
    lvgl_task_handle = xTaskGetCurrentTaskHandle();
	//ui init
    ui_init();
    
    lv_obj_add_event_cb(ui_startbutton, start_button_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(ui_stopbutton, stop_button_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(ui_clearbutton, clear_button_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(ui_startbutton2, setTime_button_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(ui_wakeupSwitch, wifi_switch_event_handler, LV_EVENT_VALUE_CHANGED, NULL);


    uint32_t task_delay_ms = EXAMPLE_LVGL_TASK_MAX_DELAY_MS;
    while (1) {
		//check notification
        uint32_t ulNotificationValue;
        // ESP_LOGI(TAG, "In task. GPIO%d = %d", EXAMPLE_PIN_NUM_TOUCH_INT, gpio_get_level(EXAMPLE_PIN_NUM_TOUCH_INT));

        // ESP_LOGI(TAG, "GPIO%d = %d", EXAMPLE_PIN_NUM_TOUCH_INT, gpio_get_level(EXAMPLE_PIN_NUM_TOUCH_INT));
		BaseType_t notify_ret = xTaskNotifyWait(0x00, ULONG_MAX, &ulNotificationValue, 0); 
		if (notify_ret == pdTRUE) {
            // check sleep?
            if (ulNotificationValue & EVENT_ENTER_SLEEP_REQUEST) {
                ESP_LOGI(TAG, "LVGL Task: Received enter sleep request.");
                
                esp_lcd_panel_disp_on_off(panel_handle, false);
                gpio_set_level(GPIO_BL, 0);
                //enter_light_sleep();
                task_delay_ms = 0; // force to next round imediately（執行 lv_timer_handler）
                //continue; // skip lvgl lock
            }
			uint32_t unprocessed_bits = ulNotificationValue & ~(EVENT_ENTER_SLEEP_REQUEST );
            if (unprocessed_bits) {
                ESP_LOGW(TAG, "LVGL Task: Received unprocessed notification bits: 0x%08lX", unprocessed_bits);
            }
        }
		
        /* Lock the mutex due to the LVGL APIs are not thread-safe */
        if (example_lvgl_lock(-1)) {
            task_delay_ms = lv_timer_handler();
            /* Release the mutex */
            example_lvgl_unlock();
        }
        if (task_delay_ms > EXAMPLE_LVGL_TASK_MAX_DELAY_MS) {
            task_delay_ms = EXAMPLE_LVGL_TASK_MAX_DELAY_MS;
        } else if (task_delay_ms < EXAMPLE_LVGL_TASK_MIN_DELAY_MS) {
            task_delay_ms = EXAMPLE_LVGL_TASK_MIN_DELAY_MS;
        }
        vTaskDelay(pdMS_TO_TICKS(task_delay_ms));
    }
}

/**
*@brief i2c scanner, to check device address on i2c bus
**/
void i2c_scanner()
{
    printf("Scanning I2C bus...\n");
	if (xSemaphoreTake(i2c_mux, 10) == pdTRUE){
        for (uint8_t addr = 1; addr < 127; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 100 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
        if (ret == ESP_OK) {
            printf("Found device at 0x%02X\n", addr);
			}
		}
         xSemaphoreGive(i2c_mux);
    }else{
            ESP_LOGI(TAG, "I2c busy! Scan fail!");
    }
}

/**
*@brief simple task for testing
**/
void simple_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Simple task running");
    vTaskDelay(pdMS_TO_TICKS(1000));
    ESP_LOGI(TAG, "Simple task finished");
    vTaskDelete(NULL);
}

/**
*@brief init spiffs file system
**/
static esp_err_t init_spiffs(void)
{
    esp_vfs_spiffs_conf_t conf = {
      .base_path = "/spiffs",
      .partition_label = NULL,
      .max_files = 5,   // 可以同時打開的最大文件數
      .format_if_mount_failed = true
    };

    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount or format filesystem.Error: %s", esp_err_to_name(ret));
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "Failed to find SPIFFS partition");
        } else {
            ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
        return ret;
    }

    size_t total = 0, used = 0;
    ret = esp_spiffs_info(NULL, &total, &used);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
    }
    return ret;
}

/**
*@brief main function
**/
void app_main(void)
{
    esp_pm_config_esp32_t pm_config = {
        .max_freq_mhz = 160,  // max freq set 
        .min_freq_mhz = 80,  // min freq set
        .light_sleep_enable = true,  // light sleep allow
    };

    esp_pm_configure(&pm_config); 
    ESP_LOGI("PM", "CPU frequency set to 80 MHz");

    static lv_disp_draw_buf_t disp_buf;
    static lv_disp_drv_t disp_drv;
	
    touch_mux = xSemaphoreCreateBinary();
    assert(touch_mux);
    i2c_mux = xSemaphoreCreateMutex();
    assert(i2c_mux);
    lvgl_mux = xSemaphoreCreateMutex();
    assert(lvgl_mux);
    s_sleep_state_mutex=xSemaphoreCreateMutex();
    assert(s_sleep_state_mutex);
	

	/* =============== Initialize nvs and spiffs=============== */
    nvs_flash_erase();
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // 如果 NVS 初始化錯誤，則擦除並重新初始化
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ret = init_spiffs();
    ESP_ERROR_CHECK(ret);

	/* =============== Initialize gpio back light =============== */
    gpio_reset_pin(GPIO_BL);
	gpio_set_direction(GPIO_BL, GPIO_MODE_OUTPUT);
	gpio_set_level(GPIO_BL, 0);
    ESP_LOGI(TAG, "Initialize GPIO BL");

	/* =============== Initialize inactivity timer =============== */
    const esp_timer_create_args_t inactivity_timer_args = {
        .callback = &inactivity_timer_cb,
        .name = "inactivity"
    };
    ESP_ERROR_CHECK(esp_timer_create(&inactivity_timer_args, &inactivity_timer));


	/* =============== Initialize adc =============== */
    adc_oneshot_unit_init_cfg_t init_cfg1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg1, &adc1_handle));

    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN_DB_11, // set decrease (according to the input voltage)
        .bitwidth = ADC_BITWIDTH_DEFAULT, // set sampling digit 
    };


	/* =============== Initialize idle watch dog =============== */
    {
        esp_err_t err;
        TaskHandle_t main_task_handle = xTaskGetCurrentTaskHandle();
        if (main_task_handle == NULL) {
            ESP_LOGE(TAG, "Failed to get current task handle for main");
        }
        err = esp_task_wdt_add(main_task_handle);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to add main task to TWDT: %d", err);
        }
    }

	/* =============== Initialize spi bus =============== */
    ESP_LOGI(TAG, "Initialize SPI bus");
    const spi_bus_config_t buscfg = ILI9341_PANEL_BUS_SPI_CONFIG(EXAMPLE_PIN_NUM_SCLK,EXAMPLE_PIN_NUM_DATA0,
                                EXAMPLE_LCD_H_RES*EXAMPLE_LCD_V_RES*sizeof(uint16_t));
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));		//spi2、dma auto

    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    const esp_lcd_panel_io_spi_config_t io_config = ILI9341_PANEL_IO_SPI_CONFIG(EXAMPLE_PIN_NUM_LCD_CS,EXAMPLE_PIN_NUM_LCD_DC,
                    example_notify_lvgl_flush_ready,&disp_drv);
    // Attach LCD to the SPI bus
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle));

    ESP_LOGI(TAG, "Install panel driver");
    //esp_lcd_panel_handle_t panel_handle = NULL;
    ili9341_vendor_config_t vendor_config = {  // Uncomment these lines if use custom initialization commands
        .init_cmds = lcd_init_cmds,
        .init_cmds_size = sizeof(lcd_init_cmds) / sizeof(ili9341_lcd_init_cmd_t),
    };
    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = EXAMPLE_PIN_NUM_LCD_RST,      // Set to -1 if not use
        .rgb_endian = LCD_RGB_ENDIAN_RGB,
        .bits_per_pixel = 16,                           // Implemented by LCD command `3Ah` (16/18)
        .vendor_config = &vendor_config,            // Uncomment this line if use custom initialization commands
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_ili9341(io_handle, &panel_config, &panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    esp_lcd_panel_set_gap(panel_handle,0,20);
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
    gpio_set_level(GPIO_BL, 1);  

	/* =============== Initialize i2c bus =============== */
    ESP_LOGI(TAG, "Initialize I2C bus");
    int i2c_master_port = I2C_MASTER_NUM;		//i2c 0

    const i2c_config_t i2c_conf = {
        .mode               = I2C_MODE_MASTER,
        .sda_io_num         = EXAMPLE_PIN_NUM_TOUCH_SDA,
        .scl_io_num         = EXAMPLE_PIN_NUM_TOUCH_SCL,
        .sda_pullup_en      = GPIO_PULLUP_ENABLE,
        .scl_pullup_en      = GPIO_PULLUP_ENABLE,
        .master.clk_speed   = 400*1000,
    };

    ESP_ERROR_CHECK(i2c_param_config(i2c_master_port, &i2c_conf));
    ESP_ERROR_CHECK(i2c_driver_install(i2c_master_port, i2c_conf.mode, 0, 0, 0));
    ESP_LOGI(TAG, "Initialize touch components");
	
    esp_lcd_panel_io_handle_t tp_io_handle    = NULL;
    const esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_CST816S_CONFIG();

    esp_lcd_touch_config_t tp_cfg = {
        .x_max = EXAMPLE_LCD_H_RES,
        .y_max = EXAMPLE_LCD_V_RES,
        .rst_gpio_num = EXAMPLE_PIN_NUM_TOUCH_RST,
        .int_gpio_num = EXAMPLE_PIN_NUM_TOUCH_INT,
        .levels = {
            .reset = 0,
            .interrupt = 0,
        },
        .flags = {
            .swap_xy = 0,
            .mirror_x = 0,
            .mirror_y = 0,
        },
        .interrupt_callback = touch_callback,
    };

    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)I2C_MASTER_NUM, &tp_io_config, &tp_io_handle));
    ESP_ERROR_CHECK(esp_lcd_touch_new_i2c_cst816s(tp_io_handle, &tp_cfg, &tp));

    {
        uint8_t val = 0;
        if (tp != NULL) {
            esp_err_t auto_sleep_err = cst816s_write_byte(tp, 0xFE, 0x01); // 寫入非零值 (0x01) 到 DisAutoSleep 寄存器 (0xFE)
            if (auto_sleep_err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to disable CST816S auto sleep: %s", esp_err_to_name(auto_sleep_err));
            } else {
                 ESP_LOGI(TAG, "CST816S auto sleep disabled.");
            }
            cst816s_write_byte(tp, 0xed, 0x64);
            cst816s_read_byte(tp,0xed,&val,1);
            ESP_LOGI(TAG, "register ed val = %x.", val);
            cst816s_write_byte(tp, 0xfa, 0x71);//original 71
            cst816s_read_byte(tp,0xfa,&val,1);
            ESP_LOGI(TAG, "register fa val = %x.", val);
            cst816s_write_byte(tp, 0xfd, 0x04);
            cst816s_read_byte(tp,0xfd,&val,1);
            ESP_LOGI(TAG, "register fd val = %x.", val);
            cst816s_read_byte(tp,0xfe,&val,1);
            ESP_LOGI(TAG, "register fe val = %x.", val);
        }
    }
	/* =============== Initialize lvgl =============== */
    ESP_LOGI(TAG, "Initialize LVGL");
    lv_init();

    /* Alloc draw buffers used by LVGL */
    /* It's recommended to choose the size of the draw buffer(s) to be at least 1/10 screen sized */
    lv_color_t *buf1 = (lv_color_t *)heap_caps_malloc(EXAMPLE_LCD_H_RES * 50 * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf1);
    lv_color_t *buf2 = (lv_color_t *)heap_caps_malloc(EXAMPLE_LCD_H_RES * 50 * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf2);
    /* Initialize LVGL draw buffers */
    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, EXAMPLE_LCD_H_RES * 50);

    ESP_LOGI(TAG, "Register display driver to LVGL");
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res    = EXAMPLE_LCD_H_RES;
    disp_drv.ver_res    = EXAMPLE_LCD_V_RES;
    disp_drv.flush_cb   = example_lvgl_flush_cb;
    disp_drv.draw_buf   = &disp_buf;
    disp_drv.user_data  = panel_handle;
    lv_disp_t *disp     = lv_disp_drv_register(&disp_drv);

    ESP_LOGI(TAG, "Register touch driver to LVGL");
    static lv_indev_drv_t indev_drv;    /* Input device driver (Touch) */
    lv_indev_drv_init(&indev_drv);
    indev_drv.type      = LV_INDEV_TYPE_POINTER;
    indev_drv.disp      = disp;
    indev_drv.read_cb   = example_lvgl_touch_cb;
    indev_drv.user_data = tp;

    lv_indev_drv_register(&indev_drv);

    ESP_LOGI(TAG, "Install LVGL tick timer");
    /* Tick interface for LVGL (using esp_timer to generate 2ms periodic event) */
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback   = &example_increase_lvgl_tick,
        .name       = "lvgl_tick"
    };
    //esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, EXAMPLE_LVGL_TICK_PERIOD_MS * 1000));

	/* =============== Task =============== */
    ESP_LOGI(TAG, "Create LVGL Task");
     i2c_scanner();
    // xTaskCreate(example_lvgl_port_task, "LVGL", EXAMPLE_LVGL_TASK_STACK_SIZE, NULL, EXAMPLE_LVGL_TASK_PRIORITY, NULL);
	// xTaskCreate(rtc_pcf85063_task, "RTC", RTC_TASK_STACK_SIZE,NULL, 1, NULL);
	


    // create lvgl task pin to app core
    xTaskCreatePinnedToCore(example_lvgl_port_task, "LVGL", EXAMPLE_LVGL_TASK_STACK_SIZE, NULL, EXAMPLE_LVGL_TASK_PRIORITY, NULL, APP_CPU_NUM);
    xTaskCreatePinnedToCore(rtc_pcf85063_task, "RTC", RTC_TASK_STACK_SIZE, NULL, NORMAL_TASK_PRIORITY, NULL, PRO_CPU_NUM);

    /* =============== Initialize clock timer =============== */
    start_clock_timer();
    const esp_timer_create_args_t timer_args = {
        .callback = &time_counter_cb,
        .name = "timer_counter "
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &time_counter));


	// create rtc task pin to pro core
    xTaskCreatePinnedToCore(qmi8658_task, "QMI8658", QMI8658_TASK_STACK_SIZE, NULL, NORMAL_TASK_PRIORITY, NULL, PRO_CPU_NUM);
    xTaskCreatePinnedToCore(battery_task, "ADC", QMI8658_TASK_STACK_SIZE, NULL, NORMAL_TASK_PRIORITY, NULL, PRO_CPU_NUM);

    /* =============== Initialize System State =============== */
    if (xSemaphoreTake(s_sleep_state_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        s_system_is_sleeping = false;
        xSemaphoreGive(s_sleep_state_mutex);
    } else {
        ESP_LOGE(TAG, "Failed to take sleep state mutex during init!");
        // 如果互斥鎖創建被 assert OK 了，這裡獲取失敗可能意味著其他任務已經在異常使用它
        while(1) { vTaskDelay(pdMS_TO_TICKS(1000)); }
    }

    /* =============== Start Inactivity Timer =============== */
    reset_inactivity_timer();

    ESP_LOGI(TAG, "App main finished, entering idle loop.");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10));		//delay
        esp_task_wdt_reset();		// reset app_main watchdog
    }
}
