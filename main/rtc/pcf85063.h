#define PCF85063_ADDR        0x51      // PCF85063 I2C address
#define I2C_TIMEOUT_MS       1000
#define I2C_MASTER_NUM       0 // I2C port number



typedef enum {
	SUNDAY=0,
	MONDAY,
	TUESDAY,
	WEDNESDAY,
	THURSDAY,
	FRIDAY,
	SATURDAY,
}DAY;

typedef enum{
	JANUARY = 0x1,
	FABRUARY,
	MARCH,
	APRIL,
	MAY,
	JUNE,
	JULY,
	AUGUST,
	SEPTEMBER,
	OCTOBER = 0x10,
	NOVEMBER,
	DECEMBER,
}MONTH;

/**
*@brief struct; Set time struct
**/
typedef struct{
	uint8_t hours; 
	uint8_t minutes; 
	uint8_t seconds; 
	uint8_t day;
	DAY weekday; 
	MONTH month; 
	uint8_t year;
}rtc_time;

/**
*@brief Read RTC register function
**/
esp_err_t rtc_read_reg(uint8_t reg_addr, uint8_t *data, size_t len);

/**
*@brief Write RTC register function
**/
esp_err_t rtc_write_reg(uint8_t reg_addr, uint8_t *data, size_t len);

/**
*@brief BCD to decimal conversion
**/
uint8_t bcd_to_dec(uint8_t val) ;

/**
*@brief Decimal to BCD conversion
**/
uint8_t dec_to_bcd(uint8_t val) ;

/**
*@brief Function to get current time from the RTC
**/
esp_err_t rtc_get_time(rtc_time *time_out) ;

/**
*@brief Function to set time on the RTC
**/
esp_err_t rtc_set_time(rtc_time *time_out) ;

/**
*@brief init rtc
**/
esp_err_t rtc_pcf85063_init() ;

/**
*@brief set alarm
**/
void rtc_set_alarm(rtc_time *time)