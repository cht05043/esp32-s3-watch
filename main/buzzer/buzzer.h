#ifndef BUZZER_H
#define BUZZER_H

/**
*@brief buzzer for alarm use Init function (Using pwm)
**/
void buzzer_pwm_init();

/**
*@brief buzzer beep function for(duration_ms) seconds
**/
void buzzer_beep(uint32_t duration_ms);

#endif