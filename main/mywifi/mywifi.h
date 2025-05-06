#ifndef WIFI_H
#define WIFI_H


#include "esp_wifi.h"
#include <esp_http_server.h>
#include <sys/param.h>
#include <time.h>
#include <sys/time.h>
#include "esp_netif.h"
#include "esp_log.h"
#include "../rtc/pcf85063.h"
#include <ctype.h>
#include <stdio.h>
#include "../qmi8658/qmi8658.h"


/**
*@brief Start wifi ap mode
**/
void start_wifi_ap();

/**
*@brief start http web server
**/
void start_web_server();

/**
*@brief stop wifi ap mode
**/
void stop_wifi_ap();

/**
*@brief stop web server
**/
void stop_web_server();

#endif