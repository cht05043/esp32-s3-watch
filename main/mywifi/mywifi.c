#include "mywifi.h"

static httpd_handle_t server = NULL;
static esp_netif_t *ap_netif = NULL;
static bool wifi_initialized = false;

/**
*@brief Start wifi ap mode
**/
void start_wifi_ap()
{ 
    if (!wifi_initialized){
        // STEP 1: init netif
        esp_netif_init();
        esp_event_loop_create_default();

        ap_netif = esp_netif_create_default_wifi_ap();
        assert(ap_netif != NULL);

        wifi_initialized = true;
    }
      
  
      // STEP 2: stop all wifi setting
      esp_wifi_stop();
      esp_wifi_deinit();
  
      // STEP 3: init wifi config
      wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
      esp_err_t ret = esp_wifi_init(&cfg);
      if (ret != ESP_OK) {
        ESP_LOGE("Wi-Fi", "WiFi init failed: %s", esp_err_to_name(ret));
        return;
    }
  
      // SET AP mode
      esp_wifi_set_mode(WIFI_MODE_AP);
  
      // Set AP config
      wifi_config_t ap_config = {
          .ap = {
              .ssid = "Watch_Setup",
              .password = "12345678",
              .max_connection = 1,
              .authmode = WIFI_AUTH_WPA_WPA2_PSK
          }
      };
      esp_wifi_set_config(WIFI_IF_AP, &ap_config);
  
      // Start WiFi ap
      esp_wifi_start();
  
  
      ESP_LOGI("Wi-Fi", "Wi-Fi AP mode start: Watch_Setup");
}

/**
*@brief translate hex value
**/
int hex_value(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return 0;
}

/**
*@brief decode recv string from recv package
**/
void url_decode(const char *src, char *dst) {
    while (*src) {
        if (*src == '%' && isxdigit((unsigned char) *(src + 1)) && isxdigit((unsigned char) *(src + 2))) {
            *dst++ = (char)((hex_value(*(src + 1)) << 4) | hex_value(*(src + 2)));
            src += 3;
        } else {
            *dst++ = *src++;
        }
    }
    *dst = '\0';
}

/**
*@brief qmi8658 step get handler
**/
esp_err_t step_get_handler(httpd_req_t *req){
    int step_count = get_step_count();
    char step_str[64];
    snprintf(step_str,sizeof(step_str),"%d",step_count);
    
    httpd_resp_set_type(req, "text/plain; charset=UTF-8");
    httpd_resp_sendstr(req, step_str);
    return ESP_OK;
}

/**
*@brief pcf85063 time get handler
**/
esp_err_t time_get_handler(httpd_req_t *req)
{
    rtc_time now;
    esp_err_t ret = rtc_get_time(&now);
    if (ret != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "RTC read failed");
        return ESP_FAIL;
    }

    char time_str[64];
    snprintf(time_str, sizeof(time_str), "%04d-%02d-%02d %02d:%02d:%02d",
             now.year + 2000 , now.month, now.day,
             now.hours, now.minutes, now.seconds);

    // printf("get time：%d-%d-%d %02d:%02d:%02d\n",
    //     now.year, now.month, now.day,
    //     now.hours, now.minutes, now.seconds);
    httpd_resp_set_type(req, "text/plain; charset=UTF-8");
    httpd_resp_sendstr(req, time_str);
    return ESP_OK;
}

/**
*@brief Set time handler
**/
esp_err_t time_post_handler(httpd_req_t *req)
{
    char buf[100];
    int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (ret <= 0) return ESP_FAIL;

    buf[ret] = '\0';

    char *time_str = strstr(buf, "datetime=");
    if (time_str) {
        time_str += 9;

        char decoded_time[64];
        url_decode(time_str, decoded_time);//%3A

        struct tm tm_time = {0};
        if (strptime(decoded_time, "%Y-%m-%dT%H:%M", &tm_time) == NULL) {
            ESP_LOGE("TIME", "strptime parsing failed for string: %s", decoded_time);
            return ESP_FAIL;
        }

        time_t t = mktime(&tm_time);
        struct timeval now = {.tv_sec = t};
        settimeofday(&now, NULL);

        rtc_time current_time;
        current_time.year = tm_time.tm_year + 1900 - 2000;
        current_time.month = tm_time.tm_mon + 1;
        current_time.day = tm_time.tm_mday;
        current_time.hours = tm_time.tm_hour ;
        current_time.minutes = tm_time.tm_min ;
        current_time.seconds = tm_time.tm_sec;
        current_time.weekday = (DAY)tm_time.tm_wday ;

        esp_err_t ret = rtc_set_time(&current_time);
        if(ret !=ESP_OK){
            ESP_LOGE("Wi-Fi:", "Failed to SET TIME: %s", esp_err_to_name(ret));
        }
        printf("Set time：%d-%d-%d %02d:%02d:%02d\n",
            current_time.year, current_time.month, current_time.day,
            current_time.hours, current_time.minutes, current_time.seconds);

    }

    httpd_resp_set_type(req, "text/plain; charset=UTF-8");
    httpd_resp_sendstr(req, "Set Time complete！");
    return ESP_OK;
}

/**
*@brief default page handler(html)
**/
esp_err_t root_get_handler(httpd_req_t *req)
{
    FILE *f = fopen("/spiffs/index.html", "r");
    if (f == NULL) {
        ESP_LOGE("Web Server", "Failed to open index.html");
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }
        // read file and send to client
        char buffer[512];
        size_t read_bytes;
        httpd_resp_set_type(req, "text/html; charset=UTF-8");
    
        while ((read_bytes = fread(buffer, 1, sizeof(buffer), f)) > 0) {
            httpd_resp_send_chunk(req, buffer, read_bytes);
        }
    
        fclose(f);  // close
        httpd_resp_send_chunk(req, NULL, 0);  // tell server all data is done
        return ESP_OK;
}

/**
*@brief static file get handler (css js)。For web can load css and javascript(init one time)
**/
esp_err_t static_file_get_handler(httpd_req_t *req)
{
    const char *uri = req->uri;
    char path[128];

    // according uri to set path right
    if (strcmp(uri, "/style.css") == 0) {
        snprintf(path, sizeof(path), "/spiffs/style.css");
    } else if (strcmp(uri, "/script.js") == 0) {
        snprintf(path, sizeof(path), "/spiffs/script.js");
    } else {
        ESP_LOGE("Web Server", "URI '%s' not found", uri);
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    // open file
    FILE *f = fopen(path, "r");
    if (f == NULL) {
        ESP_LOGE("Web Server", "Failed to open file: %s", path);
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    char buffer[512];
    size_t read_bytes;
    httpd_resp_set_type(req, "text/css; charset=UTF-8");  // set Content-Type，CSS
    if (strstr(uri, "script.js") != NULL) {
        httpd_resp_set_type(req, "application/javascript; charset=UTF-8");  // JavaScript type
    }

    while ((read_bytes = fread(buffer, 1, sizeof(buffer), f)) > 0) {
        httpd_resp_send_chunk(req, buffer, read_bytes);
    }

    fclose(f);  // close
    httpd_resp_send_chunk(req, NULL, 0);  // stop response
    return ESP_OK;
}

/**
*@brief start http web server
**/
void start_web_server()
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 8192;  // 

    httpd_start(&server, &config);

    httpd_uri_t root = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = root_get_handler
    };
    httpd_register_uri_handler(server, &root);

    httpd_uri_t set_time = {
        .uri = "/set_time",
        .method = HTTP_POST,
        .handler = time_post_handler
    };
    httpd_register_uri_handler(server, &set_time);

    httpd_uri_t get_time = {
        .uri = "/time",
        .method = HTTP_GET,
        .handler = time_get_handler
    };
    httpd_register_uri_handler(server, &get_time);

    httpd_uri_t get_step = {
        .uri = "/steps",
        .method = HTTP_GET,
        .handler = step_get_handler
    };
    httpd_register_uri_handler(server, &get_step);

    httpd_uri_t style_css = {
        .uri = "/style.css",
        .method = HTTP_GET,
        .handler = static_file_get_handler
    };
    httpd_register_uri_handler(server, &style_css);

    httpd_uri_t script_js = {
        .uri = "/script.js",
        .method = HTTP_GET,
        .handler = static_file_get_handler
    };
    httpd_register_uri_handler(server, &script_js);
}

/**
*@brief stop wifi ap mode
**/
void stop_wifi_ap()
{
    esp_wifi_stop();
    esp_wifi_deinit();
    printf("Wi-Fi CLOSE\n");
}

/**
*@brief stop web server
**/
void stop_web_server()
{
    if (server) {
        httpd_stop(server);
        server = NULL;
        printf("Web Server STOP\n");
    }
}

