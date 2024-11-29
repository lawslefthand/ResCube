#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "hal/gpio_types.h"
#include "portmacro.h"
#include "rom/ets_sys.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "soc/gpio_num.h"
#include "ssd1306.h"
#include "esp_log.h"
#include <esp_intr_alloc.h>
#include "lora.h"
#include "driver/uart.h"
#include "stdint.h"
#include "time.h"
#include "esp_timer.h"
#include "driver/timer.h"
#include <string.h>
#include <inttypes.h>

#define BUF_SIZE (1024)
#define REFERENCE_PRESSURE 101325
#define TAG "BMP180_APP"
#define SOUND_SPEED_MPS 343.0


#define TRIG_PIN 4
#define ECHO_PIN 2



#define MAX_DISTANCE_CM 500 

#define GPS_UART_PORT_NUM      0
#define GPS_UART_BAUD_RATE     9600
#define GPS_UART_TX_PIN        1
#define GPS_UART_RX_PIN        3



char latitude[12];  
char longitude[12]; 



//semaphore mutex handle
SemaphoreHandle_t xMutex;

//task handles (add or reduce as you like)
TaskHandle_t handle1 = NULL;
TaskHandle_t handle2 = NULL;
TaskHandle_t handle3 = NULL;
TaskHandle_t handle4 = NULL;
TaskHandle_t handle5 = NULL;
TaskHandle_t handle6 = NULL;
TaskHandle_t menu_str = NULL;
TaskHandle_t menu_rtr = NULL;



void task_1(void *arg) {
	if (xSemaphoreTake(xMutex, portMAX_DELAY))
	{
    ESP_LOGI(TAG, "Task 1 is running");
    i2c_driver_delete(I2C_NUM_0);
        vTaskDelay(1000/portTICK_PERIOD_MS);
        int i;
        i = 0;
        char time[30];
        SSD1306_t dev;
        i2c_master_init(&dev, 21, 22, 27);
        ssd1306_clear_screen(&dev, false);
        ssd1306_init(&dev, 128, 64);
        ssd1306_contrast(&dev, 0xff);
    xSemaphoreGive(xMutex);
    while (1) {
		ESP_LOGI(TAG,"Task 1 is sill running");
		    i++;
            ssd1306_display_text_x3(&dev, 0, "Stop", 8, false);
            ssd1306_display_text_x3(&dev, 2, "Watch", 8, false);
            sprintf(time, "%i", i);
            ssd1306_display_text(&dev, 6, time, 7, false);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            ssd1306_clear_screen(&dev, false);

    }
    }
}

void parse_nmea(const char *nmea, char *latitude, char *longitude) {
    const char *start;
    char lat[12], lon[12];

  
    start = strstr(nmea, "$GPGGA");
    if (start != NULL) {
        sscanf(start, "$GPGGA,%*f,%11[^,],%*c,%11[^,],%*c", lat, lon);
        sprintf(latitude, "%s", lat);
        sprintf(longitude, "%s", lon);
    }
}



void task_2(void *arg) {
    if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
        ESP_LOGI(TAG, "Task 2 is running");
        i2c_driver_delete(I2C_NUM_0);
        vTaskDelay(1000/portTICK_PERIOD_MS);
        SSD1306_t dev;
        i2c_master_init(&dev, 21, 22, 27);
        ssd1306_clear_screen(&dev, false);
        ssd1306_init(&dev, 128, 64);
        ssd1306_contrast(&dev, 0xff);
        xSemaphoreGive(xMutex);
        while (1) {
			ESP_LOGI(TAG,"Task 2 is sill running");
	    ssd1306_clear_screen(&dev, false);
        ssd1306_display_text(&dev, 0, "Sending" ,7 , false);
        ssd1306_display_text_x3(&dev, 1, "SOS", 3, false);
        ssd1306_display_text(&dev, 5, "TXFreq:915",10, false);
            
			const uart_config_t uart_config = {
        .baud_rate = GPS_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_param_config(GPS_UART_PORT_NUM, &uart_config);
    uart_set_pin(GPS_UART_PORT_NUM, GPS_UART_TX_PIN, GPS_UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(GPS_UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);
    uint8_t data[BUF_SIZE];
    char latitude[12] = {0};
    char longitude[12] = {0};
     lora_init();

    while (1) {
        int length = uart_read_bytes(GPS_UART_PORT_NUM, data, BUF_SIZE - 1, 20 / portTICK_PERIOD_MS);
        if (length > 0) {
		    
            data[length] = '\0'; 
            ESP_LOGI(TAG, "Received: %s", data);

          
            parse_nmea((char *)data, latitude, longitude);

            
            ESP_LOGI(TAG, "Latitude: %s, Longitude: %s", latitude, longitude);

            // final payload
            char lora_payload[50];
            snprintf(lora_payload, sizeof(lora_payload), "Lat: %s, Lon: %s", latitude, longitude);
           
            lora_set_frequency(915e6);
            lora_enable_crc();
            lora_send_packet((uint8_t*)lora_payload, strlen(lora_payload));
            printf("Packet sent...\n");
          
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
        }
        
    }
}

void task_3(void *arg) {
    if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
        ESP_LOGI(TAG, "Task 3 is running");
        i2c_driver_delete(I2C_NUM_0);
        vTaskDelay(1000/portTICK_PERIOD_MS);
        SSD1306_t dev;
        i2c_master_init(&dev, 21, 22, 27);
        ssd1306_clear_screen(&dev, false);
        ssd1306_init(&dev, 128, 64);
        ssd1306_contrast(&dev, 0xff);
        xSemaphoreGive(xMutex);
        while (1) {
	
	ESP_LOGI(TAG,"Task 3 is sill running");
	char code[15];
    strcpy(code, "Flashing sos");
    ssd1306_clear_screen(&dev, false);
    ssd1306_display_text_x3(&dev, 0, "SOS", 3, false);
    ssd1306_display_text(&dev, 4, code, 15, false);
    ssd1306_display_text(&dev, 5, "...---...", 9, false);
		
// SOS: ... --- ...

gpio_set_level(17, 1);
gpio_set_level(16, 1);
vTaskDelay(300 / portTICK_PERIOD_MS); // .
gpio_set_level(17, 0);
gpio_set_level(16, 0);
vTaskDelay(300 / portTICK_PERIOD_MS);
gpio_set_level(17, 1);
gpio_set_level(16, 1);
vTaskDelay(300 / portTICK_PERIOD_MS); // .
gpio_set_level(17, 0);
gpio_set_level(16, 0);
vTaskDelay(300 / portTICK_PERIOD_MS);
gpio_set_level(17, 1);
gpio_set_level(16, 1);
vTaskDelay(300 / portTICK_PERIOD_MS); // .
gpio_set_level(17, 0);
gpio_set_level(16, 0);
vTaskDelay(300 / portTICK_PERIOD_MS); // Inter-character gap

gpio_set_level(17, 1);
gpio_set_level(16, 1);
vTaskDelay(800 / portTICK_PERIOD_MS); // -
gpio_set_level(17, 0);
gpio_set_level(16, 0);
vTaskDelay(300 / portTICK_PERIOD_MS);
gpio_set_level(17, 1);
gpio_set_level(16, 1);
vTaskDelay(800 / portTICK_PERIOD_MS); // -
gpio_set_level(17, 0);
gpio_set_level(16, 0);
vTaskDelay(300 / portTICK_PERIOD_MS);
gpio_set_level(17, 1);
gpio_set_level(16, 1);
vTaskDelay(800 / portTICK_PERIOD_MS); // -
gpio_set_level(17, 0);
gpio_set_level(16, 0);
vTaskDelay(300 / portTICK_PERIOD_MS); // Inter-character gap

gpio_set_level(17, 1);
gpio_set_level(16, 1);
vTaskDelay(300 / portTICK_PERIOD_MS); // .
gpio_set_level(17, 0);
gpio_set_level(16, 0);
vTaskDelay(300 / portTICK_PERIOD_MS);
gpio_set_level(17, 1);
gpio_set_level(16, 1);
vTaskDelay(300 / portTICK_PERIOD_MS); // .
gpio_set_level(17, 0);
gpio_set_level(16, 0);
vTaskDelay(300 / portTICK_PERIOD_MS);
gpio_set_level(17, 1);
gpio_set_level(16, 1);
vTaskDelay(300 / portTICK_PERIOD_MS); // .
gpio_set_level(17, 0);
gpio_set_level(16, 0);
vTaskDelay(800 / portTICK_PERIOD_MS); // Inter-word gap


        }
        
    }
}

void init_timer() {
    timer_config_t config = {
        .divider = 16,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_DIS,
        .auto_reload = TIMER_AUTORELOAD_DIS,
    };
    timer_init(TIMER_GROUP_0, TIMER_0, &config);
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0x00000000ULL);
}

uint32_t measure_distance() {
    gpio_set_level(TRIG_PIN, 0);
    ets_delay_us(2);
    gpio_set_level(TRIG_PIN, 1);
    ets_delay_us(20);
    gpio_set_level(TRIG_PIN, 0);

    while (gpio_get_level(ECHO_PIN) == 0) {
    }

    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0x00000000ULL);
    timer_start(TIMER_GROUP_0, TIMER_0);

    while (gpio_get_level(ECHO_PIN) == 1) {
        
    }

    timer_pause(TIMER_GROUP_0, TIMER_0);
    uint64_t echo_time;
    timer_get_counter_value(TIMER_GROUP_0, TIMER_0, &echo_time);

    float distance = (echo_time * 0.343) / 2;
    return (uint32_t)distance;
}

void task_4(void *arg) {
    if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
        ESP_LOGI(TAG, "Task 4 is running");
        i2c_driver_delete(I2C_NUM_0);
        vTaskDelay(1000/portTICK_PERIOD_MS);
        SSD1306_t dev;
        i2c_master_init(&dev, 21, 22, 27);
        ssd1306_clear_screen(&dev, false);
        ssd1306_init(&dev, 128, 64);
        ssd1306_contrast(&dev, 0xff);
        xSemaphoreGive(xMutex);
        while (1) {
			ESP_LOGI(TAG,"Task 4 is sill running");
			uint32_t distance = measure_distance();
            distance = distance/10;
            char dist_str[15];
            snprintf(dist_str, sizeof(dist_str), "%" PRIu32 " cm", distance);
            ssd1306_clear_screen(&dev, false);
            ssd1306_display_text_x3(&dev, 0, "Range", 5, false);
            ssd1306_display_text(&dev, 4, "Finder", 6, false);
            ssd1306_display_text(&dev, 5, dist_str, strlen(dist_str), false);
            ESP_LOGI(TAG, "Distance: %" PRIu32 " cm", distance);
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        xSemaphoreGive(xMutex);
    }
}

void menu_return(void *arg)
{
	if(xSemaphoreTake(xMutex, portMAX_DELAY))
	{
	 ESP_LOGI(TAG, "Menu rtr is running");
     i2c_driver_delete(I2C_NUM_0);
     vTaskDelay(1000/portTICK_PERIOD_MS);
     SSD1306_t dev;
     i2c_master_init(&dev, 21, 22, 27);
     ssd1306_clear_screen(&dev, false);
     ssd1306_init(&dev, 128, 64);
     ssd1306_contrast(&dev, 0xff);
     xSemaphoreGive(xMutex);	
     while(1)
     {
        ssd1306_clear_screen(&dev, false);
        ssd1306_display_text_x3(&dev, 0, "ResCb", 20, false);
        ssd1306_display_text(&dev, 3, "by Aryan B. ", 20, false);
        ssd1306_display_text(&dev, 5, "Btn1 for SOS_TX", 20, false);
        ssd1306_display_text(&dev, 4, "Btn2 for StopWtch ", 20, false);
        ssd1306_display_text(&dev, 5, "Btn1 for SOS_TX", 20, false);
        ssd1306_display_text(&dev, 6, "Btn3 for Morse_SOS", 20, false);
        ssd1306_display_text(&dev, 7, "Btn4 for Menu", 20, false); 
        vTaskDelay(200/portTICK_PERIOD_MS);
	 }
	}
}

//main menu init task (this task HAS to keep running)
void menu_init(void *arg)
{
	ESP_LOGI(TAG, "Menu is running");
    vTaskDelay(1000/portTICK_PERIOD_MS);
    SSD1306_t dev;
    i2c_master_init(&dev, 21, 22, 27);
    ssd1306_clear_screen(&dev, false);
    ssd1306_init(&dev, 128, 64);
    ssd1306_contrast(&dev, 0xff);
    ssd1306_clear_screen(&dev, false);
        ssd1306_display_text_x3(&dev, 0, "ResCb", 20, false);
        ssd1306_display_text(&dev, 3, "by Aryan B. ", 20, false);
        ssd1306_display_text(&dev, 5, "Btn1 for SOS_TX", 20, false);
        ssd1306_display_text(&dev, 4, "Btn2 for StopWtch ", 20, false);
        ssd1306_display_text(&dev, 5, "Btn1 for SOS_TX", 20, false);
        ssd1306_display_text(&dev, 6, "Btn3 for Morse_SOS", 20, false);
        ssd1306_display_text(&dev, 7, "Btn4 for Menu", 20, false); 
    while (1)
    {
		ESP_LOGI(TAG, "Menu is still running");
        //example input gpios (you can use any gpio of your choice)  
       
        int sound_check_flag= 0;
        int gpio_25 = gpio_get_level(25);
        int gpio_26 = gpio_get_level(26);
        int gpio_33 = gpio_get_level(33);
        int gpio_32 = gpio_get_level(32);
        int gpio_4 = gpio_get_level(4);

        printf("GPIO 25: %d\n", gpio_25);
        vTaskDelay(400 / portTICK_PERIOD_MS);
        printf("GPIO 26: %d\n", gpio_26);
        vTaskDelay(400 / portTICK_PERIOD_MS);
        printf("GPIO 33: %d\n", gpio_33);
        vTaskDelay(400 / portTICK_PERIOD_MS); 
        printf("GPIO 32: %d\n", gpio_32);
        vTaskDelay(400 / portTICK_PERIOD_MS); 
        printf("GPIO 4: %d\n", gpio_4);
        vTaskDelay(400 / portTICK_PERIOD_MS);
      
           
       if ((gpio_4 == 0))
       {
           gpio_set_level(17, 0);
           gpio_set_level(16, 0);
          vTaskDelay(100/ portTICK_PERIOD_MS);
          gpio_set_level(17, 1);
          gpio_set_level(16, 1);
          vTaskDelay               (2000 / portTICK_PERIOD_MS); 
          gpio_set_level(17, 0);
          gpio_set_level(16, 0);
          vTaskDelay(100 / portTICK_PERIOD_MS);
          gpio_set_level(17, 1);
          gpio_set_level(16, 1);
          vTaskDelay(2000 / portTICK_PERIOD_MS);
          gpio_set_level(17, 0);
          gpio_set_level(16, 0);
          ESP_LOGI(TAG, "ALARM");
	   }
         else if (gpio_25 == 1 && handle1 == NULL) {
            xTaskCreatePinnedToCore(&task_1, "Task 1", 4096, NULL, 5, &handle1, 1);
            if (handle2) vTaskDelete(handle2);
            if (handle3) vTaskDelete(handle3);
            if (handle4) vTaskDelete(handle4);
            if (handle5) vTaskDelete(handle5);
            if (handle6) vTaskDelete(handle6);
            if (menu_rtr) vTaskDelete(handle6);
            sound_check_flag = 0;
        menu_rtr = handle2 = handle3 = handle4 = handle5 = handle6 = NULL;
        } else if (gpio_26 == 1 && handle2 == NULL) {
            xTaskCreatePinnedToCore(&task_2, "Task 2", 4096*2, NULL, 5, &handle2, 1);
            if (handle1) vTaskDelete(handle1);
            if (handle3) vTaskDelete(handle3);
            if (handle4) vTaskDelete(handle4);
            if (handle5) vTaskDelete(handle5);
            if (handle6) vTaskDelete(handle6);
            if (menu_rtr) vTaskDelete(handle6);
            sound_check_flag = 0;
          menu_rtr =  handle1 = handle3 = handle4 = handle5 = handle6 = NULL;
        } else if (gpio_33 == 1 && handle3 == NULL) {
            xTaskCreatePinnedToCore(&task_3, "Task 3", 4096, NULL, 5, &handle3, 1);
            if (handle1) vTaskDelete(handle1);
            if (handle2) vTaskDelete(handle2);
            if (handle4) vTaskDelete(handle4);
            if (handle5) vTaskDelete(handle5);
            if (handle6) vTaskDelete(handle6);
            if (menu_rtr) vTaskDelete(handle6);
            sound_check_flag = 1;
        menu_rtr = handle1 = handle2 = handle4 = handle5 = handle6 = NULL;
        } else if (gpio_32== 1 && handle4 == NULL) {
            xTaskCreatePinnedToCore(&menu_return, "Task 4", 4096, NULL, 5, &menu_rtr, 1);
            if (handle1) vTaskDelete(handle1);
            if (handle2) vTaskDelete(handle2);
            if (handle3) vTaskDelete(handle3);
            if (handle5) vTaskDelete(handle5);
            if (handle6) vTaskDelete(handle6);
            if (menu_rtr) vTaskDelete(handle6);
            sound_check_flag = 0;
         menu_rtr =  handle1 = handle2 = handle3 = handle5 = handle6 = NULL;
      
       
        } 
}
}




void app_main()
{
   	//gpio configuration
    gpio_set_direction(25, GPIO_MODE_INPUT);
    gpio_set_direction(26, GPIO_MODE_INPUT);
    gpio_set_direction(33, GPIO_MODE_INPUT);
    gpio_set_direction(32, GPIO_MODE_INPUT);
    gpio_set_direction(17, GPIO_MODE_OUTPUT);
    gpio_set_direction(16, GPIO_MODE_OUTPUT);
    gpio_set_direction(34, GPIO_MODE_INPUT);
   
    
    
    gpio_config_t io_conf;
    io_conf.intr_type= GPIO_INTR_DISABLE;
    io_conf.mode=GPIO_MODE_INPUT;
    io_conf.pin_bit_mask =  (1ULL << 25) | (1ULL << 26) | (1ULL << 33) | (1ULL << 32) | (1ULL << 4); 
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    gpio_config(&io_conf);
    
    xMutex = xSemaphoreCreateMutex();
    xSemaphoreGive(xMutex);
    xTaskCreatePinnedToCore(&menu_init, "Main menu task", 4092 * 2, NULL, 6, &menu_str, 0);
}
