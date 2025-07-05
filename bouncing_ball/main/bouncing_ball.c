#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/timer.h"
#include "esp_log.h"

#define TAG "BALL"

// === CONFIG MACROS ===
#define INIT_X 1
#define INIT_Y 1
#define INIT_VX -1
#define INIT_VY 1
#define END_X 6
#define END_Y 4
#define FRAME_RATE 5  // frames per second

// SPI & Display pins
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   5

spi_device_handle_t spi;

int ball_x = INIT_X;
int ball_y = INIT_Y;
int vel_x = INIT_VX;
int vel_y = INIT_VY;
int frame_count = 0;
volatile bool frame_ready = false;
bool finish = false;


uint8_t framebuffer[8] = {0};

// === MAX7219 SPI SEND ===
void max7219_send(uint8_t address, uint8_t data) {
    spi_transaction_t t = {
        .flags = SPI_TRANS_USE_TXDATA,
        .length = 16,
    };
    uint16_t packet = (address << 8) | data;
    t.tx_data[0] = (packet >> 8) & 0xFF;  // 高位元先送
    t.tx_data[1] = packet & 0xFF;
    spi_device_transmit(spi, &t);
}

// === MAX7219 INIT ===
void max7219_init() {
    max7219_send(0x0F, 0x00); // Test mode off
    max7219_send(0x0C, 0x01); // Shutdown off
    max7219_send(0x0B, 0x07); // Scan limit 8 rows
    max7219_send(0x0A, 0x08); // Brightness
    max7219_send(0x09, 0x00); // No decode mode

    // Clear display
    for (int i = 1; i <= 8; i++) max7219_send(i, 0x00);
}

// === FRAMEBUFFER UPDATE ===
void update_framebuffer() {
    memset(framebuffer, 0, sizeof(framebuffer));

    // Wall at x = 0 (leftmost)
    for (int i = 0; i < 8; i++) framebuffer[i] |= 0x80;

    // Ball
    if (ball_x >= 0 && ball_x <= 7 && ball_y >= 0 && ball_y <= 7) {
        framebuffer[ball_y] |= (1 << (7 - ball_x)); // x=0 is MSB
    }
}

// === DRAW TO DISPLAY ===
void draw_display() {
    for (int i = 0; i < 8; i++) {
        max7219_send(i + 1, framebuffer[i]);
    }
}

// === BALL MOVEMENT LOGIC ===
void update_ball() {
    if (ball_x >= 7 || ball_x <= 1) vel_x *= -1; // bounce from wall or right
    if (ball_y >= 7 || ball_y <= 0) vel_y *= -1; // top/bottom
    ball_x += vel_x;
    ball_y += vel_y;

    
}

// === TIMER ISR ===
bool IRAM_ATTR timer_isr_callback(void *arg) {
    update_ball();
    frame_ready = true;  // 通知主程式該畫畫了

    frame_count++;

    if (ball_x == END_X && ball_y == END_Y) {
        timer_pause(TIMER_GROUP_0, TIMER_0);
        finish = true;
    }

    return pdFALSE;
}

// === TIMER INIT ===
void timer_setup() {
    timer_config_t config = {
        .divider = 80,  // 1 us per tick
        .counter_dir = TIMER_COUNT_UP,
        .alarm_en = TIMER_ALARM_EN,
        .auto_reload = true,
    };
    timer_init(TIMER_GROUP_0, TIMER_0, &config);
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, 1000000 / FRAME_RATE);
    timer_enable_intr(TIMER_GROUP_0, TIMER_0);
    timer_isr_callback_add(TIMER_GROUP_0, TIMER_0, timer_isr_callback, NULL, 0);
    timer_start(TIMER_GROUP_0, TIMER_0);
}

// === SPI INIT ===
void spi_setup() {
    esp_err_t ret;

    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = -1,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = PIN_NUM_CS,
        .queue_size = 1,
    };



    ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(ret);  // 只有當錯誤不是「已初始化」時才報錯
    }

    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);
}

// === MAIN APP ===
void app_main(void) {
    spi_setup();
    max7219_init();
    update_framebuffer();
    draw_display();
    timer_setup();
    while (!finish) {
        if (frame_ready) {
            frame_ready = false;
            update_framebuffer();
            draw_display();  // ⛳現在在主 loop 安全使用
        }
        vTaskDelay(pdMS_TO_TICKS(1));  // 小睡一下
    }
    printf("Ball reached end. Frame count: %d\n", frame_count);
}