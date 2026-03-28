#include "rpi_uart.hpp"

#include <cstdio>
#include <cstring>

#include "coordinator.hpp"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static constexpr uart_port_t kRpiUartPort = UART_NUM_1;
static constexpr gpio_num_t  kRpiTxPin    = GPIO_NUM_43;
static constexpr gpio_num_t  kRpiRxPin    = GPIO_NUM_44;
static constexpr int         kRpiBaud     = 115200;
static constexpr int         kRxBufSize   = 256;

void rpi_uart_task(void *p)
{
    (void)p;

    const uart_config_t cfg = {
        .baud_rate           = kRpiBaud,
        .data_bits           = UART_DATA_8_BITS,
        .parity              = UART_PARITY_DISABLE,
        .stop_bits           = UART_STOP_BITS_1,
        .flow_ctrl           = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk          = UART_SCLK_DEFAULT,
    };
    uart_param_config(kRpiUartPort, &cfg);
    uart_set_pin(kRpiUartPort, kRpiTxPin, kRpiRxPin,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(kRpiUartPort, kRxBufSize, 0, 0, NULL, 0);

    char line[128];
    int  line_len = 0;

    while (true) {
        uint8_t byte;
        if (uart_read_bytes(kRpiUartPort, &byte, 1, pdMS_TO_TICKS(50)) <= 0) {
            continue;
        }

        if (byte == '\n' || byte == '\r') {
            if (line_len == 0) {
                continue;
            }
            line[line_len] = '\0';
            line_len = 0;

            float x = 0.f;
            float y = 0.f;
            float z = 0.f;
            char  extra = '\0';
            if (std::sscanf(line, "xyz %f %f %f %c", &x, &y, &z, &extra) == 3
                && coordinator_set_manual_xyz(x, y, z)) {
                uart_write_bytes(kRpiUartPort, "ok\n", 3);
            } else {
                uart_write_bytes(kRpiUartPort, "err\n", 4);
            }
        } else if (line_len < static_cast<int>(sizeof(line)) - 1) {
            line[line_len++] = static_cast<char>(byte);
        }
    }
}
