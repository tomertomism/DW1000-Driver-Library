#DW1000 Driver Library 

![](https://img.shields.io/badge/license-Apache_2.0_license-gree)

- Using library version 05.01.00

- Written in 100% C language

##What to do

- Copy all `.h` file to your folder

- Copy `my_DW1000.c` to your folder

- Copy `my_DW1000_f10x.c` OR `my_DW1000_g070.c` to your folder, they contain SPI read/write functions and the `dw1000_reset()`, remember to change GPIO pins to fit your application.

> If your project using HAL Library, `my_DW1000_g070.c` is suitable for you.

> If your project using standard peripheral library,  choose `my_DW1000_f10x.c`.

> If both aforementioned situations are not applicable to you, you can create your own `my_DW1000_xxxx.c`.

- Below table shows some functions need to be wrote by yourself

| Function name          | Description                                |
| ---------------------- | ------------------------------------------ |
| `set_spi_low()`        | Set SPI speed to lower speed.              |
| `set_spi_high()`       | Set SPI speed to higher speed.             |
| `UART_Send()`          | Sending information via USART. If you are not using USART, just delete all `UART_Send()` functions and all `sprintf()`. |
| `set_delayclk_us()`    | A delay function in microseconds (us).     |
| `set_delayclk_ms()`    | A delay function in milliseconds (ms).     |

##Variables

- Use these global variable if you want

- Beware the order sequence when read/write `addr_data`

| Name              | Description                       |
| ----------------- | --------------------------------- |
| `u8 addr_data[6]` | Read/write buffer used in SPI.    |
| `char buf[100]`   | USART sending buffer.             |

- Define it if you want

| Name            | Description                          |
| --------------- | ------------------------------------ |
| `#define DEBUG` | Print more information via USART.    |
