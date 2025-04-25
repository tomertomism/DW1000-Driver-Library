/*
https://github.com/tomertomism
Copyright 2025 Tomertomism

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/
/*
Total RAM Size = RW Data + ZI Data
Total ROM Size = Code + RO Data + RW Data
*/
#if defined(STM32G070xx)
#include "main.h"
#include "spi.h"
#include "usart.h"
#include "tim.h"
#include "gpio.h"
#endif

#include "my_DW1000.h"
#include <stdio.h>
#include <stdlib.h>


void dw1000_writeSPI(u8 reg, u16 index, u8* data, u8 body_len){
	u8 header[3] = {0};
	// reg number limited to 6-bit
	reg &= 0x3F;
	
	// write process, yes sub address
	if(index > 0) header[2] = reg | (3 << 6);
	// write process, no sub address
	else header[2] = reg | (1 << 7);
	
	// two sub address
	if(index > 127){
		header[1] = (u8)(index | 0x80);
		header[0] = (u8)(index >> 7);
	}
	// one sub address
	else header[1] = (u8)(index);
	
	#if defined(STM32G070xx)
	// enable spi, pull down cs pin
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
	
	// send first octet
	HAL_SPI_Transmit(&hspi1, &header[2], 1, 100);
	
	
	// send second octet, ifexist
	if(header[2] & (1 << 6)){
		HAL_SPI_Transmit(&hspi1, &header[1], 1, 100);
	}
	
	// send third octet, ifexist
	if(header[1] & (1 << 7)){
		HAL_SPI_Transmit(&hspi1, &header[0], 1, 100);
	}
	
	// sending tx beffer
	HAL_SPI_Transmit(&hspi1, data, body_len, 100);
	
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
	#endif
}

void dw1000_readSPI(u8 reg, u16 index, u8* data, u8 body_len){
	u8 header[3] = {0};
	reg &= 0x3f;
	
	// read process, yes sub address
	if(index > 0) header[2] = reg | (1 << 6);
	// read process, no sub address
	else header[2] = reg & ~(1 << 6);
	
	// two sub address
	if(index > 127){
		header[1] = (u8)(index | 0x80);
		header[0] = (u8)(index >> 7);
	// one sub address
	}else header[1] = (u8)(index);
	
	#if defined(STM32G070xx)
	// enable spi, pull down cs pin
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
	
	// send first octet
	HAL_SPI_Transmit(&hspi1, &header[2], 1, 100);
	
	
	// send second octet, ifexist
	if(header[2] & (1 << 6)){
		HAL_SPI_Transmit(&hspi1, &header[1], 1, 100);
	}
	
	// send third octet, ifexist
	if(header[1] & (1 << 7)){
		HAL_SPI_Transmit(&hspi1, &header[0], 1, 100);
	}
	
	// sending tx beffer
	HAL_SPI_TransmitReceive(&hspi1, &header[2], data, body_len, 100);
	
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
	#endif
}

void dw1000_reset(){
	#if defined(STM32G070xx)
	GPIO_InitTypeDef gpio;
	
	while(!(GPIOA->IDR & GPIO_PIN_8));
	gpio.Mode = GPIO_MODE_OUTPUT_PP;
	gpio.Pin = GPIO_PIN_0;
	gpio.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOA, &gpio);
	GPIOA->BRR = 1 << 0;
	
	dw1000_delayms(2);
	
	gpio.Mode = GPIO_MODE_INPUT;
	gpio.Pin = GPIO_PIN_0;
	gpio.Speed = GPIO_SPEED_FREQ_MEDIUM;
	gpio.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &gpio);
	
	dw1000_delayms(10);
	
	while(!(GPIOA->IDR & GPIO_PIN_0));
	#endif
}
