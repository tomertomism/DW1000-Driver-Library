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
#include <stm32f10x.h>
#include "my_DW1000.h"
#include <stdio.h>
#include <stdlib.h>

GPIO_InitTypeDef gpio;

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
	
	#if defined(STM32F10X_MD) || defined(STM32F10X_CL)
	// enable spi, pull down cs pin
	SPI_Cmd(SPI1, ENABLE);
	
	// send first octet
	while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE));
	SPI1->DR = header[2];
	// clear rx buffer;
	while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE));
	SPI1->DR;
	
	// send second octet, ifexist
	if(header[2] & (1 << 6)){
		while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE));
		SPI1->DR = header[1];
		// clear rx buffer;
		while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE));
		SPI1->DR;
	}
	
	// send third octet, ifexist
	if(header[1] & (1 << 7)){
		while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE));
		SPI1->DR = header[0];
		// clear rx buffer;
		while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE));
		SPI1->DR;
	}
	
	while(body_len--){
		// sending tx beffer
		while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE));
		SPI1->DR = *data++;
		// clear rx buffer;
		while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE));
		SPI1->DR;
	}
	
	// disable spi, pull up cs pin
	SPI_Cmd(SPI1, DISABLE);
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
	
	#if defined(STM32F10X_MD) || defined(STM32F10X_CL)
	// enable spi, pull down cs pin
	SPI_Cmd(SPI1, ENABLE);
	
	// send first octet
	while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE));
	SPI1->DR = header[2];
	// clear rx buffer;
	while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE));
	SPI1->DR;
	
	// send second octet, ifexist
	if(header[2] & (1 << 6)){
		while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE));
		SPI1->DR = header[1];
	// clear rx buffer;
		while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE));
		SPI1->DR;
	}
	
	// send third octet, ifexist
	if(header[1] & (1 << 7)){
		while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE));
		SPI1->DR = header[0];
	// clear rx buffer;
		while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE));
		SPI1->DR;
	}
	
	while(body_len--){
		// keeps generate clock by sending anything
		while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE));
		SPI1->DR = 0;
		// read rx buffer, get data
		while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE));
		*data++ = (u8)(SPI1->DR);
	}
	
	// disable spi, pull up cs pin
	SPI_Cmd(SPI1, DISABLE);
	#endif
}

void dw1000_reset(){
	
	#if defined(STM32F10X_MD) || defined(STM32F10X_CL)
	while(!GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8));
	
	gpio.GPIO_Mode = GPIO_Mode_Out_PP;
	gpio.GPIO_Pin = GPIO_Pin_0;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &gpio);// setting DW_RST as push pull
	GPIOA->BRR = 1 << 0;// pull down DW_RST
	
	dw1000_delayms(2);
	
	gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	gpio.GPIO_Pin = GPIO_Pin_0;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &gpio);// setting DW_RST as input
	
	dw1000_delayms(10);
	
	while(!GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0));
	#endif
}
