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
#if defined(STM32F10X_MD) || defined(STM32F10X_CL)
#include <stm32f10x.h>
#endif

#include "my_DW1000.h"
#include <stdio.h>
#include <stdlib.h>

// Defines for enable_clocks function
#define FORCE_SYS_XTI  0
#define ENABLE_ALL_SEQ 1
#define FORCE_SYS_PLL  2
#define READ_ACC_ON	7
#define READ_ACC_OFF   8
#define FORCE_OTP_ON   11
#define FORCE_OTP_OFF  12
#define FORCE_TX_PLL   13
#define FORCE_LDE	  14

// OTP addresses definitions
#define LDOTUNE_ADDRESS (0x04)
#define PARTID_ADDRESS (0x06)
#define LOTID_ADDRESS  (0x07)
#define VBAT_ADDRESS   (0x08)
#define VTEMP_ADDRESS  (0x09)
#define XTRIM_ADDRESS  (0x1E)

// Call-back data RX frames flags
#define DW1000_CB_DATA_RX_FLAG_RNG 0x1 // Ranging bit

#define FCTRL_LEN_MAX 2
#define FCTRL_ACK_REQ_MASK 0x20

dw1000 local_dw;
char buf[100];
u8 addr_data[6];
static u16 temp_16bit;
static u32 temp_32bit;

bool dw1000_init(u8 config){
	
	u8 plllockdetect = EC_CTRL_PLLLCK;
	u16 otp_addr = 0;
	u32 ldo_tune = 0;
	
	// set SPI speed below 3Mbps
	set_spi_low();
	
	sprintf(buf, "%s\n\r", DW1000_DEVICE_DRIVER_VER_STRING); //DW1000 Device Driver Version 05.01.00
	UART_Send(buf, 39);
	
	// read device id from reg 0x00
	dw1000_readdevid();
	
	// check device id
	if((u32)(local_dw.dev_id.ridtag << 16 | local_dw.dev_id.model << 8 | local_dw.dev_id.ver | local_dw.dev_id.rev | local_dw.dev_id.rev) != DW1000_DEVICE_ID){
		sprintf(buf, "dev_id: 0x%x\n\rInit Fail\n\r", 
			(u32)(local_dw.dev_id.ridtag << 16 | local_dw.dev_id.model << 8 | local_dw.dev_id.ver | local_dw.dev_id.rev | local_dw.dev_id.rev));
		UART_Send(buf, 31);
		return false;
	}else{
		sprintf(buf, "dev_id: 0x%x\n\r", 
			(u32)(local_dw.dev_id.ridtag << 16 | local_dw.dev_id.model << 8 | local_dw.dev_id.ver | local_dw.dev_id.rev | local_dw.dev_id.rev));
		
		UART_Send(buf, 20);
	}
	
	sprintf(buf, "Start Init config: 0x%x\n\r", config);
	UART_Send(buf, 25);
	
	// reset device by hardware reset
	dw1000_reset();
	
	// NOTE: set system clock to XTI - this is necessary to make
  // -sure the values read by __dw1000_otpread are reliable
	if(!((DW1000_DW_WAKE_UP & config) && 
		((DW1000_READ_OTP_TMP | DW1000_READ_OTP_BAT | DW1000_READ_OTP_LID | DW1000_READ_OTP_PID | DW1000_DW_WUP_RD_OTPREV)& config)))
			_dw1000_enableclock(FORCE_SYS_XTI);
	
	// Configure the CPLL lock detect
	dw1000_writeSPI(EXT_SYNC_ID, EC_CTRL_OFFSET, &plllockdetect, 1);
	
	if(!(DW1000_DW_WAKE_UP & config)){
		// Load LDO tune from OTP and kick it ifthere is a value actually programmed.
		ldo_tune = _dw1000_otpread(LDOTUNE_ADDRESS);
		#ifdef DEBUG
		sprintf(buf, "ldo_tune: 0x%x\n\r", ldo_tune);
		UART_Send(buf, 22);
		#endif
		if((ldo_tune & 0xff) != 0){
			addr_data[0] = OTP_SF_LDO_KICK;
			dw1000_writeSPI(OTP_IF_ID, OTP_SF, addr_data, 1);
			local_dw.sleep_mode |= AON_WCFG_ONW_LLDO;
		}
	}else{
		dw1000_readSPI(RF_CONF_ID, LDOTUNE, addr_data, 4);
		#ifdef DEBUG
		sprintf(buf, "ldo_tune(DW1000_DW_WAKE_UP & config): 0x%x\n\r", (u32)(addr_data[3] << 24 | addr_data[2] << 16 | addr_data[1] << 8 | addr_data[0]));
		UART_Send(buf, 47);
		#endif
		if((u32)(addr_data[3] << 24 | addr_data[2] << 16 | addr_data[1] << 8 | addr_data[0])!= LDOTUNE_DEFAULT)
			local_dw.sleep_mode |= AON_WCFG_ONW_LLDO;
	}
	
	if((!(DW1000_DW_WAKE_UP & config)) || ((DW1000_DW_WAKE_UP & config) && (DW1000_DW_WUP_RD_OTPREV & config))){
		// Read OTP revision number
		otp_addr = _dw1000_otpread(XTRIM_ADDRESS) & 0xffff;// Read 32 bit value, XTAL trim val is in low octet-0 (5 bits)
		local_dw.otprev = (u8)((otp_addr >> 8) & 0xff);
		#ifdef DEBUG
		sprintf(buf, "otp_addr: 0x%x\n\r", local_dw.otprev);
		UART_Send(buf, 22);
		#endif
	}else local_dw.otprev = 0;
	
	if(!(DW1000_DW_WAKE_UP & config)){
		if((otp_addr & 0x1f) == 0) otp_addr = FS_XTALT_MIDRANGE;
		dw1000_setxtaltrim((u8)otp_addr);
	}
	
	if(DW1000_READ_OTP_PID & config){
		// Load Part and Lot ID from OTP
		local_dw.partID = _dw1000_otpread(PARTID_ADDRESS);
		self_partid = local_dw.partID;
		sprintf(buf, "partID: 0x%x\n\r", local_dw.partID);
		#if defined(STM32F10X_MD) || defined(STM32F10X_CL)
		UART_Send(buf, 20);
		#endif
	}else local_dw.partID = 0;
	
	if(DW1000_READ_OTP_LID & config){
		local_dw.lotID = _dw1000_otpread(LOTID_ADDRESS);
		sprintf(buf, "lotID: 0x%x\n\r", local_dw.lotID);
		UART_Send(buf, 19);
	}else local_dw.lotID = 0;
	
	if(DW1000_READ_OTP_BAT & config){
		// Load VBAT from OTP
		local_dw.vBatP = _dw1000_otpread(VBAT_ADDRESS);
		sprintf(buf, "vBatP: 0x%x\n\r", local_dw.vBatP);
		UART_Send(buf, 20);
	}else local_dw.vBatP = 0;
	
	if(DW1000_READ_OTP_TMP & config){
		local_dw.tempP = _dw1000_otpread(VTEMP_ADDRESS);
		sprintf(buf, "tempP: 0x%x\n\r", local_dw.tempP);
		UART_Send(buf, 20);
	}else local_dw.tempP = 0;
	
	if(!(DW1000_DW_WAKE_UP & config)){
		if(DW1000_LOADUCODE & config){
			dw1000_loaducodefromrom();
			local_dw.sleep_mode |= AON_WCFG_ONW_LLDE;
		}else{
			u8 rega[2];
			dw1000_readSPI(PMSC_ID, PMSC_CTRL1_OFFSET+1, rega, 2);
			#ifdef DEBUG
			sprintf(buf, "rega: 0x%x%x\n\r", rega[0], rega[1]);
			UART_Send(buf, 14);
			#endif
			rega[0] &= 0xfd;
			#ifdef DEBUG
			sprintf(buf, "rega: 0x%x%x\n\r", rega[0], rega[1]);
			UART_Send(buf, 14);
			#endif
			dw1000_writeSPI(PMSC_ID, PMSC_CTRL1_OFFSET+1, rega, 2);
		}
	}else{
		if((DW1000_DW_WUP_NO_UCODE & config) == 0) local_dw.sleep_mode |= AON_WCFG_ONW_LLDE;
	}
	
	_dw1000_enableclock(ENABLE_ALL_SEQ); // Enable clocks for sequencing

	// The 3 bits in AON CFG1 register must be cleared to ensure proper operation of the local_dw in DEEPSLEEP mode.
	dw1000_writeSPI(AON_ID, AON_CFG1_OFFSET, 0x00, 1);
	
	// Read system register / store local copy
	// Read sysconfig register
	dw1000_readSPI(SYS_CFG_ID, 0, addr_data, 4);
	local_dw.sysCFGreg = addr_data[3] << 24 | addr_data[2] << 16 | addr_data[1] << 8 | addr_data[0];
	#ifdef DEBUG
	sprintf(buf, "sysCFGreg: 0x%x\n\r", local_dw.sysCFGreg);
	UART_Send(buf, 23);
	#endif
	//configure longFrames
	local_dw.longFrames = (local_dw.sysCFGreg & SYS_CFG_PHR_MODE_11) >> SYS_CFG_PHR_MODE_SHFT;
	#ifdef DEBUG
	sprintf(buf, "longFrames: 0x%x\n\r", local_dw.longFrames);
	UART_Send(buf, 24);
	#endif
	
	dw1000_readSPI(TX_FCTRL_ID, 0, addr_data, 4);
	local_dw.txFCTRL = addr_data[3] << 24 | addr_data[2] << 16 | addr_data[1] << 8 | addr_data[0];
	#ifdef DEBUG
	sprintf(buf, "txFCTRL: 0x%x\n\r", local_dw.txFCTRL);
	UART_Send(buf, 21);
	#endif
	
	// set SPI speed above 3Mbps
	set_spi_high();
	
	sprintf(buf, "Init Success\n\r\n");
	UART_Send(buf, 15);
	return true;
}

u8 dw1000_otprevision(){
	return local_dw.otprev;
}

void dw1000_setfinegraintxseq(u8 enable){
	if(enable){
		addr_data[1] = (u8)(PMSC_TXFINESEQ_ENABLE >> 8);
		addr_data[0] = (u8)(PMSC_TXFINESEQ_ENABLE);
		dw1000_writeSPI(PMSC_ID, PMSC_TXFINESEQ_OFFSET, addr_data, 2);
	}else{
		addr_data[1] = (u8)(PMSC_TXFINESEQ_DISABLE >> 8);
		addr_data[0] = (u8)(PMSC_TXFINESEQ_DISABLE);
		dw1000_writeSPI(PMSC_ID, PMSC_TXFINESEQ_OFFSET, addr_data, 2);
	}
}

void dw1000_setlnapamode(u8 lna_pa){
	dw1000_readSPI(GPIO_CTRL_ID, GPIO_MODE_OFFSET, addr_data, 4);
	u32 gpio_mode = addr_data[3] << 24 | addr_data[2] << 16 | addr_data[1] << 8 | addr_data[0];
	gpio_mode &= ~(GPIO_MSGP4_MASK | GPIO_MSGP5_MASK | GPIO_MSGP6_MASK);
	if(lna_pa & DW1000_LNA_ENABLE){
		gpio_mode |= GPIO_PIN6_EXTRXE;
	}
	if(lna_pa & DW1000_PA_ENABLE){
		gpio_mode |= (GPIO_PIN5_EXTTXE | GPIO_PIN4_EXTPA);
	}
	addr_data[3] = (u8)(gpio_mode >> 24);
	addr_data[2] = (u8)(gpio_mode >> 16);
	addr_data[1] = (u8)(gpio_mode >> 8);
	addr_data[0] = (u8)(gpio_mode);
	dw1000_writeSPI(GPIO_CTRL_ID, GPIO_MODE_OFFSET, addr_data, 4);
}

void dw1000_enablegpioclocks(){
	dw1000_readSPI(PMSC_ID, 0, addr_data, 4);
	u32 pmsc_clock_ctrl = addr_data[3] << 24 | addr_data[2] << 16 | addr_data[1] << 8 | addr_data[0];
	addr_data[3] = (u8)(pmsc_clock_ctrl | PMSC_CTRL0_GPCE | PMSC_CTRL0_GPRN >> 24);
	addr_data[2] = (u8)(pmsc_clock_ctrl | PMSC_CTRL0_GPCE | PMSC_CTRL0_GPRN >> 16);
	addr_data[1] = (u8)(pmsc_clock_ctrl | PMSC_CTRL0_GPCE | PMSC_CTRL0_GPRN >> 8);
	addr_data[0] = (u8)(pmsc_clock_ctrl | PMSC_CTRL0_GPCE | PMSC_CTRL0_GPRN);
	dw1000_writeSPI(PMSC_ID, 0, addr_data, 4);
}

void dw1000_setgpiodirection(u32 gpioNum, u32 direction){
	u32 command = direction | gpioNum;
	addr_data[2] = (u8)(command >> 16);
	addr_data[1] = (u8)(command >> 8);
	addr_data[0] = (u8)(command);
	dw1000_writeSPI(GPIO_CTRL_ID, GPIO_DIR_OFFSET, addr_data, 3);
}

void dw1000_setgpiovalue(u32 gpioNum, u32 value){
	u32 command = value | gpioNum;
	addr_data[2] = (u8)(command >> 16);
	addr_data[1] = (u8)(command >> 8);
	addr_data[0] = (u8)(command);
	dw1000_writeSPI(GPIO_CTRL_ID, GPIO_DOUT_OFFSET, addr_data, 3);
}

vu8 dw1000_getgpiovalue(u32 gpioNum){
	dw1000_readSPI(GPIO_CTRL_ID, GPIO_RAW_OFFSET, addr_data, 4);
	u32 value = addr_data[3] << 24 | addr_data[2] << 16 | addr_data[1] << 8 | addr_data[0];
	return ((value & gpioNum)? 1 : 0);
}

vu8 dw1000_geticrefvolt(){
	return local_dw.vBatP;
}

vu8 dw1000_geticreftemp(){
	return local_dw.tempP;
}

vu32 dw1000_getpartid(){
	return local_dw.partID;
}

vu32 dw1000_getlotid(){
	return local_dw.lotID;
}

void dw1000_readdevid(){
	u8 temp[DEV_ID_LEN];
	dw1000_readSPI(DEV_ID_ID, 0, temp, DEV_ID_LEN);
	local_dw.dev_id.rev = temp[0] & 0x0f;
	local_dw.dev_id.ver = temp[0] & 0xf0;
	local_dw.dev_id.model = temp[1];
	local_dw.dev_id.ridtag = temp[3] << 8 | temp[2];
}

void dw1000_configuretxrf(dw1000_txconfig_t* config){
	sprintf(buf, "configuretxrf: 0x%x, 0x%x\n\r",config->PGdly, config->power);
	UART_Send(buf, 33);
	// Configure RF TX PG_DELAY
	addr_data[0] = config->PGdly;
	dw1000_writeSPI(TX_CAL_ID, TC_PGDELAY_OFFSET, addr_data, 1);
	// Configure TX power
	addr_data[3] = (u8)(config->power >> 24);
	addr_data[2] = (u8)(config->power >> 16);
	addr_data[1] = (u8)(config->power >> 8);
	addr_data[0] = (u8)(config->power);
	dw1000_writeSPI(TX_POWER_ID, 0, addr_data, 4);
}

void dw1000_configurefor64plen(u8 prf){
	addr_data[0] = DEMOD_GEAR_64L;
	dw1000_writeSPI(CRTR_ID, CRTR_GEAR_OFFSET, addr_data, 1);
	
	if(prf == DW1000_PRF_16M){
		addr_data[0] = DRX_TUNE2_UNCONF_SFD_TH_PRF16;
		dw1000_writeSPI(DRX_CONF_ID, DRX_TUNE2_OFFSET+2, addr_data, 1);
	}else{
		addr_data[0] = DRX_TUNE2_UNCONF_SFD_TH_PRF64;
		dw1000_writeSPI(DRX_CONF_ID, DRX_TUNE2_OFFSET+2, addr_data, 1);
	}
}

bool dw1000_config(dw1000_config_t* config){
	sprintf(buf, "Start Config\n\r");
	UART_Send(buf, 14);
	
	u8 nsSfd_result  = 0;
	u8 useDWnsSFD = 0;
	u8 chan = config->chan;
	u32 regval;
	u16 reg16 = lde_replicaCoeff[config->rxCode];
	u8 prfIndex = config->prf - DW1000_PRF_16M;
	u8 bw = ((chan == 4) || (chan == 7)) ? 1 : 0; // Select wide or narrow band
	
	#ifdef DEBUG
	sprintf(buf, "config:\n\r\tchan: 0x%x,\n\r\tBR: 0x%x,\n\r\tSFD: 0x%x,\n\r\tPHR_Mode: 0x%x,\n\r\tprf: 0x%x,\n\r\trxCode: 0x%x,\n\r",
					config->chan, config->dataRate, config->nsSFD, config->phrMode, config->prf, config->rxCode);
	UART_Send(buf, 95);
	sprintf(buf, "\trxPAC: 0x%x,\n\r\tsfdTO: 0x%x,\n\r\ttxCode: 0x%x,\n\r\ttxPreambLength: 0x%x\n\r",
					config->rxPAC, config->sfdTO, config->txCode, config->txPreambLength);
	UART_Send(buf, 71);
	#endif
	
	// error checking
	if(!(config->dataRate <= DW1000_BR_6M8)){
		#ifdef DEBUG
		sprintf(buf, "Error Data Rate\n\r");
		UART_Send(buf, 17);
		#endif
		return false;
	}
	if(!(config->rxPAC <= DW1000_PAC64)){
		#ifdef DEBUG
		sprintf(buf, "Error Acquisition Chunk Size\n\r");
		UART_Send(buf, 30);
		#endif
		return false;
	}
	if(!((chan >= 1) && (chan <= 7) && (chan != 6))){
		#ifdef DEBUG
		sprintf(buf, "Error channel number\n\r");
		UART_Send(buf, 22);
		#endif
		return false;
	}
	if(!(((config->prf == DW1000_PRF_64M) && (config->txCode >= 9) && (config->txCode <= 24))
			|| ((config->prf == DW1000_PRF_16M) && (config->txCode >= 1) && (config->txCode <= 8)))){
		#ifdef DEBUG
		sprintf(buf, "Error Pulse Repetition Frequency & TX preamble code\n\r");
		UART_Send(buf, 53);
		#endif
		return false;
	}
	if(!(((config->prf == DW1000_PRF_64M) && (config->rxCode >= 9) && (config->rxCode <= 24))
			|| ((config->prf == DW1000_PRF_16M) && (config->rxCode >= 1) && (config->rxCode <= 8)))){
		#ifdef DEBUG
		sprintf(buf, "Error Pulse Repetition Frequency & RX preamble code\n\r");
		UART_Send(buf, 53);
		#endif
		return false;
	}
	if(!((config->txPreambLength == DW1000_PLEN_64) || (config->txPreambLength == DW1000_PLEN_128) || (config->txPreambLength == DW1000_PLEN_256)
			|| (config->txPreambLength == DW1000_PLEN_512) || (config->txPreambLength == DW1000_PLEN_1024) || (config->txPreambLength == DW1000_PLEN_1536)
			|| (config->txPreambLength == DW1000_PLEN_2048) || (config->txPreambLength == DW1000_PLEN_4096))){
		#ifdef DEBUG
		sprintf(buf, "Error txPreambLength\n\r");
		UART_Send(buf, 22);
		#endif
		return false;
	}
	if(!((config->phrMode == DW1000_PHRMODE_STD) || (config->phrMode == DW1000_PHRMODE_EXT))){
		#ifdef DEBUG
		sprintf(buf, "Error PHR mode\n\r");
		UART_Send(buf, 16);
		#endif
		return false;
	}
	
	// For 110 kbps we need a special setup
	if(config->dataRate == DW1000_BR_110K){
		local_dw.sysCFGreg |= SYS_CFG_RXM110K;
		reg16 >>= 3;
	}else{
		local_dw.sysCFGreg &= (~SYS_CFG_RXM110K);
	}
	#ifdef DEBUG
	sprintf(buf, "local_dw.sysCFGreg: 0x%x\n\r", local_dw.sysCFGreg);
	UART_Send(buf, 30);
	#endif
	
	local_dw.longFrames = config->phrMode;
	#ifdef DEBUG
	sprintf(buf, "local_dw.longFrames: 0x%x\n\r", local_dw.longFrames);
	UART_Send(buf, 25);
	#endif
	
	local_dw.sysCFGreg &= ~SYS_CFG_PHR_MODE_11;
	local_dw.sysCFGreg |= (SYS_CFG_PHR_MODE_11 & ((u32)config->phrMode << SYS_CFG_PHR_MODE_SHFT));
	#ifdef DEBUG
	sprintf(buf, "local_dw.sysCFGreg: 0x%x\n\r", local_dw.sysCFGreg);
	UART_Send(buf, 30);
	#endif
	
	addr_data[3] = (u8)(local_dw.sysCFGreg >> 24);
	addr_data[2] = (u8)(local_dw.sysCFGreg >> 16);
	addr_data[1] = (u8)(local_dw.sysCFGreg >> 8);
	addr_data[0] = (u8)(local_dw.sysCFGreg);
	dw1000_writeSPI(SYS_CFG_ID, 0, addr_data, 4);

	// Set the lde_replicaCoeff
	addr_data[1] = (u8)(reg16 >> 8);
	addr_data[0] = (u8)(reg16);
	dw1000_writeSPI(LDE_IF_ID, LDE_REPC_OFFSET, addr_data, 2);

	#ifdef DEBUG
	sprintf(buf, "reg16: 0x%x\n\r", reg16);
	UART_Send(buf, 30);
	#endif

	dw1000_configlde(prfIndex);

	// Configure PLL2/RF PLL block CFG/TUNE (for a given channel)
	addr_data[3] = (u8)(fs_pll_cfg[chan_idx[chan]] >> 24);
	addr_data[2] = (u8)(fs_pll_cfg[chan_idx[chan]] >> 16);
	addr_data[1] = (u8)(fs_pll_cfg[chan_idx[chan]] >> 8);
	addr_data[0] = (u8)(fs_pll_cfg[chan_idx[chan]]);
	dw1000_writeSPI(FS_CTRL_ID, FS_PLLCFG_OFFSET, addr_data, 4);
	
	#ifdef DEBUG
	sprintf(buf, "fs_pll_cfg[chan_idx[chan]]: 0x%x\n\r", addr_data[3] << 24 | addr_data[2] << 16 | addr_data[1] << 8 | addr_data[0]);
	UART_Send(buf, 40);
	#endif
	
	addr_data[1] = (u8)(fs_pll_tune[chan_idx[chan]] >> 8);
	addr_data[0] = (u8)(fs_pll_tune[chan_idx[chan]]);
	dw1000_writeSPI(FS_CTRL_ID, FS_PLLTUNE_OFFSET, addr_data, 2);
	
	#ifdef DEBUG
	sprintf(buf, "fs_pll_tune[chan_idx[chan]]: 0x%x\n\r", fs_pll_tune[chan_idx[chan]]);
	UART_Send(buf, 37);
	#endif
	
	// Configure RF RX blocks (for specified channel/bandwidth)
	addr_data[0] = rx_config[bw];
	dw1000_writeSPI(RF_CONF_ID, RF_RXCTRLH_OFFSET, addr_data, 1);
	
	#ifdef DEBUG
	sprintf(buf, "rx_config[bw]: 0x%x\n\r", rx_config[bw]);
	UART_Send(buf, 21);
	#endif
	
	// Configure RF TX blocks (for specified channel and PRF)
	// Configure RF TX control
	addr_data[3] = (u8)(tx_config[chan_idx[chan]] >> 24);
	addr_data[2] = (u8)(tx_config[chan_idx[chan]] >> 16);
	addr_data[1] = (u8)(tx_config[chan_idx[chan]] >> 8);
	addr_data[0] = (u8)(tx_config[chan_idx[chan]]);
	dw1000_writeSPI(RF_CONF_ID, RF_TXCTRL_OFFSET, addr_data, 4);

	#ifdef DEBUG
	sprintf(buf, "tx_config[chan_idx[chan]]: 0x%x\n\r", tx_config[chan_idx[chan]]);
	UART_Send(buf, 39);
	#endif

	// Configure the baseband parameters (for specified PRF, bit rate, PAC, and SFD settings)

	// DTUNE0
	addr_data[1] = (u8)(sftsh[config->dataRate][config->nsSFD] >> 8);
	addr_data[0] = (u8)(sftsh[config->dataRate][config->nsSFD]);
	dw1000_writeSPI(DRX_CONF_ID, DRX_TUNE0b_OFFSET, addr_data, 2);

	#ifdef DEBUG
	sprintf(buf, "sftsh[config->dataRate][config->nsSFD]: 0x%x\n\r", sftsh[config->dataRate][config->nsSFD]);
	UART_Send(buf, 59);
	#endif

	// DTUNE1
	addr_data[1] = (u8)(dtune1[prfIndex] >> 8);
	addr_data[0] = (u8)(dtune1[prfIndex]);
	dw1000_writeSPI(DRX_CONF_ID, DRX_TUNE1a_OFFSET, addr_data, 2);

	#ifdef DEBUG
	sprintf(buf, "dtune1[prfIndex]: 0x%x\n\r", dtune1[prfIndex]);
	UART_Send(buf, 26);
	#endif
	
	if(config->dataRate == DW1000_BR_110K){
		addr_data[1] = (u8)(DRX_TUNE1b_110K >> 8);
		addr_data[0] = (u8)(DRX_TUNE1b_110K);
		dw1000_writeSPI(DRX_CONF_ID, DRX_TUNE1b_OFFSET, addr_data, 2);
		#ifdef DEBUG
		sprintf(buf, "DRX_TUNE1b_110K: 0x%x\n\r", DRX_TUNE1b_110K);
		UART_Send(buf, 25);
		#endif
	}else{
		if(config->txPreambLength == DW1000_PLEN_64){
			addr_data[1] = (u8)(DRX_TUNE1b_6M8_PRE64 >> 8);
			addr_data[0] = (u8)(DRX_TUNE1b_6M8_PRE64);
			dw1000_writeSPI(DRX_CONF_ID, DRX_TUNE1b_OFFSET, addr_data, 2);
			addr_data[0] = DRX_TUNE4H_PRE64;
			dw1000_writeSPI(DRX_CONF_ID, DRX_TUNE4H_OFFSET, addr_data, 1);
			#ifdef DEBUG
			sprintf(buf, "DRX_TUNE1b_6M8_PRE64: 0x%x\n\rDRX_TUNE4H_PRE64: 0x%x\n\r", DRX_TUNE1b_6M8_PRE64, DRX_TUNE4H_PRE64);
			UART_Send(buf, 54);
			#endif
		}else{
			addr_data[1] = (u8)(DRX_TUNE1b_850K_6M8 >> 8);
			addr_data[0] = (u8)(DRX_TUNE1b_850K_6M8);
			dw1000_writeSPI(DRX_CONF_ID, DRX_TUNE1b_OFFSET, addr_data, 2);
			addr_data[0] = DRX_TUNE4H_PRE128PLUS;
			dw1000_writeSPI(DRX_CONF_ID, DRX_TUNE4H_OFFSET, addr_data, 1);
			#ifdef DEBUG
			sprintf(buf, "DRX_TUNE1b_850K_6M8: 0x%x\n\rDRX_TUNE4H_PRE128PLUS: 0x%x\n\r", DRX_TUNE1b_850K_6M8, DRX_TUNE4H_PRE128PLUS);
			UART_Send(buf, 58);
			#endif
		}
	}

	// DTUNE2
	addr_data[3] = (u8)(digital_bb_config[prfIndex][config->rxPAC] >> 24);
	addr_data[2] = (u8)(digital_bb_config[prfIndex][config->rxPAC] >> 16);
	addr_data[1] = (u8)(digital_bb_config[prfIndex][config->rxPAC] >> 8);
	addr_data[0] = (u8)(digital_bb_config[prfIndex][config->rxPAC]);
	dw1000_writeSPI(DRX_CONF_ID, DRX_TUNE2_OFFSET, addr_data, 4);
	#ifdef DEBUG
	sprintf(buf, "digital_bb_config[prfIndex][config->rxPAC]: 0x%x\n\r", digital_bb_config[prfIndex][config->rxPAC]);
	UART_Send(buf, 56);
	#endif

	// DTUNE3 (SFD timeout)
	// Don't allow 0 - SFD timeout will always be enabled
	if(config->sfdTO == 0) config->sfdTO = DW1000_SFDTOC_DEF;
	addr_data[1] = (u8)(config->sfdTO >> 8);
	addr_data[0] = (u8)(config->sfdTO);
	dw1000_writeSPI(DRX_CONF_ID, DRX_SFDTOC_OFFSET, addr_data, 2);
	#ifdef DEBUG
	sprintf(buf, "config->sfdTO: 0x%x\n\r", config->sfdTO);
	UART_Send(buf, 23);
	#endif

	// Configure AGC parameters
	addr_data[3] = (u8)(agc_config.tune2 >> 24);
	addr_data[2] = (u8)(agc_config.tune2 >> 16);
	addr_data[1] = (u8)(agc_config.tune2 >> 8);
	addr_data[0] = (u8)(agc_config.tune2);
	dw1000_writeSPI(AGC_CFG_STS_ID, 0x0C, addr_data, 4);
	#ifdef DEBUG
	sprintf(buf, "agc_config.tune2: 0x%x\n\r", agc_config.tune2);
	UART_Send(buf, 30);
	#endif
	addr_data[1] = (u8)(agc_config.tune1[prfIndex] >> 8);
	addr_data[0] = (u8)(agc_config.tune1[prfIndex]);
	dw1000_writeSPI(AGC_CFG_STS_ID, 0x04, addr_data, 2);
	#ifdef DEBUG
	sprintf(buf, "agc_config.tune1[prfIndex]: 0x%x\n\r", agc_config.tune1[prfIndex]);
	UART_Send(buf, 36);
	#endif
	addr_data[1] = (u8)(agc_config.tune3 >> 8);
	addr_data[0] = (u8)(agc_config.tune3);
	dw1000_writeSPI(AGC_CFG_STS_ID, 0x12, addr_data, 2);
	#ifdef DEBUG
	sprintf(buf, "agc_config.tune3: 0x%x\n\r", agc_config.tune3);
	UART_Send(buf, 26);
	#endif

	// Set (non-standard) user SFD for improved performance
	if(config->nsSFD){
		// Write non standard (DW) SFD length
		addr_data[0] = dwnsSFDlen[config->dataRate];
		dw1000_writeSPI(USR_SFD_ID, 0x00, addr_data, 1);
		#ifdef DEBUG
		sprintf(buf, "dwnsSFDlen[config->dataRate]: 0x%x\n\r", dwnsSFDlen[config->dataRate]);
		UART_Send(buf, 36);
		#endif
		nsSfd_result = 3;
		useDWnsSFD = 1;
	}

	regval = 	(CHAN_CTRL_TX_CHAN_MASK & (chan << CHAN_CTRL_TX_CHAN_SHIFT)) | // Transmit Channel
						(CHAN_CTRL_RX_CHAN_MASK & (chan << CHAN_CTRL_RX_CHAN_SHIFT)) | // Receive Channel
						(CHAN_CTRL_RXFPRF_MASK & ((u32)config->prf << CHAN_CTRL_RXFPRF_SHIFT)) | // RX PRF
						((CHAN_CTRL_TNSSFD|CHAN_CTRL_RNSSFD) & ((u32)nsSfd_result << CHAN_CTRL_TNSSFD_SHIFT)) | // nsSFD enable RX&TX
						(CHAN_CTRL_DWSFD & ((u32)useDWnsSFD << CHAN_CTRL_DWSFD_SHIFT)) | // Use DW nsSFD
						(CHAN_CTRL_TX_PCOD_MASK & ((u32)config->txCode << CHAN_CTRL_TX_PCOD_SHIFT)) | // TX Preamble Code
						(CHAN_CTRL_RX_PCOD_MASK & ((u32)config->rxCode << CHAN_CTRL_RX_PCOD_SHIFT)); // RX Preamble Code

	addr_data[3] = (u8)(regval >> 24);
	addr_data[2] = (u8)(regval >> 16);
	addr_data[1] = (u8)(regval >> 8);
	addr_data[0] = (u8)(regval);
	dw1000_writeSPI(CHAN_CTRL_ID, 0, addr_data, 4);
	#ifdef DEBUG
	sprintf(buf, "regval: 0x%x\n\r", regval);
	UART_Send(buf, 20);
	#endif

	// Set up TX Preamble Size, PRF and Data Rate
	local_dw.txFCTRL = ((u32)(config->txPreambLength | config->prf) << TX_FCTRL_TXPRF_SHFT) |
									 ((u32)config->dataRate << TX_FCTRL_TXBR_SHFT);

	addr_data[3] = (u8)(local_dw.txFCTRL >> 24);
	addr_data[2] = (u8)(local_dw.txFCTRL >> 16);
	addr_data[1] = (u8)(local_dw.txFCTRL >> 8);
	addr_data[0] = (u8)(local_dw.txFCTRL);
	dw1000_writeSPI(TX_FCTRL_ID, 0, addr_data, 4);
	#ifdef DEBUG
	sprintf(buf, "local_dw.txFCTRL: 0x%x\n\r", local_dw.txFCTRL);
	UART_Send(buf, 28);
	#endif

	// The SFD transmit pattern is initialized by the local_dw upon a user TX request, but (due to an IC issue) it is not done for an auto-ACK TX. 
	// The SYS_CTRL write below works around this issue, by simultaneously initiating and aborting a transmission, which correctly initializes the SFD
	// after its configuration or reconfiguration. This issue is not documented at the time of writing this code. It should be in next release of local_dw User Manual (v2.09, from July 2016).
	addr_data[0] = SYS_CTRL_TXSTRT | SYS_CTRL_TRXOFF;
	dw1000_writeSPI(SYS_CTRL_ID, SYS_CTRL_OFFSET, addr_data, 1); // Request TX start and TRX off at the same time
	#ifdef DEBUG
	sprintf(buf, "SYS_CTRL_TXSTRT | SYS_CTRL_TRXOFF: 0x%x\n\r", (u8)(SYS_CTRL_TXSTRT | SYS_CTRL_TRXOFF));
	UART_Send(buf, 41);
	#endif
	sprintf(buf, "Config Success\n\r\n");
	UART_Send(buf, 17);
	return true;
}

void dw1000_setrxantennadelay(u16 rxDelay){
	// Set the RX antenna delay for auto TX timestamp adjustment
	addr_data[1] = (u8)(rxDelay >> 8);
	addr_data[0] = (u8)(rxDelay);
	dw1000_writeSPI(LDE_IF_ID, LDE_RXANTD_OFFSET, addr_data, 2);
	#ifdef DEBUG
	sprintf(buf, "Set rxDelay: 0x%x\n\r", rxDelay);
	UART_Send(buf, 21);
	#endif
}

void dw1000_settxantennadelay(u16 txDelay){
	// Set the TX antenna delay for auto TX timestamp adjustment
	addr_data[1] = (u8)(txDelay >> 8);
	addr_data[0] = (u8)(txDelay);
	dw1000_writeSPI(TX_ANTD_ID, TX_ANTD_OFFSET, addr_data, 2);
	#ifdef DEBUG
	sprintf(buf, "Set txDelay: 0x%x\n\r", txDelay);
	UART_Send(buf, 21);
	#endif
}

bool dw1000_writetxdata(u16 txFrameLength, u8 *txFrameBytes, u16 txBufferOffset){
	// error checking
	if(!(txFrameLength >= 2)){
		#ifdef DEBUG
		sprintf(buf, "txFrameLength too short\n\r");
		UART_Send(buf, 25);
		#endif
		return false;
	}
	if(!((local_dw.longFrames && (txFrameLength <= 1023)) || (txFrameLength <= 127))){
		#ifdef DEBUG
		sprintf(buf, "txFrameLength too long\n\r");
		UART_Send(buf, 25);
		#endif
		return false;
	}
	if(!((txBufferOffset + txFrameLength) <= 1024)){
		#ifdef DEBUG
		sprintf(buf, "txFrameLength too long after offset\n\r");
		UART_Send(buf, 38);
		#endif
		return false;
	}
	// Write the data to the IC TX buffer, (-2 bytes for auto generated CRC)
	dw1000_writeSPI(TX_BUFFER_ID, txBufferOffset, txFrameBytes, txFrameLength - 2);
	return true;
}

bool dw1000_writetxdata_enable_fcs(u16 txFrameLength, u8 *txFrameBytes, u16 txBufferOffset){
	// error checking
	if(!(txFrameLength >= 2)){
		#ifdef DEBUG
		sprintf(buf, "txFrameLength too short\n\r");
		UART_Send(buf, 25);
		#endif
		return false;
	}
	if(!((local_dw.longFrames && (txFrameLength <= 1023)) || (txFrameLength <= 127))){
		#ifdef DEBUG
		sprintf(buf, "txFrameLength too long\n\r");
		UART_Send(buf, 25);
		#endif
		return false;
	}
	if(!((txBufferOffset + txFrameLength) <= 1024)){
		#ifdef DEBUG
		sprintf(buf, "txFrameLength too long after offset\n\r");
		UART_Send(buf, 38);
		#endif
		return false;
	}
	// Write the data to the IC TX buffer, (-2 bytes for auto generated CRC)
	dw1000_writeSPI(TX_BUFFER_ID, txBufferOffset, txFrameBytes, txFrameLength - 2);
	addr_data[0] = SYS_CTRL_CANSFCS;
	dw1000_writeSPI(SYS_CTRL_ID, SYS_CTRL_OFFSET, addr_data, 1);
	return true;
}

bool dw1000_writetxfctrl(u16 txFrameLength, u16 txBufferOffset, u8 ranging){
	if(!((local_dw.longFrames && (txFrameLength <= 1023)) || (txFrameLength <= 127))){
		#ifdef DEBUG
		sprintf(buf, "txFrameLength too long\n\r");
		UART_Send(buf, 25);
		#endif
		return false;
	}
	if(!((txBufferOffset + txFrameLength) <= 1024)){
		#ifdef DEBUG
		sprintf(buf, "txFrameLength too long after offset\n\r");
		UART_Send(buf, 38);
		#endif
		return false;
	}
	if(!((ranging == 0) || (ranging == 1))){
		#ifdef DEBUG
		sprintf(buf, "invalid ranging value\n\r");
		UART_Send(buf, 23);
		#endif
		return false;
	}
	// Write the frame length to the TX frame control register
	// pdw1000local->txFCTRL has kept configured bit rate information
	u32 reg32 = local_dw.txFCTRL | txFrameLength | ((u32)txBufferOffset << TX_FCTRL_TXBOFFS_SHFT) | ((u32)ranging << TX_FCTRL_TR_SHFT);
	addr_data[3] = (u8)(reg32 >> 24);
	addr_data[2] = (u8)(reg32 >> 16);
	addr_data[1] = (u8)(reg32 >> 8);
	addr_data[0] = (u8)(reg32);
	dw1000_writeSPI(TX_FCTRL_ID, 0, addr_data, 4);
	return true;
}

void dw1000_readrxdata(u8 *buffer, u16 length, u16 rxBufferOffset){
	dw1000_readSPI(RX_BUFFER_ID, rxBufferOffset, buffer, length);
}

void dw1000_readaccdata(u8 *buffer, u16 len, u16 accOffset){
	// Force on the ACC clocks ifwe are sequenced
	_dw1000_enableclock(READ_ACC_ON);
	
	dw1000_readSPI(ACC_MEM_ID, accOffset, buffer, len);
	
	// Revert clocks back
	_dw1000_enableclock(READ_ACC_OFF);
}

#define B20_SIGN_EXTEND_TEST (0x00100000UL)
#define B20_SIGN_EXTEND_MASK (0xFFF00000UL)

int dw1000_readcarrierintegrator(){
	u32  regval = 0;
	
	// Read 3 bytes into buffer (21-bit quantity)
	dw1000_readSPI(DRX_CONF_ID, DRX_CARRIER_INT_OFFSET, addr_data, DRX_CARRIER_INT_LEN);
	
	// arrange the three bytes into an unsigned integer value
	regval = addr_data[2] << 16 | addr_data[1] << 8 | addr_data[0];
	
	// sign extend bit #20 to whole word
	if(regval & B20_SIGN_EXTEND_TEST) regval |= B20_SIGN_EXTEND_MASK;
	// make sure upper bits are clear ifnot sign extending
	else regval &= DRX_CARRIER_INT_MASK;
	
	// cast unsigned value to signed quantity.
	return (int)regval;
}

void dw1000_readdiagnostics(dw1000_rxdiag_t *diagnostics){
	// Read the HW FP index
	dw1000_readSPI(RX_TIME_ID, RX_TIME_FP_INDEX_OFFSET, addr_data, 2);
	diagnostics->firstPath = addr_data[1] << 8 | addr_data[0];
	
	// LDE diagnostic data
	dw1000_readSPI(LDE_IF_ID, LDE_THRESH_OFFSET, addr_data, 2);
	diagnostics->maxNoise = addr_data[1] << 8 | addr_data[0];
	
	// Read all 8 bytes in one SPI transaction
	dw1000_readSPI(RX_FQUAL_ID, 0x0, (u8*)&diagnostics->stdNoise, 8);
	
	dw1000_readSPI(RX_TIME_ID, RX_TIME_FP_AMPL1_OFFSET, addr_data, 2);
	diagnostics->firstPathAmp1 = addr_data[1] << 8 | addr_data[0];
	
	dw1000_readSPI(RX_FINFO_ID, 0, addr_data, 4);
	diagnostics->rxPreamCount = addr_data[3] << 24 | addr_data[2] << 16 | addr_data[1] << 8 | addr_data[0];
	diagnostics->rxPreamCount &= RX_FINFO_RXPACC_MASK;
	diagnostics->rxPreamCount = diagnostics->rxPreamCount >> RX_FINFO_RXPACC_SHIFT;
}

void dw1000_readtxtimestamp(u8 *timestamp){
	// Read bytes directly into buffer
	dw1000_readSPI(TX_TIME_ID, TX_TIME_TX_STAMP_OFFSET, timestamp, TX_TIME_TX_STAMP_LEN);
}

u32 dw1000_readtxtimestamphi32(){
	// Offset is 1 to get the 4 upper bytes out of 5
	dw1000_readSPI(TX_TIME_ID, 1, addr_data, 4);
	return addr_data[3] << 24 | addr_data[2] << 16 | addr_data[1] << 8 | addr_data[0];
}

u32 dw1000_readtxtimestamplo32(){
	// Read TX TIME as a 32-bit register to get the 4 lower bytes out of 5
	dw1000_readSPI(TX_TIME_ID, 0, addr_data, 4);
	return addr_data[3] << 24 | addr_data[2] << 16 | addr_data[1] << 8 | addr_data[0];
}

void dw1000_readrxtimestamp(u8 *timestamp){
	// Get the adjusted time of arrival
	dw1000_readSPI(RX_TIME_ID, RX_TIME_RX_STAMP_OFFSET, timestamp, RX_TIME_RX_STAMP_LEN);
}

u32 dw1000_readrxtimestamphi32(){
	// Offset is 1 to get the 4 upper bytes out of 5
	dw1000_readSPI(RX_TIME_ID, 1, addr_data, 4);
	return addr_data[3] << 24 | addr_data[2] << 16 | addr_data[1] << 8 | addr_data[0];
}

u32 dw1000_readrxtimestamplo32(){
	// Read RX TIME as a 32-bit register to get the 4 lower bytes out of 5
	dw1000_readSPI(RX_TIME_ID, 0, addr_data, 4);
	return addr_data[3] << 24 | addr_data[2] << 16 | addr_data[1] << 8 | addr_data[0];
}

u32 dw1000_readsystimestamphi32(){
	// Offset is 1 to get the 4 upper bytes out of 5
	dw1000_readSPI(SYS_TIME_ID, 1, addr_data, 4);
	return addr_data[3] << 24 | addr_data[2] << 16 | addr_data[1] << 8 | addr_data[0];
}

void dw1000_readsystime(u8 *timestamp){
	dw1000_readSPI(SYS_TIME_ID, SYS_TIME_OFFSET, timestamp, SYS_TIME_LEN);
}

void dw1000_enableframefilter(u16 enable){
	dw1000_readSPI(SYS_CFG_ID, 0, addr_data, 4);
	// Read sysconfig register
	u32 sysconfig = SYS_CFG_MASK & (addr_data[3] << 24 | addr_data[2] << 16 | addr_data[1] << 8 | addr_data[0]);
	if(enable){
		// Enable frame filtering and configure frame types
		sysconfig &= ~(SYS_CFG_FF_ALL_EN); // Clear all
		sysconfig |= (enable & SYS_CFG_FF_ALL_EN) | SYS_CFG_FFE;
	} else {
		sysconfig &= ~(SYS_CFG_FFE);
	}
	local_dw.sysCFGreg = sysconfig;
	addr_data[3] = (u8)(sysconfig >> 24);
	addr_data[2] = (u8)(sysconfig >> 16);
	addr_data[1] = (u8)(sysconfig >> 8);
	addr_data[0] = (u8)(sysconfig);
	dw1000_writeSPI(SYS_CFG_ID, 0, addr_data, 4);
}

void dw1000_setpanid(u16 panID){
	// PAN ID is high 16 bits of register
	addr_data[1] = (u8)(panID >> 8);
	addr_data[0] = (u8)(panID);
	dw1000_writeSPI(PANADR_ID, PANADR_PAN_ID_OFFSET, addr_data, 2);
}

void dw1000_setaddress16(u16 shortAddress){
	// Short address into low 16 bits
	addr_data[1] = (u8)(shortAddress >> 8);
	addr_data[0] = (u8)(shortAddress);
	dw1000_writeSPI(PANADR_ID, PANADR_SHORT_ADDR_OFFSET, addr_data, 2);
}

void dw1000_seteui(u8 *eui64){
	dw1000_writeSPI(EUI_64_ID, EUI_64_OFFSET, eui64, EUI_64_LEN);
}

void dw1000_geteui(u8 *eui64){
	dw1000_writeSPI(EUI_64_ID, EUI_64_OFFSET, eui64, EUI_64_LEN);
}

void dw1000_otpread(u16 address, u32 *array, u8 length){
	// NOTE: Set system clock to XTAL - this is necessary to make sure the values read by _dw1000_otpread are reliable
	_dw1000_enableclock(FORCE_SYS_XTI);
	
	for(int i=0; i<length; i++){
		array[i] = _dw1000_otpread(address + i);
	}
	
	// Restore system clock to PLL
	_dw1000_enableclock(ENABLE_ALL_SEQ);
}

vu32 _dw1000_otpread(u8 address){

	u32 ret_data;
	
	// Write the address
	dw1000_writeSPI(OTP_IF_ID, OTP_ADDR, &address, 1);
	
	// Perform OTP Read - Manual read mode has to be set
	addr_data[0] = (OTP_CTRL_OTPREAD | OTP_CTRL_OTPRDEN);
	dw1000_writeSPI(OTP_IF_ID, OTP_CTRL, addr_data, 1);
	addr_data[0] = 0;
	dw1000_writeSPI(OTP_IF_ID, OTP_CTRL, addr_data, 1); // OTPREAD is self clearing but OTPRDEN is not
	
	// Read read data, available 40ns after rising edge of OTP_READ
	dw1000_readSPI(OTP_IF_ID, OTP_RDAT, addr_data, 4);
	ret_data = addr_data[3] << 24 | addr_data[2] << 16 | addr_data[1] << 8 | addr_data[0];
	
	// Return the 32bit of read data
	return ret_data;
}

bool _dw1000_otpsetmrregs(u8 mode){
	u16 mra = 0, mrb = 0, mr = 0;

	// PROGRAMME MRA
	// Set MRA, MODE_SEL
	addr_data[0] = 0x03;
	dw1000_writeSPI(OTP_IF_ID, OTP_CTRL + 1, addr_data, 1);

	// Load data
	switch(mode & 0x0f){
		case 0x0 :
			mr = 0x0000;
			mra = 0x0000;
			mrb = 0x0000;
			break;
		case 0x1 :
			mr = 0x1024;
			mra = 0x9220; // Enable CPP mon
			mrb = 0x000e;
			break;
		case 0x2 :
			mr = 0x1824;
			mra = 0x9220;
			mrb = 0x0003;
			break;
		case 0x3 :
			mr = 0x1824;
			mra = 0x9220;
			mrb = 0x004e;
			break;
		case 0x4 :
			mr = 0x0000;
			mra = 0x0000;
			mrb = 0x0003;
			break;
		case 0x5 :
			mr = 0x0024;
			mra = 0x0000;
			mrb = 0x0003;
			break;
		default :
			return false;
	}

	addr_data[1] = (u8)(mra >> 8);
	addr_data[0] = (u8)(mra);
	dw1000_writeSPI(OTP_IF_ID, OTP_WDAT, addr_data, 2);

	// Set WRITE_MR
	addr_data[0] = 0x08;
	dw1000_writeSPI(OTP_IF_ID, OTP_CTRL, addr_data, 1);

	// Wait?
	dw1000_delayms(2);

	// Set Clear Mode sel
	addr_data[0] = 0x02;
	dw1000_writeSPI(OTP_IF_ID, OTP_CTRL + 1, addr_data, 1);

	// Set AUX update, write MR
	addr_data[0] = 0x88;
	dw1000_writeSPI(OTP_IF_ID, OTP_CTRL, addr_data, 1);

	// Clear write MR
	addr_data[0] = 0x80;
	dw1000_writeSPI(OTP_IF_ID, OTP_CTRL, addr_data, 1);

	// Clear AUX update
	addr_data[0] = 0x00;
	dw1000_writeSPI(OTP_IF_ID, OTP_CTRL, addr_data, 1);

	/////////////////////////////////////////////
	// PROGRAM MRB
	// Set SLOW, MRB, MODE_SEL
	addr_data[0] = 0x05;
	dw1000_writeSPI(OTP_IF_ID, OTP_CTRL + 1, addr_data, 1);

	addr_data[1] = (u8)(mrb >> 8);
	addr_data[0] = (u8)(mrb);
	dw1000_writeSPI(OTP_IF_ID, OTP_WDAT, addr_data, 2);

	// Set WRITE_MR
	addr_data[0] = 0x08;
	dw1000_writeSPI(OTP_IF_ID, OTP_CTRL, addr_data, 1);

	// Wait?
	dw1000_delayms(2);

	// Set Clear Mode sel
	addr_data[0] = 0x04;
	dw1000_writeSPI(OTP_IF_ID, OTP_CTRL + 1, addr_data, 1);

	// Set AUX update, write MR
	addr_data[0] = 0x88;
	dw1000_writeSPI(OTP_IF_ID, OTP_CTRL, addr_data, 1);

	// Clear write MR
	addr_data[0] = 0x80;
	dw1000_writeSPI(OTP_IF_ID, OTP_CTRL, addr_data, 1);

	// Clear AUX update
	addr_data[0] = 0x00;
	dw1000_writeSPI(OTP_IF_ID, OTP_CTRL, addr_data, 1);

	/////////////////////////////////////////////
	// PROGRAM MR
	// Set SLOW, MODE_SEL
	addr_data[0] = 0x01;
	dw1000_writeSPI(OTP_IF_ID, OTP_CTRL + 1, addr_data, 1);

	// Load data
	addr_data[1] = (u8)(mr >> 8);
	addr_data[0] = (u8)(mr);
	dw1000_writeSPI(OTP_IF_ID, OTP_WDAT, addr_data, 2);

	// Set WRITE_MR
	addr_data[0] = 0x08;
	dw1000_writeSPI(OTP_IF_ID, OTP_CTRL, addr_data, 1);

	// Wait?
	dw1000_delayms(2);

	// Set Clear Mode sel
	addr_data[0] = 0x00;
	dw1000_writeSPI(OTP_IF_ID, OTP_CTRL + 1, addr_data, 1);

	return true;
}

bool _dw1000_otpprogword32(u32 data, u16 address){
	u8 rd_buf[1];

	// Write the data
	addr_data[3] = (u8)(data >> 24);
	addr_data[2] = (u8)(data >> 16);
	addr_data[1] = (u8)(data >> 8);
	addr_data[0] = (u8)(data);
	dw1000_writeSPI(OTP_IF_ID, OTP_WDAT, addr_data, 4);

	// Write the address [10:0]
	addr_data[1] = (u8)(address >> 8);
	addr_data[0] = (u8)(address);
	dw1000_writeSPI(OTP_IF_ID, OTP_ADDR, addr_data, 2);

	// Enable Sequenced programming
	addr_data[0] = OTP_CTRL_OTPPROG;
	dw1000_writeSPI(OTP_IF_ID, OTP_CTRL, addr_data, 1);
	addr_data[0] = 0; // And clear
	dw1000_writeSPI(OTP_IF_ID, OTP_CTRL, addr_data, 1);

	// WAIT for status to flag PRGM OK..
	u8 otp_done = 0;
	while(!otp_done){
		dw1000_delayms(1);
		dw1000_readSPI(OTP_IF_ID, OTP_STAT, rd_buf, 1);
		if((rd_buf[0] & 0x01) == 0x01) otp_done = 1;
	}
	return true;
}

u8 dw1000_otpwriteandverify(u32 value, u16 address){
	int prog_ok = DW1000_SUCCESS;
	int retry = 0;
	// Firstly set the system clock to crystal
	_dw1000_enableclock(FORCE_SYS_XTI); //set system clock to XTI
	
	//
	//!!!!!!!!!!!!!! NOTE !!!!!!!!!!!!!!!!!!!!!
	//Set the supply to 3.7V
	//
	
	_dw1000_otpsetmrregs(1); // Set mode for programming
	
	// For each value to program - the readback/check is done couple of times to verify it has programmed successfully
	do{
		_dw1000_otpprogword32(value, address);
		retry++;
	}while(!(_dw1000_otpread(address) == value) || retry > 10);
	
	// Even ifthe above does not exit before retry reaches 10, the programming has probably been successful
	
	// Set mode for reading
	_dw1000_otpsetmrregs(4);
	
	// If this does not pass please check voltage supply on VDDIO
	if(_dw1000_otpread(address) != value) prog_ok = false;
	
	// Setting OTP mode register for low RM read - resetting the device would be alternative
	_dw1000_otpsetmrregs(0); 
	
	return prog_ok;
}

void _dw1000_aonconfigupload(){
	addr_data[0] = AON_CTRL_UPL_CFG;
	dw1000_writeSPI(AON_ID, AON_CTRL_OFFSET, addr_data, 1);
	// Clear the register
	addr_data[0] = 0;
	dw1000_writeSPI(AON_ID, AON_CTRL_OFFSET, addr_data, 1);
}

void _dw1000_aonarrayupload(){
	// Clear the register
	addr_data[0] = 0;
	dw1000_writeSPI(AON_ID, AON_CTRL_OFFSET, addr_data, 1);
	addr_data[0] = AON_CTRL_SAVE;
	dw1000_writeSPI(AON_ID, AON_CTRL_OFFSET, addr_data, 1);
}

void dw1000_entersleep(){
	// Copy config to AON - upload the new configuration
	_dw1000_aonarrayupload();
	#ifdef DEBUG
	sprintf(buf, "enter sleep\n\r");
	UART_Send(buf, 13);
	#endif
}

void dw1000_configuresleepcnt(u16 sleepcnt){
	// Force system clock to crystal
	_dw1000_enableclock(FORCE_SYS_XTI);
	
	// Reset sleep configuration to make sure we don't accidentally go to sleep
	// NB: this write change the default LPCLKDIVA value which is not used anyway.
	addr_data[0] = 0;
	dw1000_writeSPI(AON_ID, AON_CFG0_OFFSET, addr_data, 1);
	addr_data[0] = 0;
	dw1000_writeSPI(AON_ID, AON_CFG1_OFFSET, addr_data, 1);
	
	// Disable the sleep counter
	_dw1000_aonconfigupload();
	
	// Set new value
	addr_data[1] = (u8)(sleepcnt >> 8);
	addr_data[0] = (u8)(sleepcnt);
	dw1000_writeSPI(AON_ID, AON_CFG0_OFFSET + AON_CFG0_SLEEP_TIM_OFFSET, addr_data, 2);
	_dw1000_aonconfigupload();
	
	// Enable the sleep counter
	addr_data[0] = AON_CFG1_SLEEP_CEN;
	dw1000_writeSPI(AON_ID, AON_CFG1_OFFSET, addr_data, 1);
	_dw1000_aonconfigupload();
	
	// Put system PLL back on
	_dw1000_enableclock(ENABLE_ALL_SEQ);
}

u16 dw1000_calibratesleepcnt(){
	u16 result;
	
	// Enable calibration of the sleep counter
	addr_data[0] = AON_CFG1_LPOSC_CAL;
	dw1000_writeSPI(AON_ID, AON_CFG1_OFFSET, addr_data, 1);
	_dw1000_aonconfigupload();
	
	// Disable calibration of the sleep counter
	addr_data[0] = 0;
	dw1000_writeSPI(AON_ID, AON_CFG1_OFFSET, addr_data, 1);
	_dw1000_aonconfigupload();
	
	// Force system clock to crystal
	_dw1000_enableclock(FORCE_SYS_XTI);
	
	dw1000_delayms(1);
	
	// Read the number of XTAL/2 cycles one LP oscillator cycle took.
	// Set up address - Read upper byte first
	addr_data[0] = AON_ADDR_LPOSC_CAL_1;
	dw1000_writeSPI(AON_ID, AON_ADDR_OFFSET, addr_data, 1);
	
	// Enable manual override
	addr_data[0] = AON_CTRL_DCA_ENAB;
	dw1000_writeSPI(AON_ID, AON_CTRL_OFFSET, addr_data, 1);
	
	// Read confirm data that was written
	addr_data[0] = AON_CTRL_DCA_ENAB | AON_CTRL_DCA_READ;
	dw1000_writeSPI(AON_ID, AON_CTRL_OFFSET, addr_data, 1);
	
	// Read back byte from AON
	dw1000_readSPI(AON_ID, AON_RDAT_OFFSET, addr_data, 1);
	result = addr_data[0] << 8;
	
	// Set up address - Read lower byte
	addr_data[0] = AON_ADDR_LPOSC_CAL_0;
	dw1000_writeSPI(AON_ID, AON_ADDR_OFFSET, addr_data, 1);
	
	// Enable manual override
	addr_data[0] = AON_CTRL_DCA_ENAB;
	dw1000_writeSPI(AON_ID, AON_CTRL_OFFSET, addr_data, 1);
	
	// Read confirm data that was written
	addr_data[0] = AON_CTRL_DCA_ENAB | AON_CTRL_DCA_READ;
	dw1000_writeSPI(AON_ID, AON_CTRL_OFFSET, addr_data, 1);
	
	// Read back byte from AON
	dw1000_readSPI(AON_ID, AON_RDAT_OFFSET, addr_data, 1);
	result |= addr_data[0];
	
	// Disable manual override
	addr_data[0] = 0;
	dw1000_writeSPI(AON_ID, AON_CTRL_OFFSET, addr_data, 1);
	
	// Put system PLL back on
	_dw1000_enableclock(ENABLE_ALL_SEQ);
	
	// Returns the number of XTAL/2 cycles per one LP OSC cycle
	// This can be converted into LP OSC frequency by 19.2 MHz/result
	return result;
}

void dw1000_configuresleep(u16 mode, u8 wake){
	// Add predefined sleep settings before writing the mode
	mode |= local_dw.sleep_mode;
	addr_data[1] = (u8)(mode >> 8);
	addr_data[0] = (u8)(mode);
	dw1000_writeSPI(AON_ID, AON_WCFG_OFFSET, addr_data, 2);
	
	addr_data[1] = (u8)(wake >> 8);
	addr_data[0] = (u8)(wake);
	dw1000_writeSPI(AON_ID, AON_CFG0_OFFSET, addr_data ,2);
	#ifdef DEBUG
	sprintf(buf, "configuresleep: mode: 0x%x, wake: 0x%x\n\r", mode, wake);
	UART_Send(buf, 44);
	#endif
}

void dw1000_entersleepaftertx(u8 enable){
	dw1000_readSPI(PMSC_ID, PMSC_CTRL1_OFFSET, addr_data, 4);
	u32 reg = addr_data[3] << 24 | addr_data[2] << 16 | addr_data[1] << 8 | addr_data[0];

	// Set the auto TX -> sleep bit
	if(enable) reg |= PMSC_CTRL1_ATXSLP;
	else reg &= ~(PMSC_CTRL1_ATXSLP);

	addr_data[3] = (u8)(reg >> 24);
	addr_data[2] = (u8)(reg >> 16);
	addr_data[1] = (u8)(reg >> 8);
	addr_data[0] = (u8)(reg);
	dw1000_writeSPI(PMSC_ID, PMSC_CTRL1_OFFSET, addr_data, 4);

	#ifdef DEBUG
	sprintf(buf, "entersleepaftertx: 0x%x\n\r", reg);
	UART_Send(buf, 31);
	#endif
}

bool dw1000_spicswakeup(u8 *buff, u16 length){
	set_spi_low();
	
	// Device was in deep sleep (the first read fails)
	dw1000_readdevid();
	if((u32)(local_dw.dev_id.ridtag << 16 | local_dw.dev_id.model << 8 | local_dw.dev_id.ver | local_dw.dev_id.rev) != DW1000_DEVICE_ID){
		// Need to keep chip select line low for at least 500us
		// Do a long read to wake up the chip (hold the chip select low)
		dw1000_readSPI(0x0, 0x0, buff, length); 
		
		// Need 5ms for XTAL to start and stabilise (could wait for PLL lock IRQ status bit !!!)
		// NOTE: Polling of the STATUS register is not possible unless frequency is < 3MHz
		dw1000_delayms(5);
	}else{
		set_spi_high();
		return true;
	}
	
	// DEBUG - check ifstill in sleep mode
	dw1000_readdevid();
	if((u32)(local_dw.dev_id.ridtag << 16 | local_dw.dev_id.model << 8 | local_dw.dev_id.ver | local_dw.dev_id.rev) != DW1000_DEVICE_ID) 
		return false;
	set_spi_high();
	return true;
}

void dw1000_configlde(u8 prfIndex){
	// 8-bit configuration register
	addr_data[0] = LDE_PARAM1;
	dw1000_writeSPI(LDE_IF_ID, LDE_CFG1_OFFSET, addr_data, 1);
	
	if(prfIndex){
		// 16-bit LDE configuration tuning register
		addr_data[1] = (u8)(LDE_PARAM3_64 >> 8);
		addr_data[0] = (u8)(LDE_PARAM3_64);
		dw1000_writeSPI( LDE_IF_ID, LDE_CFG2_OFFSET, addr_data, 2);
	}else{
		addr_data[1] = (u8)(LDE_PARAM3_16 >> 8);
		addr_data[0] = (u8)(LDE_PARAM3_16);
		dw1000_writeSPI( LDE_IF_ID, LDE_CFG2_OFFSET, addr_data, 2);
	}
}

void dw1000_loaducodefromrom(){
	// set up clocks
	_dw1000_enableclock(FORCE_LDE);
	
	// kick off the LDE load
	addr_data[1] = (u8)(OTP_CTRL_LDELOAD >> 8);
	addr_data[0] = (u8)(OTP_CTRL_LDELOAD);
	dw1000_writeSPI(OTP_IF_ID, OTP_CTRL, addr_data, 2);
	
	// Allow time for code to upload (should take up to 120 us)
	dw1000_delayms(1);
	
	// Default clocks (ENABLE_ALL_SEQ)
	_dw1000_enableclock(ENABLE_ALL_SEQ); // Enable clocks for sequencing
}

void dw1000_loadopsettabfromotp(u8 ops_sel){
	// Select defined OPS table and trigger its loading
	u16 reg = ((ops_sel << OTP_SF_OPS_SEL_SHFT) & OTP_SF_OPS_SEL_MASK) | OTP_SF_OPS_KICK;
	
	// Set up clocks
	_dw1000_enableclock(FORCE_LDE);
	
	addr_data[1] = (u8)(reg >> 8);
	addr_data[0] = (u8)(reg);
	dw1000_writeSPI(OTP_IF_ID, OTP_SF, addr_data, 2);
	
	// Default clocks (ENABLE_ALL_SEQ)
	// Enable clocks for sequencing
	_dw1000_enableclock(ENABLE_ALL_SEQ);
}

void dw1000_setsmarttxpower(u8 enable){
	// Config system register
	// Read sysconfig register
	dw1000_readSPI(SYS_CFG_ID, 0, addr_data, 4);
	local_dw.sysCFGreg = addr_data[3] << 24 | addr_data[2] << 16 | addr_data[1] << 8 | addr_data[0];
	// Disable smart power configuration
	if(enable){
		local_dw.sysCFGreg &= ~(SYS_CFG_DIS_STXP);
	}else{
		local_dw.sysCFGreg |= SYS_CFG_DIS_STXP;
	}
	addr_data[3] = (u8)(local_dw.sysCFGreg >> 24);
	addr_data[2] = (u8)(local_dw.sysCFGreg >> 16);
	addr_data[1] = (u8)(local_dw.sysCFGreg >> 8);
	addr_data[0] = (u8)(local_dw.sysCFGreg);
	dw1000_writeSPI(SYS_CFG_ID, 0, addr_data, 4);
	//#ifdef DEBUG
	sprintf(buf, "setsmarttxpower: 0x%x\n\r", local_dw.sysCFGreg);
	UART_Send(buf, 29);
	//#endif
}

void dw1000_enableautoack(u8 responseDelayTime){
	// Set auto ACK reply delay in symbols
	dw1000_writeSPI(ACK_RESP_T_ID, ACK_RESP_T_ACK_TIM_OFFSET, &responseDelayTime, 1);
	// Enable auto ACK
	local_dw.sysCFGreg |= SYS_CFG_AUTOACK;
	addr_data[3] = (u8)(local_dw.sysCFGreg >> 24);
	addr_data[2] = (u8)(local_dw.sysCFGreg >> 16);
	addr_data[1] = (u8)(local_dw.sysCFGreg >> 8);
	addr_data[0] = (u8)(local_dw.sysCFGreg);
	dw1000_writeSPI(SYS_CFG_ID, 0, addr_data, 4);
	#ifdef DEBUG
	sprintf(buf, "enableautoack: 0x%x\n\r", local_dw.sysCFGreg);
	UART_Send(buf, 27);
	#endif
}

void dw1000_setdblrxbuffmode(u8 enable){
	if(enable){
		// Enable double RX buffer mode
		local_dw.sysCFGreg &= ~SYS_CFG_DIS_DRXB;
		local_dw.dblbuffon = 1;
	}else{
		// Disable double RX buffer mode
		local_dw.sysCFGreg |= SYS_CFG_DIS_DRXB;
		local_dw.dblbuffon = 0;
	}
	addr_data[3] = (u8)(local_dw.sysCFGreg >> 24);
	addr_data[2] = (u8)(local_dw.sysCFGreg >> 16);
	addr_data[1] = (u8)(local_dw.sysCFGreg >> 8);
	addr_data[0] = (u8)(local_dw.sysCFGreg);
	dw1000_writeSPI(SYS_CFG_ID, 0, addr_data, 4);
	#ifdef DEBUG
	sprintf(buf, "setdblrxbuffmode: 0x%x\n\r", local_dw.sysCFGreg);
	UART_Send(buf, 30);
	#endif
}

void dw1000_setrxaftertxdelay(u32 rxDelayTime){
	// Read ACK_RESP_T_ID register
	dw1000_readSPI(ACK_RESP_T_ID, 0, addr_data, 4);
	u32 val = addr_data[3] << 24 | addr_data[2] << 16 | addr_data[1] << 8 | addr_data[0];

	// Clear the timer (19:0)
	val &= ~(ACK_RESP_T_W4R_TIM_MASK);

	// In UWB microseconds (e.g. turn the receiver on 20uus after TX)
	val |= (rxDelayTime & ACK_RESP_T_W4R_TIM_MASK);

	addr_data[3] = (u8)(val >> 24);
	addr_data[2] = (u8)(val >> 16);
	addr_data[1] = (u8)(val >> 8);
	addr_data[0] = (u8)(val);
	dw1000_writeSPI(ACK_RESP_T_ID, 0, addr_data, 4);

	#ifdef DEBUG
	sprintf(buf, "setrxaftertxdelay: 0x%x\n\r", val);
	UART_Send(buf, 31);
	#endif
}

void dw1000_setcallbacks(dw1000_cb_t cbTxDone, dw1000_cb_t cbRxOk, dw1000_cb_t cbRxTo, dw1000_cb_t cbRxErr){
	local_dw.cbTxDone = cbTxDone;
	local_dw.cbRxOk = cbRxOk;
	local_dw.cbRxTo = cbRxTo;
	local_dw.cbRxErr = cbRxErr;
}

u8 dw1000_checkirq(){
	dw1000_readSPI(SYS_STATUS_ID, SYS_STATUS_OFFSET, addr_data, 1);
	// Reading the lower byte only is enough for this operation
	return (addr_data[0] & SYS_STATUS_IRQS);
}

void dw1000_isr(){
	// Read status register low 32bits
	dw1000_readSPI(SYS_STATUS_ID, 0, addr_data, 4);
	u32 status = local_dw.cbData.status = addr_data[3] << 24 | addr_data[2] << 16 | addr_data[1] << 8 | addr_data[0];

	// Handle RX good frame event
	if(status & SYS_STATUS_RXFCG){
		u16 finfo16;
		u16 len;

		// Clear all receive status bits
		addr_data[3] = (u8)(SYS_STATUS_ALL_RX_GOOD >> 24);
		addr_data[2] = (u8)(SYS_STATUS_ALL_RX_GOOD >> 16);
		addr_data[1] = (u8)(SYS_STATUS_ALL_RX_GOOD >> 8);
		addr_data[0] = (u8)(SYS_STATUS_ALL_RX_GOOD);
		dw1000_writeSPI(SYS_STATUS_ID, 0, addr_data, 4);
		local_dw.cbData.rx_flags = 0;

		// Read frame info - Only the first two bytes of the register are used here.
		dw1000_readSPI(RX_FINFO_ID, RX_FINFO_OFFSET, addr_data, 2);
		finfo16 = addr_data[1] << 8 | addr_data[0];

		// Report frame length - Standard frame length up to 127, extended frame length up to 1023 bytes
		len = finfo16 & RX_FINFO_RXFL_MASK_1023;
		if(local_dw.longFrames == 0) len &= RX_FINFO_RXFLEN_MASK;
		local_dw.cbData.datalength = len;

		// Report ranging bit
		if(finfo16 & RX_FINFO_RNG) local_dw.cbData.rx_flags |= DW1000_CB_DATA_RX_FLAG_RNG;

		// Report frame control - First bytes of the received frame.
		dw1000_readSPI(RX_BUFFER_ID, 0, local_dw.cbData.fctrl, FCTRL_LEN_MAX);

		// Because of a previous frame not being received properly, AAT bit can be set upon the proper reception of a frame not requesting for
		// acknowledgement (ACK frame is not actually sent though). If the AAT bit is set, check ACK request bit in frame control to confirm (this
		// implementation works only for IEEE802.15.4-2011 compliant frames).
		// This issue is not documented at the time of writing this code. It should be in next release of local_dw User Manual (v2.09, from July 2016).
		if((status & SYS_STATUS_AAT) && ((local_dw.cbData.fctrl[0] & FCTRL_ACK_REQ_MASK) == 0)){
			// Clear AAT status bit in register
			addr_data[3] = (u8)(SYS_STATUS_AAT >> 24);
			addr_data[2] = (u8)(SYS_STATUS_AAT >> 16);
			addr_data[1] = (u8)(SYS_STATUS_AAT >> 8);
			addr_data[0] = (u8)(SYS_STATUS_AAT);
			dw1000_writeSPI(SYS_STATUS_ID, 0, addr_data, 4);

			// Clear AAT status bit in callback data register copy
			local_dw.cbData.status &= ~SYS_STATUS_AAT;
			local_dw.wait4resp = 0;
		}

		// Call the corresponding callback if present
		if(local_dw.cbRxOk != NULL) local_dw.cbRxOk(&local_dw.cbData);

		if(local_dw.dblbuffon){
			// Toggle the Host side Receive Buffer Pointer
			addr_data[0] = 1;
			dw1000_writeSPI(SYS_CTRL_ID, SYS_CTRL_HRBT_OFFSET, addr_data, 1);
		}
	}
	// Handle TX confirmation event
	if(status & SYS_STATUS_TXFRS){
		// Clear TX event bits
		addr_data[3] = (u8)(SYS_STATUS_ALL_TX >> 24);
		addr_data[2] = (u8)(SYS_STATUS_ALL_TX >> 16);
		addr_data[1] = (u8)(SYS_STATUS_ALL_TX >> 8);
		addr_data[0] = (u8)(SYS_STATUS_ALL_TX);
		dw1000_writeSPI(SYS_STATUS_ID, 0, addr_data, 4);

		// In the case where this TXFRS interrupt is due to the automatic transmission of an ACK solicited by a response (with ACK request bit set)
		// that we receive through using wait4resp to a previous TX (and assuming that the IRQ processing of that TX has already been handled), then
		// we need to handle the IC issue which turns on the RX again in this situation (i.e. because it is wrongly applying the wait4resp after the
		// ACK TX).
		// See section "Transmit and automatically wait for response" in local_dw User Manual
		if((status & SYS_STATUS_AAT) && local_dw.wait4resp){
			// Turn the RX off
			dw1000_forcetrxoff();

			// Reset in case we were late and a frame was already being received
			dw1000_rxreset();
		}

		// Call the corresponding callback if present
		if(local_dw.cbTxDone != NULL) local_dw.cbTxDone(&local_dw.cbData);
	}

	// Handle frame reception/preamble detect timeout events
	if(status & SYS_STATUS_ALL_RX_TO){
		// Clear RX timeout event bits
		addr_data[3] = (u8)(SYS_STATUS_RXRFTO >> 24);
		addr_data[2] = (u8)(SYS_STATUS_RXRFTO >> 16);
		addr_data[1] = (u8)(SYS_STATUS_RXRFTO >> 8);
		addr_data[0] = (u8)(SYS_STATUS_RXRFTO);
		dw1000_writeSPI(SYS_STATUS_ID, 0, addr_data, 4);
		local_dw.wait4resp = 0;

		// Because of an issue with receiver restart after error conditions, an RX reset must be applied after any error or timeout event to ensure
		// the next good frame's timestamp is computed correctly.
		// See section "RX Message timestamp" in local_dw User Manual.
		dw1000_forcetrxoff();
		dw1000_rxreset();

		// Call the corresponding callback if present
		if(local_dw.cbRxTo != NULL) local_dw.cbRxTo(&local_dw.cbData);
	}

	// Handle RX errors events
	if(status & SYS_STATUS_ALL_RX_ERR){
		// Clear RX error event bits
		addr_data[3] = (u8)(SYS_STATUS_ALL_RX_ERR >> 24);
		addr_data[2] = (u8)(SYS_STATUS_ALL_RX_ERR >> 16);
		addr_data[1] = (u8)(SYS_STATUS_ALL_RX_ERR >> 8);
		addr_data[0] = (u8)(SYS_STATUS_ALL_RX_ERR);
		dw1000_writeSPI(SYS_STATUS_ID, 0, addr_data, 4);
		local_dw.wait4resp = 0;

		// Because of an issue with receiver restart after error conditions, an RX reset must be applied after any error or timeout event to ensure
		// the next good frame's timestamp is computed correctly.
		// See section "RX Message timestamp" in local_dw User Manual.
		dw1000_forcetrxoff();
		dw1000_rxreset();

		// Call the corresponding callback if present
		if(local_dw.cbRxErr != NULL) local_dw.cbRxErr(&local_dw.cbData);
	}
}

void dw1000_lowpowerlistenisr(){
	// Read status register low 32bits
	dw1000_readSPI(SYS_STATUS_ID, 0, addr_data, 4);
	u32 status = local_dw.cbData.status = addr_data[3] << 24 | addr_data[2] << 16 | addr_data[1] << 8 | addr_data[0];
	u16 finfo16;
	u16 len;

	// The only interrupt handled when in low-power listening mode is RX good frame so proceed directly to the handling of the received frame.
	// Deactivate low-power listening before clearing the interrupt. If not, the local_dw will go back to sleep as soon as the interrupt is cleared.
	dw1000_setlowpowerlistening(0);

	// Clear all receive status bits
	addr_data[3] = (u8)(SYS_STATUS_ALL_RX_GOOD >> 24);
	addr_data[2] = (u8)(SYS_STATUS_ALL_RX_GOOD >> 16);
	addr_data[1] = (u8)(SYS_STATUS_ALL_RX_GOOD >> 8);
	addr_data[0] = (u8)(SYS_STATUS_ALL_RX_GOOD);
	dw1000_writeSPI(SYS_STATUS_ID, 0, addr_data, 4);
	local_dw.cbData.rx_flags = 0;

	// Read frame info - Only the first two bytes of the register are used here.
	dw1000_readSPI(RX_FINFO_ID, 0, addr_data, 2);
	finfo16 = addr_data[1] << 8 | addr_data[0];

	// Report frame length - Standard frame length up to 127, extended frame length up to 1023 bytes
	len = finfo16 & RX_FINFO_RXFL_MASK_1023;
	if(local_dw.longFrames == 0) len &= RX_FINFO_RXFLEN_MASK;
	local_dw.cbData.datalength = len;

	// Report ranging bit
	if(finfo16 & RX_FINFO_RNG) local_dw.cbData.rx_flags |= DW1000_CB_DATA_RX_FLAG_RNG;

	// Report frame control - First bytes of the received frame.
	dw1000_readSPI(RX_BUFFER_ID, 0, local_dw.cbData.fctrl, FCTRL_LEN_MAX);

	// Because of a previous frame not being received properly, AAT bit can be set upon the proper reception of a frame not requesting for
	// acknowledgement (ACK frame is not actually sent though). If the AAT bit is set, check ACK request bit in frame control to confirm (this
	// implementation works only for IEEE802.15.4-2011 compliant frames).
	// This issue is not documented at the time of writing this code. It should be in next release of local_dw User Manual (v2.09, from July 2016).
	if((status & SYS_STATUS_AAT) && ((local_dw.cbData.fctrl[0] & FCTRL_ACK_REQ_MASK) == 0)){
		// Clear AAT status bit in register
		addr_data[3] = (u8)(SYS_STATUS_AAT >> 24);
		addr_data[2] = (u8)(SYS_STATUS_AAT >> 16);
		addr_data[1] = (u8)(SYS_STATUS_AAT >> 8);
		addr_data[0] = (u8)(SYS_STATUS_AAT);
		dw1000_writeSPI(SYS_STATUS_ID, 0, addr_data, 4);

		// Clear AAT status bit in callback data register copy
		local_dw.cbData.status &= ~SYS_STATUS_AAT;
		local_dw.wait4resp = 0;
	}

	// Call the corresponding callback if present
	if(local_dw.cbRxOk != NULL) local_dw.cbRxOk(&local_dw.cbData);
}

void dw1000_setleds(u8 mode){
	u32 reg;
	if(mode & DW1000_LEDS_ENABLE){
		// Set up MFIO for LED output.
		dw1000_readSPI(GPIO_CTRL_ID, GPIO_MODE_OFFSET, addr_data, 4);
		reg = addr_data[3] << 24 | addr_data[2] << 16 | addr_data[1] << 8 | addr_data[0];
		sprintf(buf, "Start config DW1000 LED\n\r");
		UART_Send(buf, 25);
		
		reg &= ~(GPIO_MSGP2_MASK | GPIO_MSGP3_MASK);
		reg |= (GPIO_PIN2_RXLED | GPIO_PIN3_TXLED);
		addr_data[3] = (u8)(reg >> 24);
		addr_data[2] = (u8)(reg >> 16);
		addr_data[1] = (u8)(reg >> 8);
		addr_data[0] = (u8)(reg);
		dw1000_writeSPI(GPIO_CTRL_ID, GPIO_MODE_OFFSET, addr_data, 4);
		#ifdef DEBUG
		//sprintf(buf, "2setleds: 0x%x\n\r", addr_data[3] << 24 | addr_data[2] << 16 | addr_data[1] << 8 | addr_data[0]);
		//UART_Send(buf, 22);
		#endif
		
		// Enable LP Oscillator to run from counter and turn on de-bounce clock.
		dw1000_readSPI(PMSC_ID, PMSC_CTRL0_OFFSET, addr_data, 4);
		reg = addr_data[3] << 24 | addr_data[2] << 16 | addr_data[1] << 8 | addr_data[0];
		#ifdef DEBUG
		//sprintf(buf, "3setleds: 0x%x\n\r", reg);
		//UART_Send(buf, 22);
		#endif
		
		reg |= (PMSC_CTRL0_GPDCE | PMSC_CTRL0_KHZCLEN);
		addr_data[3] = (u8)(reg >> 24);
		addr_data[2] = (u8)(reg >> 16);
		addr_data[1] = (u8)(reg >> 8);
		addr_data[0] = (u8)(reg);
		dw1000_writeSPI(PMSC_ID, PMSC_CTRL0_OFFSET, addr_data, 4);
		#ifdef DEBUG
		//sprintf(buf, "4setleds: 0x%x\n\r", addr_data[3] << 24 | addr_data[2] << 16 | addr_data[1] << 8 | addr_data[0]);
		//UART_Send(buf, 22);
		#endif
		
		// Enable LEDs to blink and set default blink time.
		reg = PMSC_LEDC_BLNKEN | PMSC_LEDC_BLINK_TIME_DEF;
		#ifdef DEBUG
		//sprintf(buf, "5setleds: 0x%x\n\r", reg);
		//UART_Send(buf, 22);
		#endif
		
		// Make LEDs blink once if requested.
		if(mode & DW1000_LEDS_INIT_BLINK) reg |= PMSC_LEDC_BLINK_NOW_ALL;
		#ifdef DEBUG
		//sprintf(buf, "6setleds: 0x%x\n\r", reg);
		//UART_Send(buf, 22);
		#endif
		
		addr_data[3] = (u8)(reg >> 24);
		addr_data[2] = (u8)(reg >> 16);
		addr_data[1] = (u8)(reg >> 8);
		addr_data[0] = (u8)(reg);
		dw1000_writeSPI(PMSC_ID, PMSC_LEDC_OFFSET, addr_data, 4);
		#ifdef DEBUG
		//sprintf(buf, "7setleds: 0x%x\n\r", addr_data[3] << 24 | addr_data[2] << 16 | addr_data[1] << 8 | addr_data[0]);
		//UART_Send(buf, 22);
		#endif
		
		// Clear force blink bits if needed.
		if(mode & DW1000_LEDS_INIT_BLINK){
			reg &= ~PMSC_LEDC_BLINK_NOW_ALL;
			addr_data[3] = (u8)(reg >> 24);
			addr_data[2] = (u8)(reg >> 16);
			addr_data[1] = (u8)(reg >> 8);
			addr_data[0] = (u8)(reg);
			dw1000_writeSPI(PMSC_ID, PMSC_LEDC_OFFSET, addr_data, 4);
			sprintf(buf, "DW1000 LED config success\n\r");
			UART_Send(buf, 27);
		}
	} else {
		// Clear the GPIO bits that are used for LED control.
		dw1000_readSPI(GPIO_CTRL_ID, GPIO_MODE_OFFSET, addr_data, 4);
		reg = addr_data[3] << 24 | addr_data[2] << 16 | addr_data[1] << 8 | addr_data[0];
		reg &= ~(GPIO_MSGP2_MASK | GPIO_MSGP3_MASK);
		addr_data[3] = (u8)(reg >> 24);
		addr_data[2] = (u8)(reg >> 16);
		addr_data[1] = (u8)(reg >> 8);
		addr_data[0] = (u8)(reg);
		dw1000_writeSPI(GPIO_CTRL_ID, GPIO_MODE_OFFSET, addr_data, 4);
		#ifdef DEBUG
		sprintf(buf, "9setleds: 0x%x\n\r", addr_data[3] << 24 | addr_data[2] << 16 | addr_data[1] << 8 | addr_data[0]);
		UART_Send(buf, 22);
		#endif
	}
}

void _dw1000_enableclock(u8 clock){
	#ifdef DEBUG
		sprintf(buf, "enableclock: ");
		UART_Send(buf, 13);
	#endif
	
	dw1000_readSPI(PMSC_ID, PMSC_CTRL0_OFFSET, addr_data, 2);
	
	#ifdef DEBUG
		sprintf(buf, "0x%x, 0x%x;  ", addr_data[1], addr_data[0]);
		UART_Send(buf, 15);
	#endif
	
	switch(clock){
		case ENABLE_ALL_SEQ:
		{
				addr_data[0] = 0x00;
				addr_data[1] = addr_data[1] & 0xfe;
		}
		break;
		case FORCE_SYS_XTI:
		{
			// System and RX
			addr_data[0] = 0x01 | (addr_data[0] & 0xfc);
		}
		break;
		case FORCE_SYS_PLL:
		{
			// System
			addr_data[0] = 0x02 | (addr_data[0] & 0xfc);
		}
		break;
		case READ_ACC_ON:
		{
			addr_data[0] = 0x48 | (addr_data[0] & 0xb3);
			addr_data[1] = 0x80 | addr_data[1];
		}
		break;
		case READ_ACC_OFF:
		{
			addr_data[0] = addr_data[0] & 0xb3;
			addr_data[1] = 0x7f & addr_data[1];
		}
		break;
		case FORCE_OTP_ON:
		{
			addr_data[1] = 0x02 | addr_data[1];
		}
		break;
		case FORCE_OTP_OFF:
		{
			addr_data[1] = addr_data[1] & 0xfd;
		}
		break;
		case FORCE_TX_PLL:
		{
			addr_data[0] = 0x20 | (addr_data[0] & 0xcf);
		}
		break;
		case FORCE_LDE:
		{
			addr_data[0] = 0x01;
			addr_data[1] = 0x03;
		}
		break;
		default:
		break;
	}
	#ifdef DEBUG
	sprintf(buf, "0x%x, 0x%x\n\r", addr_data[1], addr_data[0]);
	UART_Send(buf, 15);
	#endif
	dw1000_writeSPI(PMSC_ID, PMSC_CTRL0_OFFSET, &addr_data[0], 1);
	dw1000_writeSPI(PMSC_ID, 0x1, &addr_data[1], 1);
}

// Disable sequencing and go to state "INIT"
void _dw1000_disablesequencing(){
	// Set system clock to XTI
	_dw1000_enableclock(FORCE_SYS_XTI);
	// Disable PMSC ctrl of RF and RX clk blocks
	addr_data[1] = (u8)(PMSC_CTRL1_PKTSEQ_DISABLE >> 8);
	addr_data[0] = (u8)(PMSC_CTRL1_PKTSEQ_DISABLE);
	dw1000_writeSPI(PMSC_ID, PMSC_CTRL1_OFFSET, addr_data, 2);
}

void dw1000_setdelayedtrxtime(u32 starttime){
	// Write at offset 1 as the lower 9 bits of this register are ignored
	addr_data[3] = (u8)(starttime >> 24);
	addr_data[2] = (u8)(starttime >> 16);
	addr_data[1] = (u8)(starttime >> 8);
	addr_data[0] = (u8)(starttime);
	dw1000_writeSPI(DX_TIME_ID, 1, addr_data, 4);
}

bool dw1000_starttx(u8 mode){
	bool retval = false;
	u8 temp = 0;

	if(mode & DW1000_RESPONSE_EXPECTED){
		temp = (u8)(SYS_CTRL_WAIT4RESP); // Set wait4response bit
		local_dw.wait4resp = 1;
	}

	if(mode & DW1000_START_TX_DELAYED){
		// Both SYS_CTRL_TXSTRT and SYS_CTRL_TXDLYS to correctly enable TX
		temp |= (u8)(SYS_CTRL_TXDLYS | SYS_CTRL_TXSTRT);
		addr_data[0] = temp & 0xff;
		dw1000_writeSPI(SYS_CTRL_ID, SYS_CTRL_OFFSET, addr_data, 1);
		
		// Read at offset 3 to get the upper 2 bytes out of 5
		dw1000_readSPI(SYS_STATUS_ID, 3, addr_data, 2);
		u16 checkTxOK = addr_data[1] << 8 | addr_data[0];
		// Transmit Delayed Send set over Half a Period away or Power Up error 
		//(there is enough time to send but not to power up individual blocks).
		if((checkTxOK & SYS_STATUS_TXERR) == 0){
			retval = true; // All okay
			#ifdef DEBUG
			sprintf(buf, "sent delayed!\n\r");
			UART_Send(buf, 15);
			#endif
		}else{
			// If HPDWARN or TXPUTE are set this indicates that the TXDLYS was set too late for the specified DX_TIME.
			// remedial action is to cancel delayed send and report error
			addr_data[0] = (u8)(SYS_CTRL_TRXOFF);
			dw1000_writeSPI(SYS_CTRL_ID, SYS_CTRL_OFFSET, addr_data, 1);
			retval = false; // Failed !
			#ifdef DEBUG
			sprintf(buf, "not sent!\n\r");
			UART_Send(buf, 11);
			#endif
		}
	}else{
		temp |= (u8)(SYS_CTRL_TXSTRT);
		addr_data[0] = temp & 0xff;
		dw1000_writeSPI(SYS_CTRL_ID, SYS_CTRL_OFFSET, addr_data, 1);
	}
	
	return retval;

}

bool dw1000_starttx_no_auto_fcs(u8 mode){
	bool retval = true;
	addr_data[0] = (u8)(SYS_CTRL_SFCST);
	
	if(mode & DW1000_RESPONSE_EXPECTED){
		addr_data[0] |= (u8)(SYS_CTRL_WAIT4RESP); // Set wait4response bit
		local_dw.wait4resp = 1;
	}
	
	if(mode & DW1000_START_TX_DELAYED){
		// Both SYS_CTRL_TXSTRT and SYS_CTRL_TXDLYS to correctly enable TX
		addr_data[0] |= (u8)(SYS_CTRL_TXDLYS | SYS_CTRL_TXSTRT);
		dw1000_writeSPI(SYS_CTRL_ID, SYS_CTRL_OFFSET, addr_data, 1);
		
		// Read at offset 3 to get the upper 2 bytes out of 5
		dw1000_readSPI(SYS_STATUS_ID, 3, addr_data, 2);
		u16 checkTxOK = addr_data[1] << 8 | addr_data[0];
		
		// Transmit Delayed Send set over Half a Period away or Power Up error 
		//(there is enough time to send but not to power up individual blocks).
		if((checkTxOK & SYS_STATUS_TXERR) == 0)
			retval = true; // All okay
		else{
			// If HPDWARN or TXPUTE are set this indicates that the TXDLYS was set too late for the specified DX_TIME.
			// remedial action is to cancel delayed send and report error
			addr_data[0] = (u8)(SYS_CTRL_TRXOFF);
			dw1000_writeSPI(SYS_CTRL_ID, SYS_CTRL_OFFSET, addr_data, 1);
			retval = false; // Failed !
		}
	}else{
		addr_data[0] |= (u8)(SYS_CTRL_TXSTRT);
		dw1000_writeSPI(SYS_CTRL_ID, SYS_CTRL_OFFSET, addr_data, 1);
	}
	
	return retval;
}

void dw1000_enable_auto_fcs(){
	addr_data[0] = (u8)(SYS_CTRL_CANSFCS);
	dw1000_writeSPI(SYS_CTRL_ID, SYS_CTRL_OFFSET, addr_data, 1);
}

void dw1000_forcetrxoff(){
	// Read set interrupt mask
	dw1000_readSPI(SYS_MASK_ID, 0, addr_data, 4);
	u32 mask = addr_data[3] << 24 | addr_data[2] << 16 | addr_data[1] << 8 | addr_data[0];

	// Need to beware of interrupts occurring in the middle of following read modify write cycle
	// We can disable the radio, but before the status is cleared an interrupt can be set (e.g. the
	// event has just happened before the radio was disabled)
	// thus we need to disable interrupt during this operation
	decaIrqStatus_t stat = decamutexon();

	// Clear interrupt mask - so we don't get any unwanted events
	addr_data[3] = (u8)(0 >> 24);
	addr_data[2] = (u8)(0 >> 16);
	addr_data[1] = (u8)(0 >> 8);
	addr_data[0] = (u8)(0);
	dw1000_writeSPI(SYS_MASK_ID, 0, addr_data, 4);

	// Disable the radio
	addr_data[0] = (u8)(SYS_CTRL_TRXOFF);
	dw1000_writeSPI(SYS_CTRL_ID, SYS_CTRL_OFFSET, addr_data, 1);

	// Forcing Transceiver off - so we do not want to see any new events that may have happened
	addr_data[3] = (u8)((SYS_STATUS_ALL_TX | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_GOOD) >> 24);
	addr_data[2] = (u8)((SYS_STATUS_ALL_TX | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_GOOD) >> 16);
	addr_data[1] = (u8)((SYS_STATUS_ALL_TX | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_GOOD) >> 8);
	addr_data[0] = (u8)((SYS_STATUS_ALL_TX | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_GOOD));
	dw1000_writeSPI(SYS_STATUS_ID, 0, addr_data, 4);

	dw1000_syncrxbufptrs();

	// Set interrupt mask to what it was
	addr_data[3] = (u8)(mask >> 24);
	addr_data[2] = (u8)(mask >> 16);
	addr_data[1] = (u8)(mask >> 8);
	addr_data[0] = (u8)(mask);
	dw1000_writeSPI(SYS_MASK_ID, 0, addr_data, 4);

	// Enable/restore interrupts again...
	decamutexoff(stat);
	local_dw.wait4resp = 0;
}

void dw1000_syncrxbufptrs(){
	u8  buff;
	// Need to make sure that the host/IC buffer pointers are aligned before starting RX
	// Read 1 byte at offset 3 to get the 4th byte out of 5
	dw1000_readSPI(SYS_STATUS_ID, 3, addr_data, 1);
	buff = addr_data[0];
	
	if((buff & (SYS_STATUS_ICRBP >> 24)) !=	 // IC side Receive Buffer Pointer
		((buff & (SYS_STATUS_HSRBP >> 24)) << 1) ) // Host Side Receive Buffer Pointer
	{
		addr_data[0] = 1;
		dw1000_writeSPI(SYS_CTRL_ID, SYS_CTRL_HRBT_OFFSET , addr_data, 1); // We need to swap RX buffer status reg (write one to toggle internally)
	}
}

void dw1000_setsniffmode(u8 enable, u8 timeOn, u8 timeOff){
	u32 pmsc_reg;
	if(enable){
		/* Configure ON/OFF times and enable PLL2 on/off sequencing by SNIFF mode. */
		u16 sniff_reg = (((u16)timeOff << 8) | timeOn) & RX_SNIFF_MASK;
		addr_data[1] = (u8)(sniff_reg >> 8);
		addr_data[0] = (u8)(sniff_reg);
		dw1000_writeSPI(RX_SNIFF_ID, RX_SNIFF_OFFSET, addr_data, 2);
		dw1000_readSPI(PMSC_ID, PMSC_CTRL0_OFFSET, addr_data, 4);
		pmsc_reg = addr_data[3] << 24 | addr_data[2] << 16 | addr_data[1] << 8 | addr_data[0];
		pmsc_reg |= PMSC_CTRL0_PLL2_SEQ_EN;
		addr_data[3] = (u8)(pmsc_reg >> 24);
		addr_data[2] = (u8)(pmsc_reg >> 16);
		addr_data[1] = (u8)(pmsc_reg >> 8);
		addr_data[0] = (u8)(pmsc_reg);
		dw1000_writeSPI(PMSC_ID, PMSC_CTRL0_OFFSET, addr_data, 4);
	} else {
		/* Clear ON/OFF times and disable PLL2 on/off sequencing by SNIFF mode. */
		addr_data[1] = (u8)(0 >> 8);
		addr_data[0] = (u8)(0);
		dw1000_writeSPI(RX_SNIFF_ID, RX_SNIFF_OFFSET, addr_data, 2);
		dw1000_readSPI(PMSC_ID, PMSC_CTRL0_OFFSET, addr_data, 4);
		pmsc_reg = addr_data[3] << 24 | addr_data[2] << 16 | addr_data[1] << 8 | addr_data[0];
		pmsc_reg &= ~PMSC_CTRL0_PLL2_SEQ_EN;
		addr_data[3] = (u8)(pmsc_reg >> 24);
		addr_data[2] = (u8)(pmsc_reg >> 16);
		addr_data[1] = (u8)(pmsc_reg >> 8);
		addr_data[0] = (u8)(pmsc_reg);
		dw1000_writeSPI(PMSC_ID, PMSC_CTRL0_OFFSET, addr_data, 4);
	}
}

void dw1000_setlowpowerlistening(u8 enable){
	dw1000_readSPI(PMSC_ID, PMSC_CTRL1_OFFSET, addr_data, 4);
	u32 pmsc_reg = addr_data[3] << 24 | addr_data[2] << 16 | addr_data[1] << 8 | addr_data[0];
	if(enable){
		/* Configure RX to sleep and snooze features. */
		pmsc_reg |= (PMSC_CTRL1_ARXSLP | PMSC_CTRL1_SNOZE);
	} else {
		/* Reset RX to sleep and snooze features. */
		pmsc_reg &= ~(PMSC_CTRL1_ARXSLP | PMSC_CTRL1_SNOZE);
	}
	addr_data[3] = (u8)(pmsc_reg >> 24);
	addr_data[2] = (u8)(pmsc_reg >> 16);
	addr_data[1] = (u8)(pmsc_reg >> 8);
	addr_data[0] = (u8)(pmsc_reg);
	dw1000_writeSPI(PMSC_ID, PMSC_CTRL1_OFFSET, addr_data, 4);
}

void dw1000_setsnoozetime(u8 snooze_time){
	addr_data[0] = (u8)(snooze_time);
	dw1000_writeSPI(PMSC_ID, PMSC_SNOZT_OFFSET, addr_data, 1);
}

bool dw1000_rxenable(u8 mode){
	if((mode & DW1000_NO_SYNC_PTRS) == 0) dw1000_syncrxbufptrs();
	u16 temp = (u16)SYS_CTRL_RXENAB;
	if(mode & DW1000_START_RX_DELAYED) temp |= (u16)SYS_CTRL_RXDLYE;
	addr_data[1] = (u8)(temp >> 8);
	addr_data[0] = (u8)(temp);
	dw1000_writeSPI(SYS_CTRL_ID, SYS_CTRL_OFFSET, addr_data, 2);

	// check for errors
	if(mode & DW1000_START_RX_DELAYED){
		// Read 1 byte at offset 3 to get the 4th byte out of 5
		dw1000_readSPI(SYS_STATUS_ID, 3, addr_data, 1);
		u8 temp1 = addr_data[0];

		// if delay has passed do immediate RX on unless DWT_IDLE_ON_DLY_ERR is true
		if((temp1 & (SYS_STATUS_HPDWARN >> 24)) != 0){
			dw1000_forcetrxoff(); // turn the delayed receive off
			// if DWT_IDLE_ON_DLY_ERR not set then re-enable receiver
			if((mode & DW1000_IDLE_ON_DLY_ERR) == 0) {
				addr_data[1] = (u8)(SYS_CTRL_RXENAB >> 8);
				addr_data[0] = (u8)(SYS_CTRL_RXENAB);
				dw1000_writeSPI(SYS_CTRL_ID, SYS_CTRL_OFFSET, addr_data, 2);
			}
			return false; // return warning indication
		}
	}
	return true;
}

void dw1000_setrxtimeout(u16 time){
	// Read at offset 3 to get the upper byte only
	dw1000_readSPI(SYS_CFG_ID, 3, addr_data, 1);
	u8 temp = addr_data[0];
	if(time > 0){
		addr_data[1] = (u8)(time >> 8);
		addr_data[0] = (u8)(time);
		dw1000_writeSPI(RX_FWTO_ID, RX_FWTO_OFFSET, addr_data, 2);

		// Shift RXWTOE mask as we read the upper byte only
		temp |= (u8)(SYS_CFG_RXWTOE >> 24);
		// OR in 32bit value (1 bit set), I know this is in high byte.
		local_dw.sysCFGreg |= SYS_CFG_RXWTOE;
		// Write at offset 3 to write the upper byte only
		addr_data[0] = temp;
		dw1000_writeSPI(SYS_CFG_ID, 3, addr_data, 1);
	} else {
		// Shift RXWTOE mask as we read the upper byte only
		temp &= ~(u8)(SYS_CFG_RXWTOE >> 24);
		// AND in inverted 32bit value (1 bit clear), I know this is in high byte.
		local_dw.sysCFGreg &= ~(SYS_CFG_RXWTOE);
		// Write at offset 3 to write the upper byte only
		addr_data[0] = temp;
		dw1000_writeSPI(SYS_CFG_ID, 3, addr_data, 1);
	}
}

void dw1000_setpreambledetecttimeout(u16 timeout){
	addr_data[1] = (u8)(timeout >> 8);
	addr_data[0] = (u8)(timeout);
	dw1000_writeSPI(DRX_CONF_ID, DRX_PRETOC_OFFSET, addr_data, 2);
}

void dw1000_setinterrupt(u32 bitmask, u8 operation){
	// Need to beware of interrupts occurring in the middle of following read modify write cycle
	decaIrqStatus_t stat = decamutexon();
	if(operation == 2){
		// New value
		addr_data[3] = (u8)(bitmask >> 24);
		addr_data[2] = (u8)(bitmask >> 16);
		addr_data[1] = (u8)(bitmask >> 8);
		addr_data[0] = (u8)(bitmask);
		dw1000_writeSPI(SYS_MASK_ID, 0, addr_data, 4);
	} else {
		// Read register
		dw1000_readSPI(SYS_MASK_ID, 0, addr_data, 4);
		u32 mask = addr_data[3] << 24 | addr_data[2] << 16 | addr_data[1] << 8 | addr_data[0];
		if(operation == 1) mask |= bitmask; // Set the bit
		else mask &= ~bitmask; // Clear the bit
		// New value
		addr_data[3] = (u8)(mask >> 24);
		addr_data[2] = (u8)(mask >> 16);
		addr_data[1] = (u8)(mask >> 8);
		addr_data[0] = (u8)(mask);
		dw1000_writeSPI(SYS_MASK_ID, 0, addr_data, 4);
	}
	decamutexoff(stat);
}

void dw1000_configeventcounters(u8 enable){
	// Need to clear and disable, can't just clear
	addr_data[0] = (u8)(EVC_CLR);
	dw1000_writeSPI(DIG_DIAG_ID, EVC_CTRL_OFFSET, addr_data, 1);
	if(enable){
		// Enable
		addr_data[0] = (u8)(EVC_EN);
		dw1000_writeSPI(DIG_DIAG_ID, EVC_CTRL_OFFSET, addr_data, 1);
	}
}

void dw1000_readeventcounters(dw1000_deviceentcnts_t *counters){
	u32 temp;
	
	// Read sync loss (31-16), PHE (15-0)
	dw1000_readSPI(DIG_DIAG_ID, EVC_PHE_OFFSET, addr_data, 4);
	temp = addr_data[3] << 24 | addr_data[2] << 16 | addr_data[1] << 8 | addr_data[0];
	counters->PHE = temp & 0xFFF;
	counters->RSL = (temp >> 16) & 0xFFF;
	
	// Read CRC bad (31-16), CRC good (15-0)
	dw1000_readSPI(DIG_DIAG_ID, EVC_FCG_OFFSET, addr_data, 4);
	temp = addr_data[3] << 24 | addr_data[2] << 16 | addr_data[1] << 8 | addr_data[0];
	counters->CRCG = temp & 0xFFF;
	counters->CRCB = (temp >> 16) & 0xFFF;
	
	// Overruns (31-16), address errors (15-0)
	dw1000_readSPI(DIG_DIAG_ID, EVC_FFR_OFFSET, addr_data ,4);
	temp = addr_data[3] << 24 | addr_data[2] << 16 | addr_data[1] << 8 | addr_data[0];
	counters->ARFE = temp & 0xFFF;
	counters->OVER = (temp >> 16) & 0xFFF;
	
	// Read PTO (31-16), SFDTO (15-0)
	dw1000_readSPI(DIG_DIAG_ID, EVC_STO_OFFSET, addr_data, 4);
	temp = addr_data[3] << 24 | addr_data[2] << 16 | addr_data[1] << 8 | addr_data[0];
	counters->PTO = (temp >> 16) & 0xFFF;
	counters->SFDTO = temp & 0xFFF;
	
	// Read RX TO (31-16), TXFRAME (15-0)
	dw1000_readSPI(DIG_DIAG_ID, EVC_FWTO_OFFSET, addr_data, 4);
	temp = addr_data[3] << 24 | addr_data[2] << 16 | addr_data[1] << 8 | addr_data[0];
	counters->TXF = (temp >> 16) & 0xFFF;
	counters->RTO = temp & 0xFFF;
	
	// Read half period warning events
	dw1000_readSPI(DIG_DIAG_ID, EVC_HPW_OFFSET, addr_data, 4);
	temp = addr_data[3] << 24 | addr_data[2] << 16 | addr_data[1] << 8 | addr_data[0];
	counters->HPW = temp & 0xFFF;
	
	// Power-up warning events
	counters->TXW = (temp >> 16) & 0xFFF;
}

void dw1000_rxreset(){
	// Set RX reset
	addr_data[0] = (u8)(PMSC_CTRL0_RESET_RX);
	dw1000_writeSPI(PMSC_ID, PMSC_CTRL0_SOFTRESET_OFFSET, addr_data, 1);
	
	// Clear RX reset
	addr_data[0] = (u8)(PMSC_CTRL0_RESET_CLEAR);
	dw1000_writeSPI(PMSC_ID, PMSC_CTRL0_SOFTRESET_OFFSET, addr_data, 1);
}

void dw1000_softreset(){
	set_spi_low();
	_dw1000_disablesequencing();

	// Clear any AON auto download bits (as reset will trigger AON download)
	addr_data[1] = (u8)(0 >> 8);
	addr_data[0] = (u8)(0);
	dw1000_writeSPI(AON_ID, AON_WCFG_OFFSET, addr_data, 2);

	// Clear the wake-up configuration
	addr_data[0] = (u8)(0);
	dw1000_writeSPI(AON_ID, AON_CFG0_OFFSET, addr_data, 1);

	// Upload the new configuration
	_dw1000_aonarrayupload();

	// Reset HIF, TX, RX and PMSC (set the reset bits)
	addr_data[0] = (u8)(PMSC_CTRL0_RESET_ALL);
	dw1000_writeSPI(PMSC_ID, PMSC_CTRL0_SOFTRESET_OFFSET, addr_data, 1);

	// local_dw needs a 10us sleep to let clk PLL lock after reset - the PLL will automatically lock after the reset
	// Could also have polled the PLL lock flag, but then the SPI needs to be < 3MHz !! So a simple delay is easier
	dw1000_delayms(1);

	// Clear the reset bits
	addr_data[0] = (u8)(PMSC_CTRL0_RESET_CLEAR);
	dw1000_writeSPI(PMSC_ID, PMSC_CTRL0_SOFTRESET_OFFSET, addr_data, 1);
	local_dw.wait4resp = 0;

	//set_spi_high();
}

void dw1000_setxtaltrim(u8 value){
	// The 3 MSb in this 8-bit register must be kept to 0b011 to avoid any malfunction.
	u8 reg_val = (3 << 5) | (value & FS_XTALT_MASK);
	#ifdef DEBUG
	sprintf(buf, "setxtaltrim: 0x%x\n\r", reg_val);
	UART_Send(buf, 19);
	#endif
	dw1000_writeSPI(FS_CTRL_ID, FS_XTALT_OFFSET, &reg_val, 1);
}

u8 dw1000_getxtaltrim(){
	dw1000_readSPI(FS_CTRL_ID, FS_XTALT_OFFSET, addr_data, 1);
	return (addr_data[0] & FS_XTALT_MASK);
}

bool dw1000_configcwmode(u8 chan){
	// error checking
	if(!((chan >= 1) && (chan <= 7) && (chan != 6))){
		#ifdef DEBUG
		sprintf(buf, "Error channel config\n\r");
		UART_Send(buf, 22);
		#endif
		return false;
	}
	//// Disable TX/RX RF block sequencing (needed for cw frame mode)
	//_dw1000_disablesequencing();

	// Config RF pll (for a given channel)
	// Configure PLL2/RF PLL block CFG/TUNE
	addr_data[3] = (u8)(fs_pll_cfg[chan_idx[chan]] >> 24);
	addr_data[2] = (u8)(fs_pll_cfg[chan_idx[chan]] >> 16);
	addr_data[1] = (u8)(fs_pll_cfg[chan_idx[chan]] >> 8);
	addr_data[0] = (u8)(fs_pll_cfg[chan_idx[chan]]);
	dw1000_writeSPI(FS_CTRL_ID, FS_PLLCFG_OFFSET, addr_data, 4);

	addr_data[3] = (u8)(fs_pll_tune[chan_idx[chan]] >> 24);
	addr_data[2] = (u8)(fs_pll_tune[chan_idx[chan]] >> 16);
	addr_data[1] = (u8)(fs_pll_tune[chan_idx[chan]] >> 8);
	addr_data[0] = (u8)(fs_pll_tune[chan_idx[chan]]);
	dw1000_writeSPI(FS_CTRL_ID, FS_PLLTUNE_OFFSET, addr_data, 4);
	// PLL won't be enabled until a TX/RX enable is issued later on

	// Configure RF TX blocks (for specified channel and prf)
	// Config RF TX control
	addr_data[3] = (u8)(tx_config[chan_idx[chan]] >> 24);
	addr_data[2] = (u8)(tx_config[chan_idx[chan]] >> 16);
	addr_data[1] = (u8)(tx_config[chan_idx[chan]] >> 8);
	addr_data[0] = (u8)(tx_config[chan_idx[chan]]);
	dw1000_writeSPI(RF_CONF_ID, RF_TXCTRL_OFFSET, addr_data, 4);

	//// Enable RF PLL
	//// Enable LDO and RF PLL blocks
	addr_data[3] = (u8)(RF_CONF_TXPLLPOWEN_MASK >> 24);
	addr_data[2] = (u8)(RF_CONF_TXPLLPOWEN_MASK >> 16);
	addr_data[1] = (u8)(RF_CONF_TXPLLPOWEN_MASK >> 8);
	addr_data[0] = (u8)(RF_CONF_TXPLLPOWEN_MASK);
	dw1000_writeSPI(RF_CONF_ID, 0, addr_data, 4);

	// Enable the rest of TX blocks
	addr_data[3] = (u8)(RF_CONF_TXALLEN_MASK >> 24);
	addr_data[2] = (u8)(RF_CONF_TXALLEN_MASK >> 16);
	addr_data[1] = (u8)(RF_CONF_TXALLEN_MASK >> 8);
	addr_data[0] = (u8)(RF_CONF_TXALLEN_MASK);
	dw1000_writeSPI(RF_CONF_ID, 0, addr_data, 4);

	//// Configure TX clocks
	addr_data[0] = (u8)(0x22);
	dw1000_writeSPI(PMSC_ID, PMSC_CTRL0_OFFSET, addr_data, 1);
	addr_data[0] = (u8)(0x07);
	dw1000_writeSPI(PMSC_ID, 0x1, addr_data, 1);

	// Disable fine grain TX sequencing
	dw1000_setfinegraintxseq(0);

	// Configure CW mode
	addr_data[0] = (u8)(TC_PGTEST_CW);
	dw1000_writeSPI(TX_CAL_ID, TC_PGTEST_OFFSET, addr_data, 1);
	return true;
}

void dw1000_configcontinuousframemode(u32 framerepetitionrate){
	//// Disable TX/RX RF block sequencing (needed for continuous frame mode)
	//_dw1000_disablesequencing();

	//// Enable RF PLL and TX blocks
	//// Enable LDO and RF PLL blocks
	addr_data[3] = (u8)(RF_CONF_TXPLLPOWEN_MASK >> 24);
	addr_data[2] = (u8)(RF_CONF_TXPLLPOWEN_MASK >> 16);
	addr_data[1] = (u8)(RF_CONF_TXPLLPOWEN_MASK >> 8);
	addr_data[0] = (u8)(RF_CONF_TXPLLPOWEN_MASK);
	dw1000_writeSPI(RF_CONF_ID, 0, addr_data, 4);

	// Enable the rest of TX blocks
	addr_data[3] = (u8)(RF_CONF_TXALLEN_MASK >> 24);
	addr_data[2] = (u8)(RF_CONF_TXALLEN_MASK >> 16);
	addr_data[1] = (u8)(RF_CONF_TXALLEN_MASK >> 8);
	addr_data[0] = (u8)(RF_CONF_TXALLEN_MASK);
	dw1000_writeSPI(RF_CONF_ID, 0, addr_data, 4);

	//// Configure TX clocks
	//_dw1000_enableclock(FORCE_SYS_PLL);
	_dw1000_enableclock(FORCE_TX_PLL);

	// Set the frame repetition rate
	if(framerepetitionrate < 4) framerepetitionrate = 4;
	addr_data[3] = (u8)(framerepetitionrate >> 24);
	addr_data[2] = (u8)(framerepetitionrate >> 16);
	addr_data[1] = (u8)(framerepetitionrate >> 8);
	addr_data[0] = (u8)(framerepetitionrate);
	dw1000_writeSPI(DX_TIME_ID, 0, addr_data, 4);

	//// Configure continuous frame TX
	//// Turn the TX power spectrum test mode - continuous sending of frames
	addr_data[0] = (u8)(DIAG_TMC_TX_PSTM);
	dw1000_writeSPI(DIG_DIAG_ID, DIAG_TMC_OFFSET, addr_data, 1);
}

u16 dw1000_readtempvbat(u8 fastSPI){
	u8 vbat_raw;
	u8 temp_raw;
	
	// These writes should be single writes and in sequence
	addr_data[0] = 0x80; // Enable TLD Bias
	dw1000_writeSPI(RF_CONF_ID, 0x11, addr_data, 1);
	
	// Enable TLD Bias and ADC Bias
	addr_data[0] = 0x0A;
	dw1000_writeSPI(RF_CONF_ID, 0x12, addr_data, 1);
	
	// Enable Outputs (only after Biases are up and running)
	addr_data[0] = 0x0f;
	dw1000_writeSPI(RF_CONF_ID, 0x12, addr_data, 1);	//
	
	if(fastSPI == 1){
		// Reading All SAR inputs
		addr_data[0] = 0x00;
		dw1000_writeSPI(TX_CAL_ID, TC_SARL_SAR_C, addr_data, 1);
		
		// Set SAR enable
		addr_data[0] = 0x01;
		dw1000_writeSPI(TX_CAL_ID, TC_SARL_SAR_C, addr_data, 1);
		
		// If using PLL clocks(and fast SPI rate) then this sleep is needed
		dw1000_delayms(1); 
		// Read voltage and temperature.
		dw1000_readSPI(TX_CAL_ID, TC_SARL_SAR_LVBAT_OFFSET, addr_data, 2);
	}
	else //change to a slow clock
	{
		_dw1000_enableclock(FORCE_SYS_XTI); // NOTE: set system clock to XTI - this is necessary to make sure the values read are reliable
		// Reading All SAR inputs
		addr_data[0] = 0x00;
		dw1000_writeSPI(TX_CAL_ID, TC_SARL_SAR_C, addr_data, 1);
		addr_data[0] = 0x01; // Set SAR enable
		dw1000_writeSPI(TX_CAL_ID, TC_SARL_SAR_C, addr_data, 1);

		// Read voltage and temperature.
		dw1000_readSPI(TX_CAL_ID, TC_SARL_SAR_LVBAT_OFFSET, addr_data, 2);
		// Default clocks (ENABLE_ALL_SEQ)
		_dw1000_enableclock(ENABLE_ALL_SEQ); // Enable clocks for sequencing
	}

	vbat_raw = addr_data[0];
	temp_raw = addr_data[1];

	addr_data[0] = 0x00; // Clear SAR enable
	dw1000_writeSPI(TX_CAL_ID, TC_SARL_SAR_C, addr_data, 1);

	return (((u16)temp_raw<<8)|(vbat_raw));
}

float dw1000_convertrawtemperature(u8 raw_temp){
	// error checking
	if(!(local_dw.otp_mask & DW1000_READ_OTP_TMP)){
		#ifdef DEBUG
		sprintf(buf, "Error Reading temp\n\r");
		UART_Send(buf, 20);
		#endif
		return false;
	}

	// the User Manual formula is: Temperature (°C) = ( (SAR_LTEMP - OTP_READ(Vtemp @ 23°C) ) x 1.14) + 23
	float realtemp = ((raw_temp - local_dw.tempP) * SAR_TEMP_TO_CELCIUS_CONV) + 23 ;

	return realtemp;
}

u8 dw1000_convertdegtemptoraw(short externaltemp){
	// error checking
	if(!(local_dw.otp_mask & DW1000_READ_OTP_TMP)){
		#ifdef DEBUG
		sprintf(buf, "Error Reading temp\n\r");
		UART_Send(buf, 20);
		#endif
		return false;
	}
	if(!((externaltemp > -800) && (externaltemp < 1500))){
		#ifdef DEBUG
		sprintf(buf, "Error externaltemp\n\r");
		UART_Send(buf, 20);
		#endif
		return false;
	}
	
	// the User Manual formula is: Temperature (°C) = ( (SAR_LTEMP - OTP_READ(Vtemp @ 23°C) ) x 1.14) + 23
	int raw_temp = ((externaltemp - 230 + 5) * DCELCIUS_TO_SAR_TEMP_CONV) ; //+5 for better rounding

	if(raw_temp < 0) //negative
	{
		raw_temp = (-raw_temp >> 8)  ;
		raw_temp = -raw_temp ;
	}
	else
		raw_temp = raw_temp >> 8  ;

	return (u8)(raw_temp + local_dw.tempP);
}

float dw1000_convertrawvoltage(u8 raw_voltage){
	// error checking
	if(!(local_dw.otp_mask & DW1000_READ_OTP_BAT)){
		#ifdef DEBUG
		sprintf(buf, "Error Reading bat\n\r");
		UART_Send(buf, 19);
		#endif
		return false;
	}
	
	// the User Manual formula is: Voltage (V) = ( (SAR_LVBAT ?OTP_READ(Vmeas @ 3.3 V) ) / 173 ) + 3.3
	float realvolt = ((float)(raw_voltage - local_dw.vBatP) * SAR_VBAT_TO_VOLT_CONV) + 3.3 ;
	
	return realvolt;
}

u8 dw1000_convertvoltstoraw(int externalmvolt){
	// error checking
	if(!(local_dw.otp_mask & DW1000_READ_OTP_BAT)){
		#ifdef DEBUG
		sprintf(buf, "Error Reading bat\n\r");
		UART_Send(buf, 19);
		#endif
		return false;
	}
	// the User Manual formula is: Voltage (V) = ( (SAR_LVBAT ?OTP_READ(Vmeas @ 3.3 V) ) / 173 ) + 3.3
	u32 raw_voltage = ((externalmvolt - 3300) * MVOLT_TO_SAR_VBAT_CONV) + local_dw.vBatP ;

	return (u8)(raw_voltage);
}

u8 dw1000_readwakeuptemp(){
	dw1000_readSPI(TX_CAL_ID, TC_SARL_SAR_LTEMP_OFFSET, addr_data, 1);
	return addr_data[0];
}

u8 dw1000_readwakeupvbat(){
	dw1000_readSPI(TX_CAL_ID, TC_SARL_SAR_LVBAT_OFFSET, addr_data, 1);
	return addr_data[0];
}

u8 dw1000_calcbandwidthtempadj(u16 target_count){
	u8 bit_field, curr_bw;
	u8 best_bw = 0;
	u16 delta_lowest;
	// Used to store the current values of the registers so that they can be restored after
	u8 old_pmsc_ctrl0;
	u16 old_pmsc_ctrl1;
	u32 old_rf_conf_txpow_mask;

	// Record the current values of these registers, to restore later
	dw1000_readSPI(PMSC_ID, PMSC_CTRL0_OFFSET, addr_data, 1);
	old_pmsc_ctrl0 = addr_data[0];
	dw1000_readSPI(PMSC_ID, PMSC_CTRL1_OFFSET, addr_data, 2);
	old_pmsc_ctrl1 = addr_data[1] << 8 | addr_data[0];
	dw1000_readSPI(RF_CONF_ID, 0, addr_data, 4);
	old_rf_conf_txpow_mask = addr_data[3] << 24 | addr_data[2] << 16 | addr_data[1] << 8 | addr_data[0];

	//  Set clock to XTAL
	addr_data[0] = (u8)(PMSC_CTRL0_SYSCLKS_19M);
	dw1000_writeSPI(PMSC_ID, PMSC_CTRL0_OFFSET, addr_data, 1);

	//  Disable sequencing
	addr_data[1] = (u8)(PMSC_CTRL1_PKTSEQ_DISABLE >> 8);
	addr_data[0] = (u8)(PMSC_CTRL1_PKTSEQ_DISABLE);
	dw1000_writeSPI(PMSC_ID, PMSC_CTRL1_OFFSET, addr_data, 2);

	//  Turn on CLK PLL, Mix Bias and PG
	addr_data[3] = (u8)((RF_CONF_TXPOW_MASK | RF_CONF_PGMIXBIASEN_MASK) >> 24);
	addr_data[2] = (u8)((RF_CONF_TXPOW_MASK | RF_CONF_PGMIXBIASEN_MASK) >> 16);
	addr_data[1] = (u8)((RF_CONF_TXPOW_MASK | RF_CONF_PGMIXBIASEN_MASK) >> 8);
	addr_data[0] = (u8)((RF_CONF_TXPOW_MASK | RF_CONF_PGMIXBIASEN_MASK));
	dw1000_writeSPI(RF_CONF_ID, 0, addr_data, 4);

	//  Set sys and TX clock to PLL
	addr_data[0] = (u8)(PMSC_CTRL0_SYSCLKS_125M | PMSC_CTRL0_TXCLKS_125M);
	dw1000_writeSPI(PMSC_ID, PMSC_CTRL0_OFFSET, addr_data, 1);

	// Set the MSB high for first guess
	curr_bw = 0x80;

	// Set starting bit
	bit_field = 0x80;

	// Initial lowest delta is the maximum difference that we should allow the count value to be from the target.
	// If the algorithm is successful, it will be overwritten by a smaller value where the count value is closer
	// to the target
	delta_lowest = 300;

	for (int i = 0; i < 7; i++){
		// start with 0xc0 and test.
		bit_field = bit_field >> 1;
		addr_data[0] = (u8)(curr_bw | bit_field); // Changed to index 0
		// Write bw setting to PG_DELAY register
		dw1000_writeSPI(TX_CAL_ID, TC_PGDELAY_OFFSET, addr_data, 1);
		
		// Set cal direction and time
		addr_data[0] = (u8)(TC_PGCCTRL_DIR_CONV | TC_PGCCTRL_TMEAS_MASK); // Changed to index 0
		dw1000_writeSPI(TX_CAL_ID, TC_PGCCTRL_OFFSET, addr_data, 1);
		
		// Start cal
		addr_data[0] = (u8)(TC_PGCCTRL_DIR_CONV | TC_PGCCTRL_TMEAS_MASK | TC_PGCCTRL_CALSTART); // Changed to index 0
		dw1000_writeSPI(TX_CAL_ID, TC_PGCCTRL_OFFSET, addr_data , 1);
		
		// Allow cal to complete
		dw1000_delayms(1);
		
		// Read count value from the PG cal block
		dw1000_readSPI(TX_CAL_ID, TC_PGCAL_STATUS_OFFSET, addr_data , 2);
		u16 raw_count = (addr_data[1] << 8 | addr_data[0]) & TC_PGCAL_STATUS_DELAY_MASK; // Changed to index 1, 0
		
		// Let's keep track of the closest value to the target in case we overshoot
		u16 delta_count = abs((short)raw_count - (short)target_count);
		if (delta_count < delta_lowest){
				delta_lowest = delta_count;
				best_bw = curr_bw;
		}

		// Test the count results
		if (raw_count > target_count) // Count was lower, BW was lower so increase PG DELAY
			curr_bw = curr_bw | bit_field;
		else // Count was higher
			curr_bw = curr_bw & (~(bit_field));
	}

	// Restore old register values
	addr_data[0] = (u8)(old_pmsc_ctrl0); // Changed to index 0
	dw1000_writeSPI(PMSC_ID, PMSC_CTRL0_OFFSET, addr_data, 1);

	addr_data[1] = (u8)(old_pmsc_ctrl1 >> 8);
	addr_data[0] = (u8)(old_pmsc_ctrl1);
	dw1000_writeSPI(PMSC_ID, PMSC_CTRL1_OFFSET, addr_data, 2);
	
	addr_data[3] = (u8)(old_rf_conf_txpow_mask >> 24);
	addr_data[2] = (u8)(old_rf_conf_txpow_mask >> 16);
	addr_data[1] = (u8)(old_rf_conf_txpow_mask >> 8);
	addr_data[0] = (u8)(old_rf_conf_txpow_mask);
	dw1000_writeSPI(RF_CONF_ID, 0, addr_data, 4);

	// Returns the best PG_DELAY setting
	return best_bw;
}

u32 _dw1000_computetxpowersetting(u32 ref_powerreg, int power_adj){
	u32 new_regval = 0;
	for(int i = 0; i < 4; i++){
		char da_attn_change = 0;
		char mixer_gain_change = power_adj;
		u8 current_da_attn = ((ref_powerreg >> (i*8)) & 0xE0) >> 5;
		u8 current_mixer_gain = (ref_powerreg >> (i*8)) & 0x1F;
		
		
		
		// Mixer gain gives best performance between gain value of 4 and 20
		while((current_mixer_gain + mixer_gain_change < 4) ||
			  (current_mixer_gain + mixer_gain_change > 20)){
			
			// If mixer gain goes outside bounds, adjust the DA attenuation to compensate
			if(current_mixer_gain + mixer_gain_change > 20){
				da_attn_change -= 1;
				
				//DA attenuation has reached the max value
				if(da_attn_change == 0) {
					//restore the value and exit the loop - DA is at max allowed
					da_attn_change = 1;
					break;
				}
				mixer_gain_change -= (char) (MIX_DA_FACTOR);
			}else if(current_mixer_gain + mixer_gain_change < 4){
				da_attn_change += 1;
				
				//DA attenuation has reached the min value
				if(da_attn_change == 0x8){
					//restore the value and exit the loop - DA is at min allowed
					da_attn_change = 7;
					break;
				}
				
				mixer_gain_change += (char) (MIX_DA_FACTOR);
			}
		}
		
		u8 new_da_attn = (current_da_attn + da_attn_change) & 0x7;
		u8 new_mixer_gain = (current_mixer_gain + mixer_gain_change) & 0x1F;
		
		new_regval |= ((u32) ((new_da_attn << 5) | new_mixer_gain)) << (i * 8);
	}
	
	return (u32)new_regval;
}

u32 dw1000_calcpowertempadj(u8 channel, u32 ref_powerreg, int delta_temp){
	char delta_power;
	int negative = 0;
	
	if(delta_temp < 0)
	{
		negative = 1;
		delta_temp = -delta_temp; //make (-)ve into (+)ve number
	}
	
	// Calculate the expected power differential at the current temperature
	if(channel == 5) delta_power = ((delta_temp * TEMP_COMP_FACTOR_CH5) >> 12); //>>12 is same as /4096
	else if(channel == 2) delta_power = ((delta_temp * TEMP_COMP_FACTOR_CH2) >> 12); //>>12 is same as /4096
	else delta_power = 0;
	
	
	if(negative == 1) delta_power = -delta_power; //restore the sign
	
	if(delta_power == 0) return ref_powerreg ; //no change to power register
	
	// Adjust the TX_POWER register value
	return _dw1000_computetxpowersetting(ref_powerreg, delta_power);
}

u16 dw1000_calcpgcount(u8 pgdly){
	// Perform PG count read ten times and take an average to smooth out any noise
	const int NUM_SAMPLES = 10;
	u32 sum_count = 0;
	u16 average_count = 0, count = 0;
	int i = 0;
	
	// Used to store the current values of the registers so that they can be restored after
	u8 old_pmsc_ctrl0;
	u16 old_pmsc_ctrl1;
	u32 old_rf_conf_txpow_mask;
	
	// Record the current values of these registers, to restore later
	dw1000_readSPI(PMSC_ID, PMSC_CTRL0_OFFSET, addr_data, 1);
	old_pmsc_ctrl0 = addr_data[0];
	
	dw1000_readSPI(PMSC_ID, PMSC_CTRL1_OFFSET, addr_data, 2);
	old_pmsc_ctrl1 = addr_data[1] << 8 | addr_data[0];
	
	dw1000_readSPI(RF_CONF_ID, 0, addr_data, 4);
	old_rf_conf_txpow_mask = addr_data[3] << 24 | addr_data[2] << 16 | addr_data[1] << 8 | addr_data[0];
	
	//  Set clock to XTAL
	addr_data[0] = (u8)(PMSC_CTRL0_SYSCLKS_19M);
	dw1000_writeSPI(PMSC_ID, PMSC_CTRL0_OFFSET, addr_data, 1);
	
	//  Disable sequencing
	addr_data[1] = (u8)(PMSC_CTRL1_PKTSEQ_DISABLE >> 8);
	addr_data[0] = (u8)(PMSC_CTRL1_PKTSEQ_DISABLE);
	dw1000_writeSPI(PMSC_ID, PMSC_CTRL1_OFFSET, addr_data, 2);
	
	//  Turn on CLK PLL, Mix Bias and PG
	addr_data[3] = (u8)((RF_CONF_TXPOW_MASK | RF_CONF_PGMIXBIASEN_MASK) >> 24);
	addr_data[2] = (u8)((RF_CONF_TXPOW_MASK | RF_CONF_PGMIXBIASEN_MASK) >> 16);
	addr_data[1] = (u8)((RF_CONF_TXPOW_MASK | RF_CONF_PGMIXBIASEN_MASK) >> 8);
	addr_data[0] = (u8)((RF_CONF_TXPOW_MASK | RF_CONF_PGMIXBIASEN_MASK));
	dw1000_writeSPI(RF_CONF_ID, 0, addr_data, 4);
	
	//  Set sys and TX clock to PLL
	addr_data[0] = (u8)(PMSC_CTRL0_SYSCLKS_125M | PMSC_CTRL0_TXCLKS_125M);
	dw1000_writeSPI(PMSC_ID, PMSC_CTRL0_OFFSET, addr_data, 1);
	
	for(i = 0; i < NUM_SAMPLES; i++) {
		// Write bw setting to PG_DELAY register
		addr_data[0] = (u8)(pgdly);
		dw1000_writeSPI(TX_CAL_ID, TC_PGDELAY_OFFSET, addr_data, 1);
		
		// Set cal direction and time
		addr_data[0] = (u8)(TC_PGCCTRL_DIR_CONV | TC_PGCCTRL_TMEAS_MASK);
		dw1000_writeSPI(TX_CAL_ID, TC_PGCCTRL_OFFSET, addr_data, 1);
		
		// Start cal
		addr_data[0] = (u8)(TC_PGCCTRL_DIR_CONV | TC_PGCCTRL_TMEAS_MASK | TC_PGCCTRL_CALSTART);
		dw1000_writeSPI(TX_CAL_ID, TC_PGCCTRL_OFFSET, addr_data, 1);
		
		// Allow cal to complete - the TC_PGCCTRL_CALSTART bit will clear automatically
		dw1000_delayms(1);
		
		// Read count value from the PG cal block
		dw1000_readSPI(TX_CAL_ID, TC_PGCAL_STATUS_OFFSET, addr_data, 2);
		count = (addr_data[1] << 8 | addr_data[0]) & TC_PGCAL_STATUS_DELAY_MASK;
		
		sum_count += count;
	}
	
	// Restore old register values
	addr_data[0] = (u8)(old_pmsc_ctrl0);
	dw1000_writeSPI(PMSC_ID, PMSC_CTRL0_OFFSET, addr_data, 1);
	
	addr_data[1] = (u8)(old_pmsc_ctrl1 >> 8);
	addr_data[0] = (u8)(old_pmsc_ctrl1);
	dw1000_writeSPI(PMSC_ID, PMSC_CTRL1_OFFSET, addr_data, 2);
	
	addr_data[3] = (u8)(old_rf_conf_txpow_mask >> 24);
	addr_data[2] = (u8)(old_rf_conf_txpow_mask >> 16);
	addr_data[1] = (u8)(old_rf_conf_txpow_mask >> 8);
	addr_data[0] = (u8)(old_rf_conf_txpow_mask);
	dw1000_writeSPI(RF_CONF_ID, 0, addr_data, 4);

	average_count = (int)(sum_count / NUM_SAMPLES);
	return average_count;
}


/********************************************User Define Function**********************************************/
/*
void set_delayclk(vu16 time_us){
	TIM_Cmd(TIM3, ENABLE);
	TIM3->CNT = time_us - 1;
	while(TIM3->CNT);
	TIM_Cmd(TIM3, DISABLE);
}
*/

void dw1000_delayus(vu16 time){
	set_delayclk_us(time);
}

void dw1000_delayms(vu16 time){
	set_delayclk_ms(time);
}

decaIrqStatus_t decamutexon(){
	decaIrqStatus_t s;
	#if defined(STM32F10X_LD) || defined(STM32F10X_LD_VL) || defined(STM32F10X_MD) ||\
		defined(STM32F10X_MD_VL) || defined(STM32F10X_HD) || defined(STM32F10X_HD_VL) ||\
		defined(STM32F10X_XL) || defined(STM32F10X_CL)
	
	s = ((NVIC->ISER[(((uint32_t)TIM3_IRQn) >> 5UL)] &\
	(uint32_t)(1UL << (((uint32_t)TIM3_IRQn) & 0x1FUL)) ) == (uint32_t)RESET)?(RESET):(SET);

	if(s) {
		NVIC_DisableIRQ(TIM3_IRQn); //disable the external interrupt line
	}
	#endif
	
	#if defined(STM32G070xx)
	
	#endif
	return s;
}

void decamutexoff(decaIrqStatus_t s){
	if(s) { //need to check the port state as we can't use level sensitive interrupt on the STM ARM
		#if defined(STM32F10X_LD) || defined(STM32F10X_LD_VL) || defined(STM32F10X_MD) ||\
			defined(STM32F10X_MD_VL) || defined(STM32F10X_HD) || defined(STM32F10X_HD_VL) ||\
			defined(STM32F10X_XL) || defined(STM32F10X_CL)
		NVIC_EnableIRQ(TIM3_IRQn);
		#endif
		
		#if defined(STM32G070xx)
	
		#endif
	}
}

bool dw1000_finishtx_clearflag(void){
	dw1000_readSPI(SYS_STATUS_ID, 0, addr_data, 4);
	u32 regval = (addr_data[3] << 24 | addr_data[2] << 16 | addr_data[1] << 8 | addr_data[0]);
	if(regval & SYS_STATUS_TXFRS){
		
		addr_data[3] = (u8)(SYS_STATUS_RXFCG | SYS_STATUS_TXFRS >> 24);
		addr_data[2] = (u8)(SYS_STATUS_RXFCG | SYS_STATUS_TXFRS >> 16);
		addr_data[1] = (u8)(SYS_STATUS_RXFCG | SYS_STATUS_TXFRS >> 8);
		addr_data[0] = (u8)(SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);
		dw1000_writeSPI(SYS_STATUS_ID, 0, addr_data, 4);
		return true;
	}
	return false;
}

u8 dw1000_gettxchannel(dw1000_config_t* config){
	switch(config->chan){
		case 1:
			return 0;
		case 2:
			return 1;
		case 3:
			return 2;
		case 4:
			return 3;
		case 5:
			return 4;
		case 7:
			return 5;
		default:
			return 0xff;
	}
}

u8 dw1000_gettxfreq(dw1000_config_t* config){
	return config->prf & DW1000_PRF_64M;
}

u8 dw1000_getsmarttxpower(void){
	dw1000_readSPI(SYS_CFG_ID, 0, addr_data, 4);
	temp_32bit = addr_data[3] << 24 | addr_data[2] << 16 | addr_data[1] << 8 | addr_data[0];
	return !(temp_32bit & SYS_CFG_DIS_STXP);
}

u32 dw1000_rxreadyerror(){
	u32 regval;
	do{
		dw1000_readSPI(SYS_STATUS_ID, 0, addr_data, 4);
		regval = addr_data[3] << 24 | addr_data[2] << 16 | addr_data[1] << 8 | addr_data[0];
	}while(!(regval & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_TO)));
	
	if(regval & SYS_STATUS_RXFCG){
		addr_data[3] = (u8)(SYS_STATUS_RXFCG | SYS_STATUS_TXFRS >> 24);
		addr_data[2] = (u8)(SYS_STATUS_RXFCG | SYS_STATUS_TXFRS >> 16);
		addr_data[1] = (u8)(SYS_STATUS_RXFCG | SYS_STATUS_TXFRS >> 8);
		addr_data[0] = (u8)(SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);
		dw1000_writeSPI(SYS_STATUS_ID, 0, addr_data, 4);
		#ifdef DEBUG
		sprintf(buf, "Frame received\n\r");
		UART_Send(buf, 16);
		#endif
		
			
	}else if(regval & SYS_STATUS_ALL_RX_ERR){
		#ifdef DEBUG
		if(regval & SYS_STATUS_LDEERR) sprintf(buf, "Leading edge detection processing error\n\r");
		if(regval & SYS_STATUS_AFFREJ) sprintf(buf, "Automatic Frame Filtering rejection\n\r");
		if(regval & SYS_STATUS_RXSFDTO) sprintf(buf, "Receive SFD timeout\n\r");
		if(regval & SYS_STATUS_RXRFSL) sprintf(buf, "Receiver Reed Solomon Frame Sync Loss\n\r");
		if(regval & SYS_STATUS_RXFCE) sprintf(buf, "Receiver FCS Error\n\r");
		if(regval & SYS_STATUS_RXPHE) sprintf(buf, "Receiver PHY Header Error\n\r");
		UART_Send(buf, 40);
		#endif
		addr_data[3] = (u8)(SYS_STATUS_ALL_RX_ERR >> 24);
		addr_data[2] = (u8)(SYS_STATUS_ALL_RX_ERR >> 16);
		addr_data[1] = (u8)(SYS_STATUS_ALL_RX_ERR >> 8);
		addr_data[0] = (u8)(SYS_STATUS_ALL_RX_ERR);
		dw1000_writeSPI(SYS_STATUS_ID, 0, addr_data, 4);
	}else{
		#ifdef DEBUG
		if(regval & SYS_STATUS_RXPTO) sprintf(buf, "Preamble detection timeout\n\r");
		if(regval & SYS_STATUS_RXRFTO) sprintf(buf, "Receive Frame Wait Timeout\n\r");
		UART_Send(buf, 40);
		#endif
		addr_data[3] = (u8)(SYS_STATUS_ALL_RX_TO >> 24);
		addr_data[2] = (u8)(SYS_STATUS_ALL_RX_TO >> 16);
		addr_data[1] = (u8)(SYS_STATUS_ALL_RX_TO >> 8);
		addr_data[0] = (u8)(SYS_STATUS_ALL_RX_TO);
		dw1000_writeSPI(SYS_STATUS_ID, 0, addr_data, 4);
	}
	
	
	return regval;
}

float dw1000_getFreqOffsetMul(dw1000_config_t* config){
	switch(config->dataRate){
		case DW1000_BR_110K:
			return FREQ_OFFSET_MULTIPLIER_110KB;
			break;
		case DW1000_BR_850K:
			return FREQ_OFFSET_MULTIPLIER;
			break;
		case DW1000_BR_6M8:
			return FREQ_OFFSET_MULTIPLIER;
			break;
		default:
			#pragma "Error Getting Freq_Offset_Multiplier"
			return 0;
			break;
	}
}

float dw1000_getHertzToPPM(dw1000_config_t* config){
	switch(config->chan){
		case 1:
			return HERTZ_TO_PPM_MULTIPLIER_CHAN_1;
			break;
		case 2:
			return HERTZ_TO_PPM_MULTIPLIER_CHAN_2;
			break;
		case 3:
			return HERTZ_TO_PPM_MULTIPLIER_CHAN_3;
			break;
		case 5:
			return HERTZ_TO_PPM_MULTIPLIER_CHAN_5;
			break;
		default:
			#pragma "Error Getting Hertz_To_PPM_Multiplier"
			return 0;
			break;
	}
}
