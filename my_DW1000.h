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
#pragma once
#if defined(STM32F10X_MD) || defined(STM32F10X_CL)
#include <stm32f10x.h>
#endif

#include "my_DW1000_func.h"

#ifdef __cplusplus
extern "C"{
#endif
	
	#ifndef DW1000_NUM_DW_DEV
	#define DW1000_NUM_DW_DEV (1)
	#endif

	#define DW1000_SUCCESS (0)
	#define DW1000_ERROR   (-1)

	#define DW1000_TIME_UNITS          (1.0/499.2e6/128.0) //!< = 15.65e-12 s
	
	#define UUS_TO_DW1000_TIME 65536

	#define DW1000_DEVICE_ID   (0xDECA0130)        //!< DW1000 MP device ID

	//! constants for selecting the bit rate for data TX (and RX)
	//! These are defined for write (with just a shift) the TX_FCTRL register
	#define DW1000_BR_110K     0   //!< UWB bit rate 110 kbits/s
	#define DW1000_BR_850K     1   //!< UWB bit rate 850 kbits/s
	#define DW1000_BR_6M8      2   //!< UWB bit rate 6.8 Mbits/s

	//! constants for specifying the (Nominal) mean Pulse Repetition Frequency
	//! These are defined for direct write (with a shift if necessary) to CHAN_CTRL and TX_FCTRL regs
	#define DW1000_PRF_16M     1   //!< UWB PRF 16 MHz
	#define DW1000_PRF_64M     2   //!< UWB PRF 64 MHz

	//! constants for specifying Preamble Acquisition Chunk (PAC) Size in symbols
	#define DW1000_PAC8        0   //!< PAC  8 (recommended for RX of preamble length  128 and below
	#define DW1000_PAC16       1   //!< PAC 16 (recommended for RX of preamble length  256
	#define DW1000_PAC32       2   //!< PAC 32 (recommended for RX of preamble length  512
	#define DW1000_PAC64       3   //!< PAC 64 (recommended for RX of preamble length 1024 and up

	//! constants for specifying TX Preamble length in symbols
	//! These are defined to allow them be directly written into byte 2 of the TX_FCTRL register
	//! (i.e. a four bit value destined for bits 20..18 but shifted left by 2 for byte alignment)
	#define DW1000_PLEN_4096   0x0C    //! Standard preamble length 4096 symbols
	#define DW1000_PLEN_2048   0x28    //! Non-standard preamble length 2048 symbols
	#define DW1000_PLEN_1536   0x18    //! Non-standard preamble length 1536 symbols
	#define DW1000_PLEN_1024   0x08    //! Standard preamble length 1024 symbols
	#define DW1000_PLEN_512    0x34    //! Non-standard preamble length 512 symbols
	#define DW1000_PLEN_256    0x24    //! Non-standard preamble length 256 symbols
	#define DW1000_PLEN_128    0x14    //! Non-standard preamble length 128 symbols
	#define DW1000_PLEN_64     0x04    //! Standard preamble length 64 symbols

	#define DW1000_SFDTOC_DEF              0x1041  // default SFD timeout value

	#define DW1000_PHRMODE_STD             0x0     // standard PHR mode
	#define DW1000_PHRMODE_EXT             0x3     // DW proprietary extended frames PHR mode

	// Defined constants for "mode" bitmask parameter passed into dw1000_starttx() function.
	#define DW1000_START_TX_IMMEDIATE      0
	#define DW1000_START_TX_DELAYED        1
	#define DW1000_RESPONSE_EXPECTED       2

	#define DW1000_START_RX_IMMEDIATE  0
	#define DW1000_START_RX_DELAYED    1    // Set up delayed RX, if "late" error triggers, then the RX will be enabled immediately
	#define DW1000_IDLE_ON_DLY_ERR     2    // If delayed RX failed due to "late" error then if this
																			 // flag is set the RX will not be re-enabled immediately, and device will be in IDLE when function exits
	#define DW1000_NO_SYNC_PTRS        4    // Do not try to sync IC side and Host side buffer pointers when enabling RX. This is used to perform manual RX
																			 // re-enabling when receiving a frame in double buffer mode.

	// Defined constants for "mode" bit field parameter passed to dw1000_setleds() function.
	#define DW1000_LEDS_DISABLE     0x00
	#define DW1000_LEDS_ENABLE      0x01
	#define DW1000_LEDS_INIT_BLINK  0x02

	// Defined constants for "lna_pa" bit field parameter passed to dw1000_setlnapamode() function
	#define DW1000_LNA_PA_DISABLE     0x00
	#define DW1000_LNA_ENABLE         0x01
	#define DW1000_PA_ENABLE          0x02

	//frame filtering configuration options
	#define DW1000_FF_NOTYPE_EN            0x000           // no frame types allowed (FF disabled)
	#define DW1000_FF_COORD_EN             0x002           // behave as coordinator (can receive frames with no dest address (PAN ID has to match))
	#define DW1000_FF_BEACON_EN            0x004           // beacon frames allowed
	#define DW1000_FF_DATA_EN              0x008           // data frames allowed
	#define DW1000_FF_ACK_EN               0x010           // ack frames allowed
	#define DW1000_FF_MAC_EN               0x020           // mac control frames allowed
	#define DW1000_FF_RSVD_EN              0x040           // reserved frame types allowed

	//DW1000 interrupt events
	#define DW1000_INT_TFRS            0x00000080          // frame sent
	#define DW1000_INT_LDED            0x00000400          // micro-code has finished execution
	#define DW1000_INT_RFCG            0x00004000          // frame received with good CRC
	#define DW1000_INT_RPHE            0x00001000          // receiver PHY header error
	#define DW1000_INT_RFCE            0x00008000          // receiver CRC error
	#define DW1000_INT_RFSL            0x00010000          // receiver sync loss error
	#define DW1000_INT_RFTO            0x00020000          // frame wait timeout
	#define DW1000_INT_RXOVRR          0x00100000          // receiver overrun
	#define DW1000_INT_RXPTO           0x00200000          // preamble detect timeout
	#define DW1000_INT_GPIO            0x00400000          // GPIO interrupt
	#define DW1000_INT_SFDT            0x04000000          // SFD timeout
	#define DW1000_INT_ARFE            0x20000000          // frame rejected (due to frame filtering configuration)


	//DW1000 SLEEP and WAKEUP configuration parameters
	#define DW1000_PRESRV_SLEEP 0x0100                      // PRES_SLEEP - on wakeup preserve sleep bit
	#define DW1000_LOADOPSET    0x0080                      // ONW_L64P - on wakeup load operating parameter set for 64 PSR
	#define DW1000_CONFIG       0x0040                      // ONW_LDC - on wakeup restore (load) the saved configurations (from AON array into HIF)
	#define DW1000_LOADEUI      0x0008                      // ONW_LEUI - on wakeup load EUI
	#define DW1000_RX_EN        0x0002                      // ONW_RX - on wakeup activate reception
	#define DW1000_TANDV        0x0001                      // ONW_RADC - on wakeup run ADC to sample temperature and voltage sensor values

	#define DW1000_XTAL_EN      0x10                       // keep XTAL running during sleep
	#define DW1000_WAKE_SLPCNT  0x8                        // wake up after sleep count
	#define DW1000_WAKE_CS      0x4                        // wake up on chip select
	#define DW1000_WAKE_WK      0x2                        // wake up on WAKEUP PIN
	#define DW1000_SLP_EN       0x1                        // enable sleep/deep sleep functionality

	//DW1000_DW_POWER_ON should be used when dw1000_initialise is called on cold power up of DW IC
	//When DW IC is being initialised after wake up then some of the init steps can be skipped
	//DW1000 INIT configuration parameters
	#define DW1000_LOADNONE         0x00    // no loading of micro-code or reading of OTP values
	#define DW1000_LOADUCODE        0x01    // this can be called on power up or after wake up to load ucode
	#define DW1000_DW_WAKE_UP       0x02    // init after wake up - will not load ucode / ucode will not run
	#define DW1000_DW_WUP_NO_UCODE  0x04    // init after wake up - ucode has not already been loaded / ucode is not used
	#define DW1000_DW_WUP_RD_OTPREV 0x08    // init after wakeup - read OTP rev after wake up
	#define DW1000_READ_OTP_PID     0x10    // read part ID from OTP
	#define DW1000_READ_OTP_LID     0x20    // read lot ID from OTP
	#define DW1000_READ_OTP_BAT     0x40    // read ref voltage from OTP
	#define DW1000_READ_OTP_TMP     0x80    // read ref temperature from OTP


	//DW1000 OTP operating parameter set selection
	#define DW1000_OPSET_64LEN   0x0
	#define DW1000_OPSET_TIGHT   0x1
	#define DW1000_OPSET_DEFLT   0x2
	
	static dw1000_config_t config[8] = {
		//mode 0
		{
			2,               /* Channel number. */
			DW1000_PRF_16M,     /* Pulse repetition frequency. */
			DW1000_PLEN_1024,    /* Preamble length. Used in TX only. */
			DW1000_PAC32,        /* Preamble acquisition chunk size. Used in RX only. */
			3,               /* TX preamble code. Used in TX only. */
			3,               /* RX preamble code. Used in RX only. */
			1,               /* 0 to use standard SFD, 1 to use non-standard SFD. */
			DW1000_BR_110K,      /* Data rate. */
			DW1000_PHRMODE_STD, /* PHY header mode. */
			(1025 + 64 - 32)   /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
		},
		//mode 1
		{
			2,               /* Channel number. */
			DW1000_PRF_16M,     /* Pulse repetition frequency. */
			DW1000_PLEN_128,    /* Preamble length. Used in TX only. */
			DW1000_PAC8,        /* Preamble acquisition chunk size. Used in RX only. */
			3,               /* TX preamble code. Used in TX only. */
			3,               /* RX preamble code. Used in RX only. */
			0,               /* 0 to use standard SFD, 1 to use non-standard SFD. */
			DW1000_BR_6M8,      /* Data rate. */
			DW1000_PHRMODE_STD, /* PHY header mode. */
			(129 + 8 - 8)   /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
		},
		//mode 2
		{
			2,               /* Channel number. */
			DW1000_PRF_64M,     /* Pulse repetition frequency. */
			DW1000_PLEN_1024,    /* Preamble length. Used in TX only. */
			DW1000_PAC32,        /* Preamble acquisition chunk size. Used in RX only. */
			9,               /* TX preamble code. Used in TX only. */
			9,               /* RX preamble code. Used in RX only. */
			1,               /* 0 to use standard SFD, 1 to use non-standard SFD. */
			DW1000_BR_110K,      /* Data rate. */
			DW1000_PHRMODE_STD, /* PHY header mode. */
			(1025 + 64 - 32) /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
		},
		//mode 3
		{
			2,               /* Channel number. */
			DW1000_PRF_64M,     /* Pulse repetition frequency. */
			DW1000_PLEN_128,    /* Preamble length. Used in TX only. */
			DW1000_PAC8,        /* Preamble acquisition chunk size. Used in RX only. */
			9,               /* TX preamble code. Used in TX only. */
			9,               /* RX preamble code. Used in RX only. */
			0,               /* 0 to use standard SFD, 1 to use non-standard SFD. */
			DW1000_BR_6M8,      /* Data rate. */
			DW1000_PHRMODE_STD, /* PHY header mode. */
			(129 + 8 - 8)   /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
		},
		//mode 4
		{
			5,               /* Channel number. */
			DW1000_PRF_16M,     /* Pulse repetition frequency. */
			DW1000_PLEN_1024,    /* Preamble length. Used in TX only. */
			DW1000_PAC32,        /* Preamble acquisition chunk size. Used in RX only. */
			3,               /* TX preamble code. Used in TX only. */
			3,               /* RX preamble code. Used in RX only. */
			1,               /* 0 to use standard SFD, 1 to use non-standard SFD. */
			DW1000_BR_110K,      /* Data rate. */
			DW1000_PHRMODE_STD, /* PHY header mode. */
			(1025 + 64 - 32)   /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
		},
		//mode 5
		{
			5,               /* Channel number. */
			DW1000_PRF_16M,     /* Pulse repetition frequency. */
			DW1000_PLEN_128,    /* Preamble length. Used in TX only. */
			DW1000_PAC8,        /* Preamble acquisition chunk size. Used in RX only. */
			3,               /* TX preamble code. Used in TX only. */
			3,               /* RX preamble code. Used in RX only. */
			0,               /* 0 to use standard SFD, 1 to use non-standard SFD. */
			DW1000_BR_6M8,      /* Data rate. */
			DW1000_PHRMODE_STD, /* PHY header mode. */
			(129 + 8 - 8)   /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
		},
		//mode 6
		{
			5,               /* Channel number. */
			DW1000_PRF_64M,     /* Pulse repetition frequency. */
			DW1000_PLEN_1024,    /* Preamble length. Used in TX only. */
			DW1000_PAC32,        /* Preamble acquisition chunk size. Used in RX only. */
			9,               /* TX preamble code. Used in TX only. */
			9,               /* RX preamble code. Used in RX only. */
			1,               /* 0 to use standard SFD, 1 to use non-standard SFD. */
			DW1000_BR_110K,      /* Data rate. */
			DW1000_PHRMODE_STD, /* PHY header mode. */
			(1025 + 64 - 32) /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
		},
		//mode 7
		{
			5,               /* Channel number. */
			DW1000_PRF_64M,     /* Pulse repetition frequency. */
			DW1000_PLEN_128,    /* Preamble length. Used in TX only. */
			DW1000_PAC8,        /* Preamble acquisition chunk size. Used in RX only. */
			9,               /* TX preamble code. Used in TX only. */
			9,               /* RX preamble code. Used in RX only. */
			0,               /* 0 to use standard SFD, 1 to use non-standard SFD. */
			DW1000_BR_6M8,      /* Data rate. */
			DW1000_PHRMODE_STD, /* PHY header mode. */
			(129 + 8 - 8)   /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
		},
	};
	
	static dw1000_txconfig_t txconfig[6][2][2] = {
		// tx channel 1
		{
			// smart power disable
			{
				// 16MHz
				{
					0xc9,
					0x75757575
				},
				// 64Mhz
				{
					0xc9,
					0x67676767
				},
			},
			// smart power enable
			{
				// 16MHz
				{
					0xc9,
					0x15355575
				},
				// 64Mhz
				{
					0xc9,
					0x07274767
				},
			},
		},
		// tx channel 2
		{
			// smart power disable
			{
				// 16MHz
				{
					0xc2,
					0x75757575
				},
				// 64Mhz
				{
					0xc2,
					0x67676767
				},
			},
			// smart power enable
			{
				// 16MHz
				{
					0xc2,
					0x15355575
				},
				// 64Mhz
				{
					0xc2,
					0x07274767
				},
			},
		},
		// tx channel 3
		{
			// smart power disable
			{
				// 16MHz
				{
					0xc5,
					0x6f6f6f6f
				},
				// 64Mhz
				{
					0xc5,
					0x8b8b8b8b
				},
			},
			// smart power enable
			{
				// 16MHz
				{
					0xc5,
					0x0f2f4f6f
				},
				// 64Mhz
				{
					0xc5,
					0x2b4b6b8b
				},
			},
		},
		// tx channel 4
		{
			// smart power disable
			{
				// 16MHz
				{
					0x95,
					0x5f5f5f5f
				},
				// 64Mhz
				{
					0x95,
					0x9a9a9a9a
				},
			},
			// smart power enable
			{
				// 16MHz
				{
					0x95,
					0x1f1f3f5f
				},
				// 64Mhz
				{
					0x95,
					0x3a5a7a9a
				},
			},
		},
		// tx channel 5
		{
			// smart power disable
			{
				// 16MHz
				{
					0xc0,
					0x48484848
				},
				// 64Mhz
				{
					0xc0,
					0x85858585
				},
			},
			// smart power enable
			{
				// 16MHz
				{
					0xc0,
					0x0e082848
				},
				// 64Mhz
				{
					0xc0,
					0x25456585
				},
			},
		},// tx channel 7
		{
			// smart power disable
			{
				// 16MHz
				{
					0x93,
					0x92929292
				},
				// 64Mhz
				{
					0x93,
					0xd1d1d1d1
				},
			},
			// smart power enable
			{
				// 16MHz
				{
					0x93,
					0x32527292
				},
				// 64Mhz
				{
					0x93,
					0x5171b1d1
				},
			},
		},
	};
	
	typedef int decaIrqStatus_t ; // Type for remembering IRQ status
	
	#define XMLPARAMS_VERSION   (1.17f)
	
	extern u8 addr_data[6];
	extern char buf[100];
	static u32 self_partid;
		
	void dw1000_delayus(vu16 time);
	void dw1000_delayms(vu16 time);
	decaIrqStatus_t decamutexon(void);
	void decamutexoff(decaIrqStatus_t s);
	bool dw1000_finishtx_clearflag(void);
	u8 dw1000_gettxchannel(dw1000_config_t*);
	u8 dw1000_gettxfreq(dw1000_config_t*);
	u8 dw1000_getsmarttxpower(void);
	u32 dw1000_rxreadyerror(void);
	float dw1000_getFreqOffsetMul(dw1000_config_t*);
	float dw1000_getHertzToPPM(dw1000_config_t*);
	
	static const u8 chan_idx[NUM_CH_SUPPORTED] = {0, 0, 1, 2, 3, 4, 0, 5};
	
	//-----------------------------------------
	static const u32 tx_config[NUM_CH] = {
		RF_TXCTRL_CH1,
		RF_TXCTRL_CH2,
		RF_TXCTRL_CH3,
		RF_TXCTRL_CH4,
		RF_TXCTRL_CH5,
		RF_TXCTRL_CH7,
	};
	
	//Frequency Synthesiser - PLL configuration
	static const u32 fs_pll_cfg[NUM_CH] = {
		FS_PLLCFG_CH1,
		FS_PLLCFG_CH2,
		FS_PLLCFG_CH3,
		FS_PLLCFG_CH4,
		FS_PLLCFG_CH5,
		FS_PLLCFG_CH7
	};
	
	//Frequency Synthesiser - PLL tuning
	static const u8 fs_pll_tune[NUM_CH] = {
		FS_PLLTUNE_CH1,
		FS_PLLTUNE_CH2,
		FS_PLLTUNE_CH3,
		FS_PLLTUNE_CH4,
		FS_PLLTUNE_CH5,
		FS_PLLTUNE_CH7
	};
	
	//bandwidth configuration
	static const u8 rx_config[NUM_BW] = {
		RF_RXCTRLH_NBW,
		RF_RXCTRLH_WBW
	};
	
	static const agc_cfg_struct agc_config = {
		AGC_TUNE2_VAL,
		AGC_TUNE3_VAL,
		{ AGC_TUNE1_16M , AGC_TUNE1_64M }  //adc target
	};
	
	//DW non-standard SFD length for 110k, 850k and 6.81M
	static const u8 dwnsSFDlen[NUM_BR] = {
		DW_NS_SFD_LEN_110K,
		DW_NS_SFD_LEN_850K,
		DW_NS_SFD_LEN_6M8
	};
	
	// SFD Threshold
	static const u16 sftsh[NUM_BR][NUM_SFD] = {
		{
			DRX_TUNE0b_110K_STD,
			DRX_TUNE0b_110K_NSTD
		},
		{
			DRX_TUNE0b_850K_STD,
			DRX_TUNE0b_850K_NSTD
		},
		{
			DRX_TUNE0b_6M8_STD,
			DRX_TUNE0b_6M8_NSTD
		}
	};
	
	static const u16 dtune1[NUM_PRF] = {
		DRX_TUNE1a_PRF16,
		DRX_TUNE1a_PRF64
	};
	
	static const u32 digital_bb_config[NUM_PRF][NUM_PACS] = {
		{
			DRX_TUNE2_PRF16_PAC8,
			DRX_TUNE2_PRF16_PAC16,
			DRX_TUNE2_PRF16_PAC32,
			DRX_TUNE2_PRF16_PAC64
		},
		{
			DRX_TUNE2_PRF64_PAC8,
			DRX_TUNE2_PRF64_PAC16,
			DRX_TUNE2_PRF64_PAC32,
			DRX_TUNE2_PRF64_PAC64
		}
	};
	
	static const u16 lde_replicaCoeff[PCODES] = {
    0, // No preamble code 0
    LDE_REPC_PCODE_1,
    LDE_REPC_PCODE_2,
    LDE_REPC_PCODE_3,
    LDE_REPC_PCODE_4,
    LDE_REPC_PCODE_5,
    LDE_REPC_PCODE_6,
    LDE_REPC_PCODE_7,
    LDE_REPC_PCODE_8,
    LDE_REPC_PCODE_9,
    LDE_REPC_PCODE_10,
    LDE_REPC_PCODE_11,
    LDE_REPC_PCODE_12,
    LDE_REPC_PCODE_13,
    LDE_REPC_PCODE_14,
    LDE_REPC_PCODE_15,
    LDE_REPC_PCODE_16,
    LDE_REPC_PCODE_17,
    LDE_REPC_PCODE_18,
    LDE_REPC_PCODE_19,
    LDE_REPC_PCODE_20,
    LDE_REPC_PCODE_21,
    LDE_REPC_PCODE_22,
    LDE_REPC_PCODE_23,
    LDE_REPC_PCODE_24
};

#ifdef __cplusplus
}
#endif
