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

#if !defined(u8) || !defined(u16) || !defined(u32)
#include <stdint.h>
#define u8 uint8_t
#define vu8 volatile uint8_t
#define u16 uint16_t
#define vu16 volatile uint16_t
#define u32 uint32_t
#define vu32 volatile uint32_t
#endif

#include "my_DW1000_regs.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C"{
#endif
	
	typedef struct{
		u8 rev;
		u8 ver;
		u8 model;
		u16 ridtag;
	}dw1000_dev_id_t;

	typedef struct{
		u16 pan_id;
		u16 short_addr;
	}dw1000_pan_addr_t;

	typedef struct{
		u8 tx_chan;
		u8 rx_chan;
		u8 tx_preamble;
		u8 rx_preamble;
	}dw1000_channel_t;
	
	typedef struct{
		u8   PGdly;
		//TX POWER
		//31:24     BOOST_0.125ms_PWR
		//23:16     BOOST_0.25ms_PWR-TX_SHR_PWR
		//15:8      BOOST_0.5ms_PWR-TX_PHR_PWR
		//7:0       DEFAULT_PWR-TX_DATA_PWR
		u32  power;
	}dw1000_txconfig_t ;

	typedef struct{
		//all of the below are mapped to a 12-bit register in DW1000
		u16 PHE ;                    //number of received header errors
		u16 RSL ;                    //number of received frame sync loss events
		u16 CRCG ;                   //number of good CRC received frames
		u16 CRCB ;                   //number of bad CRC (CRC error) received frames
		u16 ARFE ;                   //number of address filter errors
		u16 OVER ;                   //number of receiver overflows (used in double buffer mode)
		u16 SFDTO ;                  //SFD timeouts
		u16 PTO ;                    //Preamble timeouts
		u16 RTO ;                    //RX frame wait timeouts
		u16 TXF ;                    //number of transmitted frames
		u16 HPW ;                    //half period warn
		u16 TXW ;                    //power up warn
		
	}dw1000_deviceentcnts_t ;

	typedef struct{
    u8 chan ;           //!< channel number {1, 2, 3, 4, 5, 7 }
    u8 prf ;            //!< Pulse Repetition Frequency {DW1000_PRF_16M or DW1000_PRF_64M}
    u8 txPreambLength ; //!< DW1000_PLEN_64..DW1000_PLEN_4096
    u8 rxPAC ;          //!< Acquisition Chunk Size (Relates to RX preamble length)
    u8 txCode ;         //!< TX preamble code
    u8 rxCode ;         //!< RX preamble code
    u8 nsSFD ;          //!< Boolean should we use non-standard SFD for better performance
    u8 dataRate ;       //!< Data Rate {DW1000_BR_110K, DW1000_BR_850K or DW1000_BR_6M8}
    u8 phrMode ;        //!< PHR mode {0x0 - standard DW1000_PHRMODE_STD, 0x3 - extended frames DW1000_PHRMODE_EXT}
    u16 sfdTO ;         //!< SFD timeout value (in symbols)
	}dw1000_config_t ;
	
	typedef struct{
		u32 tune2;
		u16 tune3;
		u16 tune1[NUM_PRF];
	} agc_cfg_struct ;
	
	typedef struct{
		u16      maxNoise ;          // LDE max value of noise
		u16      firstPathAmp1 ;     // Amplitude at floor(index FP) + 1
		u16      stdNoise ;          // Standard deviation of noise
		u16      firstPathAmp2 ;     // Amplitude at floor(index FP) + 2
		u16      firstPathAmp3 ;     // Amplitude at floor(index FP) + 3
		u16      maxGrowthCIR ;      // Channel Impulse Response max growth CIR
		u16      rxPreamCount ;      // Count of preamble symbols accumulated
		u16      firstPath ;         // First path index (10.6 bits fixed point integer)
	}dw1000_rxdiag_t;
	
	// TX/RX call-back data
	typedef struct{
		u32 status;      //initial value of register as ISR is entered
		u16 datalength;  //length of frame
		u8  fctrl[2];    //frame control bytes
		u8  rx_flags;    //RX frame flags, see above
	} dw1000_cb_data_t;
	
	typedef void (*dw1000_cb_t)(const dw1000_cb_data_t *);
	
	typedef struct{
		dw1000_channel_t channel;
		dw1000_pan_addr_t pan_addr;
		dw1000_dev_id_t dev_id;
		u32      partID ;            // IC Part ID - read during initialisation
    u32      lotID ;             // IC Lot ID - read during initialisation
    u8       vBatP ;             // IC V bat read during production and stored in OTP (Vmeas @ 3V3)
    u8       tempP ;             // IC V temp read during production and stored in OTP (Tmeas @ 23C)
    u8       longFrames ;        // Flag in non-standard long frame mode
    u8       otprev ;            // OTP revision number (read during initialisation)
    u32      txFCTRL ;           // Keep TX_FCTRL register config
    u32      sysCFGreg ;         // Local copy of system config register
    u8       dblbuffon;          // Double RX buffer mode flag
    u8       wait4resp ;         // wait4response was set with last TX start command
    u16      sleep_mode;         // Used for automatic reloading of LDO tune and microcode at wake-up
    u16      otp_mask ;          // Local copy of the OTP mask used in dw1000_initialise call
		dw1000_cb_data_t cbData;           // Callback data structure
    dw1000_cb_t    cbTxDone;           // Callback for TX confirmation event
    dw1000_cb_t    cbRxOk;             // Callback for RX good frame event
    dw1000_cb_t    cbRxTo;             // Callback for RX timeout events
    dw1000_cb_t    cbRxErr;            // Callback for RX error events
	}dw1000;
	
	void dw1000_writeSPI(u8 reg, u16 index, u8* data, u8 len);
	void dw1000_readSPI(u8 reg, u16 index, u8* data, u8 len);
	void dw1000_reset(void);
	bool dw1000_init(u8 config);
	u8 dw1000_otprevision(void);
	void dw1000_setfinegraintxseq(u8 enable);
	void dw1000_setlnapamode(u8 lna_pa);
	void dw1000_enablegpioclocks(void);
	void dw1000_setgpiodirection(u32 gpioNum, u32 direction);
	void dw1000_setgpiovalue(u32 gpioNum, u32 value);
	vu8 dw1000_getgpiovalue(u32 gpioNum);
	vu8 dw1000_geticrefvolt(void);
	vu8 dw1000_geticreftemp(void);
	vu32 dw1000_getpartid(void);
	vu32 dw1000_getlotid(void);
	void dw1000_readdevid(void);
	void dw1000_configuretxrf(dw1000_txconfig_t* config);
	void dw1000_configurefor64plen(u8 prf);
	bool dw1000_config(dw1000_config_t* config);
	void dw1000_setrxantennadelay(u16 rxDelay);
	void dw1000_settxantennadelay(u16 txDelay);
	bool dw1000_writetxdata(u16 txFrameLength, u8 *txFrameBytes, u16 txBufferOffset);
	bool dw1000_writetxdata_enable_fcs(u16 txFrameLength, u8 *txFrameBytes, u16 txBufferOffset);
	bool dw1000_writetxfctrl(u16 txFrameLength, u16 txBufferOffset, u8 ranging);
	void dw1000_readrxdata(u8 *buffer, u16 length, u16 rxBufferOffset);
	void dw1000_readaccdata(u8 *buffer, u16 len, u16 accOffset);
	int dw1000_readcarrierintegrator(void);
	void dw1000_readdiagnostics(dw1000_rxdiag_t *diagnostics);
	void dw1000_readtxtimestamp(u8 *timestamp);
	u32 dw1000_readtxtimestamphi32(void);
	u32 dw1000_readtxtimestamplo32(void);
	void dw1000_readrxtimestamp(u8 *timestamp);
	u32 dw1000_readrxtimestamphi32(void);
	u32 dw1000_readrxtimestamplo32(void);
	u32 dw1000_readsystimestamphi32(void);
	void dw1000_readsystime(u8 *timestamp);
	void dw1000_enableframefilter(u16 enable);
	void dw1000_setpanid(u16 panID);
	void dw1000_setaddress16(u16 shortAddress);
	void dw1000_seteui(u8 *eui64);
	void dw1000_geteui(u8 *eui64);
	void dw1000_otpread(u16 address, u32 *array, u8 length);
	vu32 _dw1000_otpread(u8 address);
	bool _dw1000_otpsetmrregs(u8 mode);
	bool _dw1000_otpprogword32(u32 data, u16 address);
	u8 dw1000_otpwriteandverify(u32 value, u16 address);
	void _dw1000_aonconfigupload(void);
	void _dw1000_aonarrayupload(void);
	void dw1000_entersleep(void);
	void dw1000_configuresleepcnt(u16 sleepcnt);
	u16 dw1000_calibratesleepcnt(void);
	void dw1000_configuresleep(u16 mode, u8 wake);
	void dw1000_entersleepaftertx(u8 enable);
	bool dw1000_spicswakeup(u8 *buff, u16 length);
	void dw1000_configlde(u8 prfIndex);
	void dw1000_loaducodefromrom(void);
	void dw1000_loadopsettabfromotp(u8 ops_sel);
	void dw1000_setsmarttxpower(u8 enable);
	void dw1000_enableautoack(u8 responseDelayTime);
	void dw1000_setdblrxbuffmode(u8 enable);
	void dw1000_setrxaftertxdelay(u32 rxDelayTime);
	void dw1000_setcallbacks(dw1000_cb_t cbTxDone, dw1000_cb_t cbRxOk, dw1000_cb_t cbRxTo, dw1000_cb_t cbRxErr);
	void dw1000_isr(void);
	u8 dw1000_checkirq(void);
	void dw1000_lowpowerlistenisr(void);
	void dw1000_setleds(u8 mode);
	void _dw1000_enableclock(u8 clock);
	void _dw1000_disablesequencing(void);
	void dw1000_setdelayedtrxtime(u32 starttime);
	bool dw1000_starttx(u8 mode);
	bool dw1000_starttx_no_auto_fcs(u8 mode);
	void dw1000_enable_auto_fcs(void);
	void dw1000_forcetrxoff(void);
	void dw1000_syncrxbufptrs(void);
	void dw1000_setsniffmode(u8 enable, u8 timeOn, u8 timeOff);
	void dw1000_setlowpowerlistening(u8 enable);
	void dw1000_setsnoozetime(u8 snooze_time);
	bool dw1000_rxenable(u8 mode);
	void dw1000_setrxtimeout(u16 time);
	void dw1000_setpreambledetecttimeout(u16 timeout);
	void dw1000_setinterrupt(u32 bitmask, u8 operation);
	void dw1000_configeventcounters(u8 enable);
	void dw1000_readeventcounters(dw1000_deviceentcnts_t *counters);
	void dw1000_rxreset(void);
	void dw1000_softreset(void);
	void dw1000_setxtaltrim(u8 value);
	u8 dw1000_getxtaltrim(void);;
	bool dw1000_configcwmode(u8 chan);
	void dw1000_configcontinuousframemode(u32 framerepetitionrate);
	u16 dw1000_readtempvbat(u8 fastSPI);
	float dw1000_convertrawtemperature(u8 raw_temp);
	u8 dw1000_convertdegtemptoraw(short externaltemp);
	float dw1000_convertrawvoltage(u8 raw_voltage);
	u8 dw1000_convertvoltstoraw(int externalmvolt);
	u8 dw1000_readwakeuptemp(void);
	u8 dw1000_readwakeupvbat(void);
	u8 dw1000_calcbandwidthtempadj(u16 target_count);
	u32 _dw1000_computetxpowersetting(u32 ref_powerreg, int power_adj);
	u32 dw1000_calcpowertempadj(u8 channel, u32 ref_powerreg, int delta_temp);
	u16 dw1000_calcpgcount(u8 pgdly);
	
#ifdef __cplusplus
}
#endif