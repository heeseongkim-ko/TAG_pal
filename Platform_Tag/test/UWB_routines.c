
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "def_var.h"
#include "def_pin.h"
#include "UWB_routines.h"
#include "teia_routines.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf_drv_gpiote.h"
#include "dw3000_device_api.h"
#include "dw3000_regs.h"


//------------------------------------------------------------
#define OK                  (0)
#define ERR_OUT_OF_RANGE    (1)
// define GAINs constants of DW1000 radio
#define MIXER_GAIN_MASK     (0b00011111)
#define PA_GAIN_MASK         ~MIXER_GAIN_MASK
#define PA_GAIN_OFFSET      (5)

#define PA_GAIN_15dB        (uint8_t)(0 << PA_GAIN_OFFSET)
#define PA_GAIN_12_5dB      (uint8_t)(1 << PA_GAIN_OFFSET)
#define PA_GAIN_10dB        (uint8_t)(2 << PA_GAIN_OFFSET)
#define PA_GAIN_7_5dB       (uint8_t)(3 << PA_GAIN_OFFSET)
#define PA_GAIN_5dB         (uint8_t)(4 << PA_GAIN_OFFSET)
#define PA_GAIN_2_5dB       (uint8_t)(5 << PA_GAIN_OFFSET)
#define PA_GAIN_0dB         (uint8_t)(6 << PA_GAIN_OFFSET)
#define PA_OFF              (uint8_t)(7 << PA_GAIN_OFFSET)

#define MIN_ALLOWED_GAIN        0.f
#define MAX_ALLOWED_GAIN        28.f
#define MAX_POSSIBLE_PA_GAIN    15.f
#define MAX_ALLOWED_PA_GAIN     12.5f

#define REFERENCE_UWB_PACKET_LENGTH 10
#define CRC16_LENGTH 2
//
// PREAMBLE
//
//-------------------------------------------------------------------------------
/*Variables*/
data_requests_struct_t app_data_storage[NUM_OF_PRIORITIES][MAX_NUM_OF_APPS];  /*!< Array of structures, where the @f Data_Req() store the data from applications that should be send to server */
data_confirm_cb_t data_confirm_cb_arr[NUM_OF_PRIORITIES*MAX_NUM_OF_APPS]; /*!< Array of callback functions that should be called by @f Data_Conf() once the data request is done*/
volatile bool CHG_active;
volatile bool ACC_sleep;
volatile bool dw_temperature_measure;
volatile bool blink_sent;

const static uint8_t preambleCfg[8] = {
    DWT_PLEN_4096, //0
    DWT_PLEN_2048, //1
    DWT_PLEN_1536, //2
    DWT_PLEN_1024, //3
    DWT_PLEN_512, //4
    DWT_PLEN_256, //5
    DWT_PLEN_128, //6
    DWT_PLEN_64, //7
    };

//tx power
const static uint8_t txPGdelayConfig[8] =
{
    0x0,    //Channel 0 ----- this is just a place holder so the next array element is channel 1
    0xc9,   //Channel 1 PG_DELAY
    0xc2,   //Channel 2//PG_DELAY
    0xc5,   //Channel 3//PG_DELAY
    0x95,   //Channel 4//PG_DELAY
    0xc0,   //Channel 5//PG_DELAY
    0x0,    //Channel 6 ----- this is just a place holder so the next array element is channel 7//0
    0x93    //Channel 7//PG_DELAY
};

const static float compCoefTemp[8] =
{
    0.f,   //placeholder for temperature coeficient for CH0
    0.f,   //placeholder for temperature coeficient for CH1
    0.02944f,   //placeholder for temperature coeficient for CH2
    0.f,   //placeholder for temperature coeficient for CH3
    0.f,   //placeholder for temperature coeficient for CH4
    0.05104f, //temperature coeficient for CH5
    0.f,   //placeholder for temperature coeficient for CH6
    0.f    //placeholder for temperature coeficient for CH7
};

const float refFrameTxTimeUs[6] =
{
    0.f,        ////placeholder for uwbFrameTime in us for RF0
    424.34f,    ////uwbFrameTime in us for RF1
    174.2256f,  ////uwbFrameTime in us for RF2
    0.f,        ////placeholder for uwbFrameTime in us for RF3
    433.1823f,  ////uwbFrameTime in us for RF4
    178.2643f   ////uwbFrameTime in us for RF5
};

const float oneByteTxTimeUs[6] =
{
    0.f,       ////placeholder for uwbFrameTime in us for RF0
    9.36f,     ////uwbFrameTime in us for RF1
    1.1744f,   ////uwbFrameTime in us for RF2
    0.f,       ////placeholder for uwbFrameTime in us for RF3
    9.3177f,   ////uwbFrameTime in us for RF4
    1.1757f    ////uwbFrameTime in us for RF5
};


const static uint8_t dB2dw_lookup[] = {
    0xc0,0xc1,0xc2,0xc3,0xc4,0xa0,0xa1,
    0xa2,0xa3,0xa4,0x80,0x81,0x82,0x83,
    0x84,0x60,0x61,0x62,0x63,0x64,0x40,
    0x41,0x42,0x43,0x44,0x20,0x21,0x22,
    0x23,0x24,0x25,0x26,0x27,0x28,0x29,
    0x2a,0x2b,0x2c,0x2d,0x2e,0x2f,0x30,
    0x31,0x32,0x33,0x34,0x35,0x36,0x37,
    0x38,0x39,0x3a,0x3b,0x3c,0x3d,0x3e,
    0x3f};

const static float dw2dB_lookup[] = {
    15.0f,15.5f,16.0f,16.5f,17.0f,17.5f,18.0f,18.5f,19.0f,19.5f,20.0f,20.5f,21.0f,21.5f,22.0f,22.5f,
    23.0f,23.5f,24.0f,24.5f,25.0f,25.5f,26.0f,26.5f,27.0f,27.5f,28.0f,28.5f,29.0f,29.5f,30.0f,30.5f,
    12.5f,13.0f,13.5f,14.0f,14.5f,15.0f,15.5f,16.0f,16.5f,17.0f,17.5f,18.0f,18.5f,19.0f,19.5f,20.0f,
    20.5f,21.0f,21.5f,22.0f,22.5f,23.0f,23.5f,24.0f,24.5f,25.0f,25.5f,26.0f,26.5f,27.0f,27.5f,28.0f,
    10.0f,10.5f,11.0f,11.5f,12.0f,12.5f,13.0f,13.5f,14.0f,14.5f,15.0f,15.5f,16.0f,16.5f,17.0f,17.5f,
    18.0f,18.5f,19.0f,19.5f,20.0f,20.5f,21.0f,21.5f,22.0f,22.5f,23.0f,23.5f,24.0f,24.5f,25.0f,25.5f,
    7.5f,8.0f,8.5f,9.0f,9.5f,10.0f,10.5f,11.0f,11.5f,12.0f,12.5f,13.0f,13.5f,14.0f,14.5f,15.0f,15.5f,
    16.0f,16.5f,17.0f,17.5f,18.0f,18.5f,19.0f,19.5f,20.0f,20.5f,21.0f,21.5f,22.0f,22.5f,23.0f,5.0f,
    5.5f,6.0f,6.5f,7.0f,7.5f,8.0f,8.5f,9.0f,9.5f,10.0f,10.5f,11.0f,11.5f,12.0f,12.5f,13.0f,13.5f,
    14.0f,14.5f,15.0f,15.5f,16.0f,16.5f,17.0f,17.5f,18.0f,18.5f,19.0f,19.5f,20.0f,20.5f,2.5f,3.0f,3.5f,
    4.0f,4.5f,5.0f,5.5f,6.0f,6.5f,7.0f,7.5f,8.0f,8.5f,9.0f,9.5f,10.0f,10.5f,11.0f,11.5f,12.0f,12.5f,
    13.0f,13.5f,14.0f,14.5f,15.0f,15.5f,16.0f,16.5f,17.0f,17.5f,18.0f,0.0f,0.5f,1.0f,1.5f,2.0f,2.5f,
    3.0f,3.5f,4.0f,4.5f,5.0f,5.5f,6.0f,6.5f,7.0f,7.5f,8.0f,8.5f,9.0f,9.5f,10.0f,10.5f,11.0f,11.5f,12.0f,
    12.5f,13.0f,13.5f,14.0f,14.5f,15.0f,15.5f,255.0f,255.0f,255.0f,255.0f,255.0f,255.0f,255.0f,255.0f,
    255.0f,255.0f,255.0f,255.0f,255.0f,255.0f,255.0f,255.0f,255.0f,255.0f,255.0f,255.0f,255.0f,255.0f,
    255.0f,255.0f,255.0f,255.0f,255.0f,255.0f,255.0f,255.0f,255.0f};

//Voltage compensation look up table
float V_comp_factor[]={6.f,6.f,6.f,6.f,6.f,6.f,6.f,6.f,6.f,6.f,6.f,6.f,6.f,6.f,6.f,6.f,6.f,6.f,6.f,6.f,6.f,6.f,4.5f,3.5f,3.f,2.5f,2.f,2.f,1.5f,1.5f,1.f,1.f,0.f,0.f,-0.5f,-1.f,-1.f};

/*reference supply voltage in deciVolts*/
int8_t ref_voltage_dv = 33;

uint8_t seqNum = 0; //sequence number of MAC frame. It start with 0 after FW reset
uint8_t bc_ack_num[BC_ACK_NUM_SIZE] = {0};
/* gain offset - referenced to OTP CH5 gain*/
const static float gain_offset[] =
{
    0.f,    //ch0 offset
    0.f,    //ch1 offset
    2.5f,    //ch2 offset
    0.f,    //ch3 offset
    0.f,    //ch4 offset
    0.f,    //ch5 offset
    0.f,    //ch6 offset
    0.f     //ch7 offset
};

uint16_t rx_err = 0;


typedef struct {
     uint8_t fcode;
     uint8_t MAC_addr[MAC_ADDR_BYTE_SIZE];
     uint8_t seqNum;
     uint8_t BC_version;
     uint8_t BC_ack_num[BC_ACK_NUM_SIZE];
} mac_header_ba_t;

typedef struct {
     uint8_t fcode;
     uint8_t MAC_addr[MAC_ADDR_BYTE_SIZE];
     uint8_t seqNum;
} mac_header_bb_t;


typedef struct {
     uint8_t fcode;
     uint8_t MAC_addr[MAC_ADDR_BYTE_SIZE];
     uint8_t seqNum;
     uint8_t BC_version;
     uint8_t BC_ack_num[BC_ACK_NUM_SIZE];
     uint8_t BC_T_toRX;
}mac_header_bc_t;

typedef struct {
     uint8_t fcode;
     uint8_t MAC_addr[MAC_ADDR_BYTE_SIZE];
     uint8_t seqNum;
     uint8_t BC_version;
     uint8_t BC_ack_num[BC_ACK_NUM_SIZE];
     uint8_t BC_options;
}mac_header_bd_t;

typedef struct{
}mac_footer_t;


mac_header_ba_t mac_header_ba;
mac_header_bb_t mac_header_bb;
mac_header_bc_t mac_header_bc;
mac_header_bd_t mac_header_bd;
mac_footer_t mac_footer;

tag_info_msg_t info_msg;
//------------------------------------------------------------
/***
 * signalize_dwmError
 * just for developer tasks
 * 6 x 100 ms FAST LED BLINK signalize the cause
 */
void signalize_dwmError(void)
{
	for (uint8_t i = 0; i < 6; i++)
    {
		//LED1_ON();
		delay_and_sleep(100, 1, true);
		//LED1_OFF();
		delay_and_sleep(100, 1, true);
	}
}

/**
 * @brief Set reset pin of DW1OOO as output - low state.
 */
void DW_RST_ON(void)
{
    nrf_gpio_pin_clear(DW3000_RST_Pin);
    nrf_gpio_cfg_output(DW3000_RST_Pin);
}

/**
 * @brief Set reset pin of DW1OOO as High-Z input.
 */
void DW_RST_OFF(void)
{
    nrf_gpio_cfg_input(DW3000_RST_Pin,NRF_GPIO_PIN_NOPULL);
}

/**
 * @brief Set the low state at SPI Chip select pin.
 */
void DW_CS_CLR(void)
{
    nrf_gpio_pin_clear(DW3000_CS_Pin);
}

/**
 * @brief Set the high state at SPI Chip select pin.
 */
void DW_CS_SET(void)
{
    nrf_gpio_pin_set(DW3000_CS_Pin);
}

void DWM1000_initAndDoHwResetProcedure(void)
{

	//-----------------------------------------------
	// HW Reset procedure
	//-----------------------------------------------
	DW_RST_ON();
	//
	//wait 5ms (rather 6ms in case of EFM32...)
	//
	delay_and_sleep(6, 1, true); //

	//
	//DWM1000 reset pin set to normal mode
	//

	//Set reset pin to High Z (open-drain) in input mode
	DW_RST_OFF();
	//
	//wait 5ms (rather 6ms in case of EFM32...)
	//
        /*
	if(nrf_gpio_pin_read(19))//debug_khs_20250507
    {
        while(1)
        {
            nrf_delay_ms(10);
        }
    }
    */
	delay_and_sleep(6, 1, true); //
	//-----------------------------------------------
	//
	// Wakeup process procedure
	//
	//-----------------------------------------------
	//If SPI comms is not possible (the DW1000 might be in DEEPSLEEP/SLEEP)
	//then assert the WAKEUP pin and SPICSn for 500us to trigger a wakeup,
	//then check for SPI comms and proceed as from (2) above.

	//Set WAKEUP and SPICS to wakeup levels
	SPI_pins_enable();
	DW_CS_CLR(); // clear SPI_chip select pin for DWM
	//
	//wait 500us (rather 1ms in case of EFM32...)
	//
	delay_and_sleep(1, 1, true);

	//Set WAKEUP and SPICS to normal levels

	DW_CS_SET();
	//SPI_pins_disable();

	//
	//Wait 5ms (rather 6ms in case of EFM32...) until DWM go from POWER UP state to INIT state
	//
	delay_and_sleep(6, 1, true);

}

/**
 * @brief Check if DWM is alive and initialize communication
 */
static uint32_t init_DWM3000_communication(void)
{
	if(baudrate != SPI_BAUDRATE_LOW)baudrate = SPI_init(SPI_BAUDRATE_LOW); //max SPI before PLLs configured is ~4M //

	dwt_softreset();
	nrf_delay_ms(100);

	uint32_t devID = dwt_readdevid();
	if (DWT_DEVICE_ID != devID)
        {
            //if the read of devide ID fails, the DW1000 could be asleep
            DWM1000_initAndDoHwResetProcedure();
            dwt_softreset();

            DW_CS_CLR(); //
            delay_and_sleep(100, 1, true); //wake up, then waits for DW1000 XTAL to stabilize
            DW_CS_SET(); //
            delay_and_sleep(100, 1, true);
            devID = dwt_readdevid();
            //SPI not working or Unsupported Device ID
            if (DWT_DEVICE_ID != devID)
            return (-1);
	}

	//if (dwt_initialise(DWT_LOADUCODE | DWT_READ_OTP_LID | DWT_READ_OTP_PID/* FWREM | DWT_LOADTXCONFIG | DWT_LOADANTDLY | DWT_LOADXTALTRIM)*/) != DWT_SUCCESS)
        if (dwt_initialise(DWT_DW_INIT) != DWT_SUCCESS)
        {
          return (-1);
          printf("@@@@@@dwt_initialise_fail\n\r");
        }

	baudrate = SPI_init(SPI_BAUDRATE_HIGH); //increase SPI to max
	devID = dwt_readdevid();

	if (DWT_DEVICE_ID != devID) // Means it is NOT MP device
        {
              // SPI not working or Unsupported Device ID
              printf("@@@@@@read_device_error\n\r");
              return (-1);
	}
	return 0;
}

/**
 * @brief Dwt_Txcallback
 *
 * @param txd
 */
tdoa_uwb_conf_msg confRxBuff;							/**<Received message content*/
uint32_t conf_packet_len = sizeof(tdoa_uwb_conf_msg);	/**<Expected length of config message*/
bool confState = false;									/**<Valid config message was received?*/
rx_data_t rx_data;
tx_data_t tx_buffer;
dw_rx_errors_t dwt_rx_err;

volatile bool rx_frame_received = false;             /**<Back-channel data received?*/

void Dwt_Txcallback(const dwt_cb_data_t *txd)
{
	blink_sent = true;
        printf("TX_OK...\n\r"); //khs20250519_debug
}


void Dwt_Rxcallback_conf(const dwt_cb_data_t *rxd)
{
	uint8_t receivedFcode = 0;
	/*******************************************************************************
	 * SYNC msg
	 */
	 uint8_t size_of_msg = sizeof(tdoa_uwb_conf_msg);
	if (rxd->datalength == size_of_msg)
	//if (rxd->event == DWT_SIG_RX_OKAY)
	{
		dwt_readrxdata(&receivedFcode, 1, 0);

		if (receivedFcode == UWB_FCODE_CONF)
        {
			//fcode is SYNC
			dwt_readrxdata((uint8_t*) &confRxBuff, conf_packet_len, 0);

			uint64_t destAddr;
			memcpy(&(destAddr), &(confRxBuff.destAddr),
					sizeof(confRxBuff.destAddr));

			destAddr &= ~(0xFFFF000000000000);

			if (destAddr == TAG_BROADCAST_ADDR	|| destAddr == tdoaParameters.this_tag_MACaddress)
			{
				confState = true;
			}
			else
			{
				confState = false; //syncState = SYNC_TIMED_OUT;
				//re-enable receiver
				receivedFcode = 0;
				dwt_forcetrxoff();
				dwt_rxenable(0);
			}

		}
		else
        {
			//fcode is not valid for SYNC - (probably it is POLL due to same length)
			confState = false; //syncState = SYNC_TIMED_OUT;
			//re-enable receiver
			receivedFcode = 0;
			dwt_forcetrxoff();
			dwt_rxenable(0);
		} //end of if(syncedFcode == RTLS_DEMO_MSG_SANCH_SYNC)

		/*******************************************************************************
		 * RESPONSE msg or BLID RESPONSE msg
		 */
	}
	else
    {
		//Any other message was captured
		confState = false;
		//re-enable receiver
		receivedFcode = 0;
		dwt_forcetrxoff();
		dwt_rxenable(0);

	} //end of main if

}


/**
 *	Dwt_Rx_OK_callback
 * @param
 */
void Dwt_Rx_OK_callback(const dwt_cb_data_t *rxd)
{
    rx_frame_received = true;
    dwt_readrxdata(rx_data.rx_data_buffer, rxd->datalength, 0);     //read all received data and store it to rx_data_buffer
    rx_data.rx_data_len = rxd->datalength;                         //copy data length to rx_data_len for use by highrer layers
    rx_data.rx_data_rdy = true;                                    //set flag that the received data are ready
}

void Dwt_Rx_T_OUT_callback(const dwt_cb_data_t *rxd)
{
    dwt_rx_err.rx_timeout = true;
    if(rxd->status & SYS_STATUS_RXPTO_BIT_MASK)//SYS_STATUS_RXPTO.dw1000
    {
        dwt_rx_err.rx_pream_timeout = true;
    }
	else
    {
		dwt_rx_err.rx_frame_timeout = true;
	}
}


void Dwt_Rx_ERR_callback(const dwt_cb_data_t *rxd)
{
    dwt_rx_err.rx_error = true;
    if(rxd->status & SYS_STATUS_RXPHE_BIT_MASK)//SYS_STATUS_RXPHE.dw1000
    {
        dwt_rx_err.rx_err_phe = true;
    }
    if(rxd->status &  SYS_STATUS_RXFCE_BIT_MASK)//SYS_STATUS_RXFCE.dw1000
    {
        dwt_rx_err.rx_err_fce = true;
    }
    if(rxd->status &  SYS_STATUS_RXFSL_BIT_MASK)//SYS_STATUS_RXRFSL.dw1000
    {
        dwt_rx_err.rx_err_rfsl = true;
    }
    if(rxd->status &  SYS_STATUS_RXSTO_BIT_MASK)//SYS_STATUS_RXSFDTO.dw1000
    {
        dwt_rx_err.rx_err_sfdto = true;
    }
    if(rxd->status &  SYS_STATUS_ARFE_BIT_MASK)//SYS_STATUS_AFFREJ.dw1000
    {
        dwt_rx_err.rx_err_affrej = true;
    }
    if(rxd->status &  SYS_STATUS_CIAERR_BIT_MASK)//SYS_STATUS_LDEERR.dw1000
    {
        dwt_rx_err.rx_err_lde = true;
    }

    dwt_forcetrxoff();
}
/*
float cacl_RSSI(void)
{
    dwt_rxdiag_t diag;
    float RSSI =0.f;

    dwt_readdiagnostics_lite(&diag);
    RSSI = DW1000_CountRxPowerLevel(tdoaParameters.prf+1,diag.maxGrowthCIR,diag.rxPreamCount);
    return RSSI;
}
*/

/**
 * @brief Initialize DWM1000 and read partnumber to create MAC adress of tag
 */

void DWM1000_init(void)
{
	int32_t result = DWT_ERROR;

	result = init_DWM3000_communication();

	if(result == DWT_ERROR)
	{
		signalize_dwmError();
		NVIC_SystemReset();
	}
	else
	{

          //dwt_configuresleep(DWT_LOADUCODE | DWT_PRESRV_SLEEP | DWT_CONFIG | DWT_TANDV,DWT_WAKE_CS | DWT_SLP_EN); //configure the on wake parameters (upload the IC config settings)
          //dwt_configuresleep(DWT_LOADLDO | DWT_LOADBIAS | DWT_CONFIG | DWT_RUNSAR,DWT_WAKE_CSN | DWT_SLP_EN);
          dwt_configuresleep(DWT_CONFIG | DWT_PGFCAL ,DWT_PRES_SLEEP | DWT_WAKE_CSN | DWT_SLP_EN);
          //dwt_configuresleep(DWT_LOADLDO | DWT_LOADBIAS | DWT_CONFIG | DWT_RUNSAR , DWT_WAKE_CSN | DWT_SLP_EN);
          //dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | (DWT_INT_ARFE | DWT_INT_RFSL | DWT_INT_SFDT| DWT_INT_RPHE | DWT_INT_RFCE /*| DWT_INT_RFTO*/ | DWT_INT_RXPTO),1);
          dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | (DWT_INT_ARFE | DWT_INT_RFSL | DWT_INT_SFDT| DWT_INT_RPHE | DWT_INT_RFCE /*| DWT_INT_RFTO*/ | DWT_INT_RXPTO),0,1);
          dwt_setcallbacks(Dwt_Txcallback,Dwt_Rx_OK_callback, Dwt_Rx_T_OUT_callback , Dwt_Rx_ERR_callback,NULL,NULL);


          //dwt_setleds(2|1) ;
          //dwt_setlnapamode(2|1);
//dump_dwm3001c_otp_values();//debug
          //read_otp_params();
          //debug
          //debug
          dwt_entersleep(DWT_DW_IDLE_RC);
	}
}
//-----------------------------------------------------------------------------
extern volatile bool rtcDelayComplete[3][4];

/**
 * @brief Do software reset of MCU
 * @param errnum - number of error becouse of that, the softreset must be done
 */
void do_SWResetMCU(uint8_t errnum)
{
    if(errnum)
    {
            delay_and_sleep(200, 1, true);
    }
    NVIC_SystemReset();
}

/**
 *
 */
 
static void dw_restore_from_sleep(void)
{
    dwt_setinterrupt(DWT_INT_TFRS, 0, 1); //re-enable the TX/RX interrupts//org
}

/**
 * @brief Enable Interrupt Request from General Purpose Input/Output pins.
 */
void GPIO_IRQ_enable(void)
{
    NVIC_EnableIRQ(GPIOTE_IRQn);
}

/**
 * @brief Enable Interrupt Request from UWB radio chip RST_PIN.
 */
void DW_RST_IRQ_enable(void)
{
    nrf_drv_gpiote_in_event_enable(DW3000_RST_Pin, true);
    ext_irq_en.dw_rst= true;
}

/**
 * @brief Disable Interrupt Request from UWB radio chip - RST pin.
 */
void DW_RST_IRQ_disable(void)
{
	nrf_drv_gpiote_in_event_disable(DW3000_RST_Pin);
	ext_irq_en.dw_rst= false;
}

/**
 * @brief Wake up DWM module from sleep state.
 */
 void DMW1000_wake_up(void)
{
    SPI_pins_enable();                          //enable SPI pins, SPI_CS is used during wakeup
    int_flag.DW_XTAL_stable = nrf_gpio_pin_read(DW3000_RST_Pin);     //check state of RESET pin, if its high, the radio is in the IDLE mode
    GPIO_IRQ_enable();                              //enable interrupts from GPIOs
    DW_RST_IRQ_enable();                            //enable iRQ from DW reset pin - it goes high once the XTAL is stable and the radio is in IDLE mode
    DW_CS_CLR();                                //wake up the DW radio by CS pin
    nrf_delay_us(600);//khs
    set_sleep_time(DWM_WAKEUP_XTAL_STAB_TIME_TCKS,get_RTC_timestamp(false));  //set timeout - XTAL should be stabilized in


    while(!int_flag.DW_XTAL_stable && !(rtcDelayComplete[RTC_delay_instance][deep_sleep_CC_reg]))                 //wait while XTAL is not stabilized
    {
       __WFI();
    }

    DW_CS_SET();                        //set DW chip select to idle state
    DW_RST_IRQ_disable();               //disable IRQ to prevent unwanted asynchronous interrupt request
    deep_sleep_RTC_IRQ_disable();       //disable IRQ to prevent unwanted synchronour interrupt request

    nrf_delay_us(50);                       //wait until PLL locks

    uint32_t	devID = dwt_readdevid();			// check if the DWM is answering for the ID request
    if (DWT_DEVICE_ID != devID)					//if wake up was not successful
    {
            DWM1000_initAndDoHwResetProcedure();	//reset DWM //and try it again
            uint32_t	devID = dwt_readdevid();	// check if the DWM is answering for the ID request
            if (DWT_DEVICE_ID != devID)
            {
                    signalize_dwmError();
                    do_SWResetMCU(20);
            }
    }
    dw_restore_from_sleep(); 					//do after sleep procedure
}

/**
 * @brief Set DW radio into sleep state.
 */
void DM1000_enter_sleep(void)
{
    //dwt_entersleep(DWT_DW_IDLE_RC);
    dwt_entersleep(DWT_DW_IDLE_RC);
    
}

/**
 * @brief This function only wake up DWM module.
 */
void wake_up_radio(void)
{
    SPI_pins_disable();
    DMW1000_wake_up();
}

/**
 * @brief This function only enter DWM module into sleep state order to save energy.
 */
void enter_radio_sleep(void)
{
    //SPI_pins_enable();
    //dwt_configuresleep(DWT_CONFIG | DWT_PGFCAL ,DWT_PRES_SLEEP | DWT_WAKE_CSN | DWT_SLP_EN);
          //dwt_configuresleep(DWT_LOADLDO | DWT_LOADBIAS | DWT_CONFIG | DWT_RUNSAR , DWT_WAKE_CSN | DWT_SLP_EN);
          //dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | (DWT_INT_ARFE | DWT_INT_RFSL | DWT_INT_SFDT| DWT_INT_RPHE | DWT_INT_RFCE /*| DWT_INT_RFTO*/ | DWT_INT_RXPTO),1);
          //dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | (DWT_INT_ARFE | DWT_INT_RFSL | DWT_INT_SFDT| DWT_INT_RPHE | DWT_INT_RFCE /*| DWT_INT_RFTO*/ | DWT_INT_RXPTO),0,1);
          //dwt_setcallbacks(Dwt_Txcallback,Dwt_Rx_OK_callback, Dwt_Rx_T_OUT_callback , Dwt_Rx_ERR_callback,NULL,NULL);
    DM1000_enter_sleep();
    SPI_pins_disable();
    //while(1);
    //while(1)
    //{
      //printf("test_nrf_rst_pin_read = %d \n\r",nrf_gpio_pin_read(DW3000_CS_Pin));
      //nrf_delay_ms(1000);
    //}
    //SPI_pins_disable();
}

/**
* @brief Read IDs from DW1000 and compose.
* @param[out] lotid - LOT_ID of DW1000
* @param[out] partid - PART_ID of DW1000
*/
void dw_read_IDs(uint32_t* lotid, uint32_t* partid)
{
    *lotid = dwt_getlotid();            //read LOT ID from DW1000 - this is used as part of Tags MAC address
    *partid = dwt_getpartid();         //read LOT ID from DW1000 - this is used as part of Tags MAC address
   //*lotid = _dwt_otpread(LOT);
   //*partid = _dwt_otpread(PARTID_ADDRESS);
}


/**@brief     Function for calculate TX gain in decibels from DW1000 format.
 *
 * @details   This functions taken gain in DW1000 format and calculate gain in decibels.
 * @param[in] gain_DW_form TX gain in DW1000 format.
 * @param[out] gain_dB TX path gain in decibels.
 * @return    Return OK if input value is in allowed range, else return ERR_OUT_OF_RANGE.
 */
uint8_t DW_format2dB(uint8_t gain_DW_format, float* gain_dB)
{
    *gain_dB = dw2dB_lookup[gain_DW_format];

    if(*gain_dB > MAX_ALLOWED_GAIN)
    {
        *gain_dB = MAX_ALLOWED_GAIN;
        return ERR_OUT_OF_RANGE;
    }
    else if(*gain_dB < MIN_ALLOWED_GAIN)
    {
        *gain_dB = MIN_ALLOWED_GAIN;
        return ERR_OUT_OF_RANGE;
    }
     return OK;
}

/**@brief     Function for calculate TX gain in DW1000 format.
 *
 * @details   This functions taken gain in decibels and calculate DW format of this gain.
 * @param[in] gain_dB TX path gain in decibels.
 * @param[out] gain_DW_form - Calculated TX gain in DW1000 format.
 * @return    Return OK if input value is in allowed range, else return ERR_OUT_OF_RANGE.
 */
uint8_t dB2DW_format(float gain_dB, uint8_t* gain_DW_form)
{
    uint8_t ret = OK;

    if(gain_dB > MAX_ALLOWED_GAIN)
    {
        gain_dB = MAX_ALLOWED_GAIN;
        ret = ERR_OUT_OF_RANGE;
    }
    else if(gain_dB < MIN_ALLOWED_GAIN)
    {
        gain_dB = MIN_ALLOWED_GAIN;
        ret = ERR_OUT_OF_RANGE;
    }

    uint8_t lookup_index = (uint8_t)(gain_dB*2);      //step of look up table is 0.5 so gain must be multiple by 2 to get correct index of array
    *gain_DW_form = dB2dw_lookup[lookup_index];

    return ret;
}

uint8_t add_correction_to_DWgain(const uint8_t* input_gain, uint8_t* corrected_gain, float correction)
{
    uint8_t ret;
    for(uint8_t i = 0; i < 4; i++)
    {
        float gain_dB;
        ret = DW_format2dB(input_gain[i], &gain_dB);

        if(ret == ERR_OUT_OF_RANGE)
            gain_dB = MAX_ALLOWED_GAIN;
        else
        {
            gain_dB += correction;
            if(gain_dB > MAX_ALLOWED_GAIN) gain_dB = MAX_ALLOWED_GAIN;
        }
        ret += dB2DW_format(gain_dB, &corrected_gain[i]);
    }
    return ret;
}

uint8_t calc_TX_gain(const uint32_t* raw_gain, uint32_t* corr_gain, float correction_f)
{
     uint8_t ret = RET_OK;
     uint8_t *raw_gain_p;
     uint8_t *corr_gain_p;

     raw_gain_p = (uint8_t*)raw_gain;
     corr_gain_p = (uint8_t*)corr_gain;

     ret = add_correction_to_DWgain(raw_gain_p, corr_gain_p, correction_f);

     return ret;
}

void init_TX_gain(void)
{
/*
    if(tdoaParameters.tx_pwr_level_conf == 0x00000000)
    {
        load_std_compliant_TX_params(tdoaParameters.channel, tdoaParameters.RF_profile, &tdoaParameters.tx_pwr_level_raw, &tdoaParameters.tx_PG_delay);
    }
    else
    */
    //{
    tdoaParameters.tx_pwr_level_raw = tdoaParameters.tx_pwr_level_conf;
    //}


    calc_TX_gain(&tdoaParameters.tx_pwr_level_raw, &tdoaParameters.tx_pwr_level_corr, ((float)tdoaParameters.tx_pwr_correction)/2.f);
    tdoaParameters.tx_pwr_level_comp = tdoaParameters.tx_pwr_level_corr;
}


void initialize_mac_headers(uint64_t* MAC_addr)
{
    //initialize values of all mac_header_ba's members
    mac_header_ba.fcode = UWB_FCODE_BC_ACK;
    mac_header_ba.BC_version = BC_VERSION;
    mac_header_ba.seqNum = seqNum;
    memcpy((void*)&mac_header_ba.BC_ack_num,(void*)&bc_ack_num,sizeof(mac_header_ba.BC_ack_num));
    memcpy((void*)&mac_header_ba.MAC_addr,(void*)MAC_addr,sizeof(mac_header_ba.MAC_addr));

    //initialize values of all mac_header_bb's members
    mac_header_bb.fcode = UWB_FCODE_BLINK;
    mac_header_bb.seqNum = seqNum;
    memcpy((void*)&mac_header_bb.MAC_addr,(void*)MAC_addr,sizeof(mac_header_bb.MAC_addr));

    //initialize values of all mac_header_bc's members
    mac_header_bc.fcode = UWB_FCODE_BC_POLL;
    mac_header_bc.BC_version = BC_VERSION;
    mac_header_bc.seqNum = seqNum;
    mac_header_bc.BC_T_toRX = tdoaParameters.bc_tx2rx_time;
    memcpy((void*)&mac_header_bc.BC_ack_num,(void*)&bc_ack_num,sizeof(mac_header_bc.BC_ack_num));
    memcpy((void*)&mac_header_bc.MAC_addr,(void*)MAC_addr,sizeof(mac_header_bc.MAC_addr));
}

/**
 * @brief Initialize constant values in extended info blink message.
 * @details Some content of extended info blink message is invariant in time. Therefore, they are inserted into infoblink during initialization.
 * @param sysVal - System values, some of them should be send in infoblinks.
 */
void init_tag_info_payload(void)
{

    info_msg.fw_ver[0] = FW_VERSION;
    info_msg.fw_ver[1] = FW_SUBVER;
    info_msg.fw_ver[2] = FW_REVISION;
    info_msg.hw_ver[0] = HW_VERSION;
    info_msg.hw_ver[1] = HW_REVISION;
    info_msg.platform = systemValues.platform;

//
// copy user config settings into info blink
//
    info_msg.channel = tdoaParameters.channel;
    info_msg.data_rate = tdoaParameters.data_rate;
    info_msg.preamble = tdoaParameters.preamble;
    info_msg.prf = tdoaParameters.prf;
    info_msg.preamCode = tdoaParameters.preamCode;
    info_msg.nSfd = tdoaParameters.nSfd;
    info_msg.random_dev =	tdoaParameters.use_random_deviation;
    info_msg.mcr = tdoaParameters.motion_control_mode;
    info_msg.mounted_sensors = 0x00;


    if(sensors_params.ACC_mounted)
        info_msg.mounted_sensors |= 1 ;
    else
    info_msg.mounted_sensors &= ~(1);


    if(sensors_params.GYRO_mounted)
        info_msg.mounted_sensors |= (1 << 1);
    else
    info_msg.mounted_sensors &= ~(1 << 1);


    if(sensors_params.MAG_mounted)
        info_msg.mounted_sensors |= (1 << 2);
    else
    info_msg.mounted_sensors &= ~(1 << 2);


    if(sensors_params.BARO_mounted)
        info_msg.mounted_sensors |= (1 << 3);
    else
    info_msg.mounted_sensors &= ~(1 << 3);


    info_msg.active_sensors = sensors_params.EBlink_cont;	                //in EBlink_cont is stored which sensors are active
    if(sensors_params.AHRS_enable!=0) info_msg.active_sensors = 0x80;		//MSB of EBlink_cont is high when AHRS is enable

    memcpy((void*)info_msg.refresh_interval,(void*) &tdoaParameters.refresh_rate_ms,sizeof(info_msg.refresh_interval));
    memcpy((void*)info_msg.TXpower,(void*) &tdoaParameters.tx_pwr_level_corr,sizeof(info_msg.TXpower));

    info_msg.IMU_FS_range = (sensors_params.gyro_FS << 1) | (sensors_params.acc_FS >> 3);

    uint16_t mcr_threshold = 0;

    if(sensors_params.wakeup_threshold >> 14) mcr_threshold = (sensors_params.wakeup_threshold >> 14) - 1;
    else mcr_threshold = sensors_params.wakeup_threshold & ~PREDEFINED_THRESHOLD_MASK;

    memcpy((void*)info_msg.mcr_threshold,(void*) &mcr_threshold,sizeof(mcr_threshold));

    info_msg.BARO_setting = sensors_params.BARO_setting;

    memcpy((void*)info_msg.sm_refresh_interval,(void*)&tdoaParameters.no_motion_refresh_rate,sizeof(info_msg.sm_refresh_interval));

    info_msg.bc_version = BC_VERSION;
    info_msg.bc_period_ri = tdoaParameters.bc_period_ri;
    info_msg.bc_period_rism = tdoaParameters.bc_period_rism;

}

/**
 * @brief Write UWB parameters to DWM module
 *
 * @param System values
 * @return error state
 */
uint32_t DWM_reconfigure(void)
{
    uint32_t ret=0;
    // 구조체 초기화
    dwt_config_t dwt_Config;
    dwt_txconfig_t dwt_Txconfig;
    // DW3000 기본 설정
    dwt_Config.phrMode    = DWT_PHRMODE_STD;
    dwt_Config.phrRate    = DWT_PHRRATE_STD;
    dwt_Config.stsMode    = DWT_STS_MODE_OFF;
    dwt_Config.stsLength  = DWT_STS_LEN_64;
    dwt_Config.pdoaMode   = DWT_PDOA_M0;
    dwt_Config.sfdType    = 0;

    // SFD Timeout 설정 (기본값 사용)
    dwt_Config.sfdTO = DWT_SFDTOC_DEF;

    // 채널 설정 및 유효성 검사 (5 또는 9만 지원)
    dwt_Config.chan = tdoaParameters.channel;
    if (dwt_Config.chan != 5 && dwt_Config.chan != 9) {
        return 1;
    }

    // 데이터 전송 속도 설정
    dwt_Config.dataRate = tdoaParameters.data_rate;

    // 프리앰블 길이 및       PAC 크기 설정
    dwt_Config.txPreambLength = preambleCfg[tdoaParameters.preamble];
    switch (dwt_Config.txPreambLength) {
    /*
        case DWT_PLEN_4096:
        case DWT_PLEN_2048:
        case DWT_PLEN_1536:
            dwt_Config.rxPAC = DWT_PAC64;//del dw3000
            break;
    */
        case DWT_PLEN_1024:
            dwt_Config.rxPAC = DWT_PAC32;
            break;
        case DWT_PLEN_512:
        case DWT_PLEN_256:
            dwt_Config.rxPAC = DWT_PAC16;
            break;
        case DWT_PLEN_128:
        case DWT_PLEN_64:
            dwt_Config.rxPAC = DWT_PAC8;
            break;
        default:
            return 1;
    }

    // PRF 설정은 rxCode/txCode 값으로 자동 유도되므로 구조체에 따로 설정하지 않음
    // static const uint8_t prfCfg[2] = {DWT_PRF_16M, DWT_PRF_64M};
    // dwt_Config.prf = prfCfg[tdoaParameters.prf];
    // ※ DW3000에서 PRF는 txCode/rxCode로부터 유도됨
    //   - 코드 1~8   → PRF 16 MHz
    //   - 코드 9~24 → PRF 64 MHz
    //   - 코드 25~32는 STS 전용

    // 프리앰블 코드 설정
    dwt_Config.txCode = tdoaParameters.preamCode;
    dwt_Config.rxCode = tdoaParameters.preamCode;

    // 비표준 SFD 여부 설정
    dwt_Config.sfdType = tdoaParameters.nSfd;

    // PHY 설정 적용
    if (dwt_configure(&dwt_Config) != DWT_SUCCESS) {
        return 1;
    }

    // PG Delay 설정 (DW3000용  chnnel 5/9
    const uint8_t txPGdelayConfig[10] = 
    {
      0x00, // Channel 0 (placeholder, 미사용)
      0x00, // Channel 1 (미지원)
      0x00, // Channel 2 (미지원)
      0x00, // Channel 3 (미지원)
      0x00, // Channel 4 (미지원)
      0x34,//0xD0,//0x34,org // Channel 5: 권장값 (약 52) → 송신 파형 최적화용
      0x00, // Channel 6 (placeholder)
      0x2A, // Channel 7: 권장값 (약 42)
      0x00, // Channel 8 (미지원)
      0x26  // Channel 9: 권장값 (약 38)
    };

    if(tdoaParameters.tx_PG_delay == 0UL)
    {
          dwt_Txconfig.PGdly = txPGdelayConfig[dwt_Config.chan];         //if value from OTP is not correct use predefined value of PG delay
    }
    else
    {
        dwt_Txconfig.PGdly = tdoaParameters.tx_PG_delay;                //else use PG delay that was readed from OTP memory
    }

    // Smart TX Power 설정 (6.8Mbps에서만 사용 권장)
    //dwt_setsmarttxpower(dwt_Config.dataRate == DWT_BR_6M8 ? 1 : 0);//del

    // 전송 전력 설정
    dwt_Txconfig.power = tdoaParameters.tx_pwr_level_corr;
    dwt_Txconfig.PGcount = 0;         //add dw3000 khs
    // TX RF 설정 적용
    dwt_configuretxrf(&dwt_Txconfig);
    nrf_delay_ms(1);
    uint32_t written_power = dwt_read32bitoffsetreg(TX_POWER_ID, 0);
    if (written_power != dwt_Txconfig.power) {
      return 1;  // 설정 실패로 판단
    }
     

    return ret;
}

#define BC_PREAMBLE_TIMEOUT_US   (50)

/**
* @brief Set preamble timeout in DW1000. Once the RX is enabled, it is automatically disabled after defined timeout, in case of any premble symbol is detected.
* @details The timeout will be rounded up  to multiple of PAC time. One PAC take 8.15us at RF5 setting.
* @param[in] timeout_us - Preamble hunt timeout in microseconds.
*/
void dw_set_pream_timeout(uint32_t timeout_us)
{
    uint16_t timeout_PACs = (uint16_t)ceil((float)timeout_us / 8.15f);    //calculate timeout in number of PACs. 8.15 is one PAC time in us.
    if(timeout_PACs != 0) timeout_PACs -=1;                               //the timeout must be decremented by 1, because the radio add 1PAC automatically
    dwt_setpreambledetecttimeout(timeout_PACs);
}

void init_radio_timeouts(void)
{
   dw_set_pream_timeout(BC_PREAMBLE_TIMEOUT_US);
}

/**
 * @brief Enable Interrupt Request from UWB radio chip - INT pin.
 */
void UWB_IRQ_enable(void)
{
	nrf_drv_gpiote_in_event_enable(DW3000_IRQ_Pin, true);
	ext_irq_en.dw_irq = true;
}

/**
 * @brief This function send UWB parameters into DWM module via SPI.
 */
void configure_UWB(bool set_bc_timeout)
{
    SPI_pins_enable();
    wake_up_radio();                ///wake-up the UWB radio
    DWM_reconfigure();				//set UWB radio parameters
    dwt_configuresleep(DWT_CONFIG | DWT_PGFCAL ,DWT_PRES_SLEEP | DWT_WAKE_CSN | DWT_SLP_EN);//khs_add
    if(set_bc_timeout)  init_radio_timeouts();
    GPIO_IRQ_enable();              /// enable interrupt request from external sources, connected via GPIOs
    UWB_IRQ_enable();				///enable interrupt request from UWB radio chip
}

#if 0
/***
 * waitForConf
 *
 * @param sysconfbits
 * @param accInited
 */
bool waitForConf(uint32_t sysconfbits, systemValues_t *sysVal, bool rx_after_start)
{
    bool ret = false;

    uint32_t rx_on_time_s = 0;
    uint32_t rx_on_time_tcks = 0;
    uint32_t rx_off_time = 0;
    uint32_t max_duration = 0;
    uint8_t num_of_rx_periods =0;

    if(rx_after_start) rx_on_time_s = RX_ON_TIME_LIPO_BATT * 1000UL;
    num_of_rx_periods = 1;
    max_duration = rx_on_time_s*2;

    rx_on_time_tcks = convertTime2Ticks(rx_on_time_s);

    bool wdog_state = wdog_get_state();
    if(wdog_state)
    {
    watchdog_set_timeout(max_duration);
    }
    WDOG_enable(false);
    //disable transceiver
    dwt_forcetrxoff();

    //reconfigure the sysconfbits in DWM to proper receive operation
    dwt_write32bitreg(SYS_CFG_ID, sysconfbits);

    //set the callback for the interrupt handling, which is used to the sync waiting
    dwt_setcallbacks(Dwt_Txcallback,Dwt_Rxcallback_conf, Dwt_Rxcallback_conf , Dwt_Rxcallback_conf);

    //implicit state
    confState = false;

    for(uint8_t i= 0; i < num_of_rx_periods; i++)
    {
        uint32_t RTC_timestamp = 0;
       // uint32_t RTC_stop_time = 0;

        //save actual RTC state
        RTC_timestamp = get_RTC_timestamp(true);

        //calculate the value of receive stop
        //RTC_stop_time = RTC_timestamp + rx_on_time_tcks;


        //enable receiver
        dwt_rxenable(0);

        WDOG_Feed();

        while((get_RTCelapsedCount(RTC_timestamp)<rx_on_time_tcks)&& (confState == false))
        {

            //RTC_timestamp = get_RTC_timestamp(true);
        }

        // turn off the radio
        dwt_forcetrxoff();

        if (confState == true) break;

        delay_and_sleep(rx_off_time, false, true);         //tag with CR battery need some time to recovery battery before next RX state
    }

    //dwt_setcallbacks(Dwt_Txcallback,Dwt_Rx_OK_callback, Dwt_Rx_T_OUT_callback , Dwt_Rx_ERR_callback);
    dwt_setcallbacks(Dwt_Txcallback,Dwt_Rx_OK_callback, Dwt_Rx_T_OUT_callback , Dwt_Rx_ERR_callback,NULL,NULL);



    if (confState) //if new config was received
    {
            uint8_t conf_result = CheckAndCopyRxToTDOA(&confRxBuff);
            if (conf_result == CONF_OK)
            {
                    //ok
                    if (DWM_reconfigure() == CONF_OK)
        {
            if(wdog_state)
            {
                watchdog_set_timeout(2000UL*1000UL);
                WDOG_Feed();
            }
                            ret = true;
                    }
                    else
                    {
                            //err
                            do_SWResetMCU(36);
                    }
            }
            else
            {
                    //err
                    do_SWResetMCU(35);
            }
    }
    dwt_setcallbacks(Dwt_Txcallback,Dwt_Rx_OK_callback, Dwt_Rx_T_OUT_callback , Dwt_Rx_ERR_callback);

    if(wdog_state)
    {
            watchdog_set_timeout(WDOG_INTERVAL_DURING_FW_INIT);
            WDOG_Feed();
    }
    return ret;
}
#endif
/**
 * @brief Function for set the DWM into RX state and wait for new config, if new config is received its chcket if is not corrupted and stored
 * @return Return true if new config was received
 */
bool setUWB_RX_and_wait(systemValues_t *sysVal, bool rx_after_start)
{
	uint32_t sysconfbits = dwt_read32bitoffsetreg(SYS_CFG_ID, 0);
	//dwt_enableframefilter(DWT_FF_DISABLE);//DWT_FF_NOTYPE_EN=0x000 //disable frame filtering
        dwt_configureframefilter(DWT_FF_DISABLE, 0);

	//bool ret = (waitForConf(sysconfbits, sysVal, rx_after_start));
	//return ret;
        bool ret = false;
        return ret;
}

/**
 * @brief This function check if UWB RX is required and if yes, set radio to RX mode and wat for new config.
 */
bool RX_handler(bool first_rx)
{
    WDOG_Feed();
    //periodical RX and automatic RX after start is only applicable for TAG with li-ion/li-pol battery or TAG with external power supply

    if(first_rx)														//rx after start
    {
        uint32_t RX_timestamp = get_RTC_timestamp(true);				//set timestamp of last RX state
        if(setUWB_RX_and_wait(&systemValues,first_rx))					//set radio to TX state and wait for config message
        {
            tdoaTiming.last_RX_timestamp = get_RTC_timestamp(true);
            return true;
        }
        else tdoaTiming.last_RX_timestamp = RX_timestamp;
        return false;
    }


    return false;
}
//-----------------------------------------------------------------------------tx_prepare
received_payload_t received_payload;
tx_payload_buffer_t tx_payload_buff;
/**
 * @private
 * @brief The function that prepare application response as TLV and store it to the defined address.
 * @param buffer - Pointer to the address where will by data stored
 * @param app_id - 16-bites identification number of the application that prepared the response.
 * @param d_length - Length of data.
 * @param data - Pointer to the address where the data that should be send as response is prepared.
 */
static bool write_to_date_req_buffer(data_requests_struct_t* buffer,uint16_t app_id, uint8_t d_length, uint8_t* data, data_confirm_cb_t confirm_cb)
{

    if(buffer->length <= sizeof(buffer->data))
    {
        buffer->app_id = app_id;
        buffer->length = d_length;
        memcpy((void*)&buffer->data,(void*)data,buffer->length);
        buffer->rdy = true;
        buffer->data_confirm_cb = confirm_cb;
        return true;
    }
    return false;
    
}

/**
 * @private
 * @brief The function find the free slot in the response buffer and return its address as parameter @p respond_struct.
 * @param respond_struct - using this parameter the function return address of free slot in respond buffer.
 * @return Function return number of data bytes that can be stored to the slot in the response buffer. It return NULL if there is not free slot in the response buffer.
 */
static uint8_t get_resp_buffer(const int8_t priority, data_requests_struct_t** respond_struct)
{
    for(uint8_t i = 0; i < MAX_NUM_OF_APPS; i++)
    {
        if(app_data_storage[priority][i].rdy == false)
        {
            *respond_struct = &app_data_storage[priority][i];
            return sizeof(app_data_storage[priority][i].data);
        }
    }
    respond_struct = NULL;
    return 0;
   
}

/**
 * @brief This function ensures send of the data. Data are copied (deep copy) to the data buffer from which it is sent in the next UWB frame.
 * @param app_id - 16-bites identification number of the application that call the data request function.
 * @param len - Length of the data in bytes.
 * @param data - Pointer to the address where the data are prepared.
 * @param priority - Priority of the data request. The priority define in what order will be the data from buffer send.
 * @param confirm_cb - Confirmation callback - the cb function that will be called just after data request will be done.
 * @return True if the data were successfully copied into the request buffer. False if, there is no enough space in the buffer.
 */
bool Data_Req(uint16_t app_id, uint8_t len, uint8_t* data,  uint8_t priority, data_confirm_cb_t confirm_cb)
{
    data_requests_struct_t* data_req_buffer;
    uint8_t max_data_len = get_resp_buffer(priority, &data_req_buffer);

    if(max_data_len >= len)
    {
        if(write_to_date_req_buffer(data_req_buffer, app_id, len ,data,confirm_cb))
    {
        return true;
    }
    else return false;
    }
    else return false;
}


/**
 * @brief Prepare info msg that will be sent to server.
 *
 */
static bool prepare_tag_info_payload(void)
{
    bool ret = true;
    info_msg.battVoltage = systemValues.actual_batt_voltage_raw;

    info_msg.AHRS_representation = (sensors_params.AHRS_enable & 0x0F);			//only low nibble represent AHRS representation
    info_msg.AHRS_representation |= ((uint8_t)sensors_params.MAG_just_calib) << 7;	//highest bit in 'tx_uwb_message_tag_eblink_info.AHRS_representation' is high just after calibration of magnetometer
    info_msg.AHRS_representation |= ((uint8_t)sensors_params.IMU_just_calib) << 6;	//second highest bit in 'tx_uwb_message_tag_eblink_info.AHRS_representation' is high just after calibration of IMU

    //-------------------------------------------
    // Write IBlink to response buffer
    memcpy(info_msg.TXpower, &tdoaParameters.tx_pwr_level_conf, sizeof(info_msg.TXpower));//add khs
    //-------------------------------------------
    ret = Data_Req(MSGTYPE_TAG_INFO, sizeof(info_msg), (uint8_t*)&info_msg, INFO_MSG_PRIORITY, NULL);
    return ret;
}

/**
 * @brief Prepare battery voltage as msg that will be sent to server.
 *
 */
static bool prepare_battery_payload(void)
{
    bool ret = true;
    battery_msg_t batt_msg;

    batt_msg.battVoltage = systemValues.actual_batt_voltage_raw;   //put battery voltage to the batt payload

    // Write message to response buffer
    ret = Data_Req(MSGTYPE_BATT, sizeof(batt_msg), (uint8_t*)&batt_msg, BATT_MSG_PRIORITY, NULL);
    return ret;
}


/**
 * @brief Prepare next blink.
 * @param systemValues
 * @return blink_length in number of bytes
 */
void prepare_system_apps_response(void)
{
  //battery message and info message counters
    static uint8_t batt_msg_cnt = BATT_MSG_PERIOD;
    static uint8_t info_msg_cnt = INFO_MSG_PERIOD-1;

    uint8_t msgType = 0;

    if(tdoaValues.on_start_info_msg)
    {
      tdoaValues.on_start_info_msg--;
      prepare_tag_info_payload();
      msgType = MSGTYPE_TAG_INFO;
      if(!tdoaValues.on_start_info_msg)
      {
          sensors_params.IMU_just_calib = false;				//just after calib bites are set only in 3 infoblinks after calibration
          sensors_params.MAG_just_calib = false;				//just after calib bites are set only in 3 infoblinks after calibration
      }
    }

    else if(batt_msg_cnt == 0)
    {
        batt_msg_cnt = BATT_MSG_PERIOD;
        if(info_msg_cnt == 0)
        {
            info_msg_cnt = INFO_MSG_PERIOD;
            prepare_tag_info_payload();
            msgType = MSGTYPE_TAG_INFO;
        }
        else
        {
            msgType = MSGTYPE_BATT;
        }
        info_msg_cnt--;                //info blink period is based on battery blink period
    }

    if(msgType == MSGTYPE_BATT)
    {
        prepare_battery_payload();
    }
    /*
    if(CHG_active && (systemValues.platform != TEIA_CAR)  && (systemValues.platform != TEIA_TOOL))    //if charging is active, the baro data must be put into blink - but its not appliable for TEIA tags if(CHG_active && (systemValues.platform != TEIA_CAR)  && (systemValues.platform != TEIA_TOOL))    //if charging is active, the baro data must be put into blink - but its not appliable for TEIA tags
    {
        prepare_sensors_payload(true);
    }
    if(sensors_params.EBlink_cont)
    {
        prepare_sensors_payload(false);
    }
    else if(sensors_params.AHRS_enable==1) prepare_angles_payload();
    else if(sensors_params.AHRS_enable==2) prepare_quaternion_payload();
    else if(sensors_params.AHRS_enable==3)
    {
        prepare_quaternion_payload();
        prepare_sensors_payload(true);
    }
    */

    batt_msg_cnt--;
    return;
}

void get_tx_payload_buff_p(tx_payload_buffer_t** tx_payload_p)
{
    *tx_payload_p = &tx_payload_buff;
}

void get_rx_payload_p(received_payload_t** rx_payload)
{
    *rx_payload = &received_payload;
}


/**
 * @private
 * @brief This function check all slots in data request buffer and push all prepared data into the payload_buffer, that will be added into the next UWB frame.
 */

void process_prepared_data(void)
{
    uint8_t d_confirm_cb_i = 0;
    uint8_t num_of_unprocessed_msg = 0;
    uint8_t msg_type = UNIV_MSG_TYPE;
    tx_payload_buffer_t* payload_buff_p;
    
    get_tx_payload_buff_p(&payload_buff_p);
    payload_buff_p->length = 0;
    
    payload_buff_p->data[0] = msg_type;
    payload_buff_p->length += sizeof(msg_type);

    for(int8_t prio = DATA_MIN_PRIORITY; prio < DATA_MAX_PRIORITY; prio++)
    {
        for(uint8_t i = 0; i < MAX_NUM_OF_APPS; i++)//for(uint8_t i = 0; i <= MAX_NUM_OF_APPS; i++)??
        {
            if(app_data_storage[prio][i].rdy)
            {
                size_t remaining_space = sizeof(payload_buff_p->data)- payload_buff_p->length;
                size_t length_of_app_data = app_data_storage[prio][i].length + sizeof(app_data_storage[prio][i].length) + sizeof(app_data_storage[prio][i].app_id);
                if(length_of_app_data <= remaining_space)
                {
                    memcpy((void*)&payload_buff_p->data[payload_buff_p->length], (void*)&app_data_storage[prio][i].app_id, length_of_app_data);
                    payload_buff_p->length += length_of_app_data;
                    payload_buff_p->rdy = true;
                    data_confirm_cb_arr[d_confirm_cb_i++] = app_data_storage[prio][i].data_confirm_cb;        //register data confirmation table
                    app_data_storage[prio][i].rdy = false;
                }
                else num_of_unprocessed_msg++;
            }
        }
    }
}
//************************************************************************
//**************************************************************************************
float TX_gain_CompensatePowerByTemperature(float actualUWBTemp, float calibUwbTemp, float coefTemp)
{

	float tempOffsetDB;

	// Calculate offset in dB
	tempOffsetDB = (actualUWBTemp - calibUwbTemp) * coefTemp;

    // Return calculated offset
	return tempOffsetDB;
}

//**************************************************************************************
float uwbCorrection_getCompensatePowerByDataLength(uint8_t calibLen,uint8_t txFrameLength, float calibFrameTimeUs, float oneByteTimeUs)
{
	float tempOffsetDB = 0.f;
	float t_tranmissionUs;
	int8_t diffLen;

	// Calculate differente calib and frame length
	diffLen = txFrameLength - calibLen;

	// Do nothing with diff length equal 0
	if(diffLen != 0)
    {
		t_tranmissionUs = calibFrameTimeUs + (float)(txFrameLength - calibLen) * oneByteTimeUs;

		// Basic equation
		//offsetDB = 10.0*log10f(t_calib) - 10.0*log10f(t_tranmission);

		// Simplify Basic equation
		//offsetDB = 10.0 * ( log10f(t_calib) - log10f(t_tranmission) );

		// Math upgrade of "Simplify Basic equation" by WolframAlpha to reduce number log functions
		// koefEqLen is part of equation pre-calculate at start up of Anchor

		tempOffsetDB = 10.f * log10f(calibFrameTimeUs / t_tranmissionUs);
	}
	// Return calculated offset
	return tempOffsetDB;
}

float read_temperature(void)
{
    //extern volatile int8_t teplota;         //debug variable
    int16_t T_raw = 0;
    float T_celsius = 0.f;
    int16_t ref_temp_degC = ref_local.temp;

    T_raw = dwt_readwakeuptemp();
    T_celsius = (float)(T_raw-otp_refs.tempP) * 1.14f+ (float)ref_temp_degC;

    //teplota = round(T_celsius);

    return T_celsius;
}

float convert_raw_to_volts(uint8_t voltage_raw, uint8_t platform)
{
    float voltage_v = 0;
    uint32_t temp = 0;

    temp = (uint32_t)voltage_raw * 2500UL;
    voltage_v = (float)temp / 128;
    
    return voltage_v;
}

float calc_temperature_compensation_of_TXgain(uint8_t uwbChannel,bool compensateNow)
{
    static float old_DW_temperature = NAN;
    static float gain_offset = 0.f;
    float DW_temperature;

    if(compensateNow)
    {
        DW_temperature = read_temperature();

        if(DW_temperature != old_DW_temperature)
        {
            gain_offset = TX_gain_CompensatePowerByTemperature(DW_temperature, ref_local.temp, compCoefTemp[uwbChannel]);
            old_DW_temperature = DW_temperature;
        }
    }
    return gain_offset;
}

float calc_packet_lenth_compensation_of_TXgain(uint8_t uwb_packet_len, uint8_t RF_profile)
{
    static uint8_t old_packet_len = 0;
    static float gain_offset = 0.f;

    if(uwb_packet_len != old_packet_len)
    {
        gain_offset = uwbCorrection_getCompensatePowerByDataLength(REFERENCE_UWB_PACKET_LENGTH, uwb_packet_len, refFrameTxTimeUs[tdoaParameters.RF_profile], oneByteTxTimeUs[tdoaParameters.RF_profile]);
        old_packet_len = uwb_packet_len;
    }
    return gain_offset;
}

float volt_comp(uint8_t supply_voltage_raw, uint8_t platform)
{

     static float gain_offset = 0.f;
     static uint8_t supply_voltage_raw_old = 0;


     if(supply_voltage_raw != supply_voltage_raw_old)
    {
       float supply_voltage_v;
       int8_t supply_voltage_dv;


       supply_voltage_v= convert_raw_to_volts(supply_voltage_raw,platform);
       supply_voltage_dv = (int8_t)ceilf(supply_voltage_v/100.f);

       if(supply_voltage_dv > ref_voltage_dv) supply_voltage_dv = ref_voltage_dv;         //platforms Leonardo Persona/iMU/Vehicle is equipped with 3.3V LDO
       
       gain_offset= V_comp_factor[supply_voltage_dv];
       supply_voltage_raw_old= supply_voltage_raw;
    }
     return gain_offset;

}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_configuretxgain()
 *
 * @brief This function provides the API for the configuration of the TX the power  The input is a tx_gain in DW format.
 *
 * input parameters
 * @param tx_gain    -   tx gain in DW1000 format
 *
 * output parameters
 *
 * no return value
 */
void dwt_configuretxgain(uint32_t tx_gain)
{
    // Configure TX power
    dwt_write32bitreg(TX_POWER_ID, tx_gain);
}


/**
 * @brief Write UWB parameters to DWM module
 *
 * @param TX gain in DW format

 */
void DWM_set_gain(uint32_t tx_gain_dw_form)
{
    //configure tx gain of dw1000 via spi
	dwt_configuretxgain(tx_gain_dw_form);
	return;
}

bool bc_period_counter(void)
{
    static uint8_t BC_cnt = 1;
    bool bc_now = false;
    uint8_t bc_period;


    if(ACC_sleep) bc_period = tdoaParameters.bc_period_rism;
    else bc_period = tdoaParameters.bc_period_ri;

    if(bc_period == 0) return false;

    if(BC_cnt >= bc_period)
    {
        BC_cnt = 1;
        bc_now = true;
    }
    else
    {
        BC_cnt++;
    }
    return bc_now;
}

void get_received_data_p(rx_data_t** rx_data_p)
{
    *rx_data_p = &rx_data;
}

void get_transmit_buffer_p(tx_data_t** tx_buffer_p)
{
    *tx_buffer_p = &tx_buffer;
}

void compensate_tx_gain(uint8_t uwb_packet_len)
{
    static float old_offset = 0.f;        //old value of tx gain offset
    float new_offset;                     //new calculated offset

    new_offset = 0.f;

    /*calculate offset of tx_gain based on supply voltage*/
    new_offset += volt_comp(systemValues.actual_batt_voltage_raw, systemValues.platform);

    /*calculate offset of tx_gain based on uwb packet length*/
    new_offset += calc_packet_lenth_compensation_of_TXgain(uwb_packet_len, tdoaParameters.RF_profile);


    if(dw_temperature_measure)
    {
        /*calculate offset of tx_gain based on uwb radio chip temperature*/
        new_offset += calc_temperature_compensation_of_TXgain(tdoaParameters.channel,true);
        dw_temperature_measure = false;
        shedule_time_of_next_measurement();
    }
    else
    {
        new_offset += calc_temperature_compensation_of_TXgain(tdoaParameters.channel,false);
    }


    if (new_offset != old_offset)
    {
        uint32_t OTP_gain = tdoaParameters.tx_pwr_level_corr;
        uint32_t new_gain = 0;
        float correction = (floorf(new_offset*2.f))/2.f;

        calc_TX_gain(&OTP_gain, &new_gain, correction);             //TODO add debug controll of return value from this function

        tdoaParameters.tx_pwr_level_comp = new_gain;
        DWM_set_gain(new_gain);
        old_offset = new_offset;

    }
}
static void update_mac_header(uint8_t frameType)
{
    if(frameType == UWB_FCODE_BC_ACK)        //if fcode == 0xBA
    {
        mac_header_ba.seqNum = seqNum++;                                                                //increment seqNum and put new value int MAC header
        memcpy((void*)&mac_header_ba.BC_ack_num,(void*)&bc_ack_num,sizeof(mac_header_ba.BC_ack_num));   //copy current value of bc_ack_numeber to mac header
    }
    else if(frameType == UWB_FCODE_BLINK)    //if fcode == 0xBB
    {
        mac_header_bb.seqNum = seqNum++;     //increment seqNum and put new value int MAC header

    }
    else if(frameType == UWB_FCODE_BC_POLL)  //if fcode == 0xBC
    {
        mac_header_bc.seqNum = seqNum++;                                                                //increment seqNum and put new value int MAC header
        memcpy((void*)&mac_header_bc.BC_ack_num,(void*)&bc_ack_num,sizeof(mac_header_bc.BC_ack_num));   //copy current value of bc_ack_numeber to mac header
    }
}

static int8_t add_header_to_frame(uint8_t frameType,uint8_t* MAC_frame)
{
    size_t header_length = 0;

    if(frameType == UWB_FCODE_BC_ACK)        //if fcode == 0xBA
    {
        memcpy((void*)MAC_frame, (void*)&mac_header_ba, sizeof(mac_header_ba));   //copy MAC header to MAC frame
        header_length = sizeof(mac_header_ba);
    }
    else if(frameType == UWB_FCODE_BLINK)    //if fcode == 0xBB
    {
        memcpy((void*)MAC_frame, (void*)&mac_header_bb, sizeof(mac_header_bb));   //copy MAC header to MAC frame
        header_length = sizeof(mac_header_bb);

    }
    else if(frameType == UWB_FCODE_BC_POLL)  //if fcode == 0xBC
    {
        memcpy((void*)MAC_frame, (void*)&mac_header_bc, sizeof(mac_header_bc));   //copy MAC header to MAC frame
        header_length = sizeof(mac_header_bc);
    }
    return header_length;
}


static uint8_t add_mark_to_frame(uint8_t mark,uint8_t* MAC_frame, uint8_t offset)
{
    if(mark)
    {
        MAC_frame[offset] = mark;
        return sizeof(mark);
    }
    else return 0;
}

static int8_t add_prepared_payload_to_frame(uint8_t* MAC_frame,uint8_t offset)
{
   uint8_t data_len;
   if(tx_payload_buff.rdy)
   {
       data_len = tx_payload_buff.length;
       memcpy((void*)(&MAC_frame[offset]),(void*)tx_payload_buff.data,data_len);
       tx_payload_buff.rdy = false;
       tx_payload_buff.length = 0;

       return data_len;
   }
   return 0;
}

uint8_t compose_MAC_frame(uint8_t frameType, uint8_t mark)
{
    tx_data_t *tx_buffer_p;
    uint8_t frame_size = 0;  //MAC frame is empty, so frame size must be set to zero

    get_transmit_buffer_p(&tx_buffer_p);
    //update_MAC_header
    update_mac_header(frameType);

    //copy MAC header into frame which is preparing
    frame_size += add_header_to_frame(frameType, (uint8_t*)&tx_buffer_p->tx_data);

    //add special mark to the frame - it signalize that the tag is in sleep mode or the tag is charged.
    frame_size += add_mark_to_frame(mark, (uint8_t*)&tx_buffer_p->tx_data, frame_size);

    //copy prepared data (from system and apps) into frame that is preparing
    frame_size += add_prepared_payload_to_frame((uint8_t*)&tx_buffer_p->tx_data, frame_size);

    //calculate_crc16((uint16_t*)&(MAC_frame[frame_size]),MAC_frame,frame_size);
    //frame_size += SIZE_OF_CRC16;

    tx_buffer_p->tx_data_len = frame_size;
    tx_buffer_p->tx_data_rdy = true;

    return frame_size;
}

void set_wait4rest_time(uint32_t time)
{
    dwt_setrxaftertxdelay(time);
}

/**
 *
 * @brief Set UWB radio to TX mode and send prepared blink.
 */
static void start_UWB_TX(uint8_t delayed, uint8_t wait4resp)
{
    delayed &= 0x01;
    wait4resp <<= 1;
    wait4resp &= 0x02;
    dwt_starttx(delayed | wait4resp);
    //printf("tx_test_state = %d \n\r",dwt_starttx(delayed | wait4resp));
}
static void transmit_frame(bool delayed, bool wait4resp)
{
    /**send prepared blink*/
    start_UWB_TX((uint8_t)delayed,(uint8_t)wait4resp);
    
    while(!blink_sent)  __WFI();		//wait while sending is not complete

    blink_sent = false;
}

void transmit_prepared_frame(bool delayed, bool wait4resp)
{
    if(tx_buffer.tx_data_rdy)
    {
        dwt_writetxdata(tx_buffer.tx_data_len + CRC16_LENGTH,(uint8_t*)&tx_buffer.tx_data, 0); // write the frame data
        dwt_writetxfctrl(tx_buffer.tx_data_len + CRC16_LENGTH , 0, false);                  //write length of message

        compensate_tx_gain(tx_buffer.tx_data_len + CRC16_LENGTH);                          //calculate and set gain according to current supply voltage, dw chip temperature and uwb packet length
        transmit_frame(delayed, wait4resp);      //transmit the data
    }
    tx_buffer.tx_data_rdy = false;
}

uint8_t BC_receive_data(uint32_t tx2rx_delay)
{
    bool bc_data_received = false;
    rx_data_t* rx_data_p;

    //tx_timestamp = (dwt_readtxtimestamphi32());

	//dwt_setdelayedtrxtime(tx_timestamp + tx2rx_delay);

    rx_frame_received = false;
    dwt_rx_err.rx_error = false;
    dwt_rx_err.rx_timeout = false;

    //UWB_IRQ_disable();
    while(!rx_frame_received && !dwt_rx_err.rx_error && !dwt_rx_err.rx_timeout)
    {
        __WFI();
        //while(!GPIO_PinInGet(19));
        //dwt_isr();

    }
    //UWB_IRQ_enable();



    get_received_data_p(&rx_data_p);
    bc_data_received = rx_data_p->rx_data_rdy;


    return bc_data_received;
}

//*****************************************************
uint8_t process_recieved_frame(const uint8_t* frame, size_t frame_length, uint8_t* num_of_ack, bool* immediate_ack)
{
    uint8_t err_code = BC_DATA_NOK;
    uint8_t fcode = 0;

    fcode = frame[0];       //at first possition of the received frame should be fcode, that describe what kind of UWB frame it is

    if(fcode == UWB_FCODE_BC_DATA)
    {
        uint64_t destAddr;
        mac_header_bd_t* mac_header_p;

        mac_header_p = (mac_header_bd_t*)frame;
        memcpy((void*)&destAddr,(void*)&mac_header_p->MAC_addr,sizeof(mac_header_p->MAC_addr));
        destAddr &= MAC_ADDR_48b_MASK;

        if(destAddr == tdoaParameters.this_tag_MACaddress || destAddr == TAG_BROADCAST_ADDR)
        {
            size_t data_part_len;
            memcpy((void*)&bc_ack_num,(void*)&mac_header_p->BC_ack_num,sizeof(bc_ack_num));

            *immediate_ack = (mac_header_p->BC_options) & IMM_ACK_MASK;
            *num_of_ack = ((mac_header_p->BC_options) & NUM_OF_ACK_MASK) >> NUM_OF_ACK_OFFSET;

            data_part_len = frame_length-(sizeof(mac_header_bd)+sizeof(mac_footer));
            if(data_part_len > 0)
            {
                received_payload.rdy = true;
                received_payload.length = data_part_len;
                received_payload.data = (uint8_t*)&frame[sizeof(mac_header_bd)];
            }
            err_code = BC_DATA_OK;
        }
    }
    return  err_code;
}
/*
uint32_t dws_change_pream_length(uint8_t pream_len)
{
    //get current value of pdw1000local->txFCTRL
    uint32_t orig_val =  pdw3000local->txFCTRL;
    uint32_t temp = pdw3000local->txFCTRL;
    //change value of the txpsr and pe bits to pream_len value
    temp &= ~ (TX_FCTRL_TXPSR_PE_MASK);
    temp |= (uint32_t)pream_len << TX_FCTRL_TXPRF_SHFT;

    pdw3000local->txFCTRL = temp;

    return orig_val;
}

void dws_restore_pream_length(uint32_t txFCTRL_val)
{
    pdw3000local->txFCTRL = txFCTRL_val;
    dwt_write32bitreg(TX_FCTRL_ID, pdw3000local->txFCTRL);
}
*/

void transmit_imm_ack(void)
{
    //uint32_t tx_fctrl = dws_change_pream_length(DWT_PLEN_64);
    //inverted_transmit(false,false);
    transmit_prepared_frame(false,false);
    //dws_restore_pream_length(tx_fctrl);
}

void radio_routine(void)
{
    static uint8_t remaining_acks = 0;                // 아직 처리하지 않은        ACK 개수
    bool immediate_ack = 0;                           // 즉시   ACK를 전송할지 여부
    bool bc_now = 0;                                  // 현재 주기가      Backchannel 주기인지 여부
    bool bc_data_received = false;                    // Backchannel 수신 여부
    uint8_t acc_chg_mark = 0;                         // 충전 또는 모션 상태 마크

    if(CHG_active)                                    // 충전 중이면 마크 설정
        acc_chg_mark = CHARGING_MARK;
    else if(ACC_sleep)                                // 움직임이 없으면 마크 설정
        acc_chg_mark = NO_MOTION_MARK;

    bc_now = bc_period_counter();                     // 현재  Backchannel 주기인지 확인
    //SPI_pins_disable();//khs_wakeup_test
    DMW1000_wake_up_test();//test
    //test
    //wake_up_radio();                                  // 라디오를 웨이크업 (슬립 → 동작 상태)

    if(bc_now)                                        // Backchannel 동작 수행
    {
        uint8_t mac_frame_len;
        uint16_t tx_duration_us;
        uint16_t rx2tx_time_us;

        mac_frame_len = compose_MAC_frame(UWB_FCODE_BC_POLL, acc_chg_mark); // POLL 프레임 구성

        tx_duration_us = (uint16_t)((float)mac_frame_len * DATA_BYTE_DURSTION_US)
                         + PREAM_128_DURATION_US + SFD_DURATION_US;          // TX 예상 시간 계산

        rx2tx_time_us = tdoaParameters.bc_tx2rx_time * 10 - tx_duration_us + PREAM_64_DURATION_US;
        rx2tx_time_us = (uint16_t)((float)rx2tx_time_us / 1.024f);           // RX 전환 타이밍 계산

        set_wait4rest_time(rx2tx_time_us);            // RX 대기 타이밍 설정

        transmit_prepared_frame(false, true);         // POLL 프레임 전송

        bc_data_received = BC_receive_data(rx2tx_time_us);  // 수신 대기 및 수신 시도

        if(bc_data_received)                          // 수신 성공 시
        {
            rx_data_t* rx_data_p;

            get_received_data_p(&rx_data_p);          // 수신 버퍼 포인터 획득

            process_recieved_frame(                   // 수신 프레임 파싱 및 처리
                (const uint8_t*)&rx_data_p->rx_data_buffer,
                rx_data_p->rx_data_len,
                &remaining_acks,
                &immediate_ack
            );

            rx_data_p->rx_data_rdy = false;           // 수신 완료 → 플래그 초기화

            if(remaining_acks && immediate_ack)       // 즉시 ACK 전송 조건
            {
                compose_MAC_frame(UWB_FCODE_BC_ACK, acc_chg_mark); // ACK 프레임 구성
                transmit_imm_ack();                                // 즉시  ACK 전송
                remaining_acks--;
            }
        }
    }
    else                                              // 일반 주기 (Blink 또는 남은 ACK 전송)
    {
        if(remaining_acks)                            // 이전에 처리하지 못한 ACK가 있다면
        {
            remaining_acks--;                         // 전송 생략하고 카운트만 감소
        }
        else
        {
            compose_MAC_frame(UWB_FCODE_BLINK, acc_chg_mark); // Blink 프레임 구성
        }

        transmit_prepared_frame(false, false);        // Blink 또는 ACK 프레임 전송
    }

    //enter_radio_sleep();                              // 라디오를 슬립 모드로 전환
}

void uwb_msg_trx(void)
{
    radio_routine();
}

/**
 * @brief The confirmation callback is called just after sending the UWB frame. The callback is registered in Data_Req() function.
 */

void Data_Conf(void)
{
    /*call all registered data confirmation callbacks*/
    for(uint8_t cb_i=0; cb_i < (sizeof(data_confirm_cb_arr)/sizeof(data_confirm_cb_arr[0])); cb_i++)
    {
         //check if the app with this index is registered in running_apps array
        if(data_confirm_cb_arr[cb_i] != NULL)
        {
            //call registered callback
            data_confirm_cb_arr[cb_i]();
            //clear callback from the table
            data_confirm_cb_arr[cb_i] = NULL;
        }
    }
    /*clear callback table*/
}


void DMW1000_wake_up_test(void)
{
    SPI_pins_enable();                          //enable SPI pins, SPI_CS is used during wakeup
    int_flag.DW_XTAL_stable = nrf_gpio_pin_read(DW3000_RST_Pin);     //check state of RESET pin, if its high, the radio is in the IDLE mode
    GPIO_IRQ_enable();                              //enable interrupts from GPIOs
    DW_RST_IRQ_enable();                            //enable iRQ from DW reset pin - it goes high once the XTAL is stable and the radio is in IDLE mode
    DW_CS_CLR();                                //wake up the DW radio by CS pin

    set_sleep_time(DWM_WAKEUP_XTAL_STAB_TIME_TCKS,get_RTC_timestamp(false));  //set timeout - XTAL should be stabilized in

    nrf_delay_us(3000); //khs
    //while (!dwt_checkidlerc()) // check in IDLE_RC before proceeding
   // {
    //}
    while(!int_flag.DW_XTAL_stable && !(rtcDelayComplete[RTC_delay_instance][deep_sleep_CC_reg]))                 //wait while XTAL is not stabilized
    {
       __WFI();
    }

    DW_CS_SET();                        //set DW chip select to idle state
    DW_RST_IRQ_disable();               //disable IRQ to prevent unwanted asynchronous interrupt request
    deep_sleep_RTC_IRQ_disable();       //disable IRQ to prevent unwanted synchronour interrupt request

    nrf_delay_us(50);                       //wait until PLL locks

    uint32_t	devID = dwt_readdevid();			// check if the DWM is answering for the ID request
    if (DWT_DEVICE_ID != devID)					//if wake up was not successful
    {
            DWM1000_initAndDoHwResetProcedure();	//reset DWM //and try it again
            uint32_t	devID = dwt_readdevid();	// check if the DWM is answering for the ID request
            if (DWT_DEVICE_ID != devID)
            {
                    signalize_dwmError();
                    do_SWResetMCU(20);
            }
    }
    
    dwt_restoreconfig();//khs
    dw_restore_from_sleep(); 					//do after sleep procedure
    configure_UWB(false);//khs
    //DWM_reconfigure();//khs add 20250701
    //dwt_setxtaltrim(0x31);
    dwt_write32bitreg(TX_POWER_ID, TX_LEVEL_DEFAULT);//khs add 20250701
    //DWM_reconfigure();//khs add 20250701
    dump_dwm3001c_register_config();//khs add 20250701
    
// PLL 상태 확인 로그
uint32_t sys_status = dwt_read32bitreg(SYS_STATUS_ID);
uint32_t sys_state  = dwt_read32bitreg(0x19);
printf("[PLL CHECK] SYS_STATUS: 0x%08X, CPLOCK: %d\r\n", sys_status, (sys_status >> 1) & 0x1);
printf("[PLL CHECK] SYS_STATE : 0x%08X\r\n", sys_state);

// TX 딜레이 보정 적용 (DW3210 전용)
dwt_write32bitreg(TX_ANTD_ID, 0x00006400);

//void dw3210_force_pll_ldo_bias_config(void)
//{
    // 1. LDO 튜닝 수동 적용 (DW3001 기준값)
    dwt_write32bitreg(LDO_TUNE_LO_ID, 0x87777688);   // LDO_LO: 안정화
    dwt_write32bitreg(LDO_TUNE_HI_ID, 0x06067878);   // LDO_HI: 안정화

    // 2. BIAS 설정 (OTP 미사용 시 필수)
    dwt_write32bitreg(BIAS_CTRL_ID, 0x0000106F);     // 안정적 기본값

    // 3. PLL 설정 강제 주입 (PLL_CFG 일부 문서엔 생략됨)
    dwt_write32bitoffsetreg(PLL_CFG_ID, 0, 0x09000407); // PLL 구성
    dwt_write32bitoffsetreg(PLL_CAL_ID, 0, 0x00000081); // PLL Calibration kick

    // 4. XTAL 트리밍 (OTP 미사용 시 필수)
    //dwt_setxtaltrim(0x30); // 수동 조정 필요 시 0x2C~0x3F 범위 내 변경 가능

    // 5. PLL Lock 상태 대기 (CPLOCK 올라올 때까지)
    int retry = 0;
    while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_CP_LOCK_BIT_MASK)) {
        nrf_delay_us(50);
        retry++;
        if (retry > 200) {
            printf("PLL Lock FAIL: CPLOCK not set\r\n");
            break;
        }
    }
//}
}

//*******************************************************debug test zon uwb
/**
 * @brief Initialize DWM1000 and read partnumber to create MAC adress of tag
 */

void DWM1000_init_debug(void)
{
	int32_t result = DWT_ERROR;

	result = init_DWM3000_communication();

	if(result == DWT_ERROR)
	{
		signalize_dwmError();
		NVIC_SystemReset();
	}
	else
	{

          //dwt_configuresleep(DWT_LOADUCODE | DWT_PRESRV_SLEEP | DWT_CONFIG | DWT_TANDV,DWT_WAKE_CS | DWT_SLP_EN); //configure the on wake parameters (upload the IC config settings)
          //dwt_configuresleep(DWT_LOADLDO | DWT_LOADBIAS | DWT_CONFIG | DWT_RUNSAR,DWT_WAKE_CSN | DWT_SLP_EN);
          dwt_configuresleep(DWT_CONFIG | DWT_PGFCAL ,DWT_PRES_SLEEP | DWT_WAKE_CSN | DWT_SLP_EN);
          //dwt_configuresleep(DWT_LOADLDO | DWT_LOADBIAS | DWT_CONFIG | DWT_RUNSAR , DWT_WAKE_CSN | DWT_SLP_EN);
          //dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | (DWT_INT_ARFE | DWT_INT_RFSL | DWT_INT_SFDT| DWT_INT_RPHE | DWT_INT_RFCE /*| DWT_INT_RFTO*/ | DWT_INT_RXPTO),1);
          dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | (DWT_INT_ARFE | DWT_INT_RFSL | DWT_INT_SFDT| DWT_INT_RPHE | DWT_INT_RFCE /*| DWT_INT_RFTO*/ | DWT_INT_RXPTO),0,1);
          dwt_setcallbacks(Dwt_Txcallback,Dwt_Rx_OK_callback, Dwt_Rx_T_OUT_callback , Dwt_Rx_ERR_callback,NULL,NULL);


          //dwt_setleds(2|1) ;
          //dwt_setlnapamode(2|1);
//dump_dwm3001c_otp_values();//debug
          //read_otp_params();
          //debug
          //debug
          //dwt_entersleep(DWT_DW_IDLE_RC);
	}
}