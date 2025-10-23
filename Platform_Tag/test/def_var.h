#ifndef DEF_VAR_H_INCLUDED
#define DEF_VAR_H_INCLUDED

//======================================================//
//----------------- TDOA SYSTEM PARAMETERS -------------//
//======================================================//


#define MAC_ADDR_BYTE_SIZE    			(6)
#define UWB_FCODE_BLINK                         (0xBB)
#define UWB_FCODE_CONF                          (0xCC)
#define UWB_FCODE_BC_POLL			(0xBC)
#define UWB_FCODE_BC_DATA			(0xBD)
#define UWB_FCODE_BC_ACK			(0xBA)
#define UWB_FCODE_BC_REPORT			(0xBE)

#define TAG_BROADCAST_ADDR			(0xFFFFFFFFFFFF)

#define WDOG_INTERVAL_DURING_FW_INIT            5000000UL
#define WD_GUARD_INTERVAL_US                    100000UL

//extern volatile uint16_t baudrate;
#define SPI_BAUDRATE_LOW                    1000 /**Set baudrate to 1 MBaud/s (1000kHz)*/
#define SPI_BAUDRATE_HIGH                   8000 /**Set baudrate to 8 MBaud/s (8000kHz)*/

#define TEIA_CAR    (9)  //custom personal
#define TEIA_TOOL   (8)  //custom vehicle
// Firmware version
//
#define FW_VERSION			(1UL)//(3UL)		//TDOA 1st stage
#define FW_SUBVER			(3UL)//(129UL) 	//this need to be incremented if changes has been done
#define FW_REVISION			(1UL)//(104UL)		// thats are pre-release revisions
#define HW_VERSION			(1)//(1) //
#define HW_REVISION			(0)//(0) //
#define PLATFORM                        TEIA_CAR

//
// Battery management
//

#define MIN_BATT_V_LIPO_BATT				    (3300) //mV for periodical ADC check
#define MAX_BATT_V_LIPO_BATT				    (4160) //mV for periodical ADC check
#define MIN_BATT_V_FOR_DFU_LIPO_BATT            (3500) //mV minimal voltage for reliable reception of new FW over BLE
#define LOW_BATT_V_LIPO_BATT                    (3500)
#define WAKE_UP_V_DURING_CHARGING               (3400)


#define MIN_BATT_V_LIPO_BATT_RAW			    (MIN_BATT_V_LIPO_BATT * 1/2500) //(MIN_BATT_V_LIPO_BATT * 128/2500)
#define MAX_BATT_V_LIPO_BATT_RAW      		    (MAX_BATT_V_LIPO_BATT * 128/2500) //
#define LOW_BATT_V_LIPO_BATT_RAW                (LOW_BATT_V_LIPO_BATT * 128/2500)
#define MIN_BATT_V_FOR_DFU_LIPO_BATT_RAW        (MIN_BATT_V_FOR_DFU_LIPO_BATT  * 128/2500)
#define WAKE_UP_V_DURING_CHARGING_RAW           (WAKE_UP_V_DURING_CHARGING * 128/2500)

#define MIN_BATT_V_COIN_BATT					(2100) //mV for periodical ADC check
#define MAX_BATT_V_COIN_BATT					(3150) //mV for periodical ADC check
#define LOW_BATT_V_COIN_BATT					(2300) //mV for periodical ADC check
#define MIN_BATT_V_FOR_RX_COIN_BATT             (2900) //mV minimal voltage for reliable reception of RX config
#define MIN_BATT_V_FOR_DFU_COIN_BATT            (2500) //mV minimal voltage for reliable reception of new FW over BLE

#define MIN_BATT_V_COIN_BATT_RAW				(MIN_BATT_V_COIN_BATT * 255/3750)
#define MAX_BATT_V_COIN_BATT_RAW				(MAX_BATT_V_COIN_BATT * 255/3750)
#define LOW_BATT_V_COIN_BATT_RAW                (LOW_BATT_V_COIN_BATT * 256/3750)
#define MIN_BATT_V_FOR_DFU_COIN_BATT_RAW        (MIN_BATT_V_FOR_DFU_COIN_BATT * 256/3750)
#define MIN_BATT_V_FOR_RX_COIN_BATT_RAW         (MIN_BATT_V_FOR_RX_COIN_BATT * 256/3750)

////////////////BLINK LIMIT REFRESHRATES///////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define MAX_RR_MS 			(60000)//ms //1 min
#define MAX_RRSM_MS 	    (129600000) //ms /////36 hod


#define MIN_RR_MS_LIPO_BATT			(10)//ms
#define MIN_RRSM_MS_LIPO_BATT       (MIN_RR_MS_LIPO_BATT)

#define MIN_RR_MS_COIN_BATT			(50)//ms - CR TAG MINIMAL RR
#define MIN_RRSM_MS_COIN_BATT       (MIN_RR_MS_COIN_BATT)

#define LOW_THRESHOLD_FLAG			(1 << 14)
#define MEDIUM_THRESHOLD_FLAG		(2 << 14)
#define HIGH_THRESHOLD_FLAG			(3 << 14)
#define PREDEFINED_THRESHOLD_MASK	(3 << 14)

#define ONE_SEC_RTC_TCKS	32768UL
#define TWO_SEC_RTC_TCKS	65536UL
#define FIVE_SEC_RTC_TCKS	163840UL

#define DWM_WAKEUP_LINE_ACTIVE_TIME 			550 //us
#define DWM_WAKEUP_LINE_ACTIVE_TICKS_RTC		21 //ticks
#define DWM_WAKEUP_XTAL_STAB_TIME		 		4 //ms
#define DWM_WAKEUP_XTAL_STAB_TIME_TCKS		 	328 //tcks//131
#define DWM_WAKEUP_LINE_ACTIVE_LONG_TIME 		1000 //us

#define  RTC_delay_instance     2
#define  delay_CC_reg           1
#define  deep_sleep_CC_reg      2
#define  PB_CC_reg              3
//--------------------------------------------------------------------------------------------------teia_set
/** Default UWB channel. */
#define CHANNEL_DEFAULT							(5)

/** Default RF_profile */
#define RF_PROFILE_DEFAULT						(5)//(4)//(5)//(5)

/** Default data rate [2 -> 6M8]  */
#define DR_MODE_DEFAULT							(2)//(1)//(2) //1 rf4

/** Default preamble length [6 -> 128 b]  */
#define PREAMBLE_DEFAULT						(6)//(5)//(6)//5 rf4

/** Default Pulse Repetition Frequency [1 -> 64 MHz]  */ //khs not use
#define PRF_DEFAULT							(1)

/** Default Preamble code [12]*/
#define PREAM_CODE_DEFAULT						(12)

/** Default (Non) Standard Frame Delimiter ([0 -> standard]*/
#define NSFD_DEFAULT							(0)//(1)//(0)//1 rf4

/** Default setting for random deviation of refresh rate [1 -> enable] */
#define RANDOM_DEVIATION_DEFAULT					(1)

/** Default setting for sleep modes - motion controlled ranging [0 -disable, 1 -delayed sleep]*/
#define SLEEP_MODE_DEFAULT						(0)


/** Default threshold of acceleration for Leonardo tag[48 mg] */
#define ACC_SENS_DEFAULT                                                (48UL | LOW_THRESHOLD_FLAG)

/** Default setting for sensors data sending  [0 -> all sensors disabled] */
#define EBLINK_DEFAULT                                                  (0)

/** Default setting for Spatial Rotation determination [0 -> disable] */
#define AHRS_DEFAULT                                                    (0)

/** Default setting for RX period [0 -> disable] */
#define RX_PERIOD_DEFAULT						(0)

/** Default setting for RX state duration [1 s] */
#define RX_DURATION_DEFAULT						(1000) // [ms]


/** Default setting refresh rate [] */
#define REFRESH_RATE_DEFAULT_LIPO_BATT                                  (250) //[ms]
#define REFRESH_RATE_DEFAULT_COIN_BATT                                  (1000) //[ms]


/** Default setting no motion refresh rate [0 -> disable] */
#define NO_MOT_REFRESH_RATE_DEFAULT                                     (0)  //[ms]

/** Default setting for barometer precision [0 ->disable] */
#define BARO_DEFAULT     						(0x00)

/** Default setting for gyroscope full scale range [0 -> +-250 dps] */
#define GYRO_FS_DEFAULT							(0)

/** Default setting for accelerometer full scale range [0 -> +-2 g] */
#define ACC_FS_DEFAULT							(0)


/** Default setting for magnetometer calibration mode [0 -> Standard] */
#define MAG_CALIB_MODE_DEFAULT						(0)

/** Default setting for UWB radio transmitter power [0x00000000 -> nominal power from OTP] */
#define TX_LEVEL_DEFAULT						(0xfdfdfdfd)//(0xfdfdfdfd)//(0x00000000)
//#define TX_LEVEL_DEFAULT							(0x23436383)



/** Default setting of Pushbutton On function */
#define PB_ON_OFF_DEFAULT					(true)

/** Default setting of switch to UWB RX mode using PB - Asset only */
#define PB_ENTER_UWB_CONFIG_DEFAULT				(true)

/** Default setting of switch to DFU mode using Pushbutton */
#define PB_ENTER_DFU_DEFAULT					(false)

/** Default setting of Reset the configuration to factory default using Pushbutton */
#define PB_FACTORY_RESET_DEFAULT				(true)

/** Default setting of functionality for send special blink with information that a button has been pressed*/
#define PB_SPECIAL_BLINK_ON_PRESS_DEFAULT			(true)


/** Default setting of Pushbutton pin (disable/enable input on MCU) */
#define PB_ENABLE_DEFAULT     					(true)

/** Default setting of Low battery signalization by LED */
#define LOW_BAT_SIGNAL_DEFAULT                                  (false)

/** Default setting of TX_power_correction */
#define TX_POWER_CORRECTION_DEFAULT                 (int8_t)(0)


/** Default setting of BC period in normal mode (0 = disable) */
#define BC_PERIOD_RI_DEFAULT                        (0)

/** Default setting of BC period during sleep (0 = disable) */
#define BC_PERIOD_RISM_DEFAULT                      (0)

/** Default setting of BC period during sleep [x10ms] */
#define BC_TX2RX_TIME_DEFAULT                        (88)

/** Default setting of UWB protocol version */
#define UWB_PROTOCOL_DEFAULT                        (2)
//--------------------------------------------------------------------------------------------------
#define PRF_16      (0)
#define PRF_64      (1)

#define DR_110k     (0)
#define DR_850k     (1)
#define DR_6M8      (2)

#define PREAM_L_2048 (2)
#define PREAM_L_1024 (3)
#define PREAM_L_256  (5)
#define PREAM_L_128  (6)

#define nSFD_STD     (0)
#define nSFD_nSTD    (1)

//======================================================//
//----------------- TDOA SYSTEM PARAMETERS -------------//
//======================================================//


#define MAC_ADDR_BYTE_SIZE    						(6)
#define MAC_ADDR_BYTE_OFFSET                        (1)
#define BC_DATA_OFFSET                              (MAC_ADDR_BYTE_OFFSET + MAC_ADDR_BYTE_SIZE)
#define TAG_BROADCAST_ADDR							(0xFFFFFFFFFFFF)
#define MAC_ADDR_48b_MASK                           (0xFFFFFFFFFFFFULL)
#define MAC_ADDR_44b_MASK                           (0xFFFFFFFFFFULL)
#define MAC_ADDR_COMPOSITION_MARK                   (0x300000000000ULL)     //highest nibble of composed MAC address determine the version of composition process, current version is 3

#define UWB_FCODE_BLINK                             (0xBB)
#define UWB_FCODE_CONF								(0xCC)
#define UWB_FCODE_BC_POLL							(0xBC)
#define UWB_FCODE_BC_DATA							(0xBD)
#define UWB_FCODE_BC_ACK							(0xBA)
#define UWB_FCODE_BC_REPORT							(0xBE)

#define RET_OK       (0)
#define RET_ERR      (1)

#define BC_VERSION          (1)     //current version of back channel implementation
#define BC_ACK_NUM_SIZE     (5)


//
// Timeout to run with Accelerometer without motion detected
//
#define	ACC_TIME_TO_WAIT_INMOTION				(30000) //ms
#define	RR_IN_IDLE_MODE						(5000) //ms

//
//Extended Info blink settings
//
#define DELAY_BETWEEN_INFOBLINKS				(1000)                          /**< period of infoblinks send on start [ms] */
#define NUM_OF_IBLINK_SENDIG_AT_THE_BEGIN		(3)                             /**< number of infoblinks send after start of tag */
#define EXTENDED_BLINK_BATT_FREQ				(15)                            /**< period of blinks that contain info about battery level */
#define IBLINK_FREQ                             EXTENDED_BLINK_BATT_FREQ * 15   /**< period of infoblinks */

#define INFO_MSG_PERIOD                         (15)
#define BATT_MSG_PERIOD                         (15)

#define RX_ON_TIME_COIN_BATT				(100)//ms
#define RX_OFF_TIME_COIN_BATT				(200)//ms
#define RX_ON_TIME_LIPO_BATT				(1000)//ms
//-------------------------------------------------------------------------------------ADC
#define ADC_VDD_channel      0
#define ADC_BATT_channel     1

#define MEAS_BATT_AFTER_RR  (15) //cycle of RR

//bat level
#define Full_cutoff_voltage 203 //4.1V
#define GREEN_LIMIT_POWER   	185 // 3.65
#define ORANGE_LIMIT_POWER 	178 // 3.5
#define RED_LIMIT_POWER   		170 // 3.35
//-------------------------------------------------------------------------------------I2C
#define TWI_INSTANCE_ID     1               /**< I2C instance index. */

#define I2C_FREQ_100k       (26738688)      /**< I2C frequency = 100 kHz. */
#define I2C_FREQ_250k       (67108864)      /**< I2C frequency = 250 kHz. */
#define I2C_FREQ_400k       (104857600)     /**< I2C frequency = 400 kHz. */

#define I2C_FREQ_STD                        100 /**SCL frequency in kHZ*/
#define I2C_FREQ_250                        250 /**SCL frequency in kHZ*/
#define I2C_FREQ_FAST                       400 /**SCL frequency in kHZ*/
//-------------------------------------------------------------------------------------ACC
#define LIS2DH12    (1<<2)

#define ACC_FS_2g    0x00
#define ACC_FS_4g    0x10
#define ACC_FS_8g    0x20
#define ACC_FS_16g   0x30

//
// Acceleration thresholds for wakeup from sleep
//
#define LIS2DH12_THRESHOLD_STEP		(16)	//it must be set in multiplies of 16
#define LIS2DH12_LOW_THRESHOLD		(48)
#define LIS2DH12_MIN_THRESHOLD		(LIS2DH12_LOW_THRESHOLD)
#define LIS2DH12_MEDIUM_THRESHOLD	(256)
#define LIS2DH12_HIGH_THRESHOLD		(1024)
#define LIS2DH12_MAX_THRESHOLD		(8001)
//-------------------------------------------------------------------------------------

#define SEW_CRITICAL_PRIORITY   (0)
#define SEW_INFO_PRIORITY       (1)
#define SEW_STD_PRIORITY        (3)
#define SEW_LOW_PRIORITY        (5)

#define USER_HIGH_PRIORITY      (2)
#define USER_MEDIUM_PRIORITY    (4)
#define USER_LOW_PRIORITY       (6)

#define INFO_MSG_PRIORITY   (SEW_INFO_PRIORITY)
#define BATT_MSG_PRIORITY   (SEW_STD_PRIORITY)

#define INFO_MSG_PERIOD                         (15)
#define BATT_MSG_PERIOD                         (15)

#define DATA_MAX_PRIORITY 6
#define DATA_MIN_PRIORITY 0

#define NUM_OF_PRIORITIES (DATA_MAX_PRIORITY - DATA_MIN_PRIORITY + 1)

#define MAX_NUM_OF_APPS 4

#define UNIV_MSG_TYPE  0x64U
#define CORRUPTED_DATA 0xFF
#define APP_NOT_EXIST  0xFE

#define CHARGING_MARK                     (252) /**<If the tag charged msg type is 252 after msg type the paylod defined by tag setting is appended*/
#define NO_MOTION_MARK                    (253) /**<If the tag is not in motion and still send blinks the msg type is 253 after msg type the paylod - defined by tag setting is appended*/

#define PREAM_64_DURATION_US    (65)
#define PREAM_128_DURATION_US   (130)
#define SFD_DURATION_US         (36)
#define DATA_BYTE_DURSTION_US   (1.176f)

#define  TEMP_MEAS_RTC_instance 1
#define  TEMP_MEAS_RTC_chan     3

#define DW_TEMPERATURE_MEAS_PERIOD_S            (10UL)
#define DW_TEMPERATURE_MEAS_PERIOD_TCKS         (DW_TEMPERATURE_MEAS_PERIOD_S * 32768UL)

#define BC_DATA_OK  0
#define BC_DATA_NOK 1

#define IMM_ACK_MASK        0x01
#define NUM_OF_ACK_MASK     0b00011110
#define NUM_OF_ACK_OFFSET   1


typedef struct
{
    bool motion_action_detected;			/**<motion was detected by accelerometer*/
    bool pushbutton_pressed;				/**<user pushbutton was pressed*/
    bool blink_sent;						/**<sending of blink was completed*/
    bool UWB_msg_received;					/**<new config message was received via UWB*/
    bool wake_up;                           /**<generall wake_up flag */
    bool RTC_CMP0_achieved;					/**<RTC counter achieved to value of compare register 0 - RTC delay | RX required | TX reuired during EM2 sleep*/
    bool RTC_CMP1_achieved;					/**<RTC counter achieved to value of compare register 1 - used for measure of time of button pressed*/
    bool DW_XTAL_stable;
}int_flag_t;
extern int_flag_t int_flag;						/**< interrupt requests flags*/

//====================================================================================================================khs_uart
/*structure of external irq enable flags - each member hold info about eneable/disable of the specific external irq source en*/

typedef struct
{
    bool acc_irq;
    bool nfc_fd;
    bool chg_status;
    bool dw_irq;
    bool pushbutton;
    bool dw_rst;
} ext_irq_en_t;
extern ext_irq_en_t ext_irq_en;

// Wireless Config RX message
//
// +-------+----------+--------+---------+----------+----------+-----+-----------+------+---------+----------+-------------+-------------+----+------------+----------+-----+
// | fcode | destAddr | seqNum | channel | datarate | preamble | prf | preamCode | nSfd | randDev | acc_mode |  EblinkCont | AHRS_enable | RR | no_mot_RR  | TX power | CRC |
// |  1B   |    6B    |   1B   |   1B    |    1B    |    1B    | 1B  |    1B     |  1B  |   1B    |   1B     |      1B     |      1B     | 4B |     1B     |    4B    | 2B  |
// +-------+----------+--------+---------+----------+----------+-----+-----------+------+---------+----------+-------------+-------------+----+------------+----------+-----+

typedef struct {
    uint8_t fcode;					  /**<Type of message - OxCC for wireless config message*/
    uint8_t destAddr[MAC_ADDR_BYTE_SIZE]; /**<Address of tag that should be set*/
    uint8_t seqNum;				       /**<Sequence number*/
    uint8_t channel;			/**<UWB radio channel*/
    uint8_t datarate;			/**<Data rate - UWB radio*/
    uint8_t preamble;			/**<Preamle length - UWB radio*/
    uint8_t prf;				/**<Pulse repetition frequency*/
    uint8_t preamCode;			/**<UWB radio preamble code*/
    uint8_t nSfd;				/**<Start frame delimiter - standard/non-standard*/
    uint8_t randomDev;			/**<Random deviation of TDOA refresh rate*/
    uint8_t acc_mode;			/**<Mode of sleep controled by acceleration of tag - no motion detection*/
    uint8_t EblinkCont;		 	/**<Setting of sensors whose data should be put into extended blink*/
    uint8_t AHRS_enable;		/**< Rotation determination setting*/
    uint8_t RR[4];				/**<TDOA refreshrate - period of UWB blink*/
    uint8_t no_mot_RR[4];		/**<TDOA refreshrate during no motion*/
    uint8_t tx_power[4];		/**<Transmiter power - UWB radio*/
    uint8_t	MCR_sens[2];		/**<Threshold of acceleration for wakeup tag from deep sleep - motion controled ranging*/
    uint8_t IMU_FS_range;		/**<Full scae range of gyroscope and accelerometer*/
    uint8_t RX_period[4];		/**<Period of RX state - UWB radio*/
    uint8_t RX_duration[2];		/**<Duration of RX state - UWB radio*/
    uint8_t BARO_setting;		/**<Barometer precision*/
    uint8_t CALIB_START;		/**<Sensors calbration start*/
    uint8_t crc[2];				/**<placeholder for CRC - calculated in UWB radio*/
} tdoa_uwb_conf_msg;

/**
 * A structure to represent TDOA parameters which are USER/DEVELOPER definable
 */
typedef struct {
    uint64_t this_tag_MACaddress;		/**< MAC address of the tag*/
    uint8_t channel;					/**<UWB radio channel*/
    uint8_t RF_profile;					/**<UWB radio - RF profile - Data rate, preamble len, preamble code, PRF, nSfd*/
    uint8_t data_rate;					/**<UWB radio data rate*/
    uint8_t preamble;					/**<Preamble length - UWB radio*/
    uint8_t prf;						/**<Pulse repetition frequency - UWB radio*/
    uint8_t preamCode;					/**<Preamble code -UWB radio*/
    uint8_t nSfd;						/**<Start frame delimiter - UWB radio*/
    uint32_t use_random_deviation;		/**<Random deviation of TDOA refresh rate*/
    uint32_t frequency_of_Eblink;		/**<Frequency of battery blink*/
    uint8_t motion_control_mode;		/**<No motion detection mode - tag is in slee while tag is in no motion state*/
    uint32_t refresh_rate_ms;			/**<TDOA refreshrate - period of blink in miliseconds*/
    uint32_t no_motion_refresh_rate;    /**<TDOA refreshrate - during no motion state*/
//	uint32_t RX_period_ms;				/**<Period of UWB RX state*/
//	uint32_t RX_duration_ms;			/**<Duration of UWB RX state*/
    uint32_t tx_pwr_level_conf;			/**<Gain of UWB transmitter - raw value in DW format*/
    uint32_t tx_pwr_level_raw;			/**<Gain of UWB transmitter - raw value in DW format*/
    uint32_t tx_pwr_level_corr;			/**<Gain of UWB transmitter - corrected (raw + correction) value in DW format*/
    uint32_t tx_pwr_level_comp;			/**<Gain of UWB transmitter - compensated (corrected + compensation (V,T,length)*/
    uint8_t tx_PG_delay;                /**<Propagation_delay of UWB transmitter*/
    int8_t tx_pwr_correction;          /**<Correction of TX gain - it is added to tx_pwr_level - LSB = 0.5dB*/
    uint8_t bc_period_ri;               /**<Back-channel period during normal operation [every Xth refresh interval] */
    uint8_t bc_period_rism;             /**<Back-channel period during sleep moe [every Xth refresh interval in sleep mode] */
    uint8_t bc_tx2rx_time;                  /**<Back-channel time space between transmit of BC frame to RX. It is represented as time between TX Rmarker and RX Rmarker. */
    uint8_t uwb_protocol;                /**<Version of used  */
} tdoaParameters_t;
extern tdoaParameters_t tdoaParameters;		/**<Parameters of tdoa*/

typedef struct {
    int32_t temperature_MCU;                /**<measured temperature of MCU. */
    int8_t temeperature_DW;
    uint32_t need_to_measure_battery;       /**<Counter for periodical battery measurement. */
    uint8_t lowbattcount;                   /**<Counter of battery measurement with lower result than defined value. */
    uint8_t actual_batt_voltage_raw;        /**<Battery measurement - ADC output. */
    uint8_t min_batt_voltage_raw;           /**<Battery measurement - ADC output. */
    uint8_t max_batt_voltage_raw;           /**<Battery measurement - ADC output. */
    uint8_t low_batt_voltage_raw;           /**<Battery measurement - ADC output. */
    uint8_t min_batt_voltage_for_DFU_raw;   /**<Battery measurement - ADC output. */
    uint8_t platform;                       /**<Tag Leonardo edition on which this FW running. */
    uint8_t hw_version;                     /**<Hardware version for which this HW is intended. */
    uint8_t hw_revision;                    /**<Hardware revision for which this HW is intended. */
    uint8_t fw_version;                     /**<Version of firmware. */
    uint8_t fw_subversion;                  /**<Subversion of firmware. */
    uint8_t fw_revision;                    /**<Revision of firmware. */
    bool    periodical_rx_possibility;      /**<The periodical RX is only allowed on some platform. True if allowed on current platform. */
    uint32_t min_rr_ms;                     /**<Minimal allowed refresh rate for current platform */
    uint32_t max_rr_ms;                     /**<Maximal allowed refresh rate for current platform */
    uint32_t min_rrsm_ms;                   /**<Minimal allowed refresh rate in sleep mode for current platform */
    uint32_t max_rrsm_ms;                   /**<Maximal allowed refresh rate in sleep mode for current platform */

    uint8_t MAC_addr[6];
	volatile bool is_shutdown;          /**<Flag which signalize, that the tag is in shutdown mod. */
} systemValues_t;
extern systemValues_t systemValues;            /**<Structure that contain general information about HW and FW. */

typedef struct{
    bool rx_data_rdy;
    uint8_t rx_data_len;
    uint8_t rx_data_buffer[128];
}rx_data_t;

typedef struct{
    bool tx_data_rdy;
    uint8_t tx_data_len;
    uint8_t tx_data[128];
}tx_data_t;

typedef struct{
    bool rx_err_phe;
    bool rx_err_fce;
    bool rx_err_rfsl;
    bool rx_err_sfdto;
    bool rx_err_affrej;
    bool rx_err_lde;
    bool rx_pto;
    bool rx_fto;
    bool rx_pream_timeout;
    bool rx_frame_timeout;
    bool rx_timeout;
    bool rx_error;
}dw_rx_errors_t;

typedef struct {
    uint8_t 	  ACC_mounted;			/**<if 0 accelerometer is not mounted, if 1 mounted accelerometer is MMA8453, if 2 mounted accelerometer is MPU9250*/
    bool    	  MAG_mounted;			/**<if true magnetometer is mounted on tag*/
    bool    	  GYRO_mounted;			/**<if true gyro is mounted on tag*/
    bool          BARO_mounted;
    uint8_t       EBlink_cont;			/**<this byte determines which sensor data will be sent in extended blink*/
    uint8_t       sens_corr_data;		/**<this byte determines if the data from sensors will be corrected or true RAW*/
    bool          IMU_just_calib;		/**<true if IMU was just calibrated*/
    bool          MAG_just_calib;		/**<true if magnetometer was just calibrated*/
    uint8_t	  AHRS_enable;          /**<determine if AHRS should be calculate*/
    uint8_t	  motion_control_mode;  /**<*/
    uint16_t	  wakeup_threshold ;	/**<threshold of acceleration to wake up MCU form acc sleep*/
    uint8_t	  gyro_FS;				/**<dynamic range of gyroscope*/
    uint8_t	  acc_FS;				/**<dynamic range of accelerometer*/
    uint8_t 	  BARO_setting;			/**<setting of BAROMETER*/
    bool          BARO_cont_mode;       /**<if true BAROMETER measure continuously in background mode, else Command mode*/
    uint16_t      measure_period_ms;    /**<period of barometer measurements in milliseconds*/
    uint16_t      measure_period_tck;   /**<period of barometer measurements in RTC ticks*/
    bool          MAG_cont_mode;        /**<if true MAGNETOMETER measure continuously in background mode, else Command mode*/
    uint8_t 	  mag_calib_mode;		/**<mode of magnetometer calibration*/
    uint8_t	  ACC_LP_period;		/**<accelerometer Low power period in ms>*/
} sensors_params_t;
extern sensors_params_t sensors_params;

typedef struct {
    int8_t q0;
    int8_t q1;
    int8_t q2;
    int8_t q3;
}quatQ8_t;							   /**<Quaternion in fixed point Q8 representation*/

typedef struct
{
    int16_t x_axis;                     /**<Data from sensors x-axis*/
    int16_t y_axis;                     /**<Data from sensors y-axis*/
    int16_t z_axis;                     /**<Data from sensors z-axis*/
} sens3D_data_t;						/**<Data from 3D sensor*/

typedef struct {
    float q0;
    float q1;
    float q2;
    float q3;
}quat_float_t;							/**<Quaternion in floating point representation*/

typedef struct {
    uint8_t yaw;                        /**<Yaw angle that describe rotation of tag around Z axis. */
    uint8_t pitch;                      /**<Pitch angle that describe rotation of tag around X axis. */
    uint8_t roll;                       /**<Roll angle that describe rotation of tag around Y axis. */
}TB_angles_t; 

typedef struct {
    sens3D_data_t acc_offsets;			/**<offsets for each axes*/
    sens3D_data_t mag_offsets;			/**<offsets for each axes*/
    uint16_t	  geo_mag_flux;			/**<magnetic flux density of geo-magnetic field*/
    sens3D_data_t gyro_offsets;			/**<offsets for each axes*/
    sens3D_data_t acc_offsets_SC;		/**<offsets for each axes - scaled for chosen dynamic range*/
    sens3D_data_t gyro_offsets_SC;		/**<offsets for each axes - scaled for chosen dynamic range*/
    sens3D_data_t acc_data;				/**<accelerometer raw data*/
    sens3D_data_t mag_data;				/**<magnetometer raw data*/
    sens3D_data_t gyro_data;			/**<gyroscope raw data*/
    uint32_t	  baro_data;			/**<pressure*/
    int32_t		  baro_offset;			/**<pressure offset*/
    uint8_t		  temp_data;			/**<temperature from baro sensor*/
    TB_angles_t   angles;				/**<Tilt-Brian angles which determines orientation of tag*/
    quatQ8_t	  quaternion;			/**<quaternion which determines orientation of tag - stored in fixed point format Q8*/
    quat_float_t  f_quaternion;		    /**<quaternion which determines orientation of tag - stored in floating point format*/
}sensors_data_t;
extern sensors_data_t sensors_data;

typedef struct
{
    bool PB_tag_switch_on_off;          /**<Pushbutton functionality for switch the tag ON/OFF*/
    bool PB_enter_UWB_config;           /**<Pushbutton functionality that allows switch the tag to RX state to receive config via UWB*/
    bool PB_enter_DFU;                  /**<Pushbutton functionality that allows switch the tag to DFU mode*/
    bool PB_factory_reset;              /**<Pushbutton functionality for reset the tag configuration to factory default configuration*/
    bool PB_special_blink_on_press;     /**<Pushbutton functionality for send special blink with information that a button has been pressed*/
    bool PB_enable;                     /**<Set PB pin as input if enable, if disable the pin is disabled*/
    bool LED_Low_batt_sig;              /**<Signalize low battery state*/
}optional_functionality_t;

typedef struct {
	uint32_t RefreshRate_us; 				/**<TDOA refresh rate(blink period) in microseconds*/
	uint32_t RefreshRate_ms; 				/**<TDOA refresh rate(blink period) in milliseconds*/
	uint32_t RefreshRate_tck;  				/**<TDOA refresh rate(blink period) in RTC ticks*/
	uint32_t RRwaitTime_tck;				/**<Time to wait before next blink = RR - time need to sent blink - overhead time*/
	uint32_t RRwaitTime_tck_actual;			/**<Actual time for wait before next blink - used if random deviation is enabled*/
	uint32_t random_rr_value_actual;        /**<Wait time that will be used in current period*/
	uint32_t random_rr_value_old;           /**<Wait time that was used in last period*/
	uint32_t random_rr_deviation_tck;       /**<Deviation calculated for this blink period*/
	uint32_t random_rr_deviation_half_tck;  /**<Deviation calculated for this blink period divided by 2*/
	int32_t  diff_from_RR;                  /**<Difference between defined refresh rate and deviated RR*/
	uint32_t RR_calculated;                 /**<save the calculated refresh rate here*/

	uint32_t RefreshRate_no_mot_ms;		/**<TDOA refresh rate(blink period) in microseconds, during the no motion sleep*/
	uint32_t RefreshRate_no_mot_tck;	/**<TDOA refresh rate(blink period) in RTC ticks, during the no motion sleep*/


	uint32_t RR_in_sleep_calculated; 		/**<Calculated refresh rate in sleep*/


	uint32_t last_Blink_timestamp;			/**<Timestamp of last UWB blink - state of RTC*/

	uint32_t no_motion_delay;				/**<Delay between no motion detection and sleep state of tag*/

//	uint32_t  tcks_to_RX;					/**<Ticks of RTC remaining to next RX state*/
//	uint32_t RX_period_ms;					/**<Period of RX state (UWB radio) in milliseconds*/
//	uint32_t RX_period_tck;					/**<Period of RX state (UWB radio) in RTC ticks*/
//	uint32_t RX_duration_ms;				/**<Required duration of RX state (UWB radio) in milliseconds*/
	uint32_t last_RX_timestamp;			    /**<Timestamp of last RX state - RTC count*/
	bool 	 RX_required;					/**<Is RX state of UWB radio now required?*/
	bool 	 random_dev_en;					/**<Random deviation of TDOA refresh rate*/
} tdoaTiming_t;
extern tdoaTiming_t tdoaTiming;

typedef struct {
  uint32_t no_motion_cycle; 				/**<if motion ranging control mode is set to 1, MCU must wait several cycles (blink periods) before go to sleep*/
  uint32_t send_Eblink_after_RR;			/**<period of sending messages with battery level*/
  uint32_t RR_counter;					/**<counter of blink period*/
  uint32_t inf_b_counter;					/**<counter of battery blink period*/
  uint8_t seq_num;						/**<sequence number of current blink*/
  uint8_t on_start_info_msg;				/**<after FW start tag send some infoblinks, this define haw many infoblinks remain*/
  bool is_info_blink_requested;		    /**<determine if next blink will be extended info blink*/
} tdoaValues_t;
extern tdoaValues_t tdoaValues;            /**<Values that is used to determine which type of blink should be currently used etc.*/
//--------------------------------------------------------------------------------------

#define MSGTYPE_BATT		    	 (1)  /**<Message with information about battery state.*/
#define MSGTYPE_ANGLES      		 (2)  /**<Message with information about spatial rotation of tag - that is represented as Tait-Bryan angles.*/
#define MSGTYPE_SENS_RAW		     (5)  /**<Message type with RAW data from sensors mounted on tag*/
#define MSGTYPE_QUATERNION           (9)  /**<Message with information about spatial rotation of tag - that is represented as quaternion in fixed point format*/
#define MSGTYPE_TAG_INFO			(14)  /**<Message type with informations about tag HW,FW,user settings etc..*/
#define MSGTYPE_CUSTOM_PAYLOAD		(255) /**<Blink message with user defined payload*/

#define MSGTYPE_GENERIC              (0)  /**<Basic message - non-battery, non-info data. */
#define MSGTYPE_BATT_ANGLES      	 (3)  /**<Message with information about spatial rotation of tag - that is represented as Tait-Bryan angles.*/
#define MSGTYPE_BATT_SENS_RAW		 (6)  /**<Message type with RAW data from sensors mounted on tag*/
#define MSGTYPE_BATT_QUATERNION      (10)  /**<Message with information about spatial rotation of tag - that is represented as quaternion in fixed point format*/
#define MSGTYPE_QUATERNION_BARO      (11)  /**<Message with information about spatial rotation of tag - that is represented as quaternion in fixed point format*/
#define MSGTYPE_BATT_QUATERNION_BARO (12)  /**<Message with information about spatial rotation of tag - that is represented as quaternion in fixed point format*/
#define MSGTYPE_BATTCUSTOM_PAYLOAD	 (254) /**<Blink message with  user defined payload*/




typedef struct {
      uint8_t battVoltage;				/**<Battery level in RAW form.*/
      uint8_t platform;					/**<Type of tag.*/
      uint8_t hw_ver[2];					/**<Hardware version and revision.*/
      uint8_t fw_ver[3];					/**<Firmware version, subversion and revision.*/
      uint8_t channel;					/**<Used UWB radio channel*/
      uint8_t data_rate;					/**<UWB radio data rate*/
      uint8_t preamble;					/**<UWB radio - preamble code length*/
      uint8_t prf;						/**<UWB radio - pulse repetition frequency*/
      uint8_t preamCode;					/**<UWB radio preamble code*/
      uint8_t nSfd;						/**<Start frame delimiter*/
      uint8_t mcr;						/**<No motion sleep mode*/
      uint8_t random_dev;					/**<Random deviation of TDOA blink refreshrate*/
      uint8_t refresh_interval[4];		/**<TDOA Blink refresh rate*/
      uint8_t sm_refresh_interval[4];     /**<TDOA Blink refresh rate - no motion sleep mode*/
      uint8_t TXpower[4];					/**<UWB radio transmitter power*/
      uint8_t mounted_sensors;			/**<Each bite represent one sensor, if set : concrete sensor is mounted on tag*/
      uint8_t active_sensors;				/**<Each bite represent one sensor, if set : concrete sensor is active*/
      uint8_t mcr_threshold[2];			/**<No motion - acceleration threshold*/
      uint8_t IMU_FS_range;				/**<Inertial measurement unit (accelerometer and gyroscope full scale ranges*/
      uint8_t BARO_setting;				/**<Barometer precision setting*/
      uint8_t AHRS_representation;		/**<Spatial rotation representation. 0 - disabled, 1 - Tait-brian angles, 2 - Quaternion */
      uint8_t bc_version;                 /**<Current version of back-channel protocol. */
      uint8_t bc_period_ri;               /**<Back-channel period during normal operation [every Xth refresh interval] */
      uint8_t bc_period_rism;             /**<Back-channel period during sleep mode [every Xth refresh interval in sleep mode] */
} tag_info_msg_t;

/**
 * @brief Battery blinks. It contain battery level represented as ADC output.
 */
typedef struct {
	uint8_t battVoltage;					/**<Battery level in RAW form.*/
} battery_msg_t;

//---------------------------------------------------------------------------------NFC
#define FW_VER_OFFSET                           (16)
#define FW_SUBVER_OFFSET                        (8)
#define FW_REV_OFFSET                           (0)
#define CONFIGURATION_VERSION        ((FW_VERSION << FW_VER_OFFSET) | (FW_SUBVER << FW_SUBVER_OFFSET) |(FW_REVISION << FW_REV_OFFSET))
#define EEPROM_NUM_OF_RW_ATTEMPTS               (3)

//for validity check
#define RANDDEV_CNT					(2)
#define MCR_CNT						(4)

#define CONF_OK						(0)
#define ERR_CHANNEL					(0x00000001)      /**<Wrong channel -error code*/
#define ERR_DR						(0x00000002)
#define ERR_PREAMBLE                                    (0x00000003)
#define ERR_RANDDEV					(0x00000004)
#define ERR_MCR						(0x00000005)
#define ERR_RR						(0x00000006)
#define ERR_TX_POW					(0x00000007)
#define ERR_PRF						(0x00000008)
#define ERR_PREAMCODE                                   (0x00000009)
#define ERR_NSFD                                        (0x0000000A)
#define ERR_ACC_FS					(0x0000000B)
#define ERR_AHRS					(0x0000000C)
#define ERR_BARO					(0x0000000D)
#define ERR_EBLINK					(0x0000000E)
#define ERR_GYRO_FS                                     (0x0000000F)
#define ERR_RX_PERIOD                                   (0x00000010)
#define ERR_RX_DURATION                                 (0x00000011)
#define ERR_PB_ON_OFF                                   (0x00000012)
#define ERR_PB_UWB_CONF                                 (0x00000013)
#define ERR_PB_DFU                                      (0x00000014)
#define ERR_PB_RESET                                    (0x00000015)
#define ERR_PB_SPECIAL_BLINK                            (0x00000016)
#define ERR_PB_ENABLE					(0x00000017)
#define ERR_LED_LOW_BATT_SIG				(0x00000018)
#define ERR_TX_POW_CORR     				(0x00000019)

//
// Data rates
//
#define DATARATE_COUNT 					(3)  /**<Number of usable data rates*/
#define DR0_INDEX 					(0)  /**<Data rate index for data_rate 110k*/
#define DR1_INDEX 					(1)  /**<Data rate index for data_rate 850k*/
#define DR2_INDEX 					(2)  /**<Data rate index for data_rate 6M8*/
#define PREAMBLE_COUNT					(8)  /**<Number of usable preamble lengths*/

typedef struct
{
	int16_t x_axis;                     /**<Offset for X axis of some sensor. */
	int16_t y_axis;                     /**<Offset for Y axis of some sensor. */
	int16_t z_axis;                     /**<Offset for Z axis of some sensor. */
} sens_offsets_t;                       /**<Structure that contain offsets for  3-axis sensors.*/


typedef struct {
    uint32_t version;                       /**<Check value - if 0 dta is not stored in EEPROM, else the configuration version should be stored there*/
	uint8_t channel;				        /**<Ultra wide band radio channel setting*/
	uint8_t RF_profile;				        /**<RF profile - Data rate, Preamble length, PRF, Preamble code, nSfd*/
	uint8_t data_rate;				        /**<Data rate (UWB radio) setting*/
	uint8_t preamble;				        /**<Preamble length setting*/
	uint8_t prf;				            /**<Pulse repetition frequency setting*/
	uint8_t preamCode;				        /**<Preamble code setting*/
	uint8_t nSfd;					        /**<Start frame delimiter setting*/
	uint8_t use_random_deviation;	        /**<Setting the random deviation for refresh rate*/
	uint8_t acc_mode;				        /**<Setting the mode of acceleration sleep*/
	uint16_t acc_sens;				        /**<Setting the threshold of acceleration for wakeup the tag from sleep*/
	uint32_t user_refresh_rate;		        /**<Setting the refresh rate*/
    uint32_t no_motion_refresh_rate;        /**<Setting the refresh rate during no motion state*/
	uint32_t rx_period;			            /**<Setting the period of RX state (UWB radio) */
	uint32_t rx_duration;			        /**<Setting the duration of RX state (UWB radio) */
	uint32_t tx_pwr_level;			        /**<Transmitter power setting (UWB radio)*/
	uint8_t Eblink_cont;			        /**<Each bit in this byte determines that a particular sensor is active and its data is sent in eblink*/
	uint8_t sens_corr_data;			        /**<Each bit in this byte determines whether data from a particular sensor is being sent in a corrected form*/
	uint8_t GYRO_FS;				        /**<Gyroscope full scale range*/
	uint8_t ACC_FS;					        /**<Accelerometer full scale range*/
	uint8_t BARO_setting;			        /**<Barometer precision setting*/
	uint8_t AHRS_enable;			        /**<Cyclic redundancy check*/
	uint8_t mag_calib_mode;			        /**<Mode of magnetometer calibration*/
	uint16_t geo_mag_flux;			        /**<magnetic flux density of geo-magnetic field*/
	sens_offsets_t acc_offset;		        /**<Calculated offset of accelerometer data in all 3 axis*/
	sens_offsets_t mag_offset;		        /**<Calculated offset of magnetometer data in all 3 axis*/
	sens_offsets_t gyro_offset;		        /**<Calculated offset of gyroscope data in all 3 axis*/
	optional_functionality_t optional_func; /**<Optional functionalities of Pushbutton and LED*/
    int8_t TX_power_correction;             /**<Correction offset of TX power - it will be added to TX power - to correct effect of neighboring objects. LSB represent 0.5dB*/
    uint32_t configuration_timestamp;       /**<Epoch time in the seconds - the timestamp is taken when the NFC_config request is send to the tag*/
	uint8_t bc_period_ri;                   /**<Back-channel period during normal operation [every Xth refresh interval] */
	uint8_t bc_period_rism;                 /**<Back-channel period during sleep mode [every Xth refresh interval in sleep mode] */
	uint8_t bc_tx2rx_time;                  /**<Back-channel time space between transmit of BC frame to RX. It is represented as time between TX Rmarker and RX Rmarker. */
	uint8_t uwb_protocol;                   /**<UwB protocol version used for compose of the UWBframes. */
	uint16_t crc;                           /**<Cyclic redundancy check*/
} userData_t;

#endif  /* DEF_VAR_H_INCLUDED */