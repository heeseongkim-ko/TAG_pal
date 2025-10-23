#ifndef UWB_ROUTINES_H_INCLUDED
#define UWB_ROUTINES_H_INCLUDED


#define DWT_DEVICE_ID (0xDECA0302)
extern volatile uint16_t baudrate;
extern volatile bool CHG_active;
extern volatile bool ACC_sleep;
extern volatile bool dw_temperature_measure;
extern volatile bool blink_sent;

/** Definition of confirmation callback function type. It can used for confirmation of data request done event. */
typedef void (*data_confirm_cb_t)(void);

/**
 * @private
 * @brief Definition of structure of date requests.
 */
typedef struct {
    uint16_t app_id;                    /*!< Application ID. */
    uint8_t length;                     /*!< Length of data in bytes. */
    uint8_t data[110];                  /*!< Application output data array.*/
    bool rdy;                           /*!< Data ready flag. */
    data_confirm_cb_t data_confirm_cb;  /*!< The callback, that should be called once the data was successfully send. */
}data_requests_struct_t;

typedef struct{
    bool rdy;
    uint8_t length;
    uint8_t* data;
}received_payload_t;

typedef struct{
bool rdy;
uint8_t length;
uint8_t data[110];
}tx_payload_buffer_t;

/* data types definitions */
typedef struct ref_values {
    uint8_t   pgdly;
    uint32_t  power;
    int16_t   temp;   // the DW IC raw register value, which needs conversion if you want a value
    uint16_t  pgcnt;
} ref_values_t;
extern ref_values_t ref_local;

// -------------------------------------------------------------------------------------------------------------------
// Structure to hold device data
typedef struct
{
    uint8_t  vBatP;             /**<IC V bat read during production and stored in OTP (Vmeas @ 3V3) */
    uint8_t  tempP;             /**<IC V temp read during production and stored in OTP (Tmeas @ 23C) */
    uint8_t  otprev;            /**<OTP revision number (read during initialisation) */
    uint16_t otp_mask;          /**<Local copy of the OTP mask used in dwt_initialise call */
    uint32_t TXpwr_ch1_prf16;   /**<TX power for channel 1 and PRF16 - read from OTP */
    uint32_t TXpwr_ch1_prf64;   /**<TX power for channel 1 and PRF64 - read from OTP */
    uint32_t TXpwr_ch2_prf16;   /**<TX power for channel 2 and PRF16 - read from OTP */
    uint32_t TXpwr_ch2_prf64;   /**<TX power for channel 2 and PRF64 - read from OTP */
    uint32_t TXpwr_ch3_prf16;   /**<TX power for channel 3 and PRF16 - read from OTP */
    uint32_t TXpwr_ch3_prf64;   /**<TX power for channel 3 and PRF64 - read from OTP */
    uint32_t TXpwr_ch4_prf16;   /**<TX power for channel 4 and PRF16 - read from OTP */
    uint32_t TXpwr_ch4_prf64;   /**<TX power for channel 4 and PRF64 - read from OTP */
    uint32_t TXpwr_ch5_prf16;   /**<TX power for channel 5 and PRF16 - read from OTP */
    uint32_t TXpwr_ch5_prf64;   /**<TX power for channel 5 and PRF64 - read from OTP */
    uint32_t TXpwr_ch7_prf16;   /**<TX power for channel 7 and PRF16 - read from OTP */
    uint32_t TXpwr_ch7_prf64;   /**<TX power for channel 7 and PRF64 - read from OTP */
} otp_data_t;
extern otp_data_t otp_refs;

//extern volatile bool blink_sent;
void DWM1000_initAndDoHwResetProcedure(void);
void DWM1000_init(void);
void wake_up_radio(void);
void DM1000_enter_sleep(void);
void enter_radio_sleep(void);
void dw_read_IDs(uint32_t* lotid, uint32_t* partid);
uint8_t DW_format2dB(uint8_t gain_DW_format, float* gain_dB);
uint8_t dB2DW_format(float gain_dB, uint8_t* gain_DW_form);
uint8_t add_correction_to_DWgain(const uint8_t* input_gain, uint8_t* corrected_gain, float correction);
uint8_t calc_TX_gain(const uint32_t* raw_gain, uint32_t* corr_gain, float correction_f);
void init_TX_gain(void);
void initialize_mac_headers(uint64_t* MAC_addr);
void init_tag_info_payload(void);
uint32_t DWM_reconfigure(void);
void configure_UWB(bool set_bc_timeout);
void UWB_IRQ_enable(void);
bool RX_handler(bool first_rx);
void prepare_system_apps_response(void);
void process_prepared_data(void);
void uwb_msg_trx(void);
void radio_routine(void);
void Data_Conf(void);
void DMW1000_wake_up_test(void);
void do_SWResetMCU(uint8_t errnum);

//******************************************debug
void DWM1000_init_debug(void);
#if 0
/*
// 기본 레지스터 주소 (레지스터 ID)
#define DEV_ID_ID                      0x00  // Device ID
#define SYS_CFG_ID                     0x04  // System Configuration
#define SYS_STATUS_ID                  0x0F  // System Status
#define PMSC_CTRL0_ID                  0x36  // Power Management (Clock Enable 등)
#define PMSC_CTRL1_ID                  0x36  // INIT 상태 진입, sub-address 사용
#define OTP_RDAT_ID                    0x2D  // OTP 읽기 결과 저장
#define OTP_CFG_ID                     0x2E  // OTP 설정 (LDO kick 등)
#define OTP_ADDR_ID                    0x2B  // OTP 주소 지정
#define OTP_CTRL_ID                    0x2C  // OTP 동작 트리거
#define AON_DIG_CFG_ID                 0x2A  // AON sleep configuration
#define AON_CTRL_ID                    0x2C  // AON control register
#define FS_XTALT_ID                    0x1E  // XTAL Trim register (1byte)

// PMSC: XTAL Enable + Digital Clocks (INIT 상태 유지 시)
#define PMSC_CTRL0_XTAL_ON             0x0200F00F  // Enable DPLL & clocks
#define PMSC_CTRL1_GOTO_INIT           0x00000004  // Move to INIT state

// SYS_CFG 기본 설정
#define SYS_CFG_DEFAULT                0x00001200  // Frame check enable, RX timeouts enable 등

// SYS_STATUS PLL Lock 확인용 마스크
#define SYS_STATUS_CLKPLL_LL_BIT_MASK  0x80000000UL  // Bit 31 = PLL Locked

// OTP 관련 비트
#define OTP_CFG_LDO_KICK_BIT_MASK      0x80         // LDO 튠 kick (bit 7)

// AON 관련
#define AON_DIG_CFG_KEEP_LDO           0x0E         // Sleep 중 유지할 블럭 설정
#define AON_CTRL_ARRAY_SAVE_BIT_MASK   0x01         // Save current config to AON
#define AON_CTRL_CONFIG_UPLOAD_BIT_MASK 0x02        // Apply sleep config from AON

// XTAL Trim Mask (0x1F)
#define XTAL_TRIM_MASK                 0x1F

#define LDOTUNE_ADDRESS                0x04  // LDO tuning value
#define XTAL_TRIM_ADDRESS              0x0E  // Crystal trim
#define OTP_PART_ID_ADDRESS            0x06  // Part ID
#define OTP_LOT_ID_ADDRESS             0x08  // Lot ID
#define OTP_BAT_ADDRESS                0x0A  // Battery voltage reference
#define OTP_TMP_ADDRESS                0x0C  // Temperature reference
*/
//************************************************************************test
typedef struct
{
    uint8_t chan;
    uint8_t prf;
    uint8_t pacSize;
    uint8_t txCode;
    uint8_t rxCode;
    uint8_t nsSFD;
    uint8_t dataRate;
    uint8_t phrMode;
    uint8_t phrRate;
    uint16_t sfdTO;
    uint8_t stsMode;
    uint8_t stsLength;
    uint8_t pdoaMode;
} dwt_config_custom_t;

void dw3000_wakeup(void);
void dw3000_reset(void);
//uint32_t dw3000_read_register(uint8_t reg_addr, uint8_t sub_addr, uint8_t length);
//int dw3000_write_register(uint8_t reg_addr, uint8_t sub_addr, const uint8_t *data, uint8_t length);
//void dw3000_write_register_by_addr(uint32_t reg_addr, uint8_t sub_addr, uint8_t length, uint32_t value);
//uint32_t dw3000_read_register_by_addr(uint32_t reg_addr, uint8_t sub_addr, uint8_t length);
uint32_t dw3000_fast_command(uint8_t Fast_command, uint8_t length);
uint32_t dw3000_short_addr_read(uint8_t reg_addr, uint8_t length);
uint32_t dw3000_sub_addr_read(uint8_t reg_addr, uint8_t sub_addr, uint8_t length);
int dw3000_short_addr_write(uint8_t reg_addr, const uint8_t *data, uint8_t length);
int dw3000_sub_addr_write(uint8_t reg_addr, uint8_t sub_addr, const uint8_t *data, uint8_t length);
//int dw3000_write_fast(uint8_t reg_addr, const uint8_t *data, uint8_t length);
//uint32_t dw3000_read_sub(uint8_t reg_addr, uint8_t sub_addr, uint8_t length);
//int dw3000_write_sub(uint8_t reg_addr, uint8_t sub_addr, const uint8_t *data, uint8_t length);
//int dw3000_write_masked(uint8_t reg_addr, uint8_t sub_addr, uint8_t mask, uint8_t data);

void dw3000_send_tx_packet(const uint8_t* tx_data, uint8_t length);
//int dwt_configure_custom(dwt_config_custom_t* config);
//************************************************************************uart
void dwm3001c_spi_init(void);
void dw_irq_init(void);
//-----------------------------
void dwm3001c_spim_init(void);
//************************************************************************uart
int readfromspi2(uint16_t headerLength, uint8_t *headerBuffer,
                uint16_t readLength, uint8_t *readBuffer);
int writetospi2(uint16_t headerLength, const uint8_t *headerBuffer,
               uint16_t bodyLength, const uint8_t *bodyBuffer);
#endif
#endif  /* UWB_ROUTINES_H_INCLUDED */

