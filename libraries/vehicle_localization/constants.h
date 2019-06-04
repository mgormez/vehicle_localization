



#ifndef CONSTANTS_H
#define CONSTANTS_H


/* ------------------------------------- */
/* REGISTERS MAP                         */
/* ------------------------------------- */
#define DEV_ID_LEN		4
#define SYS_MASK_LEN	4
#define SYS_STATUS_LEN 	5
#define SYS_STATE_LEN	5
#define SYS_CTRL_LEN	4
#define SYS_CFG_LEN		4
#define TIME_STAMP_LEN	5
#define RX_FLEN_LEN		1

#define DEV_ID      0x00
#define EUI         0x01
#define PANADR      0x03
#define SYS_CFG     0x04
#define SYS_TIME    0x06
#define TX_FCTRL    0x08
#define TX_BUFFER   0x09
#define DX_TIME     0x0A
#define RX_FWTO     0X0C
#define SYS_CTRL    0x0D
#define SYS_MASK    0x0E
#define SYS_STATUS  0x0F
#define RX_FINFO    0x10
#define RX_BUFFER   0x11
#define RX_FQUAL    0x12
#define RX_TTCKI    0x13
#define RX_TTCKO    0x14
#define RX_TIME     0x15
#define TX_TIME     0x17
#define TX_ANTD     0x18
#define SYS_STATE   0x19
#define ACK_RESP_T  0x1A
#define RX_SNIFF    0x1D
#define TX_POWER    0x1E
#define CHAN_CTRL   0x1F
#define USR_SFD     0x21
#define AGC_CTRL    0x23
#define EXT_SYNC    0x24
#define ACC_MEM     0x25
#define GPIO_CTRL   0x26
#define DRX_CONF    0x27
#define RF_CONF     0x28
#define TX_CAL      0x2A
#define FS_CTRL     0x2B
#define AON         0x2C
#define OTP_IF      0x2D
#define LDE_CTRL    0x2E
#define DIG_DIAG    0x2F
#define PMSC        0x36

#define PMSC_SYSCLKS_SUB	0x00
#define PMSC_SOFTRESET_SUB	0x03
#define	OTP_CTRL_SUB		0x06	// for LDE loading

/* ------------------------------------- */
/* REGISTERS BIT MAP                     */
/* ------------------------------------- */
#define TRXOFF_BIT			6
#define TXSTRT_BIT			1
#define TX_OK_BIT			7
#define RX_FINISHED_BIT		13
#define RX_NO_ERROR_BIT		14
#define WAIT4RESP_BIT		7

/* ------------------------------------- */
/* SPI PARAMETERS	                     */
/* ------------------------------------- */

// SPI_FREQUENCY has to be at most 3 MHz during configuration phase, and up to 
// 20 MHz afterwards.
// The slow_read_spi and slow_write_spi functions are used during configuration 
// and are not affected by SPI_FREQUENCY parameter, instead their frequency is
// fixed at 3MHz
#define SPI_FREQUENCY	20000000L	// 20MHz
// #define SPI_FREQUENCY	3000000L	// 3MHz

// spi headers for the different operations, already shifted the correct amount of times
#define NO_SUB		0xFFFFU	//place holder value that doesn't actually mean anything, it just has to be a value that signifies no subheader. 0x00 works...
#define WRITE 		0x80U
#define WRITE_SUB 	0xC0U
#define READ 		0x00U
#define READ_SUB 	0x40U	
#define RW_SUB_EXT 	0x80U
#define JUNK 		0x00U	//for reading from SPI

/* ------------------------------------- */
/* LOCALIZATION PARAMETERS               */
/* ------------------------------------- */
// #define ANTENNA_DELAY 		65610U //Offset for error at -50cm  //0x8066 // precis 10cm
// #define ANTENNA_DELAY		65670U
// #define ANTENNA_DELAY 		65772U // 2m calibration (aggregate tag+anchor value)
#define ANTENNA_DELAY 		65767U // 5m calibration (aggregate tag+anchor value)

//index of the byte containing given data in a message exchange
#define ANCHOR_NB_INDEX				0U
#define STATE_MACHINE_STATE_INDEX	1U
#define TIME_STAMP_INDEX			2U

#define MESSAGE_SIZE	1	// size of messages exchanged during the ranging
							// This message simply need to carry information about 
							// which step of the ranging is currently being undertaken
#define TIME_MESSAGE_SIZE	15	// Size of the message containing the times measured by the anchor
// #define NB_OF_ANCHORS	3	// hard code the number of anchors
#define NB_OF_ANCHORS	1	// for calibration

// // fixed anchors positions (m)
// #define ANCHOR_1_POS_X	0.0
// #define ANCHOR_1_POS_Y	0.0

// #define ANCHOR_2_POS_X	7.5
// #define ANCHOR_2_POS_Y	0.0

// #define ANCHOR_3_POS_X	1.80
// #define ANCHOR_3_POS_Y	49.28

// fixed anchors positions (m)
#define ANCHOR_1_POS_X	0.0
#define ANCHOR_1_POS_Y	0.0

#define ANCHOR_2_POS_X	7.33
#define ANCHOR_2_POS_Y	0.0

#define ANCHOR_3_POS_X	-1.5
#define ANCHOR_3_POS_Y	34.2

// for testing localization algorithm (legacy)
#define TAG_X			-0.05
#define TAG_Y			2.85			

#define REGISTER_VALUE_TO_S (1/(128*499.2*1000000)) //15.65 ps = 1.565*10^-11
#define SPEED_OF_LIGHT 299792458L	//m/s
#define TIMEOUT 500U


#endif
