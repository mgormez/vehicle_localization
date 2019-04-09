



#ifndef CONSTANTS_H
#define CONSTANTS_H


/* ------------------------------------- */
/* REGISTERS MAP                         */
/* ------------------------------------- */
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


#define SPI_FREQUENCY	3000000L	// 3MHz

#define NO_SUB		0xFFU	//CHECK THIS VALUE (how many bytes??)
// #define NO_SUB		0xFFFFU	//CHECK THIS VALUE (how many bytes??)
#define WRITE 		0x80U	// spi header for writing to slave (already shifted to the left to be the 2 MSB)
#define WRITE_SUB 	0xC0U	//
#define READ 		0x00U	//
#define READ_SUB 	0x40U	//
#define RW_SUB_EXT 	0x80U	//

#define JUNK 		0x00U	//for reading from SPI
#endif
