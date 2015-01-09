#ifndef __MAC9260_H__
#define __MAC9260_H__

struct desc_type {
        uint32_t   status;                /*!< Status */
        uint32_t   conbufsize;     /*!< Control and Buffer1, Buffer2 lengths */
        uint32_t   buf1addr;           /*!< Buffer1 address pointer */
        uint32_t   buf2nextdescaddr;   /*!< Buffer2 or next descriptor address pointer */
};

struct board_info
{	
	struct desc_type  *tx_desc;
	struct desc_type  *rx_desc;
        struct eth_device ndev;
        u32 recv_len;
	uint8_t *p_rx;
	uint8_t *p_tx;
        uint8_t *buf;
};

typedef unsigned char byte;

void set_pin(void);
void set_phy_mode(int phy);
void set_mac_mode(int mac);
void set_work_mode(int mode);
void set_mac_address(unsigned int high,unsigned int low);
int alp_dma_map(void);
void alp_dma_unmap(void);
void mac_init(void);
void mac_dma_enable(void);
void eth_send_init(struct desc_type *txtab, uint8_t *tbuf, uint32_t tbufcount);
void eth_recv_init(struct desc_type *rxtab, uint8_t *rbuf, uint32_t rbufcount);
void eth_it_config(struct desc_type *dmarxdesc);
void eth_insert_config(struct desc_type *dmatxdesc, uint32_t DMATxDesc_Checksum);
void mac_clk_enable(void);
void mac_clk_disable(void);
void eth_reset(void);
void software_reset(void);
void mac_loopback(void);
void set_csr_clock(void);
void set_interrupt(void);
void mac_stop(void);
int packet_check_err(void);

int mac9260_init(struct eth_device *ndev,bd_t *bis);
void mac9260_halt(struct eth_device *ndev);
int mac9260_send(struct eth_device *ndev, volatile void *buf, int len);
int mac9260_recv(struct eth_device *ndev);
static void mac9260_phy_write(int reg, uint32_t value);
static unsigned long mac9260_phy_read(int reg);


#define CARDNAME	"mac9260"
#define DRV_VERSION	"uboot_v1.01"

#define MII_Mode 1
#define RMII_Mode 0

#define PHY_ADDRESS                      0x01		/* Relative to STM3210C-EVAL Board */
//#define PHY_ADDRESS 0                     /*dm9161*/

#define PHY_BCR                          0          	/*!< Tranceiver Basic Control Register */

#define ETH_MACMIIAR_PA   ((uint32_t)0x0000F800)  /* Physical layer address */ 
#define ETH_MACMIIAR_MR   ((uint32_t)0x000007C0)  /* MII register in the selected PHY */ 
#define ETH_MACMIIAR_MW   ((uint32_t)0x00000002)  /* MII write */ 
#define ETH_MACMIIAR_MB   ((uint32_t)0x00000001)  /* MII busy */ 

#define PHY_READ_TO                     ((uint32_t)0x0004FFFF)
#define PHY_WRITE_TO                    ((uint32_t)0x0004FFFF)
#define PHY_ResetDelay                  ((uint32_t)0x000FFFFF) 

#define ETH_RXBUFNB        2
#define ETH_TXBUFNB        2
#define ETH_MAX_PACKET_SIZE    1536    					/*!< ETH_HEADER + ETH_EXTRA + MAX_ETH_PAYLOAD + ETH_CRC */
#define ETH_DMATxDesc_TCH                     ((uint32_t)0x01000000)  	/*!< Second Address Chained */
#define ETH_DMATxDesc_OWN                     ((uint32_t)0x80000000)  	/*!< OWN bit: descriptor is owned by DMA engine */
#define ETH_DMARxDesc_OWN         	      ((uint32_t)0x80000000)  /*!< OWN bit: descriptor is owned by DMA engine  */
#define ETH_DMARxDesc_RCH   		      ((uint32_t)0x01000000)  /*!< Second Address Chained */
#define ETH_DMATxDesc_FS                      ((uint32_t)0x20000000)  	/*!< First Segment */
#define ETH_DMATxDesc_LS                      ((uint32_t)0x40000000)  	/*!< Last Segment */
#define ETH_DMATxDesc_ChecksumTCPUDPICMPFull  ((uint32_t)0x18000000)   	
#define ETH_DMARxDesc_ES         	      ((uint32_t)0x00008000) 
#define ETH_DMARxDesc_FS         	      ((uint32_t)0x00000200)  /*!< First descriptor of the frame  */
#define ETH_DMARxDesc_LS          	      ((uint32_t)0x00000100)  /*!< Last descriptor of the frame  */ 
#define ETH_DMARxDesc_Error_Summary	      ((uint32_t)0x00008000)

#define STATUS_CRC 			      0x00000002
#define STATUS_RECV			      0x00000008
#define STATUS_WTDG			      0x00000010
#define STATUS_COLLISION		      0x00000040
#define STATUS_IPV4_CHECKSUM		      0x00000080
#define STATUS_OVER_FLOW		      0x00000800
#define STATUS_DESC			      0x00004000
#if 0
//MAC REGISTER
#define HW_MACCLKDIV          	0x800401F8
#define HW_ETH_BASE_ADDR        0x80500000
#define HW_ETH_MACCR            HW_ETH_BASE_ADDR + 0x0000
#define HW_ETH_MACFFR           HW_ETH_BASE_ADDR + 0x0004
#define HW_ETH_MACHTHR          HW_ETH_BASE_ADDR + 0x0008
#define HW_ETH_MACHTLR          HW_ETH_BASE_ADDR + 0x000C
#define HW_ETH_MACMIIAR         HW_ETH_BASE_ADDR + 0x0010
#define HW_ETH_MACMIIDR         HW_ETH_BASE_ADDR + 0x0014
#define HW_ETH_MACFCR           HW_ETH_BASE_ADDR + 0x0018
#define HW_ETH_MACVLANTR        HW_ETH_BASE_ADDR + 0x001C
#define HW_ETH_MACVR            HW_ETH_BASE_ADDR + 0x0020    //read only
#define HW_ETH_MACRWUFFR        HW_ETH_BASE_ADDR + 0x0028
#define HW_ETH_MACPMTCSR        HW_ETH_BASE_ADDR + 0x002C
#define HW_ETH_MACDBGR          HW_ETH_BASE_ADDR + 0x0034
#define HW_ETH_MACISR           HW_ETH_BASE_ADDR + 0x0038
#define HW_ETH_MACIMR           HW_ETH_BASE_ADDR + 0x003C
#define HW_ETH_MACA0HR          HW_ETH_BASE_ADDR + 0x0040
#define HW_ETH_MACA0LR          HW_ETH_BASE_ADDR + 0x0044
#define HW_ETH_MACA1HR          HW_ETH_BASE_ADDR + 0x0048
#define HW_ETH_MACA1LR          HW_ETH_BASE_ADDR + 0x004C
#define HW_ETH_MACA2HR          HW_ETH_BASE_ADDR + 0x0050
#define HW_ETH_MACA2LR          HW_ETH_BASE_ADDR + 0x0054
#define HW_ETH_MACA3HR          HW_ETH_BASE_ADDR + 0x0058
#define HW_ETH_MACA3LR          HW_ETH_BASE_ADDR + 0x005C
#define HW_ETH_MACA4HR          HW_ETH_BASE_ADDR + 0x0060
#define HW_ETH_MACA4LR          HW_ETH_BASE_ADDR + 0x0064
#define HW_ETH_MMCCR            HW_ETH_BASE_ADDR + 0x0100
#define HW_ETH_MMCRIR           HW_ETH_BASE_ADDR + 0x0104
#define HW_ETH_MMCTIR           HW_ETH_BASE_ADDR + 0x0108
#define HW_ETH_MMCRIMR          HW_ETH_BASE_ADDR + 0x010C
#define HW_ETH_MMCTIMR          HW_ETH_BASE_ADDR + 0x0110
#define HW_ETH_MMCTGFSCCR       HW_ETH_BASE_ADDR + 0x014C
#define HW_ETH_MMCTGFMSCCR      HW_ETH_BASE_ADDR + 0x0150
#define HW_ETH_MMCTGFCR         HW_ETH_BASE_ADDR + 0x0168
#define HW_ETH_MMCRFCECR        HW_ETH_BASE_ADDR + 0x0194
#define HW_ETH_MMCRFAECR        HW_ETH_BASE_ADDR + 0x0198
#define HW_ETH_MMCRGUFCR        HW_ETH_BASE_ADDR + 0x01C4
#define HW_ETH_PTPTSCR          HW_ETH_BASE_ADDR + 0x0700
#define HW_ETH_PTPSSIR          HW_ETH_BASE_ADDR + 0x0704
#define HW_ETH_PTPTSHR          HW_ETH_BASE_ADDR + 0x0708
#define HW_ETH_PTPTSLR          HW_ETH_BASE_ADDR + 0x070C
#define HW_ETH_PTPTSHUR         HW_ETH_BASE_ADDR + 0x0710
#define HW_ETH_PTPTSLUR         HW_ETH_BASE_ADDR + 0x0714
#define HW_ETH_PTPTSAR          HW_ETH_BASE_ADDR + 0x0718
#define HW_ETH_PTPTTHR          HW_ETH_BASE_ADDR + 0x071C
#define HW_ETH_PTPTTLR          HW_ETH_BASE_ADDR + 0x0720
//#define HW_ETH_PTPTSSR          HW_ETH_BASE_ADDR + 0x0728
//#define HW_ETH_PTPPPSCR         HW_ETH_BASE_ADDR + 0x072C
#define HW_ETH_DMABMR           HW_ETH_BASE_ADDR + 0x1000
#define HW_ETH_DMATPDR          HW_ETH_BASE_ADDR + 0x1004
#define HW_ETH_DMARPDR          HW_ETH_BASE_ADDR + 0x1008
#define HW_ETH_DMARDLAR         HW_ETH_BASE_ADDR + 0x100C
#define HW_ETH_DMATDLAR         HW_ETH_BASE_ADDR + 0x1010
#define HW_ETH_DMASR            HW_ETH_BASE_ADDR + 0x1014
#define HW_ETH_DMAOMR           HW_ETH_BASE_ADDR + 0x1018
#define HW_ETH_DMAIER           HW_ETH_BASE_ADDR + 0x101C
#define HW_ETH_DMAMFBOCR        HW_ETH_BASE_ADDR + 0x1020
//#define HW_ETH_DMARSWTR         HW_ETH_BASE_ADDR + 0x1024
#define HW_ETH_DMACHTDR         HW_ETH_BASE_ADDR + 0x1048
#define HW_ETH_DMACHRDR         HW_ETH_BASE_ADDR + 0x104C
#define HW_ETH_DMACHTBAR        HW_ETH_BASE_ADDR + 0x1050
#define HW_ETH_DMACHRBAR        HW_ETH_BASE_ADDR + 0x1054
#endif
#endif /* __MAC9260_H__ */
