/*
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2020-2023. All rights reserved.
 */

#ifndef __EDMACV310_H__
#define __EDMACV310_H__

/* debug control */
extern int edmacv310_trace_level_n;
#define EDMACV310_TRACE_LEVEL 5
#define EDMACV310_TRACE_LEVEL_INFO 4
#define EDMACV310_TRACE_LEVEL_DEBUG 6

#define EDMACV310_TRACE_FMT KERN_INFO

typedef void DMAC_ISR(unsigned int channel, int status);

#define EDMACV310_32BIT 32
#define EDMAC_UPDATE_TIMEOUT  (30 * HZ)
#define EDMAC_TRANS_MAXSIZE  (64 * 1024 - 1)

#ifdef  DEBUG_EDMAC

#define edmacv310_trace(level, msg...) do { \
	if ((level) >= edmacv310_trace_level_n) { \
		printk(EDMACV310_TRACE_FMT"%s:%d: ", __func__, __LINE__); \
		printk(msg); \
		printk("\n"); \
	} \
} while (0)


#define edmacv310_assert(cond) do { \
	if (!(cond)) { \
		printk(KERN_ERR "Assert:edmacv310:%s:%d\n", \
				__func__, \
				__LINE__); \
		BUG(); \
	} \
} while (0)

#define edmacv310_error(s...) do { \
	printk(KERN_ERR "edmacv310:%s:%d: ", __func__, __LINE__); \
	printk(s); \
	printk("\n"); \
} while (0)

#else

#define edmacv310_trace(level, msg...) do { } while (0)
#define edmacv310_assert(level, msg...) do { } while (0)
#define edmacv310_error(level, msg...) do { } while (0)

#endif

#define edmacv310_readl(addr) ((unsigned int)readl((void *)(addr)))

#define edmacv310_writel(v, addr) do { writel(v, (void *)(addr)); \
} while (0)


#define MAX_TRANSFER_BYTES  0xffff

#define EDMAC_ERR_REG_NUM 3
#define DMAC_FINISHED_WAIT_TIME 10

/* reg offset */
#define EDMAC_INT_STAT                  0x0
#define EDMAC_INT_TC1                   0x4
#define EDMAC_INT_TC2                   0x8
#define EDMAC_INT_ERR1                  0xc
#define EDMAC_INT_ERR2                  0x10
#define EDMAC_INT_ERR3                  0x14
#define EDMAC_INT_TC1_MASK              0x18
#define EDMAC_INT_TC2_MASK              0x1c
#define EDMAC_INT_ERR1_MASK             0x20
#define EDMAC_INT_ERR2_MASK             0x24
#define EDMAC_INT_ERR3_MASK             0x28

#define EDMAC_INT_TC1_RAW               0x600
#define EDMAC_INT_TC2_RAW               0x608
#define EDMAC_INT_ERR1_RAW              0x610
#define EDMAC_INT_ERR2_RAW              0x618
#define EDMAC_INT_ERR3_RAW              0x620

#define edmac_cx_curr_cnt0(cn)           (0x404 + (cn) * 0x20)
#define edmac_cx_curr_src_addr_l(cn)     (0x408 + (cn) * 0x20)
#define edmac_cx_curr_src_addr_h(cn)     (0x40c + (cn) * 0x20)
#define edmac_cx_curr_dest_addr_l(cn)    (0x410 + (cn) * 0x20)
#define edmac_cx_curr_dest_addr_h(cn)    (0x414 + (cn) * 0x20)

#define EDMAC_CH_PRI                    0x688
#define EDMAC_CH_STAT                   0x690
#define EDMAC_DMA_CTRL                  0x698

#define edmac_cx_base(cn)               (0x800 + (cn) * 0x40)
#define edmac_cx_lli_l(cn)              (0x800 + (cn) * 0x40)
#define edmac_cx_lli_h(cn)              (0x804 + (cn) * 0x40)
#define edmac_cx_cnt0(cn)               (0x81c + (cn) * 0x40)
#define edmac_cx_src_addr_l(cn)         (0x820 + (cn) * 0x40)
#define edmac_cx_src_addr_h(cn)         (0x824 + (cn) * 0x40)
#define edmac_cx_dest_addr_l(cn)        (0x828 + (cn) * 0x40)
#define edmac_cx_dest_addr_h(cn)        (0x82c + (cn) * 0x40)
#define edmac_cx_config(cn)             (0x830 + (cn) * 0x40)

#define EDMAC_CXCONFIG_M2M            0xCFF33000
#define EDMAC_CXCONFIG_M2M_LLI        0xCFF00000
#define EDMAC_CXCONFIG_CHN_START  0x1
#define EDMAC_CX_DISABLE           0x0

#define EDMAC_ALL_CHAN_CLR        0xff
#define EDMAC_INT_ENABLE_ALL_CHAN 0xff


#define EDMAC_CONFIG_SRC_INC          (1 << 31)
#define EDMAC_CONFIG_DST_INC          (1 << 30)

#define EDMAC_CONFIG_SRC_WIDTH_SHIFT  16
#define EDMAC_CONFIG_DST_WIDTH_SHIFT  12
#define EDMAC_WIDTH_8BIT              0x0
#define EDMAC_WIDTH_16BIT             0x1
#define EDMAC_WIDTH_32BIT             0x10
#define EDMAC_WIDTH_64BIT             0x11

#define EDMAC_MAX_BURST_WIDTH         16
#define EDMAC_MIN_BURST_WIDTH         1
#define EDMAC_CONFIG_SRC_BURST_SHIFT  24
#define EDMAC_CONFIG_DST_BURST_SHIFT  20

#define EDMAC_LLI_ALIGN   0x40
#define EDMAC_LLI_DISABLE 0x0
#define EDMAC_LLI_ENABLE 0x2

#define EDMAC_CXCONFIG_SIGNAL_SHIFT   0x4
#define EDMAC_CXCONFIG_MEM_TYPE       0x0
#define EDMAC_CXCONFIG_DEV_MEM_TYPE   0x1
#define EDMAC_CXCONFIG_TSF_TYPE_SHIFT 0x2
#define EDMAC_CXCONFIG_LLI_START      0x1

#define EDMAC_CXCONFIG_ITC_EN     0x1
#define EDMAC_CXCONFIG_ITC_EN_SHIFT   0x1

#define CCFG_EN 0x1

/* DMAC peripheral structure */
typedef struct edmac_peripheral {
	/* peripherial ID */
	unsigned int peri_id;
	/* peripheral data register address */
	unsigned long peri_addr;
	/* config requset */
	int host_sel;
#define DMAC_HOST0 0
#define DMAC_HOST1 1
#define DMAC_NOT_USE (-1)
	/* default channel configuration word */
	unsigned int transfer_cfg;
	/* default channel configuration word */
	unsigned int transfer_width;
	unsigned int dynamic_periphery_num;
} edmac_peripheral;


#define PERI_ID_OFFSET  4
#define EDMA_SRC_WIDTH_OFFSET 16
#define EDMA_DST_WIDTH_OFFSET 12
#define EDMA_CH_ENABLE  1

#define PERI_8BIT_MODE  0
#define PERI_16BIT_MODE 1
#define PERI_32BIT_MODE 2
#define PERI_64BIT_MODE 3

#define EDMAC_LLI_PAGE_NUM 0x4
#endif
