#include "phytium_can_port.h"

#include "fcan.h"
#include "fio_mux.h"
#include "fcan_hw.h"
#include "fparameters.h"
#include "ftypes.h"
#include "sdkconfig.h"
#include <stdio.h>
#include <string.h>

#ifndef PHYTIUM_CAN_ID
#define PHYTIUM_CAN_ID FCAN0_ID
#endif

#ifndef PHYTIUM_CAN_BAUDRATE
#define PHYTIUM_CAN_BAUDRATE 1000000U
#endif

#define PHYTIUM_CAN0_BASE 0x2800A000U
#define PHYTIUM_CAN1_BASE 0x2800B000U
#define PHYTIUM_CAN_CTRL_OFFSET 0x000U
#define PHYTIUM_CAN_INTR_OFFSET 0x004U
#define PHYTIUM_CAN_XFER_STS_OFFSET 0x030U
#define PHYTIUM_CAN_ERR_CNT_OFFSET 0x034U
#define PHYTIUM_CAN_FIFO_CNT_OFFSET 0x038U
#define PHYTIUM_CAN_XFER_EN_OFFSET 0x040U

static FCanCtrl g_can;
static int g_can_ready = 0;
static PhytiumCanDebugState g_can_debug = {
    .init_ret = -99,
    .last_send_ret = -99,
    .can_id = PHYTIUM_CAN_ID,
    .baudrate = PHYTIUM_CAN_BAUDRATE,
};

static uintptr phytium_can_base(void)
{
    return PHYTIUM_CAN_ID == FCAN1_ID ? PHYTIUM_CAN1_BASE : PHYTIUM_CAN0_BASE;
}

static uint32_t phytium_can_read_reg(uint32_t offset)
{
    return *(volatile uint32_t *)(phytium_can_base() + offset);
}

static void phytium_can_update_regs(void)
{
    g_can_debug.reg_ctrl = phytium_can_read_reg(PHYTIUM_CAN_CTRL_OFFSET);
    g_can_debug.reg_intr = phytium_can_read_reg(PHYTIUM_CAN_INTR_OFFSET);
    g_can_debug.reg_xfer_sts = phytium_can_read_reg(PHYTIUM_CAN_XFER_STS_OFFSET);
    g_can_debug.reg_err_cnt = phytium_can_read_reg(PHYTIUM_CAN_ERR_CNT_OFFSET);
    g_can_debug.reg_fifo_cnt = phytium_can_read_reg(PHYTIUM_CAN_FIFO_CNT_OFFSET);
    g_can_debug.reg_xfer_en = phytium_can_read_reg(PHYTIUM_CAN_XFER_EN_OFFSET);
}

const PhytiumCanDebugState *phytium_can_get_debug_state(void)
{
    if (g_can_ready) {
        phytium_can_update_regs();
    }
    return &g_can_debug;
}

int phytium_can_init(void)
{
    FError ret;
    FCanBaudrateConfig arb_segment_config;

    if (g_can_ready) {
        g_can_debug.init_ret = 0;
        return 0;
    }

    memset(&g_can, 0, sizeof(g_can));
    g_can_debug.can_id = PHYTIUM_CAN_ID;
    g_can_debug.baudrate = PHYTIUM_CAN_BAUDRATE;

    printf("phytium_can_init: can_id=%u baudrate=%u\r\n",
           (unsigned)PHYTIUM_CAN_ID,
           (unsigned)PHYTIUM_CAN_BAUDRATE);

    FIOMuxInit();
    printf("phytium_can_init: FIOMuxInit done\r\n");

    FIOPadSetCanMux(PHYTIUM_CAN_ID);
    printf("phytium_can_init: FIOPadSetCanMux done\r\n");

    ret = FCanCfgInitialize(&g_can, FCanLookupConfig(PHYTIUM_CAN_ID));
    if (ret != FCAN_SUCCESS) {
        g_can_debug.init_ret = -1;
        printf("phytium_can_init: FCanCfgInitialize failed ret=%d\r\n", ret);
        return -1;
    }
    printf("phytium_can_init: FCanCfgInitialize ok\r\n");

    FCanFdEnable(&g_can, FALSE);
    FCanSetMode(&g_can, FCAN_PROBE_NORMAL_MODE);

    memset(&arb_segment_config, 0, sizeof(arb_segment_config));

    arb_segment_config.baudrate = PHYTIUM_CAN_BAUDRATE;
    /*
     * Match the Linux SocketCAN timing that has already moved the motor:
     * bitrate 1000000, sample-point 0.750, brp 10, prop 7,
     * phase_seg1 7, phase_seg2 5, sjw 2, CAN clock 200MHz.
     */
    arb_segment_config.auto_calc = FALSE;
    arb_segment_config.segment = FCAN_ARB_SEGMENT;
    arb_segment_config.sample_point = 750;
    arb_segment_config.prop_seg = 7;
    arb_segment_config.phase_seg1 = 7;
    arb_segment_config.phase_seg2 = 5;
    arb_segment_config.sjw = 2;
    arb_segment_config.brp = 10;

    ret = FCanBaudrateSet(&g_can, &arb_segment_config);
    if (ret != FCAN_SUCCESS) {
        g_can_debug.init_ret = -2;
        printf("phytium_can_init: FCanBaudrateSet arb failed ret=%d\r\n", ret);
        return -2;
    }
    printf("phytium_can_init: FCanBaudrateSet arb ok\r\n");

    FCanEnable(&g_can, TRUE);
    printf("phytium_can_init: FCanEnable done\r\n");

    g_can_ready = 1;
    g_can_debug.init_ret = 0;
    phytium_can_update_regs();
    return 0;
}

int phytium_can_send(const Jc4010CanFrame *frame)
{
    FError ret;
    FCanFrame send_frame;

    if (!frame) {
        g_can_debug.last_send_ret = -1;
        return -1;
    }

    if (!g_can_ready) {
        ret = phytium_can_init();
        if (ret != 0) {
            g_can_debug.last_send_ret = -2;
            return -2;
        }
    }

    memset(&send_frame, 0, sizeof(send_frame));

    send_frame.canid = frame->id & CAN_SFF_MASK;
    send_frame.candlc = frame->dlc;
    for (int i = 0; i < frame->dlc && i < 8; ++i) {
        send_frame.data[i] = frame->data[i];
    }

    g_can_debug.last_frame_id = send_frame.canid;
    g_can_debug.last_frame_dlc = send_frame.candlc;
    for (int i = 0; i < 8; ++i) {
        g_can_debug.last_frame_data[i] = send_frame.data[i];
    }

    printf("phytium_can_send: id=0x%03x dlc=%u data=%02x %02x %02x %02x %02x %02x %02x %02x\r\n",
           (unsigned)send_frame.canid,
           (unsigned)send_frame.candlc,
           send_frame.data[0], send_frame.data[1], send_frame.data[2], send_frame.data[3],
           send_frame.data[4], send_frame.data[5], send_frame.data[6], send_frame.data[7]);

    ret = FCanSend(&g_can, &send_frame);
    if (ret != FCAN_SUCCESS) {
        g_can_debug.last_send_ret = -3;
        phytium_can_update_regs();
        printf("phytium_can_send: FCanSend failed ret=%d\r\n", ret);
        return -3;
    }

    g_can_debug.last_send_ret = 0;
    g_can_debug.send_count++;
    phytium_can_update_regs();
    printf("phytium_can_regs: ctrl=0x%08x intr=0x%08x xfer=0x%08x err=0x%08x fifo=0x%08x en=0x%08x\r\n",
           (unsigned)g_can_debug.reg_ctrl,
           (unsigned)g_can_debug.reg_intr,
           (unsigned)g_can_debug.reg_xfer_sts,
           (unsigned)g_can_debug.reg_err_cnt,
           (unsigned)g_can_debug.reg_fifo_cnt,
           (unsigned)g_can_debug.reg_xfer_en);
    printf("phytium_can_send: FCanSend ok count=%u\r\n", (unsigned)g_can_debug.send_count);
    return 0;
}
