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

static FCanCtrl g_can;
static int g_can_ready = 0;
static PhytiumCanDebugState g_can_debug = {
    .init_ret = -99,
    .last_send_ret = -99,
    .can_id = PHYTIUM_CAN_ID,
    .baudrate = PHYTIUM_CAN_BAUDRATE,
};

const PhytiumCanDebugState *phytium_can_get_debug_state(void)
{
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
    arb_segment_config.auto_calc = TRUE;
    arb_segment_config.segment = FCAN_ARB_SEGMENT;

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
        printf("phytium_can_send: FCanSend failed ret=%d\r\n", ret);
        return -3;
    }

    g_can_debug.last_send_ret = 0;
    g_can_debug.send_count++;
    printf("phytium_can_send: FCanSend ok count=%u\r\n", (unsigned)g_can_debug.send_count);
    return 0;
}
