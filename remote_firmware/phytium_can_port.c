#include "phytium_can_port.h"

#include "fcan.h"
#include "fio_mux.h"
#include "fcan_hw.h"
#include "fparameters.h"
#include "ftypes.h"
#include "sdkconfig.h"
#include <string.h>

#ifndef PHYTIUM_CAN_ID
#define PHYTIUM_CAN_ID FCAN0_ID
#endif

#ifndef PHYTIUM_CAN_BAUDRATE
#define PHYTIUM_CAN_BAUDRATE 1000000U
#endif

static FCanCtrl g_can;
static int g_can_ready = 0;

int phytium_can_init(void)
{
    FError ret;
    FCanBaudrateConfig arb_segment_config;
    FCanBaudrateConfig data_segment_config;

    if (g_can_ready) {
        return 0;
    }

    memset(&g_can, 0, sizeof(g_can));

    FIOMuxInit();
    FIOPadSetCanMux(PHYTIUM_CAN_ID);

    ret = FCanCfgInitialize(&g_can, FCanLookupConfig(PHYTIUM_CAN_ID));
    if (ret != FCAN_SUCCESS) {
        return -1;
    }

    FCanFdEnable(&g_can, FALSE);
    FCanSetMode(&g_can, FCAN_PROBE_NORMAL_MODE);

    memset(&arb_segment_config, 0, sizeof(arb_segment_config));
    memset(&data_segment_config, 0, sizeof(data_segment_config));

    arb_segment_config.baudrate = PHYTIUM_CAN_BAUDRATE;
    arb_segment_config.auto_calc = TRUE;
    arb_segment_config.segment = FCAN_ARB_SEGMENT;

    data_segment_config.baudrate = PHYTIUM_CAN_BAUDRATE;
    data_segment_config.auto_calc = TRUE;
    data_segment_config.segment = FCAN_DATA_SEGMENT;

    ret = FCanBaudrateSet(&g_can, &arb_segment_config);
    if (ret != FCAN_SUCCESS) {
        return -2;
    }

    ret = FCanBaudrateSet(&g_can, &data_segment_config);
    if (ret != FCAN_SUCCESS) {
        return -3;
    }

    FCanEnable(&g_can, TRUE);

    g_can_ready = 1;
    return 0;
}

int phytium_can_send(const Jc4010CanFrame *frame)
{
    FError ret;
    FCanFrame send_frame;

    if (!frame) {
        return -1;
    }

    if (!g_can_ready) {
        ret = phytium_can_init();
        if (ret != 0) {
            return -2;
        }
    }

    memset(&send_frame, 0, sizeof(send_frame));

    send_frame.canid = frame->id & CAN_SFF_MASK;
    send_frame.candlc = frame->dlc;
    for (int i = 0; i < frame->dlc && i < 8; ++i) {
        send_frame.data[i] = frame->data[i];
    }

    ret = FCanSend(&g_can, &send_frame);
    if (ret != FCAN_SUCCESS) {
        return -3;
    }

    return 0;
}
