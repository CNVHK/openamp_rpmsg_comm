#include "phytium_bmi088_port.h"

#include "fio_mux.h"
#include "fsleep.h"
#include "fspim.h"
#include "fgpio.h"
#include "ftypes.h"

#include <stdint.h>
#include <string.h>

#ifndef PHYTIUM_BMI088_SPI_ID
#define PHYTIUM_BMI088_SPI_ID FSPI0_ID
#endif

#ifndef PHYTIUM_BMI088_SPI_HZ
#define PHYTIUM_BMI088_SPI_HZ 1000000U
#endif

#ifndef PHYTIUM_BMI088_ACCEL_CS_GPIO_ID
#define PHYTIUM_BMI088_ACCEL_CS_GPIO_ID FGPIO_ID(FGPIO_CTRL_1, FGPIO_PIN_12)
#endif

#ifndef PHYTIUM_BMI088_GYRO_CS_GPIO_ID
#define PHYTIUM_BMI088_GYRO_CS_GPIO_ID FGPIO_ID(FGPIO_CTRL_4, FGPIO_PIN_12)
#endif

#ifndef PHYTIUM_BMI088_ACCEL_CS_GPIO_CTRL
#define PHYTIUM_BMI088_ACCEL_CS_GPIO_CTRL FGPIO_CTRL_1
#endif

#ifndef PHYTIUM_BMI088_ACCEL_CS_GPIO_PIN
#define PHYTIUM_BMI088_ACCEL_CS_GPIO_PIN FGPIO_PIN_12
#endif

#ifndef PHYTIUM_BMI088_GYRO_CS_GPIO_CTRL
#define PHYTIUM_BMI088_GYRO_CS_GPIO_CTRL FGPIO_CTRL_4
#endif

#ifndef PHYTIUM_BMI088_GYRO_CS_GPIO_PIN
#define PHYTIUM_BMI088_GYRO_CS_GPIO_PIN FGPIO_PIN_12
#endif

#define BMI088_ACCEL_CHIP_ID_REG 0x00U
#define BMI088_GYRO_CHIP_ID_REG  0x00U
#define BMI088_ACCEL_DATA_REG    0x12U
#define BMI088_GYRO_DATA_REG     0x02U
#define BMI088_ACCEL_CHIP_ID     0x1EU
#define BMI088_GYRO_CHIP_ID      0x0FU

typedef enum {
    BMI088_DEV_ACCEL = 0,
    BMI088_DEV_GYRO = 1
} Bmi088Dev;

static FSpim g_spim;
static FGpio g_accel_cs;
static FGpio g_gyro_cs;
static uint8_t g_ready;
static PhytiumBmi088DebugState g_dbg = {
    .init_ret = -99,
    .last_ret = -99,
};

static void bmi088_select(Bmi088Dev dev, boolean on)
{
    FGpio *cs = (dev == BMI088_DEV_ACCEL) ? &g_accel_cs : &g_gyro_cs;
    (void)FGpioSetOutputValue(cs, on ? FGPIO_PIN_LOW : FGPIO_PIN_HIGH);
}

static int bmi088_transfer(const uint8_t *tx, uint8_t *rx, uint32_t len)
{
    FError ret = FSpimTransferPollFifo(&g_spim, tx, rx, len);
    if (ret != FSPIM_SUCCESS) {
        g_dbg.last_ret = -2;
        return -2;
    }

    return 0;
}

static int bmi088_read_reg(Bmi088Dev dev, uint8_t reg, uint8_t *data, uint32_t len)
{
    uint8_t tx[16];
    uint8_t rx[16];
    uint32_t dummy = (dev == BMI088_DEV_ACCEL) ? 1U : 0U;
    uint32_t total = len + 1U + dummy;

    if (len == 0U || total > sizeof(tx)) {
        g_dbg.last_ret = -3;
        return -3;
    }

    memset(tx, 0xff, total);
    memset(rx, 0, total);
    tx[0] = (uint8_t)(reg | 0x80U);

    bmi088_select(dev, TRUE);
    int ret = bmi088_transfer(tx, rx, total);
    bmi088_select(dev, FALSE);

    if (ret != 0) {
        return ret;
    }

    memcpy(data, &rx[1U + dummy], len);
    g_dbg.last_ret = 0;
    return 0;
}

int phytium_bmi088_init(void)
{
    FSpimConfig spim_cfg;
    FError ret;

    if (g_ready) {
        g_dbg.init_ret = 0;
        return 0;
    }

    FIOMuxInit();
    FIOPadSetSpimMux(PHYTIUM_BMI088_SPI_ID);
    FIOPadSetGpioMux(PHYTIUM_BMI088_ACCEL_CS_GPIO_CTRL, PHYTIUM_BMI088_ACCEL_CS_GPIO_PIN);
    FIOPadSetGpioMux(PHYTIUM_BMI088_GYRO_CS_GPIO_CTRL, PHYTIUM_BMI088_GYRO_CS_GPIO_PIN);

    const FGpioConfig *accel_cs_cfg = FGpioLookupConfig(PHYTIUM_BMI088_ACCEL_CS_GPIO_ID);
    const FGpioConfig *gyro_cs_cfg = FGpioLookupConfig(PHYTIUM_BMI088_GYRO_CS_GPIO_ID);
    if (accel_cs_cfg == NULL || gyro_cs_cfg == NULL) {
        g_dbg.init_ret = -5;
        return -5;
    }

    if (FGpioCfgInitialize(&g_accel_cs, accel_cs_cfg) != FGPIO_SUCCESS ||
        FGpioCfgInitialize(&g_gyro_cs, gyro_cs_cfg) != FGPIO_SUCCESS) {
        g_dbg.init_ret = -6;
        return -6;
    }

    FGpioSetDirection(&g_accel_cs, FGPIO_DIR_OUTPUT);
    FGpioSetDirection(&g_gyro_cs, FGPIO_DIR_OUTPUT);
    (void)FGpioSetOutputValue(&g_accel_cs, FGPIO_PIN_HIGH);
    (void)FGpioSetOutputValue(&g_gyro_cs, FGPIO_PIN_HIGH);

    const FSpimConfig *base_cfg = FSpimLookupConfig(PHYTIUM_BMI088_SPI_ID);
    if (base_cfg == NULL) {
        g_dbg.init_ret = -1;
        return -1;
    }

    spim_cfg = *base_cfg;
    spim_cfg.en_test = FALSE;
    spim_cfg.en_dma = FALSE;
    spim_cfg.slave_dev_id = FSPIM_SLAVE_DEV_0;
    spim_cfg.cpol = FSPIM_CPOL_HIGH;
    spim_cfg.cpha = FSPIM_CPHA_2_EDGE;
    spim_cfg.n_bytes = FSPIM_1_BYTE;
    spim_cfg.sclk_hz = PHYTIUM_BMI088_SPI_HZ;
    spim_cfg.trans_way = TRANS_WAY_POLL;

    ret = FSpimCfgInitialize(&g_spim, &spim_cfg);
    if (ret != FSPIM_SUCCESS) {
        g_dbg.init_ret = -2;
        return -2;
    }

    ret = FSpimSetOption(&g_spim, FSPIM_FREQUENCY_OPTION, PHYTIUM_BMI088_SPI_HZ);
    if (ret != FSPIM_SUCCESS) {
        g_dbg.init_ret = -3;
        return -3;
    }

    FSpimSetChipSelection(&g_spim, FALSE);
    fsleep_millisec(10);

    (void)bmi088_read_reg(BMI088_DEV_ACCEL, BMI088_ACCEL_CHIP_ID_REG, &g_dbg.accel_chip_id, 1);
    (void)bmi088_read_reg(BMI088_DEV_GYRO, BMI088_GYRO_CHIP_ID_REG, &g_dbg.gyro_chip_id, 1);

    if (g_dbg.accel_chip_id != BMI088_ACCEL_CHIP_ID ||
        g_dbg.gyro_chip_id != BMI088_GYRO_CHIP_ID) {
        g_dbg.init_ret = -4;
        return -4;
    }

    g_ready = 1;
    g_dbg.init_ret = 0;
    return 0;
}

int phytium_bmi088_read_sample(void)
{
    uint8_t buf[6];
    int ret = phytium_bmi088_init();
    if (ret != 0) {
        g_dbg.last_ret = (int8_t)ret;
        return ret;
    }

    ret = bmi088_read_reg(BMI088_DEV_ACCEL, BMI088_ACCEL_DATA_REG, buf, sizeof(buf));
    if (ret != 0) {
        return ret;
    }
    g_dbg.accel_raw[0] = (int16_t)(((uint16_t)buf[1] << 8) | buf[0]);
    g_dbg.accel_raw[1] = (int16_t)(((uint16_t)buf[3] << 8) | buf[2]);
    g_dbg.accel_raw[2] = (int16_t)(((uint16_t)buf[5] << 8) | buf[4]);

    ret = bmi088_read_reg(BMI088_DEV_GYRO, BMI088_GYRO_DATA_REG, buf, sizeof(buf));
    if (ret != 0) {
        return ret;
    }
    g_dbg.gyro_raw[0] = (int16_t)(((uint16_t)buf[1] << 8) | buf[0]);
    g_dbg.gyro_raw[1] = (int16_t)(((uint16_t)buf[3] << 8) | buf[2]);
    g_dbg.gyro_raw[2] = (int16_t)(((uint16_t)buf[5] << 8) | buf[4]);
    g_dbg.read_count++;
    g_dbg.last_ret = 0;

    return 0;
}

const PhytiumBmi088DebugState *phytium_bmi088_get_debug_state(void)
{
    return &g_dbg;
}
