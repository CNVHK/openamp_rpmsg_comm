#include "phytium_can_port.h"

int phytium_can_init(void)
{
    /*
     * TODO: Fill this function with the real Phytium Standalone SDK CAN init.
     *
     * Integration checklist:
     * 1. Enable CAN in the standalone config.
     * 2. Configure CAN pins / IOPAD according to the Phytium Pi schematic.
     * 3. Set nominal bitrate to match the motor bus.
     * 4. Configure RX filters for 0x581 and 0x582 if feedback is needed.
     * 5. Start CAN controller and enable RX interrupt or polling.
     */
    return 0;
}

int phytium_can_send(const Jc4010CanFrame *frame)
{
    /*
     * TODO: Replace with the real CAN send API from Phytium Standalone SDK.
     *
     * Required frame fields:
     *   frame->id      standard CAN ID, e.g. 0x601 or 0x602
     *   frame->dlc     8
     *   frame->data[]  classic CAN payload
     *
     * This placeholder returns success so protocol code can compile first.
     */
    (void)frame;
    return 0;
}

