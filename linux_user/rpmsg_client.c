#include "../src/rpmsg_protocol.h"
#include <fcntl.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>

/* Minimal Linux-side rpmsg client skeleton.
 * Build on board after /dev/rpmsgX is created:
 * gcc linux_user/rpmsg_client.c src/rpmsg_protocol.c -I./src -o rpmsg_client
 */
int main(int argc, char **argv)
{
    const char *dev = argc > 1 ? argv[1] : "/dev/rpmsg0";
    int fd = open(dev, O_RDWR);
    if (fd < 0) {
        perror("open rpmsg device");
        return 1;
    }

    uint8_t payload[] = {0};
    uint8_t frame[32];
    size_t size = rpmsg_encode(CMD_HEARTBEAT, 1, payload, sizeof(payload), frame, sizeof(frame));

    if (write(fd, frame, size) != (ssize_t)size) {
        perror("write");
        close(fd);
        return 1;
    }

    printf("heartbeat frame sent to %s\n", dev);
    close(fd);
    return 0;
}

