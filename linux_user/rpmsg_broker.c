#define _GNU_SOURCE

#include "../src/rpmsg_protocol.h"
#include "../src/rpmsg_transport.h"

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <pthread.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <time.h>
#include <unistd.h>

#define BROKER_VERSION "1.0.0"
#define DEFAULT_DEVICE "/dev/rpmsg0"
#define TRANSACTION_TIMEOUT_MS 5000

typedef struct {
    int client_fd;
} ClientContext;

static int g_device_fd = -1;
static int g_server_fd = -1;
static uint8_t g_sequence;
static volatile sig_atomic_t g_stopping;
static pthread_mutex_t g_device_lock = PTHREAD_MUTEX_INITIALIZER;

static int64_t monotonic_ms(void)
{
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (int64_t)now.tv_sec * 1000 + now.tv_nsec / 1000000;
}

static void request_stop(int signo)
{
    (void)signo;
    g_stopping = 1;
    if (g_server_fd >= 0) {
        close(g_server_fd);
        g_server_fd = -1;
    }
}

static int transact(const uint8_t *request_data, size_t request_size,
                    uint8_t *response_data, size_t response_capacity,
                    size_t *response_size)
{
    uint8_t tx[RPMSG_MAX_PAYLOAD + 5U];
    uint8_t rx[RPMSG_MAX_PAYLOAD + 5U];
    RpmsgFrame request;
    RpmsgFrame response;
    uint8_t broker_sequence;
    size_t tx_size;
    int64_t deadline;
    int result = -1;

    if (!rpmsg_decode(request_data, request_size, &request)) {
        errno = EPROTO;
        return -1;
    }
    pthread_mutex_lock(&g_device_lock);
    broker_sequence = ++g_sequence;
    if (broker_sequence == 0U) {
        broker_sequence = ++g_sequence;
    }
    tx_size = rpmsg_encode(request.type, broker_sequence, request.payload,
                           request.length, tx, sizeof(tx));
    if (tx_size == 0U ||
        write(g_device_fd, tx, tx_size) != (ssize_t)tx_size) {
        goto done;
    }

    deadline = monotonic_ms() + TRANSACTION_TIMEOUT_MS;
    while (!g_stopping) {
        struct pollfd poll_fd = { .fd = g_device_fd, .events = POLLIN };
        int remaining = (int)(deadline - monotonic_ms());
        ssize_t rx_size;
        int ready;

        if (remaining <= 0) {
            errno = ETIMEDOUT;
            goto done;
        }
        ready = poll(&poll_fd, 1U, remaining);
        if (ready <= 0) {
            errno = ready == 0 ? ETIMEDOUT : errno;
            goto done;
        }
        rx_size = read(g_device_fd, rx, sizeof(rx));
        if (rx_size <= 0) {
            goto done;
        }
        if (!rpmsg_decode(rx, (size_t)rx_size, &response) ||
            response.seq != broker_sequence) {
            continue;
        }
        *response_size = rpmsg_encode(
            response.type, request.seq, response.payload, response.length,
            response_data, response_capacity);
        if (*response_size == 0U) {
            errno = EMSGSIZE;
            goto done;
        }
        result = 0;
        break;
    }

done:
    pthread_mutex_unlock(&g_device_lock);
    return result;
}

static void *serve_client(void *argument)
{
    ClientContext *context = argument;
    uint8_t request[RPMSG_MAX_PAYLOAD + 5U];
    uint8_t response[RPMSG_MAX_PAYLOAD + 5U];
    int client_fd = context->client_fd;

    free(context);
    while (!g_stopping) {
        size_t response_size = 0U;
        ssize_t request_size = recv(client_fd, request, sizeof(request), 0);
        if (request_size == 0) {
            break;
        }
        if (request_size < 0) {
            if (errno == EINTR) {
                continue;
            }
            break;
        }
        if (transact(request, (size_t)request_size, response,
                     sizeof(response), &response_size) != 0 ||
            send(client_fd, response, response_size, MSG_NOSIGNAL) !=
                (ssize_t)response_size) {
            break;
        }
    }
    close(client_fd);
    return NULL;
}

static void usage(const char *program)
{
    fprintf(stderr, "Usage: %s [--device PATH] [--socket PATH]\n", program);
}

int main(int argc, char **argv)
{
    const char *device = DEFAULT_DEVICE;
    const char *socket_path = RPMSG_BROKER_DEFAULT_SOCKET;
    struct sockaddr_un address;
    int option;

    for (option = 1; option < argc; ++option) {
        if (strcmp(argv[option], "--device") == 0 && option + 1 < argc) {
            device = argv[++option];
        } else if (strcmp(argv[option], "--socket") == 0 &&
                   option + 1 < argc) {
            socket_path = argv[++option];
        } else {
            usage(argv[0]);
            return 2;
        }
    }
    if (strlen(socket_path) >= sizeof(address.sun_path)) {
        fprintf(stderr, "broker socket path is too long\n");
        return 2;
    }

    signal(SIGINT, request_stop);
    signal(SIGTERM, request_stop);
    signal(SIGPIPE, SIG_IGN);
    g_device_fd = open(device, O_RDWR | O_CLOEXEC);
    if (g_device_fd < 0) {
        perror("open RPMsg device");
        return 1;
    }
    g_server_fd = socket(AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC, 0);
    if (g_server_fd < 0) {
        perror("create broker socket");
        close(g_device_fd);
        return 1;
    }
    memset(&address, 0, sizeof(address));
    address.sun_family = AF_UNIX;
    memcpy(address.sun_path, socket_path, strlen(socket_path) + 1U);
    unlink(socket_path);
    if (bind(g_server_fd, (const struct sockaddr *)&address,
             sizeof(address)) != 0) {
        perror("bind broker socket");
        close(g_server_fd);
        close(g_device_fd);
        unlink(socket_path);
        return 1;
    }
    if (chmod(socket_path, 0660) != 0) {
        perror("set broker socket permissions");
        close(g_server_fd);
        close(g_device_fd);
        unlink(socket_path);
        return 1;
    }
    if (listen(g_server_fd, 16) != 0) {
        perror("listen on broker socket");
        close(g_server_fd);
        close(g_device_fd);
        unlink(socket_path);
        return 1;
    }

    printf("rpmsg-broker %s: device=%s socket=%s\n",
           BROKER_VERSION, device, socket_path);
    fflush(stdout);
    while (!g_stopping) {
        ClientContext *context;
        pthread_t thread;
        int client_fd = accept4(g_server_fd, NULL, NULL, SOCK_CLOEXEC);
        if (client_fd < 0) {
            if (errno == EINTR || g_stopping) {
                continue;
            }
            perror("accept broker client");
            break;
        }
        context = malloc(sizeof(*context));
        if (context == NULL) {
            close(client_fd);
            continue;
        }
        context->client_fd = client_fd;
        if (pthread_create(&thread, NULL, serve_client, context) != 0) {
            close(client_fd);
            free(context);
            continue;
        }
        pthread_detach(thread);
    }
    if (g_server_fd >= 0) {
        close(g_server_fd);
    }
    close(g_device_fd);
    unlink(socket_path);
    return 0;
}
