#define _DEFAULT_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <signal.h>
#include <time.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/stat.h>

#define SOF      0x01
#define ACK      0x06
#define NAK      0x15
#define CAN      0x18

#define REQUEST  0x00
#define MAX_FRAME 256

#define DCH_Z110_NODE3 3
#define DCH_Z110_NODE4 4

static volatile sig_atomic_t running = 1;

struct dch_z110_state {
    int contact_known;
    int contact_open;

    int battery_known;
    int battery_low;
    int battery_percent;

    int sensor03_known;
    int sensor03_value;

    int sensor01_known;
    int sensor01_raw;

    time_t last_seen;
};

static struct dch_z110_state node3;
static struct dch_z110_state node4;

static struct dch_z110_state *state_for_node(uint8_t node_id)
{
    if (node_id == DCH_Z110_NODE3)
        return &node3;

    if (node_id == DCH_Z110_NODE4)
        return &node4;

    return NULL;
}

static void handle_signal(int sig)
{
    (void)sig;
    running = 0;
}

static void timestamp(char *buf, size_t size)
{
    time_t now;
    struct tm tm_now;

    now = time(NULL);
    localtime_r(&now, &tm_now);

    strftime(buf, size, "%Y-%m-%d %H:%M:%S", &tm_now);
}

static int wait_readable(int fd, int timeout_ms)
{
    fd_set rfds;
    struct timeval tv;
    int r;

    for (;;) {
        FD_ZERO(&rfds);
        FD_SET(fd, &rfds);

        tv.tv_sec = timeout_ms / 1000;
        tv.tv_usec = (timeout_ms % 1000) * 1000;

        r = select(fd + 1, &rfds, NULL, NULL, &tv);

        if (r < 0 && errno == EINTR) {
            if (!running)
                return 0;
            continue;
        }

        return r;
    }
}

static int read_byte_timeout(int fd, uint8_t *b, int timeout_ms)
{
    int r;

    r = wait_readable(fd, timeout_ms);

    if (r <= 0)
        return r;

    for (;;) {
        ssize_t n = read(fd, b, 1);

        if (n == 1)
            return 1;

        if (n < 0 && errno == EINTR)
            continue;

        if (n < 0 &&
            (errno == EAGAIN || errno == EWOULDBLOCK))
            return 0;

        return -1;
    }
}

static int write_all(int fd, const uint8_t *buf, size_t len)
{
    size_t off = 0;

    while (off < len) {
        ssize_t n = write(fd, buf + off, len - off);

        if (n > 0) {
            off += (size_t)n;
            continue;
        }

        if (n < 0 && errno == EINTR)
            continue;

        if (n < 0 &&
            (errno == EAGAIN || errno == EWOULDBLOCK)) {
            fd_set wfds;
            struct timeval tv;
            int r;

            FD_ZERO(&wfds);
            FD_SET(fd, &wfds);

            tv.tv_sec = 1;
            tv.tv_usec = 0;

            r = select(fd + 1, NULL, &wfds, NULL, &tv);

            if (r > 0)
                continue;

            return -1;
        }

        return -1;
    }

    return 0;
}

static int send_control(int fd, uint8_t c)
{
    return write_all(fd, &c, 1);
}

static uint8_t zw_checksum(const uint8_t *p, size_t n)
{
    uint8_t c = 0xFF;
    size_t i;

    for (i = 0; i < n; i++)
        c ^= p[i];

    return c;
}

static int setup_serial(const char *dev)
{
    struct termios tio;
    int fd;
    int modem = 0;

    fd = open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);

    if (fd < 0) {
        perror("open");
        return -1;
    }

    if (tcgetattr(fd, &tio) < 0) {
        perror("tcgetattr");
        close(fd);
        return -1;
    }

    cfmakeraw(&tio);

    tio.c_cflag &= ~(CSIZE | PARENB | CSTOPB);

#ifdef CRTSCTS
    tio.c_cflag &= ~CRTSCTS;
#endif

    tio.c_cflag |= CS8 | CLOCAL | CREAD;

    if (cfsetispeed(&tio, B115200) < 0 ||
        cfsetospeed(&tio, B115200) < 0) {
        perror("cfsetspeed");
        close(fd);
        return -1;
    }

    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;

    if (tcsetattr(fd, TCSANOW, &tio) < 0) {
        perror("tcsetattr");
        close(fd);
        return -1;
    }

    tcflush(fd, TCIOFLUSH);

    if (ioctl(fd, TIOCMGET, &modem) == 0) {
        modem &= ~(TIOCM_DTR | TIOCM_RTS);
        ioctl(fd, TIOCMSET, &modem);
    }

    return fd;
}

static int receive_frame(int fd,
                         uint8_t *frame,
                         size_t frame_size,
                         size_t *frame_len)
{
    uint8_t b;
    uint8_t len;
    size_t total;
    size_t pos;
    int r;

    *frame_len = 0;

    while (running) {
        r = read_byte_timeout(fd, &b, 500);

        if (r < 0)
            return -1;

        if (r == 0)
            continue;

        if (b == SOF)
            break;
    }

    if (!running)
        return 1;

    frame[0] = SOF;

    r = read_byte_timeout(fd, &len, 1000);

    if (r != 1)
        return -1;

    total = (size_t)len + 2;

    if (len < 3 || total > frame_size) {
        send_control(fd, NAK);
        return -1;
    }

    frame[1] = len;
    pos = 2;

    while (pos < total) {
        r = read_byte_timeout(fd, &frame[pos], 1000);

        if (r != 1)
            return -1;

        pos++;
    }

    if (zw_checksum(&frame[1], total - 2) !=
        frame[total - 1]) {
        send_control(fd, NAK);
        return -1;
    }

    if (send_control(fd, ACK) < 0)
        return -1;

    *frame_len = total;

    return 0;
}


/*
 * BERNAL HOME JSON STATE v0.2
 *
 * Runtime state is written under /tmp (RAM-backed on OpenWrt).
 * Atomic publication:
 *
 *   state.json.tmp -> rename() -> state.json
 *
 * No flash writes for normal sensor events.
 */
#define BERNAL_STATE_DIR  "/tmp/bernal-home"
#define BERNAL_STATE_FILE "/tmp/bernal-home/state.json"
#define BERNAL_STATE_TMP  "/tmp/bernal-home/state.json.tmp"
#define BERNAL_HISTORY_FILE "/tmp/bernal-home/history.jsonl"

static int ensure_state_directory(void)
{
    if (mkdir(BERNAL_STATE_DIR, 0755) == 0)
        return 0;

    if (errno == EEXIST)
        return 0;

    perror("mkdir bernal-home");
    return -1;
}

static void write_node_json(FILE *fp,
                            uint8_t node_id,
                            const struct dch_z110_state *state,
                            int trailing_comma)
{
    char last_seen[32];

    if (state->last_seen != 0) {
        struct tm tm_seen;

        localtime_r(&state->last_seen, &tm_seen);
        strftime(last_seen,
                 sizeof(last_seen),
                 "%Y-%m-%d %H:%M:%S",
                 &tm_seen);
    } else {
        snprintf(last_seen, sizeof(last_seen), "never");
    }

    fprintf(fp, "    \"%u\": {\n",
            (unsigned int)node_id);
    fprintf(fp, "      \"node\": %u,\n",
            (unsigned int)node_id);
    fprintf(fp, "      \"device\": \"DCH-Z110\",\n");
    fprintf(fp, "      \"online\": %s,\n",
            state->last_seen != 0 ? "true" : "false");

    fprintf(fp, "      \"contact\": \"%s\",\n",
            !state->contact_known ? "unknown" :
            state->contact_open ? "open" : "closed");

    fprintf(fp, "      \"battery_known\": %s,\n",
            state->battery_known ? "true" : "false");

    fprintf(fp, "      \"battery_low\": %s,\n",
            state->battery_low ? "true" : "false");

    if (state->battery_known &&
        !state->battery_low &&
        state->battery_percent >= 0)
        fprintf(fp, "      \"battery_percent\": %d,\n",
                state->battery_percent);
    else
        fprintf(fp, "      \"battery_percent\": null,\n");

    if (state->sensor03_known) {
        fprintf(fp, "      \"sensor03\": %d,\n",
                state->sensor03_value);
        fprintf(fp, "      \"luminance_percent\": %d,\n",
                state->sensor03_value);
    } else {
        fprintf(fp, "      \"sensor03\": null,\n");
        fprintf(fp, "      \"luminance_percent\": null,\n");
    }

    if (state->sensor01_known) {
        fprintf(fp, "      \"sensor01_raw\": %d,\n",
                state->sensor01_raw);
        fprintf(fp, "      \"temperature_f\": %d,\n",
                state->sensor01_raw);
        fprintf(fp, "      \"temperature_c\": %.1f,\n",
                (state->sensor01_raw - 32.0) * 5.0 / 9.0);
    } else {
        fprintf(fp, "      \"sensor01_raw\": null,\n");
        fprintf(fp, "      \"temperature_f\": null,\n");
        fprintf(fp, "      \"temperature_c\": null,\n");
    }

    fprintf(fp, "      \"last_seen\": \"%s\"\n",
            last_seen);

    fprintf(fp, "    }%s\n",
            trailing_comma ? "," : "");
}

static int write_state_json(void)
{
    FILE *fp;

    if (ensure_state_directory() < 0)
        return -1;

    fp = fopen(BERNAL_STATE_TMP, "w");

    if (!fp) {
        perror("fopen state.json.tmp");
        return -1;
    }

    fprintf(fp, "{\n");
    fprintf(fp, "  \"controller\": \"DCH-G020\",\n");
    fprintf(fp, "  \"nodes\": {\n");

    write_node_json(fp, DCH_Z110_NODE3, &node3, 1);
    write_node_json(fp, DCH_Z110_NODE4, &node4, 0);

    fprintf(fp, "  }\n");
    fprintf(fp, "}\n");

    if (fclose(fp) != 0) {
        perror("fclose state.json.tmp");
        unlink(BERNAL_STATE_TMP);
        return -1;
    }

    if (rename(BERNAL_STATE_TMP,
               BERNAL_STATE_FILE) != 0) {
        perror("rename state.json");
        unlink(BERNAL_STATE_TMP);
        return -1;
    }

    return 0;
}

static void append_history_event(uint8_t node_id,
                                 int contact_open)
{
    FILE *fp;
    char ts[32];
    const char *name;

    if (ensure_state_directory() < 0)
        return;

    timestamp(ts, sizeof(ts));

    if (node_id == DCH_Z110_NODE4)
        name = "Puerta principal";
    else if (node_id == DCH_Z110_NODE3)
        name = "Puerta pérgola";
    else
        name = "Desconocida";

    fp = fopen(BERNAL_HISTORY_FILE, "a");

    if (!fp) {
        perror("fopen history.jsonl");
        return;
    }

    fprintf(fp,
            "{\"time\":\"%s\",\"node\":%u,"
            "\"name\":\"%s\",\"state\":\"%s\"}\n",
            ts,
            (unsigned int)node_id,
            name,
            contact_open ? "open" : "closed");

    fclose(fp);
}

static void print_contact_state(uint8_t node_id,
                                struct dch_z110_state *state)
{
    char ts[32];

    timestamp(ts, sizeof(ts));

    printf("[%s] NODE%u CONTACT %s\n",
           ts,
           (unsigned int)node_id,
           state->contact_open ? "OPEN" : "CLOSED");

    fflush(stdout);
}

static void decode_embedded(uint8_t node_id,
                            struct dch_z110_state *state,
                            const uint8_t *cmd,
                            size_t len)
{
    if (!cmd || len < 2)
        return;

    if (cmd[0] == 0x80 &&
        cmd[1] == 0x03 &&
        len >= 3) {

        state->battery_known = 1;

        if (cmd[2] == 0xFF) {
            /*
             * DCH-Z110 observed value. Do not interpret 0xFF as
             * low battery until its behaviour is validated.
             */
            state->battery_low = 0;
            state->battery_percent = -1;
        } else if (cmd[2] <= 100) {
            state->battery_low = 0;
            state->battery_percent = cmd[2];
        }

        return;
    }

    /*
     * Experimental mapping confirmed physically on Node4:
     *
     * notification event 0x16 = magnet removed = OPEN
     * notification event 0x17 = magnet present = CLOSED
     */
    if (cmd[0] == 0x71 &&
        cmd[1] == 0x05 &&
        len >= 8 &&
        cmd[6] == 0x06) {

        if (cmd[7] == 0x16) {
            if (!state->contact_known ||
                !state->contact_open)
                append_history_event(node_id, 1);

            state->contact_known = 1;
            state->contact_open = 1;
            print_contact_state(node_id, state);
        } else if (cmd[7] == 0x17) {
            if (!state->contact_known ||
                state->contact_open)
                append_history_event(node_id, 0);

            state->contact_known = 1;
            state->contact_open = 0;
            print_contact_state(node_id, state);
        }

        return;
    }

    if (cmd[0] == 0x31 &&
        cmd[1] == 0x05 &&
        len >= 5) {

        if (cmd[2] == 0x03 &&
            cmd[3] == 0x01) {
            state->sensor03_known = 1;
            state->sensor03_value = cmd[4];
        }

        if (cmd[2] == 0x01 &&
            cmd[3] == 0x0A &&
            len >= 6) {
            state->sensor01_known = 1;
            state->sensor01_raw =
                ((int)cmd[4] << 8) |
                (int)cmd[5];
        }
    }
}

static void decode_multi_cmd(uint8_t node_id,
                             struct dch_z110_state *state,
                             const uint8_t *cmd,
                             size_t len)
{
    uint8_t count;
    size_t pos;
    unsigned int i;

    if (!cmd || len < 3)
        return;

    if (cmd[0] != 0x8F || cmd[1] != 0x01)
        return;

    count = cmd[2];
    pos = 3;

    for (i = 0; i < count; i++) {
        uint8_t embedded_len;

        if (pos >= len)
            return;

        embedded_len = cmd[pos++];

        if (embedded_len == 0 ||
            (size_t)embedded_len > len - pos)
            return;

        decode_embedded(node_id, state,
                        &cmd[pos], embedded_len);

        pos += embedded_len;
    }
}

static void decode_application_command(const uint8_t *frame,
                                       size_t frame_len)
{
    const uint8_t *d;
    size_t data_len;
    uint8_t source_node;
    uint8_t command_len;
    const uint8_t *command;
    struct dch_z110_state *state;

    if (!frame || frame_len < 8)
        return;

    if (frame[0] != SOF ||
        frame[2] != REQUEST ||
        frame[3] != 0x04)
        return;

    d = &frame[4];
    data_len = frame_len - 5;

    if (data_len < 3)
        return;

    source_node = d[1];
    command_len = d[2];

    if ((size_t)command_len > data_len - 3)
        return;

    command = &d[3];
    state = state_for_node(source_node);

    if (!state)
        return;

    state->last_seen = time(NULL);

    if (command_len >= 3 &&
        command[0] == 0x8F &&
        command[1] == 0x01) {

        decode_multi_cmd(source_node,
                         state,
                         command,
                         command_len);

        if (write_state_json() < 0)
            fprintf(stderr,
                    "[!] Could not publish Bernal Home state\n");
    }
}

static void print_node_summary(uint8_t node_id,
                               const struct dch_z110_state *state)
{
    char ts[32];

    printf(" NODE%u / DCH-Z110\n",
           (unsigned int)node_id);

    if (state->contact_known)
        printf(" Contact     : %s\n",
               state->contact_open ? "OPEN" : "CLOSED");
    else
        printf(" Contact     : UNKNOWN\n");

    if (state->battery_known) {
        if (state->battery_low)
            printf(" Battery     : LOW WARNING\n");
        else if (state->battery_percent >= 0)
            printf(" Battery     : %d%%\n",
                   state->battery_percent);
        else
            printf(" Battery     : NOT CALIBRATED\n");
    } else {
        printf(" Battery     : UNKNOWN\n");
    }

    if (state->sensor03_known)
        printf(" Luminance   : %d%%\n",
               state->sensor03_value);
    else
        printf(" Luminance   : UNKNOWN\n");

    if (state->sensor01_known)
        printf(" Temperature : %.1f C\n",
               (state->sensor01_raw - 32.0) * 5.0 / 9.0);
    else
        printf(" Temperature : UNKNOWN\n");

    if (state->last_seen != 0) {
        struct tm tm_seen;

        localtime_r(&state->last_seen, &tm_seen);
        strftime(ts, sizeof(ts),
                 "%Y-%m-%d %H:%M:%S", &tm_seen);

        printf(" Last seen   : %s\n", ts);
    } else {
        printf(" Last seen   : NEVER\n");
    }
}

static void print_summary(void)
{
    printf("\n");
    printf("========================================\n");
    printf(" BERNAL HOME - MULTI-NODE STATE\n");
    printf("========================================\n");

    print_node_summary(DCH_Z110_NODE3, &node3);

    printf("----------------------------------------\n");

    print_node_summary(DCH_Z110_NODE4, &node4);

    printf("========================================\n");
}

int main(int argc, char **argv)
{
    const char *device = "/dev/ttyACM0";
    uint8_t frame[MAX_FRAME];
    size_t frame_len;
    int fd;

    if (argc == 2)
        device = argv[1];

    memset(&node3, 0, sizeof(node3));
    memset(&node4, 0, sizeof(node4));

    /*
     * Publish an initial UNKNOWN/OFFLINE state before the
     * first Z-Wave event arrives.
     */
    if (write_state_json() < 0)
        fprintf(stderr,
                "[!] Initial Bernal Home state unavailable\n");

    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);

    printf("========================================\n");
    printf(" BERNAL Z-WAVE DAEMON v0.4 MULTI-NODE\n");
    printf("========================================\n");
    printf(" Serial API : %s\n", device);
    printf(" Nodes      : 3 + 4 / DCH-Z110\n");
    printf(" Mode       : PASSIVE\n");
    printf(" RF TX      : NONE\n");
    printf("========================================\n");

    fd = setup_serial(device);

    if (fd < 0)
        return 1;

    printf("[+] Serial API ready\n");
    printf("[+] Waiting for Node3 + Node4 events...\n\n");
    fflush(stdout);

    while (running) {
        int r;

        r = receive_frame(fd,
                          frame,
                          sizeof(frame),
                          &frame_len);

        if (r == 1)
            break;

        if (r < 0) {
            if (running)
                fprintf(stderr,
                        "[!] Serial frame receive error\n");
            continue;
        }

        decode_application_command(frame, frame_len);
    }

    print_summary();

    close(fd);

    printf("[+] Bernal Z-Wave daemon stopped\n");

    return 0;
}
