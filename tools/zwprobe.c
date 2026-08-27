#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <sys/select.h>

#define SOF 0x01
#define ACK 0x06
#define NAK 0x15
#define CAN 0x18

#define REQUEST  0x00
#define RESPONSE 0x01

#define MAX_FRAME 256
#define MAX_RETRIES 3

static uint8_t discovered_nodes[232];
static size_t discovered_node_count = 0;

static void dump_hex(const char *tag, const uint8_t *b, size_t n)
{
    size_t i;

    printf("%s (%zu bytes):", tag, n);
    for (i = 0; i < n; i++)
        printf(" %02X", b[i]);
    printf("\n");
}

/*
 * Z-Wave Serial API checksum:
 *
 *   0xFF XOR LEN XOR TYPE XOR FUNC XOR DATA...
 *
 * El SOF no participa.
 */
static uint8_t zw_checksum(const uint8_t *p, size_t n)
{
    uint8_t c = 0xFF;
    size_t i;

    for (i = 0; i < n; i++)
        c ^= p[i];

    return c;
}

static int wait_readable(int fd, int timeout_ms)
{
    fd_set rfds;
    struct timeval tv;
    int r;

    for (;;) {
        FD_ZERO(&rfds);
        FD_SET(fd, &rfds);

        tv.tv_sec  = timeout_ms / 1000;
        tv.tv_usec = (timeout_ms % 1000) * 1000;

        r = select(fd + 1, &rfds, NULL, NULL, &tv);

        if (r < 0 && errno == EINTR)
            continue;

        return r;
    }
}

static int read_byte_timeout(int fd, uint8_t *b, int timeout_ms)
{
    int r;

    r = wait_readable(fd, timeout_ms);

    if (r == 0)
        return 0;

    if (r < 0) {
        perror("select");
        return -1;
    }

    for (;;) {
        ssize_t n = read(fd, b, 1);

        if (n == 1)
            return 1;

        if (n < 0 && errno == EINTR)
            continue;

        if (n < 0 &&
            (errno == EAGAIN || errno == EWOULDBLOCK))
            return 0;

        if (n < 0)
            perror("read");

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

            fprintf(stderr, "[-] timeout escribiendo puerto\n");
            return -1;
        }

        perror("write");
        return -1;
    }

    return 0;
}

static int send_control(int fd, uint8_t c)
{
    if (write_all(fd, &c, 1) < 0)
        return -1;

    printf("TX CONTROL: %02X (%s)\n",
           c,
           c == ACK ? "ACK" :
           c == NAK ? "NAK" :
           c == CAN ? "CAN" : "?");

    return 0;
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

    printf("[+] open(%s) OK, fd=%d\n", dev, fd);

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

    if (cfsetispeed(&tio, B115200) < 0) {
        perror("cfsetispeed");
        close(fd);
        return -1;
    }

    if (cfsetospeed(&tio, B115200) < 0) {
        perror("cfsetospeed");
        close(fd);
        return -1;
    }

    tio.c_cc[VMIN]  = 0;
    tio.c_cc[VTIME] = 0;

    if (tcsetattr(fd, TCSANOW, &tio) < 0) {
        perror("tcsetattr");
        close(fd);
        return -1;
    }

    /*
     * Limpiamos cualquier byte residual ANTES de comenzar
     * una sesión Serial API nueva.
     */
    tcflush(fd, TCIOFLUSH);

    printf("[+] termios: 115200 8N1 raw, CLOCAL+CREAD\n");

    if (ioctl(fd, TIOCMGET, &modem) == 0) {
        printf("[+] modem bits antes: 0x%08X\n", modem);

        modem &= ~(TIOCM_DTR | TIOCM_RTS);

        if (ioctl(fd, TIOCMSET, &modem) < 0) {
            perror("TIOCMSET");
        } else {
            printf("[+] DTR=0 RTS=0\n");
        }

        if (ioctl(fd, TIOCMGET, &modem) == 0)
            printf("[+] modem bits despues: 0x%08X\n", modem);
    } else {
        perror("TIOCMGET");
        printf("[!] driver sin modem bits; continuamos\n");
    }

    return fd;
}

/*
 * Espera el ACK/NAK/CAN correspondiente a nuestra petición.
 *
 * Si aparece SOF antes del ACK, lo señalamos de forma distinta
 * para no consumir silenciosamente una respuesta.
 */
static int wait_request_ack(int fd, uint8_t *early_sof)
{
    uint8_t b;
    int i;

    *early_sof = 0;

    for (i = 0; i < 20; i++) {
        int r = read_byte_timeout(fd, &b, 100);

        if (r < 0)
            return -1;

        if (r == 0)
            continue;

        printf("RX CONTROL: %02X", b);

        if (b == ACK) {
            printf(" (ACK)\n");
            return ACK;
        }

        if (b == NAK) {
            printf(" (NAK)\n");
            return NAK;
        }

        if (b == CAN) {
            printf(" (CAN)\n");
            return CAN;
        }

        if (b == SOF) {
            printf(" (SOF temprano)\n");
            *early_sof = 1;
            return SOF;
        }

        printf(" (ignorado esperando ACK)\n");
    }

    printf("[-] timeout esperando ACK\n");
    return 0;
}

/*
 * Lee una trama Serial API completa.
 *
 * Al entrar:
 *   have_sof=1 -> SOF ya fue consumido.
 *   have_sof=0 -> buscamos SOF.
 */
static int receive_frame(int fd,
                         uint8_t *frame,
                         size_t frame_size,
                         size_t *frame_len,
                         int have_sof)
{
    uint8_t b;
    uint8_t len;
    size_t total;
    size_t pos;
    int r;
    int attempts;

    *frame_len = 0;

    if (!have_sof) {
        for (attempts = 0; attempts < 30; attempts++) {
            r = read_byte_timeout(fd, &b, 100);

            if (r < 0)
                return -1;

            if (r == 0)
                continue;

            if (b == SOF)
                break;

            if (b == ACK || b == NAK || b == CAN) {
                printf("[!] control inesperado antes de SOF: %02X\n", b);
                continue;
            }

            printf("[!] byte inesperado antes de SOF: %02X\n", b);
        }

        if (attempts == 30) {
            printf("[-] timeout esperando SOF\n");
            return -1;
        }
    }

    frame[0] = SOF;

    r = read_byte_timeout(fd, &len, 1000);

    if (r != 1) {
        printf("[-] no se pudo leer LENGTH\n");
        return -1;
    }

    /*
     * LENGTH cuenta TYPE + FUNC + DATA + CHECKSUM.
     * Total almacenado = SOF + LENGTH-byte + LENGTH bytes.
     */
    total = (size_t)len + 2;

    if (len < 3 || total > frame_size) {
        printf("[-] LENGTH invalido: %u\n", len);
        send_control(fd, NAK);
        return -1;
    }

    frame[1] = len;
    pos = 2;

    while (pos < total) {
        r = read_byte_timeout(fd, &frame[pos], 1000);

        if (r != 1) {
            printf("[-] timeout leyendo trama (%zu/%zu)\n",
                   pos, total);
            return -1;
        }

        pos++;
    }

    *frame_len = total;

    dump_hex("RX FRAME", frame, total);

    /*
     * Checksum recibido = último byte.
     * Calculamos sobre LENGTH, TYPE, FUNC y DATA.
     */
    if (zw_checksum(&frame[1], total - 2) !=
        frame[total - 1]) {

        printf("[-] CHECKSUM incorrecto: calc=%02X rx=%02X\n",
               zw_checksum(&frame[1], total - 2),
               frame[total - 1]);

        send_control(fd, NAK);
        return -1;
    }

    printf("[+] checksum OK\n");

    /*
     * Confirmamos inmediatamente la trama válida.
     */
    if (send_control(fd, ACK) < 0)
        return -1;

    return 0;
}

/*
 * Construye una trama REQUEST Serial API con DATA.
 *
 * IMPORTANTE:
 *   Esta funcion NO escribe en el puerto.
 *   Solo construye bytes en memoria.
 *
 * Frame:
 *
 *   SOF LEN REQUEST FUNC DATA... CHECKSUM
 */
static int build_request_frame(uint8_t func,
                               const uint8_t *data,
                               size_t data_len,
                               uint8_t *frame,
                               size_t frame_size,
                               size_t *frame_len)
{
    size_t total;

    if (!frame || !frame_len) {
        printf("[-] build_request_frame: argumento invalido\n");
        return -1;
    }

    if (data_len && !data) {
        printf("[-] build_request_frame: DATA NULL\n");
        return -1;
    }

    if (data_len > MAX_FRAME - 5) {
        printf("[-] build_request_frame: payload demasiado grande\n");
        return -1;
    }

    total = data_len + 5;

    if (total > frame_size) {
        printf("[-] build_request_frame: buffer insuficiente\n");
        return -1;
    }

    frame[0] = SOF;
    frame[1] = (uint8_t)(data_len + 3);
    frame[2] = REQUEST;
    frame[3] = func;

    if (data_len)
        memcpy(&frame[4], data, data_len);

    frame[4 + data_len] =
        zw_checksum(&frame[1], data_len + 3);

    *frame_len = total;

    return 0;
}


/*
 * V6.1:
 *
 * Construimos las tramas de ADD_NODE START/STOP,
 * pero NO abrimos ni escribimos el puerto Z-Wave.
 *
 * START:
 *   FUNC = 0x4A
 *   mode = ADD_NODE_ANY (0x01)
 *   callback id = 0x01
 *
 * STOP:
 *   FUNC = 0x4A
 *   mode = ADD_NODE_STOP (0x05)
 *   callback id = 0x01
 */

/*
 * Estados reportados por ZW_ADD_NODE_TO_NETWORK.
 *
 * Parser OFFLINE: no toca el puerto serie.
 */
static const char *add_node_status_name(uint8_t status)
{
    switch (status) {
    case 0x01:
        return "LEARN_READY";
    case 0x02:
        return "NODE_FOUND";
    case 0x03:
        return "ADDING_SLAVE";
    case 0x04:
        return "ADDING_CONTROLLER";
    case 0x05:
        return "PROTOCOL_DONE";
    case 0x06:
        return "DONE";
    case 0x07:
        return "FAILED";
    default:
        return "UNKNOWN";
    }
}

static int decode_add_node_callback(const uint8_t *f, size_t n)
{
    const uint8_t *d;
    size_t data_len;
    uint8_t callback_id;
    uint8_t status;

    if (!f || n < 7) {
        printf("[-] ADD_NODE callback demasiado corto\n");
        return 1;
    }

    if (f[0] != SOF) {
        printf("[-] ADD_NODE callback sin SOF\n");
        return 1;
    }

    if (f[2] != REQUEST) {
        printf("[-] ADD_NODE callback TYPE inesperado: %02X\n",
               f[2]);
        return 1;
    }

    if (f[3] != 0x4A) {
        printf("[-] ADD_NODE callback FUNC inesperado: %02X\n",
               f[3]);
        return 1;
    }

    /*
     * Callback Serial API:
     *
     * DATA[0] = callback/function id
     * DATA[1] = status
     * DATA[...] depende del estado.
     */
    d = &f[4];
    data_len = n - 5;

    if (data_len < 2) {
        printf("[-] ADD_NODE callback sin status\n");
        return 1;
    }

    callback_id = d[0];
    status = d[1];

    printf("[+] ADD_NODE callback id     : 0x%02X\n",
           callback_id);

    printf("[+] ADD_NODE status          : 0x%02X (%s)\n",
           status,
           add_node_status_name(status));

    /*
     * Algunos estados incluyen Node ID a continuación.
     * De momento solo lo mostramos, sin interpretar todavía
     * el resto del payload.
     */
    if (data_len >= 3) {
        printf("[+] ADD_NODE node id         : %u (0x%02X)\n",
               d[2], d[2]);
    }

    if (data_len > 3) {
        size_t i;

        printf("[+] ADD_NODE extra data      :");

        for (i = 3; i < data_len; i++)
            printf(" %02X", d[i]);

        printf("\n");
    }

    return 0;
}


/*
 * Construye un callback REQUEST 0x4A sintético y lo pasa
 * por el decoder.
 *
 * TOTALMENTE OFFLINE:
 *   - no abre ttyACM0
 *   - no escribe puerto serie
 *   - no modifica ninguna red Z-Wave
 */
static int selftest_add_node_frame(uint8_t callback_id,
                                   uint8_t status,
                                   uint8_t node_id,
                                   int include_node)
{
    uint8_t frame[MAX_FRAME];
    size_t data_len;
    size_t total;

    /*
     * Callback:
     *
     * SOF LEN REQUEST 4A CALLBACK_ID STATUS [NODE] CHECKSUM
     */
    frame[0] = SOF;
    frame[2] = REQUEST;
    frame[3] = 0x4A;
    frame[4] = callback_id;
    frame[5] = status;

    data_len = 2;

    if (include_node) {
        frame[6] = node_id;
        data_len++;
    }

    /*
     * LEN cuenta:
     *   TYPE + FUNC + DATA + CHECKSUM
     */
    frame[1] = (uint8_t)(data_len + 3);

    total = data_len + 5;

    frame[total - 1] =
        zw_checksum(&frame[1], total - 2);

    dump_hex("SELFTEST RX FRAME", frame, total);

    /*
     * Verificación independiente usando exactamente la
     * misma convención empleada por receive_frame().
     */
    if (zw_checksum(&frame[1], total - 2) !=
        frame[total - 1]) {
        printf("[-] SELFTEST checksum incorrecto\n");
        return 1;
    }

    printf("[+] SELFTEST checksum OK\n");

    return decode_add_node_callback(frame, total);
}

static int run_add_node_callback_selftest(void)
{
    struct test_case {
        uint8_t status;
        uint8_t node;
        int include_node;
    };

    static const struct test_case tests[] = {
        { 0x01, 0x00, 0 },  /* LEARN_READY */
        { 0x02, 0x00, 0 },  /* NODE_FOUND */
        { 0x03, 0x02, 1 },  /* ADDING_SLAVE */
        { 0x05, 0x02, 1 },  /* PROTOCOL_DONE */
        { 0x06, 0x02, 1 },  /* DONE */
        { 0x07, 0x00, 0 }   /* FAILED */
    };

    const uint8_t callback_id = 0x01;
    size_t i;

    printf("\n");
    printf("========================================\n");
    printf(" V6 ADD_NODE CALLBACK SELFTEST\n");
    printf("========================================\n");
    printf("[+] OFFLINE: no se abrira ttyACM0\n");
    printf("[+] OFFLINE: no se transmitira ningun byte\n");

    for (i = 0; i < sizeof(tests) / sizeof(tests[0]); i++) {
        printf("\n");
        printf("----------------------------------------\n");
        printf(" TEST %zu: %s\n",
               i + 1,
               add_node_status_name(tests[i].status));
        printf("----------------------------------------\n");

        if (selftest_add_node_frame(callback_id,
                                    tests[i].status,
                                    tests[i].node,
                                    tests[i].include_node)) {
            printf("[-] SELFTEST fallo en TEST %zu\n", i + 1);
            return 1;
        }
    }

    printf("\n");
    printf("========================================\n");
    printf(" CALLBACK SELFTEST COMPLETE: %zu/6\n",
           sizeof(tests) / sizeof(tests[0]));
    printf("========================================\n");

    return 0;
}



/*
 * ============================================================
 * V6.4 - ADD_NODE OFFLINE STATE MACHINE
 * ============================================================
 *
 * No abre ttyACM0.
 * No transmite bytes.
 * No modifica la red Z-Wave.
 */

enum add_node_sm_state {
    ADD_SM_IDLE = 0,
    ADD_SM_LEARN_READY,
    ADD_SM_NODE_FOUND,
    ADD_SM_ADDING_NODE,
    ADD_SM_PROTOCOL_DONE,
    ADD_SM_DONE,
    ADD_SM_FAILED
};

struct add_node_sm {
    uint8_t callback_id;
    uint8_t node_id;
    int have_node;
    enum add_node_sm_state state;
};

static const char *add_node_sm_state_name(enum add_node_sm_state state)
{
    switch (state) {
    case ADD_SM_IDLE:
        return "IDLE";
    case ADD_SM_LEARN_READY:
        return "LEARN_READY";
    case ADD_SM_NODE_FOUND:
        return "NODE_FOUND";
    case ADD_SM_ADDING_NODE:
        return "ADDING_NODE";
    case ADD_SM_PROTOCOL_DONE:
        return "PROTOCOL_DONE";
    case ADD_SM_DONE:
        return "DONE";
    case ADD_SM_FAILED:
        return "FAILED";
    default:
        return "UNKNOWN";
    }
}

static int add_node_sm_feed(struct add_node_sm *sm,
                            uint8_t callback_id,
                            uint8_t status,
                            int have_node,
                            uint8_t node_id)
{
    enum add_node_sm_state old_state;

    if (!sm)
        return 1;

    if (callback_id != sm->callback_id) {
        printf("[-] SM callback ID incorrecto: "
               "esperado=0x%02X recibido=0x%02X\n",
               sm->callback_id, callback_id);
        return 1;
    }

    old_state = sm->state;

    printf("[+] SM status                : 0x%02X (%s)\n",
           status, add_node_status_name(status));

    switch (status) {
    case 0x01:
        if (sm->state != ADD_SM_IDLE)
            return 1;
        sm->state = ADD_SM_LEARN_READY;
        break;

    case 0x02:
        if (sm->state != ADD_SM_LEARN_READY)
            return 1;
        sm->state = ADD_SM_NODE_FOUND;
        break;

    case 0x03:
    case 0x04:
        if (sm->state != ADD_SM_NODE_FOUND)
            return 1;

        if (!have_node || node_id == 0 || node_id > 232)
            return 1;

        sm->node_id = node_id;
        sm->have_node = 1;
        sm->state = ADD_SM_ADDING_NODE;
        break;

    case 0x05:
        if (sm->state != ADD_SM_ADDING_NODE)
            return 1;

        if (have_node && sm->have_node &&
            node_id != sm->node_id)
            return 1;

        sm->state = ADD_SM_PROTOCOL_DONE;
        break;

    case 0x06:
        if (sm->state != ADD_SM_PROTOCOL_DONE)
            return 1;

        if (have_node && sm->have_node &&
            node_id != sm->node_id)
            return 1;

        sm->state = ADD_SM_DONE;
        break;

    case 0x07:
        if (sm->state == ADD_SM_IDLE ||
            sm->state == ADD_SM_DONE ||
            sm->state == ADD_SM_FAILED)
            return 1;

        sm->state = ADD_SM_FAILED;
        break;

    default:
        return 1;
    }

    printf("[+] SM transition            : %s -> %s\n",
           add_node_sm_state_name(old_state),
           add_node_sm_state_name(sm->state));

    if (sm->have_node)
        printf("[+] SM node                  : %u (0x%02X)\n",
               sm->node_id, sm->node_id);

    return 0;
}


/*
 * ============================================================
 * V6.5 - ADD_NODE CALLBACK PIPELINE OFFLINE
 * ============================================================
 *
 * Une:
 *
 *   Serial API frame
 *          |
 *          v
 *   validacion REQUEST / FUNC 0x4A
 *          |
 *          v
 *   extraccion callback/status/node
 *          |
 *          v
 *   ADD_NODE state machine
 *
 * OFFLINE:
 *   - no abre ttyACM0
 *   - no transmite bytes
 *   - no modifica la red Z-Wave
 */

static int add_node_process_frame(struct add_node_sm *sm,
                                  const uint8_t *f,
                                  size_t n)
{
    const uint8_t *d;
    size_t data_len;
    uint8_t callback_id;
    uint8_t status;
    uint8_t node_id = 0;
    int have_node = 0;

    if (!sm || !f || n < 7) {
        printf("[-] PIPELINE frame demasiado corto\n");
        return 1;
    }

    if (f[0] != SOF) {
        printf("[-] PIPELINE sin SOF\n");
        return 1;
    }

    /*
     * LENGTH + SOF/LENGTH deben describir exactamente
     * el tamaño del frame recibido.
     */
    if ((size_t)f[1] + 2 != n) {
        printf("[-] PIPELINE LENGTH inconsistente: "
               "len=%u total=%zu\n",
               f[1], n);
        return 1;
    }

    if (zw_checksum(&f[1], n - 2) != f[n - 1]) {
        printf("[-] PIPELINE checksum incorrecto\n");
        return 1;
    }

    if (f[2] != REQUEST) {
        printf("[-] PIPELINE TYPE inesperado: %02X\n",
               f[2]);
        return 1;
    }

    if (f[3] != 0x4A) {
        printf("[-] PIPELINE FUNC inesperado: %02X\n",
               f[3]);
        return 1;
    }

    d = &f[4];
    data_len = n - 5;

    if (data_len < 2) {
        printf("[-] PIPELINE callback sin status\n");
        return 1;
    }

    callback_id = d[0];
    status = d[1];

    /*
     * ADDING_SLAVE / ADDING_CONTROLLER necesitan Node ID.
     * PROTOCOL_DONE y DONE pueden llevarlo también.
     */
    if (data_len >= 3) {
        node_id = d[2];
        have_node = 1;
    }

    printf("[+] PIPELINE callback id      : 0x%02X\n",
           callback_id);
    printf("[+] PIPELINE status           : 0x%02X (%s)\n",
           status,
           add_node_status_name(status));

    if (have_node)
        printf("[+] PIPELINE node             : %u (0x%02X)\n",
               node_id, node_id);

    return add_node_sm_feed(sm,
                            callback_id,
                            status,
                            have_node,
                            node_id);
}


static int add_node_pipeline_make_frame(struct add_node_sm *sm,
                                        uint8_t callback_id,
                                        uint8_t status,
                                        int have_node,
                                        uint8_t node_id)
{
    uint8_t frame[MAX_FRAME];
    size_t data_len = 2;
    size_t total;

    memset(frame, 0, sizeof(frame));

    frame[0] = SOF;
    frame[2] = REQUEST;
    frame[3] = 0x4A;
    frame[4] = callback_id;
    frame[5] = status;

    if (have_node) {
        frame[6] = node_id;
        data_len++;
    }

    frame[1] = (uint8_t)(data_len + 3);
    total = data_len + 5;

    frame[total - 1] =
        zw_checksum(&frame[1], total - 2);

    dump_hex("PIPELINE RX FRAME", frame, total);

    return add_node_process_frame(sm, frame, total);
}


static int run_add_node_pipeline_selftest(void)
{
    struct add_node_sm sm;
    const uint8_t callback_id = 0x01;
    int steps = 0;

    memset(&sm, 0, sizeof(sm));

    sm.callback_id = callback_id;
    sm.state = ADD_SM_IDLE;

    printf("\n");
    printf("========================================\n");
    printf(" V6.5 ADD_NODE CALLBACK PIPELINE SELFTEST\n");
    printf("========================================\n");
    printf("[+] OFFLINE: no se abrira ttyACM0\n");
    printf("[+] OFFLINE: no se transmitira ningun byte\n");

#define PIPE_STEP(status_, have_node_, node_)                     \
    do {                                                          \
        printf("\n----- PIPELINE STEP %d -----\n", steps + 1);     \
        if (add_node_pipeline_make_frame(&sm,                     \
                                         callback_id,             \
                                         status_,                  \
                                         have_node_,               \
                                         node_)) {                 \
            printf("[-] PIPELINE fallo STEP %d\n", steps + 1);    \
            return 1;                                             \
        }                                                         \
        steps++;                                                  \
    } while (0)

    PIPE_STEP(0x01, 0, 0); /* LEARN_READY */
    PIPE_STEP(0x02, 0, 0); /* NODE_FOUND */
    PIPE_STEP(0x03, 1, 2); /* ADDING_SLAVE */
    PIPE_STEP(0x05, 1, 2); /* PROTOCOL_DONE */
    PIPE_STEP(0x06, 1, 2); /* DONE */

#undef PIPE_STEP

    if (sm.state != ADD_SM_DONE ||
        !sm.have_node ||
        sm.node_id != 2) {
        printf("[-] PIPELINE estado final incorrecto\n");
        return 1;
    }

    printf("\n");
    printf("========================================\n");
    printf(" PIPELINE SELFTEST COMPLETE: %d/5\n", steps);
    printf(" FINAL STATE: %s\n",
           add_node_sm_state_name(sm.state));
    printf(" FINAL NODE : %u\n", sm.node_id);
    printf("========================================\n");

    return 0;
}


/*
 * ============================================================
 * V6.6 - ADD_NODE REAL RECEIVE PATH
 * ============================================================
 *
 * Conecta la ruta real de recepcion:
 *
 *   fd
 *    |
 *    v
 *   receive_frame()
 *    |
 *    v
 *   add_node_process_frame()
 *    |
 *    v
 *   add_node_sm_feed()
 *
 * IMPORTANTE:
 *
 * Esta capa NO inicia inclusion.
 * NO envia ZW_ADD_NODE_TO_NETWORK.
 *
 * El selftest utiliza socketpair(AF_UNIX).
 * No abre ttyACM0.
 */

static int add_node_receive_one(int fd,
                                struct add_node_sm *sm,
                                int timeout_ms)
{
    uint8_t frame[MAX_FRAME];
    size_t frame_len = 0;
    int r;

    if (!sm) {
        printf("[-] RX PATH state machine NULL\n");
        return -1;
    }

    /*
     * Esta espera exterior nos permite distinguir:
     *
     *   1  -> callback recibido y procesado
     *   0  -> timeout limpio
     *  -1  -> error
     */
    r = wait_readable(fd, timeout_ms);

    if (r == 0) {
        printf("[!] RX PATH timeout: no hay callback\n");
        return 0;
    }

    if (r < 0) {
        perror("select RX PATH");
        return -1;
    }

    if (receive_frame(fd,
                      frame,
                      sizeof(frame),
                      &frame_len,
                      0) < 0) {
        printf("[-] RX PATH receive_frame fallo\n");
        return -1;
    }

    printf("[+] RX PATH frame recibido : %zu bytes\n",
           frame_len);

    if (add_node_process_frame(sm,
                               frame,
                               frame_len)) {
        printf("[-] RX PATH pipeline rechazo frame\n");
        return -1;
    }

    return 1;
}


/*
 * Construye y transmite una trama callback sintetica
 * por un socket UNIX local.
 *
 * El otro extremo pasa por receive_frame() REAL.
 */

/*
 * Forward declaration.
 * La implementacion esta mas abajo junto al RX selftest.
 */
static int add_node_rx_test_send(int fd,
                                 uint8_t callback_id,
                                 uint8_t status,
                                 int have_node,
                                 uint8_t node_id);


/*
 * ============================================================
 * V6.7 - ADD_NODE CALLBACK LOOP
 * ============================================================
 *
 * Consume callbacks ya disponibles en un descriptor.
 *
 * IMPORTANTE:
 *
 * Esta capa NO inicia inclusion.
 * NO envia ZW_ADD_NODE_TO_NETWORK.
 *
 * Termina cuando:
 *
 *   ADD_SM_DONE    -> exito
 *   ADD_SM_FAILED  -> fallo
 *   timeout        -> fallo
 *   error RX       -> fallo
 */
static int add_node_callback_loop(int fd,
                                  struct add_node_sm *sm,
                                  int callback_timeout_ms,
                                  unsigned int max_callbacks)
{
    unsigned int callbacks = 0;

    if (!sm) {
        printf("[-] CALLBACK LOOP state machine NULL\n");
        return 1;
    }

    if (callback_timeout_ms <= 0) {
        printf("[-] CALLBACK LOOP timeout invalido\n");
        return 1;
    }

    if (max_callbacks == 0) {
        printf("[-] CALLBACK LOOP max_callbacks invalido\n");
        return 1;
    }

    printf("\n");
    printf("========================================\n");
    printf(" ADD_NODE CALLBACK LOOP\n");
    printf("========================================\n");
    printf("[+] callback timeout        : %d ms\n",
           callback_timeout_ms);
    printf("[+] max callbacks           : %u\n",
           max_callbacks);

    for (;;) {
        int r;

        if (sm->state == ADD_SM_DONE) {
            printf("[+] CALLBACK LOOP terminal   : DONE\n");
            return 0;
        }

        if (sm->state == ADD_SM_FAILED) {
            printf("[-] CALLBACK LOOP terminal   : FAILED\n");
            return 1;
        }

        if (callbacks >= max_callbacks) {
            printf("[-] CALLBACK LOOP limite alcanzado: %u\n",
                   callbacks);
            return 1;
        }

        printf("\n");
        printf("----- CALLBACK %u -----\n",
               callbacks + 1);

        r = add_node_receive_one(fd,
                                 sm,
                                 callback_timeout_ms);

        if (r < 0) {
            printf("[-] CALLBACK LOOP error de recepcion\n");
            return 1;
        }

        if (r == 0) {
            printf("[-] CALLBACK LOOP timeout esperando callback\n");
            return 1;
        }

        callbacks++;

        printf("[+] CALLBACK LOOP procesados : %u\n",
               callbacks);
        printf("[+] CALLBACK LOOP estado     : %s\n",
               add_node_sm_state_name(sm->state));
    }
}



/*
 * ============================================================
 * V6.8 - ADD_NODE TRANSACTION
 * ============================================================
 *
 * Camino de transporte:
 *
 *   START request
 *       |
 *       v
 *   ACK
 *       |
 *       v
 *   callback loop
 *       |
 *       v
 *   STOP request
 *       |
 *       v
 *   ACK
 *
 * STOP se intenta incluso si callback_loop() falla.
 */

static int add_node_send_request_wait_ack(int fd,
                                          uint8_t mode,
                                          uint8_t callback_id,
                                          const char *name)
{
    uint8_t data[2];
    uint8_t frame[MAX_FRAME];
    uint8_t early_sof = 0;
    size_t frame_len = 0;
    int ctrl;

    data[0] = mode;
    data[1] = callback_id;

    if (build_request_frame(0x4A,
                            data,
                            sizeof(data),
                            frame,
                            sizeof(frame),
                            &frame_len)) {
        printf("[-] %s: no se pudo construir frame\n", name);
        return 1;
    }

    printf("\n");
    printf("----- %s -----\n", name);

    dump_hex("TX FRAME", frame, frame_len);

    if (write_all(fd, frame, frame_len) < 0) {
        printf("[-] %s: fallo TX\n", name);
        return 1;
    }

    ctrl = wait_request_ack(fd, &early_sof);

    if (ctrl != ACK) {
        printf("[-] %s: ACK no recibido (ctrl=%02X)\n",
               name,
               ctrl < 0 ? 0xFF : ctrl);
        return 1;
    }

    printf("[+] %s ACK recibido\n", name);

    return 0;
}


static int add_node_transaction(int fd,
                                uint8_t callback_id,
                                int callback_timeout_ms,
                                unsigned int max_callbacks,
                                struct add_node_sm *sm)
{
    int start_ok = 0;
    int loop_ok = 0;
    int stop_ok = 0;

    if (!sm) {
        printf("[-] TRANSACTION state machine NULL\n");
        return 1;
    }

    memset(sm, 0, sizeof(*sm));

    sm->callback_id = callback_id;
    sm->state = ADD_SM_IDLE;

    printf("\n");
    printf("========================================\n");
    printf(" ADD_NODE TRANSACTION\n");
    printf("========================================\n");
    printf("[+] callback id            : 0x%02X\n",
           callback_id);

    /*
     * START = ADD_NODE_ANY (0x01)
     */
    if (add_node_send_request_wait_ack(fd,
                                       0x01,
                                       callback_id,
                                       "ADD_NODE START"))
        goto cleanup;

    start_ok = 1;

    /*
     * Procesamos callbacks hasta DONE/FAILED/timeout/error.
     */
    if (add_node_callback_loop(fd,
                               sm,
                               callback_timeout_ms,
                               max_callbacks) == 0)
        loop_ok = 1;

cleanup:

    /*
     * STOP = ADD_NODE_STOP (0x05)
     *
     * Deliberadamente se intenta siempre que START llegó
     * a ser aceptado.
     */
    if (start_ok) {
        if (add_node_send_request_wait_ack(fd,
                                           0x05,
                                           callback_id,
                                           "ADD_NODE STOP") == 0)
            stop_ok = 1;
    }

    printf("\n");
    printf("========================================\n");
    printf(" ADD_NODE TRANSACTION RESULT\n");
    printf(" START : %s\n", start_ok ? "OK" : "ERROR");
    printf(" LOOP  : %s\n", loop_ok ? "OK" : "ERROR");
    printf(" STOP  : %s\n",
           start_ok ? (stop_ok ? "OK" : "ERROR")
                    : "NOT STARTED");
    printf(" STATE : %s\n",
           add_node_sm_state_name(sm->state));
    printf(" NODE  : %u\n", sm->node_id);
    printf("========================================\n");

    return (start_ok && loop_ok && stop_ok) ? 0 : 1;
}


/*
 * Lee exactamente una REQUEST Serial API del extremo
 * emulado y comprueba FUNC/mode/callback.
 *
 * No usa receive_frame(), porque receive_frame() enviaria
 * ACK automáticamente y aquí el emulador necesita controlar
 * cuándo devuelve ese ACK.
 */
static int add_node_emulator_expect_request(int fd,
                                            uint8_t expected_mode,
                                            uint8_t expected_callback)
{
    uint8_t frame[MAX_FRAME];
    uint8_t b;
    size_t total;
    size_t pos;

    if (read_byte_timeout(fd, &b, 1000) != 1 ||
        b != SOF) {
        printf("[-] EMULATOR: no recibio SOF\n");
        return 1;
    }

    frame[0] = b;

    if (read_byte_timeout(fd, &frame[1], 1000) != 1) {
        printf("[-] EMULATOR: no recibio LENGTH\n");
        return 1;
    }

    total = (size_t)frame[1] + 2;

    if (total > sizeof(frame) || total < 7) {
        printf("[-] EMULATOR: LENGTH invalido\n");
        return 1;
    }

    pos = 2;

    while (pos < total) {
        if (read_byte_timeout(fd,
                              &frame[pos],
                              1000) != 1) {
            printf("[-] EMULATOR: frame incompleto\n");
            return 1;
        }

        pos++;
    }

    dump_hex("EMULATOR RX FRAME", frame, total);

    if (zw_checksum(&frame[1], total - 2) !=
        frame[total - 1]) {
        printf("[-] EMULATOR: checksum incorrecto\n");
        return 1;
    }

    if (frame[2] != REQUEST ||
        frame[3] != 0x4A ||
        frame[4] != expected_mode ||
        frame[5] != expected_callback) {

        printf("[-] EMULATOR: request inesperada\n");
        return 1;
    }

    printf("[+] EMULATOR request correcta"
           " mode=0x%02X callback=0x%02X\n",
           expected_mode,
           expected_callback);

    return 0;
}


/*
 * Controlador Z-Wave sintético.
 *
 * Secuencia:
 *
 *   recibe START
 *   envia ACK
 *   envia 5 callbacks
 *   recibe ACK por cada callback
 *   recibe STOP
 *   envia ACK
 */
static int add_node_transaction_emulator(int fd)
{
    static const struct {
        uint8_t status;
        int have_node;
        uint8_t node_id;
    } events[] = {
        { 0x01, 0, 0 },
        { 0x02, 0, 0 },
        { 0x03, 1, 2 },
        { 0x05, 1, 2 },
        { 0x06, 1, 2 }
    };

    uint8_t b;
    unsigned int i;

    printf("\n");
    printf("========================================\n");
    printf(" V6.8 SYNTHETIC CONTROLLER\n");
    printf("========================================\n");

    if (add_node_emulator_expect_request(fd,
                                         0x01,
                                         0x01))
        return 1;

    if (write_all(fd, (const uint8_t[]){ ACK }, 1) < 0)
        return 1;

    printf("[+] EMULATOR START ACK enviado\n");

    for (i = 0;
         i < sizeof(events) / sizeof(events[0]);
         i++) {

        printf("\n");
        printf("----- EMULATOR CALLBACK %u -----\n",
               i + 1);

        if (add_node_rx_test_send(fd,
                                  0x01,
                                  events[i].status,
                                  events[i].have_node,
                                  events[i].node_id))
            return 1;

        if (read_byte_timeout(fd, &b, 1000) != 1 ||
            b != ACK) {
            printf("[-] EMULATOR: callback sin ACK\n");
            return 1;
        }

        printf("[+] EMULATOR callback ACK recibido\n");
    }

    if (add_node_emulator_expect_request(fd,
                                         0x05,
                                         0x01))
        return 1;

    if (write_all(fd, (const uint8_t[]){ ACK }, 1) < 0)
        return 1;

    printf("[+] EMULATOR STOP ACK enviado\n");

    return 0;
}



/*
 * ============================================================
 * V6.8.1 - ADD_NODE FAILURE PATH SELFTESTS
 * ============================================================
 *
 * Verificamos dos condiciones antes de permitir hardware real:
 *
 *   1. FAILED callback
 *   2. callback timeout
 *
 * En AMBOS casos START ya fue aceptado, por tanto la
 * transaccion DEBE intentar STOP.
 *
 * Todo se ejecuta sobre socketpair().
 * No se abre ttyACM0.
 */


/*
 * Emulador del caso FAILED.
 *
 * Secuencia:
 *
 *   START
 *   ACK
 *   LEARN_READY
 *   FAILED
 *   STOP
 *   ACK
 */
static int add_node_failure_emulator(int fd)
{
    uint8_t b;

    printf("\n");
    printf("========================================\n");
    printf(" V6.8.1 SYNTHETIC CONTROLLER: FAILED\n");
    printf("========================================\n");

    if (add_node_emulator_expect_request(fd,
                                         0x01,
                                         0x01))
        return 1;

    if (write_all(fd,
                  (const uint8_t[]){ ACK },
                  1) < 0)
        return 1;

    printf("[+] FAILED EMULATOR START ACK enviado\n");

    /*
     * LEARN_READY
     */
    if (add_node_rx_test_send(fd,
                              0x01,
                              0x01,
                              0,
                              0))
        return 1;

    if (read_byte_timeout(fd, &b, 1000) != 1 ||
        b != ACK) {
        printf("[-] FAILED EMULATOR: LEARN_READY sin ACK\n");
        return 1;
    }

    printf("[+] FAILED EMULATOR LEARN_READY ACK recibido\n");

    /*
     * FAILED
     */
    if (add_node_rx_test_send(fd,
                              0x01,
                              0x07,
                              0,
                              0))
        return 1;

    if (read_byte_timeout(fd, &b, 1000) != 1 ||
        b != ACK) {
        printf("[-] FAILED EMULATOR: FAILED sin ACK\n");
        return 1;
    }

    printf("[+] FAILED EMULATOR FAILED ACK recibido\n");

    /*
     * Aunque la inclusión haya fallado, esperamos STOP.
     */
    if (add_node_emulator_expect_request(fd,
                                         0x05,
                                         0x01)) {
        printf("[-] FAILED EMULATOR: STOP NO recibido\n");
        return 1;
    }

    printf("[+] FAILED EMULATOR STOP recibido\n");

    if (write_all(fd,
                  (const uint8_t[]){ ACK },
                  1) < 0)
        return 1;

    printf("[+] FAILED EMULATOR STOP ACK enviado\n");

    return 0;
}


/*
 * Emulador del caso TIMEOUT.
 *
 * Secuencia:
 *
 *   START
 *   ACK
 *
 *   <ningun callback>
 *
 *   STOP
 *   ACK
 */
static int add_node_timeout_emulator(int fd)
{
    printf("\n");
    printf("========================================\n");
    printf(" V6.8.1 SYNTHETIC CONTROLLER: TIMEOUT\n");
    printf("========================================\n");

    if (add_node_emulator_expect_request(fd,
                                         0x01,
                                         0x01))
        return 1;

    if (write_all(fd,
                  (const uint8_t[]){ ACK },
                  1) < 0)
        return 1;

    printf("[+] TIMEOUT EMULATOR START ACK enviado\n");
    printf("[+] TIMEOUT EMULATOR no envia callbacks\n");

    /*
     * El padre agotará su callback timeout y después
     * necesariamente deberá enviarnos STOP.
     */
    if (add_node_emulator_expect_request(fd,
                                         0x05,
                                         0x01)) {
        printf("[-] TIMEOUT EMULATOR: STOP NO recibido\n");
        return 1;
    }

    printf("[+] TIMEOUT EMULATOR STOP recibido\n");

    if (write_all(fd,
                  (const uint8_t[]){ ACK },
                  1) < 0)
        return 1;

    printf("[+] TIMEOUT EMULATOR STOP ACK enviado\n");

    return 0;
}


/*
 * Ejecuta una prueba de fallo usando fork/socketpair.
 *
 * expected_state permite comprobar que el error observado
 * es exactamente el esperado.
 */
static int add_node_run_failure_case(
    const char *name,
    int (*emulator)(int),
    enum add_node_sm_state expected_state,
    int callback_timeout_ms)
{
    int sv[2] = { -1, -1 };
    pid_t pid;
    int status = 0;
    int parent_rc;
    int child_rc;
    struct add_node_sm sm;

    printf("\n");
    printf("========================================\n");
    printf(" FAILURE CASE: %s\n", name);
    printf("========================================\n");

    if (socketpair(AF_UNIX,
                   SOCK_STREAM,
                   0,
                   sv) < 0) {
        perror("socketpair");
        return 1;
    }

    pid = fork();

    if (pid < 0) {
        perror("fork");
        close(sv[0]);
        close(sv[1]);
        return 1;
    }

    if (pid == 0) {
        int rc;

        close(sv[0]);

        rc = emulator(sv[1]);

        close(sv[1]);

        _exit(rc ? 1 : 0);
    }

    close(sv[1]);
    sv[1] = -1;

    /*
     * IMPORTANTE:
     *
     * Para FAILED/TIMEOUT esperamos que add_node_transaction()
     * devuelva ERROR.
     *
     * Eso es precisamente lo correcto.
     */
    parent_rc = add_node_transaction(sv[0],
                                     0x01,
                                     callback_timeout_ms,
                                     10,
                                     &sm);

    close(sv[0]);
    sv[0] = -1;

    if (waitpid(pid, &status, 0) < 0) {
        perror("waitpid");
        return 1;
    }

    child_rc =
        WIFEXITED(status) &&
        WEXITSTATUS(status) == 0
        ? 0 : 1;

    printf("\n");
    printf("----------------------------------------\n");
    printf(" FAILURE CASE RESULT: %s\n", name);
    printf("----------------------------------------\n");
    printf(" TRANSACTION RC : %d (esperado != 0)\n",
           parent_rc);
    printf(" CONTROLLER     : %s\n",
           child_rc ? "ERROR" : "OK");
    printf(" FINAL STATE    : %s\n",
           add_node_sm_state_name(sm.state));
    printf(" EXPECTED STATE : %s\n",
           add_node_sm_state_name(expected_state));
    printf("----------------------------------------\n");

    /*
     * El caso pasa si:
     *
     * - la transacción detectó fallo,
     * - el controlador sintético vio STOP y terminó OK,
     * - el estado final es el esperado.
     */
    if (parent_rc == 0) {
        printf("[-] ERROR: transaccion debia fallar\n");
        return 1;
    }

    if (child_rc) {
        printf("[-] ERROR: controlador sintetico fallo\n");
        return 1;
    }

    if (sm.state != expected_state) {
        printf("[-] ERROR: estado final inesperado\n");
        return 1;
    }

    printf("[+] FAILURE CASE %s: OK\n", name);

    return 0;
}


static int run_add_node_failure_selftest(void)
{
    int failed_rc;
    int timeout_rc;

    printf("\n");
    printf("========================================\n");
    printf(" V6.8.1 ADD_NODE FAILURE SELFTEST\n");
    printf("========================================\n");
    printf("[+] OFFLINE: socketpair(AF_UNIX)\n");
    printf("[+] OFFLINE: synthetic controllers\n");
    printf("[+] OFFLINE: no se abrira ttyACM0\n");
    printf("[+] OFFLINE: no se transmitira Z-Wave\n");

    /*
     * Caso 1: callback FAILED.
     */
    failed_rc =
        add_node_run_failure_case(
            "FAILED",
            add_node_failure_emulator,
            ADD_SM_FAILED,
            500
        );

    /*
     * Caso 2: timeout sin callback.
     *
     * Como no llega ningún evento, el estado permanece IDLE.
     */
    timeout_rc =
        add_node_run_failure_case(
            "TIMEOUT",
            add_node_timeout_emulator,
            ADD_SM_IDLE,
            250
        );

    printf("\n");
    printf("========================================\n");
    printf(" V6.8.1 FAILURE SELFTEST COMPLETE\n");
    printf(" FAILED PATH  : %s\n",
           failed_rc ? "ERROR" : "OK");
    printf(" TIMEOUT PATH : %s\n",
           timeout_rc ? "ERROR" : "OK");
    printf(" STOP CLEANUP : %s\n",
           (!failed_rc && !timeout_rc)
               ? "OK"
               : "ERROR");
    printf("========================================\n");

    return (failed_rc || timeout_rc) ? 1 : 0;
}


static int run_add_node_transaction_selftest(void)
{
    int sv[2] = { -1, -1 };
    pid_t pid;
    int status = 0;
    int parent_rc;
    int child_rc;
    struct add_node_sm sm;

    printf("\n");
    printf("========================================\n");
    printf(" V6.8 ADD_NODE TRANSACTION SELFTEST\n");
    printf("========================================\n");
    printf("[+] OFFLINE: socketpair(AF_UNIX)\n");
    printf("[+] OFFLINE: synthetic controller\n");
    printf("[+] OFFLINE: no se abrira ttyACM0\n");
    printf("[+] OFFLINE: no se transmitira Z-Wave\n");

    if (socketpair(AF_UNIX,
                   SOCK_STREAM,
                   0,
                   sv) < 0) {
        perror("socketpair");
        return 1;
    }

    pid = fork();

    if (pid < 0) {
        perror("fork");
        close(sv[0]);
        close(sv[1]);
        return 1;
    }

    if (pid == 0) {
        int rc;

        close(sv[0]);

        rc = add_node_transaction_emulator(sv[1]);

        close(sv[1]);

        _exit(rc ? 1 : 0);
    }

    close(sv[1]);
    sv[1] = -1;

    parent_rc = add_node_transaction(sv[0],
                                     0x01,
                                     1000,
                                     10,
                                     &sm);

    close(sv[0]);
    sv[0] = -1;

    if (waitpid(pid, &status, 0) < 0) {
        perror("waitpid");
        return 1;
    }

    child_rc =
        WIFEXITED(status) && WEXITSTATUS(status) == 0
        ? 0 : 1;

    printf("\n");
    printf("========================================\n");
    printf(" TRANSACTION SELFTEST COMPLETE\n");
    printf(" PARENT      : %s\n",
           parent_rc ? "ERROR" : "OK");
    printf(" CONTROLLER  : %s\n",
           child_rc ? "ERROR" : "OK");
    printf(" FINAL STATE : %s\n",
           add_node_sm_state_name(sm.state));
    printf(" FINAL NODE  : %u\n",
           sm.node_id);
    printf("========================================\n");

    if (parent_rc ||
        child_rc ||
        sm.state != ADD_SM_DONE ||
        sm.node_id != 2)
        return 1;

    return 0;
}


/*
 * ============================================================
 * V6.7 - CALLBACK LOOP SELFTEST OFFLINE
 * ============================================================
 *
 * socketpair(AF_UNIX):
 *
 *   sv[0] -> productor sintetico
 *   sv[1] -> callback loop real
 *
 * No abre ttyACM0.
 * No transmite Z-Wave.
 */
static int run_add_node_callback_loop_selftest(void)
{
    struct add_node_sm sm;
    int sv[2] = { -1, -1 };
    uint8_t ack;
    unsigned int i;
    int rc = 1;

    struct test_event {
        uint8_t status;
        int have_node;
        uint8_t node_id;
    };

    static const struct test_event events[] = {
        { 0x01, 0, 0 }, /* LEARN_READY */
        { 0x02, 0, 0 }, /* NODE_FOUND */
        { 0x03, 1, 2 }, /* ADDING_SLAVE */
        { 0x05, 1, 2 }, /* PROTOCOL_DONE */
        { 0x06, 1, 2 }, /* DONE */
    };

    memset(&sm, 0, sizeof(sm));

    sm.callback_id = 0x01;
    sm.state = ADD_SM_IDLE;

    printf("\n");
    printf("========================================\n");
    printf(" V6.7 ADD_NODE CALLBACK LOOP SELFTEST\n");
    printf("========================================\n");
    printf("[+] OFFLINE: socketpair(AF_UNIX)\n");
    printf("[+] OFFLINE: no se abrira ttyACM0\n");
    printf("[+] OFFLINE: no se transmitira Z-Wave\n");

    if (socketpair(AF_UNIX,
                   SOCK_STREAM,
                   0,
                   sv) < 0) {
        perror("socketpair");
        return 1;
    }

    /*
     * Precargamos todos los callbacks en el socket.
     * El loop del otro extremo los consumira mediante
     * receive_frame() -> add_node_process_frame() -> SM.
     */
    for (i = 0;
         i < sizeof(events) / sizeof(events[0]);
         i++) {

        printf("\n");
        printf("----- PRELOAD EVENT %u -----\n",
               i + 1);

        if (add_node_rx_test_send(sv[0],
                                  0x01,
                                  events[i].status,
                                  events[i].have_node,
                                  events[i].node_id))
            goto out;
    }

    /*
     * Ejecutamos el loop real.
     */
    if (add_node_callback_loop(sv[1],
                               &sm,
                               500,
                               10))
        goto out;

    /*
     * receive_frame() ha debido generar un ACK
     * por cada uno de los cinco callbacks.
     */
    for (i = 0;
         i < sizeof(events) / sizeof(events[0]);
         i++) {

        if (read_byte_timeout(sv[0],
                              &ack,
                              500) != 1) {
            printf("[-] SELFTEST no recibio ACK %u\n",
                   i + 1);
            goto out;
        }

        printf("[+] SELFTEST ACK %u           : %02X\n",
               i + 1, ack);

        if (ack != ACK) {
            printf("[-] SELFTEST ACK inesperado\n");
            goto out;
        }
    }

    if (sm.state != ADD_SM_DONE) {
        printf("[-] CALLBACK LOOP estado final inesperado: %s\n",
               add_node_sm_state_name(sm.state));
        goto out;
    }

    if (!sm.have_node ||
        sm.node_id != 2) {
        printf("[-] CALLBACK LOOP Node ID final inesperado\n");
        goto out;
    }

    printf("\n");
    printf("========================================\n");
    printf(" CALLBACK LOOP SELFTEST COMPLETE\n");
    printf(" FINAL STATE : %s\n",
           add_node_sm_state_name(sm.state));
    printf(" FINAL NODE  : %u\n",
           sm.node_id);
    printf(" CALLBACKS   : 5\n");
    printf(" ACKS        : 5/5\n");
    printf("========================================\n");

    rc = 0;

out:
    if (sv[0] >= 0)
        close(sv[0]);

    if (sv[1] >= 0)
        close(sv[1]);

    return rc;
}


static int add_node_rx_test_send(int fd,
                                 uint8_t callback_id,
                                 uint8_t status,
                                 int have_node,
                                 uint8_t node_id)
{
    uint8_t frame[MAX_FRAME];
    size_t data_len;
    size_t total;
    size_t pos;
    uint8_t ack;
    int r;

    data_len = have_node ? 3 : 2;

    pos = 0;

    frame[pos++] = SOF;
    frame[pos++] = (uint8_t)(data_len + 3);
    frame[pos++] = REQUEST;
    frame[pos++] = 0x4A;
    frame[pos++] = callback_id;
    frame[pos++] = status;

    if (have_node)
        frame[pos++] = node_id;

    total = pos + 1;

    frame[pos] =
        zw_checksum(&frame[1], total - 2);

    dump_hex("SELFTEST SOCKET TX",
             frame,
             total);

    if (write_all(fd, frame, total) < 0) {
        printf("[-] SELFTEST socket write fallo\n");
        return 1;
    }

    /*
     * El ACK se comprobara DESPUES de que
     * receive_frame() haya procesado la trama.
     *
     * Aquí no esperamos todavía porque ambos extremos
     * se ejecutan secuencialmente en el mismo proceso.
     */
    (void)ack;
    (void)r;

    return 0;
}


static int add_node_rx_test_expect_ack(int fd)
{
    uint8_t ack = 0;
    int r;

    r = read_byte_timeout(fd, &ack, 500);

    if (r != 1) {
        printf("[-] SELFTEST no recibio ACK\n");
        return 1;
    }

    printf("[+] SELFTEST control recibido: %02X\n",
           ack);

    if (ack != ACK) {
        printf("[-] SELFTEST esperaba ACK=%02X\n",
               ACK);
        return 1;
    }

    printf("[+] SELFTEST ACK correcto\n");

    return 0;
}


static int run_add_node_rx_path_selftest(void)
{
    struct add_node_sm sm;
    int sv[2] = { -1, -1 };
    unsigned int passed = 0;
    int r;
    int rc = 1;

    memset(&sm, 0, sizeof(sm));

    sm.callback_id = 0x01;
    sm.state = ADD_SM_IDLE;

    printf("\n");
    printf("========================================\n");
    printf(" V6.6 ADD_NODE RX PATH SELFTEST\n");
    printf("========================================\n");
    printf("[+] OFFLINE: socketpair(AF_UNIX)\n");
    printf("[+] OFFLINE: no se abrira ttyACM0\n");
    printf("[+] OFFLINE: no se transmitira Z-Wave\n");

    if (socketpair(AF_UNIX,
                   SOCK_STREAM,
                   0,
                   sv) < 0) {
        perror("socketpair");
        return 1;
    }

#define RX_STEP(NUM, STATUS, HAVE_NODE, NODE)                    \
    do {                                                         \
        printf("\n");                                            \
        printf("----- RX STEP %u -----\n",                       \
               (unsigned int)(NUM));                             \
                                                                 \
        if (add_node_rx_test_send(sv[0],                         \
                                  0x01,                          \
                                  (STATUS),                      \
                                  (HAVE_NODE),                   \
                                  (NODE)))                       \
            goto out;                                            \
                                                                 \
        r = add_node_receive_one(sv[1],                          \
                                 &sm,                            \
                                 500);                           \
                                                                 \
        if (r != 1) {                                            \
            printf("[-] RX STEP %u fallo: %d\n",                 \
                   (unsigned int)(NUM), r);                       \
            goto out;                                            \
        }                                                        \
                                                                 \
        if (add_node_rx_test_expect_ack(sv[0]))                  \
            goto out;                                            \
                                                                 \
        passed++;                                                \
    } while (0)

    RX_STEP(1, 0x01, 0, 0); /* LEARN_READY */
    RX_STEP(2, 0x02, 0, 0); /* NODE_FOUND */
    RX_STEP(3, 0x03, 1, 2); /* ADDING_SLAVE */
    RX_STEP(4, 0x05, 1, 2); /* PROTOCOL_DONE */
    RX_STEP(5, 0x06, 1, 2); /* DONE */

#undef RX_STEP

    printf("\n");
    printf("----- RX TIMEOUT TEST -----\n");

    r = add_node_receive_one(sv[1],
                             &sm,
                             100);

    if (r != 0) {
        printf("[-] timeout test devolvio %d\n", r);
        goto out;
    }

    if (sm.state != ADD_SM_DONE) {
        printf("[-] estado final inesperado: %s\n",
               add_node_sm_state_name(sm.state));
        goto out;
    }

    if (!sm.have_node ||
        sm.node_id != 2) {
        printf("[-] Node ID final inesperado\n");
        goto out;
    }

    printf("\n");
    printf("========================================\n");
    printf(" RX PATH SELFTEST COMPLETE: %u/5\n",
           passed);
    printf(" FINAL STATE: %s\n",
           add_node_sm_state_name(sm.state));
    printf(" FINAL NODE : %u\n",
           sm.node_id);
    printf(" ACK PATH   : OK\n");
    printf(" TIMEOUT    : OK\n");
    printf("========================================\n");

    rc = 0;

out:
    if (sv[0] >= 0)
        close(sv[0]);

    if (sv[1] >= 0)
        close(sv[1]);

    return rc;
}

static int run_add_node_state_selftest(void)
{
    struct add_node_sm sm;
    const uint8_t callback_id = 0x01;
    int steps = 0;

    memset(&sm, 0, sizeof(sm));

    sm.callback_id = callback_id;
    sm.state = ADD_SM_IDLE;

    printf("\n");
    printf("========================================\n");
    printf(" V6.4 ADD_NODE STATE SELFTEST\n");
    printf("========================================\n");
    printf("[+] OFFLINE: no se abrira ttyACM0\n");
    printf("[+] OFFLINE: no se transmitira ningun byte\n");

#define STEP(status, have_node, node)                         \
    do {                                                      \
        printf("\n----- STEP %d -----\n", steps + 1);          \
        if (add_node_sm_feed(&sm, callback_id,                \
                             status, have_node, node)) {       \
            printf("[-] fallo en STEP %d\n", steps + 1);      \
            return 1;                                         \
        }                                                     \
        steps++;                                              \
    } while (0)

    STEP(0x01, 0, 0);
    STEP(0x02, 0, 0);
    STEP(0x03, 1, 2);
    STEP(0x05, 1, 2);
    STEP(0x06, 1, 2);

#undef STEP

    if (sm.state != ADD_SM_DONE ||
        !sm.have_node ||
        sm.node_id != 2) {
        printf("[-] estado final incorrecto\n");
        return 1;
    }

    printf("\n");
    printf("========================================\n");
    printf(" STATE SELFTEST COMPLETE: %d/5\n", steps);
    printf(" FINAL STATE: %s\n",
           add_node_sm_state_name(sm.state));
    printf(" FINAL NODE : %u\n", sm.node_id);
    printf("========================================\n");

    return 0;
}

static int run_add_node_dry_run(void)
{
    uint8_t frame[MAX_FRAME];
    size_t frame_len;

    const uint8_t start_data[] = {
        0x01,       /* ADD_NODE_ANY */
        0x01        /* callback id */
    };

    const uint8_t stop_data[] = {
        0x05,       /* ADD_NODE_STOP */
        0x01        /* callback id */
    };

    printf("\n");
    printf("========================================\n");
    printf(" V6 ADD_NODE DRY RUN\n");
    printf("========================================\n");
    printf("[+] NO se abrira el puerto Z-Wave\n");
    printf("[+] NO se transmitira ningun byte\n");

    printf("\n");
    printf("----- ADD_NODE START -----\n");

    if (build_request_frame(0x4A,
                            start_data,
                            sizeof(start_data),
                            frame,
                            sizeof(frame),
                            &frame_len))
        return 1;

    dump_hex("DRY TX FRAME", frame, frame_len);

    printf("\n");
    printf("----- ADD_NODE STOP -----\n");

    if (build_request_frame(0x4A,
                            stop_data,
                            sizeof(stop_data),
                            frame,
                            sizeof(frame),
                            &frame_len))
        return 1;

    dump_hex("DRY TX FRAME", frame, frame_len);

    printf("\n");
    printf("[+] DRY RUN terminado\n");
    printf("[+] Ningun byte enviado al controlador\n");

    return 0;
}


static int serial_api_query_data(int fd,
                                 uint8_t func,
                                 const char *name,
                                 const uint8_t *data,
                                 size_t data_len,
                                 uint8_t *response,
                                 size_t response_size,
                                 size_t *response_len)
{
    uint8_t request[MAX_FRAME];
    uint8_t early_sof;
    size_t request_len;
    int attempt;

    /*
     * Z-Wave Serial API REQUEST:
     *
     * SOF LEN REQ FUNC DATA... CHK
     *
     * LEN cuenta TYPE + FUNC + DATA + CHECKSUM.
     */
    if (data_len > MAX_FRAME - 5) {
        printf("[-] payload demasiado grande\n");
        return -1;
    }

    request_len = data_len + 5;

    request[0] = SOF;
    request[1] = (uint8_t)(data_len + 3);
    request[2] = REQUEST;
    request[3] = func;

    if (data_len)
        memcpy(&request[4], data, data_len);

    request[4 + data_len] =
        zw_checksum(&request[1], data_len + 3);

    printf("\n========================================\n");
    printf(" QUERY: %s (FUNC_ID=0x%02X)\n", name, func);
    printf("========================================\n");

    for (attempt = 1; attempt <= MAX_RETRIES; attempt++) {
        int ctrl;

        printf("[+] intento %d/%d\n", attempt, MAX_RETRIES);

        dump_hex("TX FRAME", request, request_len);

        if (write_all(fd, request, request_len) < 0)
            return -1;

        ctrl = wait_request_ack(fd, &early_sof);

        if (ctrl == ACK) {
            if (receive_frame(fd,
                              response,
                              response_size,
                              response_len,
                              0) < 0) {
                printf("[-] respuesta invalida\n");
                return -1;
            }

        } else if (ctrl == SOF && early_sof) {
            if (receive_frame(fd,
                              response,
                              response_size,
                              response_len,
                              1) < 0) {
                printf("[-] respuesta temprana invalida\n");
                return -1;
            }

        } else if (ctrl == CAN || ctrl == NAK || ctrl == 0) {
            printf("[!] peticion no aceptada; reintentando\n");
            usleep(100000);
            continue;

        } else {
            printf("[-] estado inesperado del enlace\n");
            return -1;
        }

        if (*response_len < 5) {
            printf("[-] respuesta demasiado corta\n");
            return -1;
        }

        if (response[2] != RESPONSE) {
            printf("[-] TYPE inesperado: %02X\n", response[2]);
            return -1;
        }

        if (response[3] != func) {
            printf("[-] FUNC_ID inesperado: esperado=%02X recibido=%02X\n",
                   func, response[3]);
            return -1;
        }

        printf("[+] RESPONSE FUNC_ID correcto: 0x%02X\n", func);
        return 0;
    }

    printf("[-] agotados los reintentos\n");
    return -1;
}

static int serial_api_query(int fd,
                            uint8_t func,
                            const char *name,
                            uint8_t *response,
                            size_t response_size,
                            size_t *response_len)
{
    return serial_api_query_data(fd,
                                 func,
                                 name,
                                 NULL,
                                 0,
                                 response,
                                 response_size,
                                 response_len);
}

static void decode_get_version(const uint8_t *f, size_t n)
{
    size_t data_len;
    size_t i;

    if (n < 6)
        return;

    /*
     * SOF LEN TYPE FUNC DATA... CHK
     */
    data_len = n - 5;

    printf("[+] GET_VERSION data:");

    for (i = 0; i < data_len; i++)
        printf(" %02X", f[4 + i]);

    printf("\n");

    printf("[+] version string: ");

    for (i = 0; i < data_len; i++) {
        uint8_t c = f[4 + i];

        if (c == 0)
            break;

        if (c >= 32 && c <= 126)
            putchar(c);
        else
            putchar('.');
    }

    printf("\n");
}

static void decode_memory_get_id(const uint8_t *f, size_t n)
{
    if (n < 10) {
        printf("[!] MEMORY_GET_ID demasiado corto para decodificar\n");
        return;
    }

    printf("[+] Home ID : %02X %02X %02X %02X\n",
           f[4], f[5], f[6], f[7]);

    printf("[+] Node ID : %u (0x%02X)\n",
           f[8], f[8]);
}



static void decode_init_data(const uint8_t *f, size_t n)
{
    const uint8_t *d;
    size_t data_len;
    size_t mask_len;
    size_t available;
    size_t i;
    unsigned int node;

    discovered_node_count = 0;

    /*
     * Frame:
     * SOF LEN RESPONSE FUNC
     * SERIAL_API_VERSION
     * CAPABILITIES
     * NODELIST_LENGTH
     * NODELIST...
     * CHIP_TYPE
     * CHIP_VERSION
     * CHECKSUM
     */
    if (n < 10) {
        printf("[!] SERIAL_API_GET_INIT_DATA demasiado corto\n");
        return;
    }

    d = &f[4];

    /*
     * n incluye SOF, LEN, TYPE, FUNC, DATA y CHECKSUM.
     * Por tanto DATA = n - 5.
     */
    data_len = n - 5;

    if (data_len < 3) {
        printf("[!] INIT_DATA payload demasiado corto\n");
        return;
    }

    printf("[+] Serial API init version : %u\n", d[0]);
    printf("[+] Init capabilities       : 0x%02X\n", d[1]);

    mask_len = d[2];
    available = data_len - 3;

    printf("[+] Node bitmask length     : %zu bytes\n", mask_len);

    if (mask_len > available) {
        printf("[!] Node bitmask truncado: esperado=%zu disponible=%zu\n",
               mask_len, available);
        return;
    }

    printf("[+] Node bitmask            :");
    for (i = 0; i < mask_len; i++)
        printf(" %02X", d[3 + i]);
    printf("\n");

    printf("[+] Nodes present           :");

    for (i = 0; i < mask_len; i++) {
        unsigned int bit;

        for (bit = 0; bit < 8; bit++) {
            if (d[3 + i] & (1U << bit)) {
                node = (unsigned int)(i * 8 + bit + 1);

                if (node <= 232) {
                    printf(" %u", node);

                    if (discovered_node_count <
                        sizeof(discovered_nodes)) {
                        discovered_nodes[discovered_node_count++] =
                            (uint8_t)node;
                    }
                }
            }
        }
    }

    if (!discovered_node_count)
        printf(" ninguno");

    printf("\n");
    printf("[+] Node count              : %zu\n",
           discovered_node_count);

    /*
     * Después del bitmask suelen venir chip type/version.
     */
    if (available >= mask_len + 2) {
        printf("[+] Chip type               : 0x%02X\n",
               d[3 + mask_len]);
        printf("[+] Chip version            : 0x%02X\n",
               d[3 + mask_len + 1]);
    }
}

static void decode_node_protocol_info(const uint8_t *f, size_t n)
{
    const uint8_t *d;
    size_t data_len;

    if (n < 11) {
        printf("[!] NODE_PROTOCOL_INFO demasiado corto\n");
        return;
    }

    d = &f[4];
    data_len = n - 5;

    printf("[+] Protocol info raw       :");
    for (size_t i = 0; i < data_len; i++)
        printf(" %02X", d[i]);
    printf("\n");

    if (data_len >= 6) {
        printf("[+] Capability              : 0x%02X\n", d[0]);
        printf("[+] Security                : 0x%02X\n", d[1]);
        printf("[+] Reserved                : 0x%02X\n", d[2]);
        printf("[+] Basic device class      : 0x%02X\n", d[3]);
        printf("[+] Generic device class    : 0x%02X\n", d[4]);
        printf("[+] Specific device class   : 0x%02X\n", d[5]);

        printf("[+] Listening               : %s\n",
               (d[0] & 0x80) ? "SI" : "NO");

        printf("[+] Routing                 : %s\n",
               (d[0] & 0x40) ? "SI" : "NO");
    }
}

static void decode_controller_capabilities(const uint8_t *f, size_t n)
{
    uint8_t caps;

    if (n < 6) {
        printf("[!] CONTROLLER_CAPABILITIES demasiado corto\n");
        return;
    }

    caps = f[4];

    printf("[+] Controller capabilities raw: 0x%02X\n", caps);

    printf("[+] Secondary controller       : %s\n",
           (caps & 0x01) ? "SI" : "NO");

    printf("[+] On other network           : %s\n",
           (caps & 0x02) ? "SI" : "NO");

    printf("[+] SIS present                : %s\n",
           (caps & 0x04) ? "SI" : "NO");

    printf("[+] Real primary               : %s\n",
           (caps & 0x08) ? "SI" : "NO");

    printf("[+] SUC                        : %s\n",
           (caps & 0x10) ? "SI" : "NO");
}

static void decode_serial_capabilities(const uint8_t *f, size_t n)
{
    size_t data_len;
    size_t mask_len;
    size_t i;
    unsigned int func;
    const uint8_t *d;
    const uint8_t *mask;

    if (n < 13) {
        printf("[!] SERIAL_API_GET_CAPABILITIES demasiado corto\n");
        return;
    }

    /*
     * Layout:
     *
     * SOF LEN RESPONSE FUNC
     * API_VERSION
     * API_REVISION
     * MANUFACTURER_ID[2]
     * PRODUCT_TYPE[2]
     * PRODUCT_ID[2]
     * FUNC_ID_BITMASK...
     * CHECKSUM
     */

    data_len = n - 5;
    d = &f[4];

    if (data_len < 8) {
        printf("[!] payload de capabilities demasiado corto\n");
        return;
    }

    printf("[+] Serial API version : %u\n", d[0]);
    printf("[+] Serial API revision: %u\n", d[1]);

    printf("[+] Manufacturer ID    : 0x%02X%02X\n",
           d[2], d[3]);

    printf("[+] Product Type       : 0x%02X%02X\n",
           d[4], d[5]);

    printf("[+] Product ID         : 0x%02X%02X\n",
           d[6], d[7]);

    mask = &d[8];
    mask_len = data_len - 8;

    printf("[+] Function bitmask   : %zu bytes\n", mask_len);

    printf("[+] Supported FUNC_IDs :");

    for (i = 0; i < mask_len; i++) {
        unsigned int bit;

        for (bit = 0; bit < 8; bit++) {
            if (mask[i] & (1U << bit)) {
                func = (unsigned int)(i * 8 + bit + 1);

                if (func <= 0xFF)
                    printf(" %02X", func);
            }
        }
    }

    printf("\n");

    /*
     * Destacamos algunas funciones importantes sin ejecutar
     * ninguna de ellas.
     */
    {
        static const struct {
            uint8_t id;
            const char *name;
        } interesting[] = {
            { 0x02, "SERIAL_API_GET_INIT_DATA" },
            { 0x05, "ZW_GET_CONTROLLER_CAPABILITIES" },
            { 0x07, "SERIAL_API_GET_CAPABILITIES" },
            { 0x15, "ZW_GET_VERSION" },
            { 0x20, "MEMORY_GET_ID" },
            { 0x41, "ZW_GET_NODE_PROTOCOL_INFO" },
            { 0x42, "ZW_SET_DEFAULT" },
            { 0x4A, "ZW_ADD_NODE_TO_NETWORK" },
            { 0x4B, "ZW_REMOVE_NODE_FROM_NETWORK" },
            { 0x4C, "ZW_CREATE_NEW_PRIMARY" },
            { 0x4D, "ZW_CONTROLLER_CHANGE" },
            { 0x56, "ZW_GET_SUC_NODE_ID" }
        };

        size_t x;

        printf("[+] Funciones destacadas:\n");

        for (x = 0;
             x < sizeof(interesting) / sizeof(interesting[0]);
             x++) {

            unsigned int id = interesting[x].id;
            unsigned int idx = (id - 1) / 8;
            unsigned int bit = (id - 1) % 8;
            int supported = 0;

            if (idx < mask_len)
                supported = !!(mask[idx] & (1U << bit));

            printf("    0x%02X %-31s : %s\n",
                   id,
                   interesting[x].name,
                   supported ? "SI" : "NO");
        }
    }
}

static int run_query(int fd, uint8_t func, const char *name)
{
    uint8_t response[MAX_FRAME];
    size_t response_len = 0;
    int rc;

    rc = serial_api_query(fd,
                          func,
                          name,
                          response,
                          sizeof(response),
                          &response_len);

    if (rc < 0)
        return 1;

    if (func == 0x15)
        decode_get_version(response, response_len);
    else if (func == 0x20)
        decode_memory_get_id(response, response_len);
    else if (func == 0x02)
        decode_init_data(response, response_len);
    else if (func == 0x05)
        decode_controller_capabilities(response, response_len);
    else if (func == 0x07)
        decode_serial_capabilities(response, response_len);

    return 0;
}

static int run_node_info(int fd, uint8_t node)
{
    uint8_t response[MAX_FRAME];
    size_t response_len = 0;
    uint8_t data[1];
    int rc;

    data[0] = node;

    printf("[+] Consultando Node ID %u (0x%02X)\n",
           node, node);

    rc = serial_api_query_data(fd,
                               0x41,
                               "ZW_GET_NODE_PROTOCOL_INFO",
                               data,
                               sizeof(data),
                               response,
                               sizeof(response),
                               &response_len);

    if (rc < 0)
        return 1;

    decode_node_protocol_info(response, response_len);

    return 0;
}

static int run_inventory(int fd)
{
    size_t i;

    /*
     * GET_INIT_DATA ya rellena:
     *
     *   discovered_nodes[]
     *   discovered_node_count
     *
     * No modificamos la red. Todas las operaciones realizadas
     * por este modo son consultas de lectura.
     */
    printf("\n");
    printf("========================================\n");
    printf(" Z-WAVE NETWORK INVENTORY\n");
    printf("========================================\n");

    if (run_query(fd,
                  0x02,
                  "SERIAL_API_GET_INIT_DATA")) {
        printf("[-] no se pudo obtener el mapa de nodos\n");
        return 1;
    }

    printf("\n");
    printf("========================================\n");
    printf(" INVENTORY: %zu NODE(S)\n",
           discovered_node_count);
    printf("========================================\n");

    if (!discovered_node_count) {
        printf("[!] No hay nodos en el mapa Z-Wave\n");
        return 0;
    }

    for (i = 0; i < discovered_node_count; i++) {
        uint8_t node = discovered_nodes[i];

        printf("\n");
        printf("----------------------------------------\n");
        printf(" NODE %u\n", node);
        printf("----------------------------------------\n");

        if (run_node_info(fd, node)) {
            printf("[!] NODE %u: error consultando protocol info\n",
                   node);
            return 1;
        }
    }

    printf("\n");
    printf("========================================\n");
    printf(" INVENTORY COMPLETE: %zu NODE(S)\n",
           discovered_node_count);
    printf("========================================\n");

    return 0;
}


/*
 * ============================================================
 * V6.9 - REAL ADD_NODE MODE
 * ============================================================
 *
 * ATENCION:
 *
 * Esta funcion SI utiliza el controlador Z-Wave real.
 * Debe llamarse solamente despues de setup_serial().
 *
 * Toda la transaccion reutiliza la infraestructura validada:
 *
 *   START
 *     -> ACK
 *     -> callback loop
 *     -> state machine
 *     -> STOP cleanup
 *
 * El STOP se intenta tanto después de DONE como ante error/
 * timeout mediante add_node_transaction().
 */
static int run_add_node_real(int fd)
{
    struct add_node_sm sm;
    const uint8_t callback_id = 0x01;
    int rc;

    memset(&sm, 0, sizeof(sm));

    sm.callback_id = callback_id;
    sm.state = ADD_SM_IDLE;

    printf("\n");
    printf("========================================\n");
    printf(" V6.9 REAL Z-WAVE ADD_NODE\n");
    printf("========================================\n");
    printf("[!] ATENCION: modo REAL\n");
    printf("[!] Se iniciara inclusion en el controlador Z-Wave\n");
    printf("[+] callback id            : 0x%02X\n",
           callback_id);
    printf("\n");
    printf("[+] Pon el dispositivo Z-Wave que quieras incluir\n");
    printf("[+] en modo inclusion cuando aparezca LEARN_READY.\n");

    rc = add_node_transaction(fd,
                              callback_id,
                              30000,
                              32,
                              &sm);

    /*
     * V6.9.1:
     *
     * Algunos controladores Serial API reales pueden completar
     * la inclusion y dejar el ultimo callback observado en
     * PROTOCOL_DONE sin entregar posteriormente DONE.
     *
     * NO consideramos PROTOCOL_DONE suficiente por si solo.
     *
     * Si tenemos un Node ID valido, verificamos contra
     * SERIAL_API_GET_INIT_DATA que dicho nodo aparece realmente
     * en el mapa persistido del controlador.
     */
    if (rc != 0 &&
        sm.state == ADD_SM_PROTOCOL_DONE &&
        sm.have_node &&
        sm.node_id != 0) {

        size_t i;
        int node_verified = 0;

        printf("\n");
        printf("========================================\n");
        printf(" V6.9.1 POST-INCLUSION VERIFY\n");
        printf("========================================\n");
        printf("[+] PROTOCOL_DONE recibido sin DONE\n");
        printf("[+] Node candidato          : %u (0x%02X)\n",
               sm.node_id,
               sm.node_id);
        printf("[+] Verificando INIT_DATA...\n");

        if (run_query(fd,
                      0x02,
                      "SERIAL_API_GET_INIT_DATA") == 0) {

            for (i = 0; i < discovered_node_count; i++) {
                if (discovered_nodes[i] == sm.node_id) {
                    node_verified = 1;
                    break;
                }
            }
        }

        if (node_verified) {
            printf("[+] Node %u CONFIRMADO en mapa Z-Wave\n",
                   sm.node_id);
            printf("[+] Inclusion verificada tras PROTOCOL_DONE\n");

            sm.state = ADD_SM_DONE;
            rc = 0;
        } else {
            printf("[-] Node %u NO aparece en mapa Z-Wave\n",
                   sm.node_id);
            printf("[-] PROTOCOL_DONE NO se acepta como exito\n");
        }

        printf("========================================\n");
    }

    printf("\n");
    printf("========================================\n");
    printf(" REAL ADD_NODE RESULT\n");
    printf(" RC    : %d\n", rc);
    printf(" STATE : %s\n",
           add_node_sm_state_name(sm.state));
    printf(" NODE  : %u\n",
           sm.node_id);
    printf("========================================\n");

    if (rc == 0 && sm.state == ADD_SM_DONE) {
        printf("[+] Inclusion terminada correctamente\n");
        return 0;
    }

    printf("[-] Inclusion NO completada\n");
    return 1;
}



/*
 * ============================================================
 * V7.0 - PASSIVE SERIAL API LISTENER
 * ============================================================
 *
 * Escucha tramas espontaneas procedentes del controlador.
 *
 * NO envia comandos Z-Wave.
 * NO inicia inclusion/exclusion.
 * NO ejecuta ZW_SEND_DATA.
 *
 * receive_frame() envia el ACK de transporte requerido por
 * Serial API cuando recibe una trama valida.
 */


/*
 * V7.1 - ZW_SEND_DATA TRANSPORT
 *
 * Construye una peticion FUNC_ID_ZW_SEND_DATA (0x13).
 *
 * Payload Serial API:
 *
 *   NODE
 *   DATA_LEN
 *   COMMAND_CLASS COMMAND ...
 *   TX_OPTIONS
 *   CALLBACK_ID
 *
 * Esta funcion solo construye la trama.
 * NO abre ningun puerto y NO transmite nada.
 */
static int build_zw_send_data_frame(uint8_t node_id,
                                    const uint8_t *command,
                                    size_t command_len,
                                    uint8_t tx_options,
                                    uint8_t callback_id,
                                    uint8_t *frame,
                                    size_t frame_size,
                                    size_t *frame_len)
{
    uint8_t data[MAX_FRAME];
    size_t data_len;

    if (node_id == 0 || node_id > 232) {
        printf("[-] ZW_SEND_DATA Node ID invalido: %u\n",
               node_id);
        return -1;
    }

    if (!command || command_len == 0) {
        printf("[-] ZW_SEND_DATA comando vacio\n");
        return -1;
    }

    /*
     * NODE + LENGTH + COMMAND + TX_OPTIONS + CALLBACK
     */
    data_len = command_len + 4;

    if (data_len > sizeof(data)) {
        printf("[-] ZW_SEND_DATA payload demasiado grande\n");
        return -1;
    }

    data[0] = node_id;
    data[1] = (uint8_t)command_len;

    memcpy(&data[2], command, command_len);

    data[2 + command_len] = tx_options;
    data[3 + command_len] = callback_id;

    return build_request_frame(0x13,
                               data,
                               data_len,
                               frame,
                               frame_size,
                               frame_len);
}


/*
 * V7.1 OFFLINE SELFTEST
 *
 * No llama setup_serial().
 * No abre /dev/ttyACM0.
 * No transmite Z-Wave.
 *
 * Construimos:
 *
 *   Node       = 2
 *   Command    = 84 05
 *   TX options = 0x25
 *   Callback   = 0x01
 *
 * El objetivo de V7.1 es validar exclusivamente
 * el transporte/frame de ZW_SEND_DATA.
 */
static int run_zw_send_data_selftest(void)
{
    uint8_t frame[MAX_FRAME];
    size_t frame_len = 0;

    static const uint8_t command[] = {
        0x84, 0x05
    };

    printf("\n");
    printf("========================================\n");
    printf(" V7.1 ZW_SEND_DATA OFFLINE SELFTEST\n");
    printf("========================================\n");
    printf("[+] OFFLINE: no se abrira ttyACM0\n");
    printf("[+] OFFLINE: no se transmitira Z-Wave\n");
    printf("[+] Node ID                : 2\n");
    printf("[+] Command                : 84 05\n");
    printf("[+] TX options             : 0x25\n");
    printf("[+] Callback ID            : 0x01\n");

    if (build_zw_send_data_frame(2,
                                 command,
                                 sizeof(command),
                                 0x25,
                                 0x01,
                                 frame,
                                 sizeof(frame),
                                 &frame_len) < 0) {
        printf("[-] No se pudo construir ZW_SEND_DATA\n");
        return 1;
    }

    dump_hex("ZW_SEND_DATA DRY FRAME",
             frame,
             frame_len);

    /*
     * Esperamos:
     *
     * SOF
     * LEN = 9
     * REQUEST
     * FUNC = 13
     * NODE = 02
     * CMDLEN = 02
     * CMD = 84 05
     * TXOPT = 25
     * CALLBACK = 01
     * CHECKSUM
     */
    if (frame_len != 11) {
        printf("[-] Longitud inesperada: %zu (esperada=11)\n",
               frame_len);
        return 1;
    }

    if (frame[0] != SOF ||
        frame[1] != 0x09 ||
        frame[2] != REQUEST ||
        frame[3] != 0x13 ||
        frame[4] != 0x02 ||
        frame[5] != 0x02 ||
        frame[6] != 0x84 ||
        frame[7] != 0x05 ||
        frame[8] != 0x25 ||
        frame[9] != 0x01) {
        printf("[-] Contenido ZW_SEND_DATA inesperado\n");
        return 1;
    }

    if (frame[frame_len - 2] != 0x01) {
        printf("[-] Callback ID inesperado: %02X\n",
               frame[frame_len - 2]);
        return 1;
    }

    if (zw_checksum(&frame[1], frame_len - 2) !=
        frame[frame_len - 1]) {
        printf("[-] Checksum ZW_SEND_DATA incorrecto\n");
        return 1;
    }

    printf("[+] FUNC_ID_ZW_SEND_DATA   : 0x13\n");
    printf("[+] Frame estructuralmente correcto\n");
    printf("[+] Checksum correcto\n");
    printf("[+] Ningun byte transmitido\n");
    printf("========================================\n");

    return 0;
}


static void decode_application_command_handler(const uint8_t *f,
                                               size_t n)
{
    const uint8_t *d;
    size_t data_len;
    uint8_t rx_status;
    uint8_t source_node;
    uint8_t command_len;
    size_t i;

    /*
     * APPLICATION_COMMAND_HANDLER:
     *
     * REQUEST 0x04
     *
     * DATA:
     *   [0] RX status
     *   [1] source node
     *   [2] command length
     *   [3...] command
     */
    if (!f || n < 8) {
        printf("[!] APPLICATION_COMMAND_HANDLER demasiado corto\n");
        return;
    }

    if (f[0] != SOF ||
        f[2] != REQUEST ||
        f[3] != 0x04) {
        printf("[!] No es APPLICATION_COMMAND_HANDLER\n");
        return;
    }

    d = &f[4];
    data_len = n - 5;

    if (data_len < 3) {
        printf("[!] APPLICATION_COMMAND_HANDLER sin cabecera completa\n");
        return;
    }

    rx_status = d[0];
    source_node = d[1];
    command_len = d[2];

    printf("[+] APPLICATION_COMMAND_HANDLER\n");
    printf("[+] RX status              : 0x%02X\n",
           rx_status);
    printf("[+] Source Node            : %u (0x%02X)\n",
           source_node,
           source_node);
    printf("[+] Command length         : %u\n",
           command_len);

    if ((size_t)command_len > data_len - 3) {
        printf("[!] Command length invalido: "
               "declarado=%u disponible=%zu\n",
               command_len,
               data_len - 3);
        return;
    }

    printf("[+] Z-Wave command         :");

    for (i = 0; i < command_len; i++)
        printf(" %02X", d[3 + i]);

    printf("\n");

    if (command_len >= 2) {
        uint8_t cc = d[3];
        uint8_t cmd = d[4];

        printf("[+] Command Class          : 0x%02X\n", cc);
        printf("[+] Command                : 0x%02X\n", cmd);

        /*
         * V7.11 - DCH-Z110 PASSIVE COMMAND CLASS DECODER
         *
         * SOLO interpreta bytes ya recibidos.
         * NO envia ZW_SEND_DATA.
         * NO responde al nodo.
         * NO modifica asociaciones/configuracion.
         */
        printf("[+] V7.11 decode            : ");

        switch (cc) {
        case 0x84:
            printf("COMMAND_CLASS_WAKE_UP");

            if (cmd == 0x07)
                printf(" / WAKE_UP_NOTIFICATION");
            else if (cmd == 0x08)
                printf(" / WAKE_UP_NO_MORE_INFORMATION");
            else
                printf(" / command 0x%02X", cmd);

            printf("\n");
            break;

        case 0x80:
            printf("COMMAND_CLASS_BATTERY");

            if (cmd == 0x03 && command_len >= 3) {
                uint8_t level = d[5];

                printf(" / BATTERY_REPORT");

                if (level == 0xFF)
                    printf(" / LOW BATTERY WARNING");
                else if (level <= 100)
                    printf(" / level=%u%%", level);
                else
                    printf(" / raw=0x%02X", level);
            } else {
                printf(" / command 0x%02X", cmd);
            }

            printf("\n");
            break;

        case 0x71:
            printf("COMMAND_CLASS_NOTIFICATION");

            if (cmd == 0x05)
                printf(" / NOTIFICATION_GET");
            else if (cmd == 0x06)
                printf(" / NOTIFICATION_REPORT");
            else if (cmd == 0x07)
                printf(" / NOTIFICATION_SET");
            else
                printf(" / command 0x%02X", cmd);

            printf("\n");
            break;

        case 0x30:
            printf("COMMAND_CLASS_SENSOR_BINARY");

            if (cmd == 0x02)
                printf(" / SENSOR_BINARY_GET");
            else if (cmd == 0x03) {
                printf(" / SENSOR_BINARY_REPORT");

                if (command_len >= 3)
                    printf(" / value=0x%02X", d[5]);
            } else {
                printf(" / command 0x%02X", cmd);
            }

            printf("\n");
            break;

        case 0x31:
            printf("COMMAND_CLASS_SENSOR_MULTILEVEL");

            if (cmd == 0x04)
                printf(" / SENSOR_MULTILEVEL_GET");
            else if (cmd == 0x05)
                printf(" / SENSOR_MULTILEVEL_REPORT");
            else
                printf(" / command 0x%02X", cmd);

            printf("\n");
            break;

        case 0x85:
            printf("COMMAND_CLASS_ASSOCIATION");

            if (cmd == 0x01)
                printf(" / ASSOCIATION_SET");
            else if (cmd == 0x02)
                printf(" / ASSOCIATION_GET");
            else if (cmd == 0x03)
                printf(" / ASSOCIATION_REPORT");
            else if (cmd == 0x04)
                printf(" / ASSOCIATION_REMOVE");
            else if (cmd == 0x05)
                printf(" / ASSOCIATION_GROUPINGS_GET");
            else if (cmd == 0x06)
                printf(" / ASSOCIATION_GROUPINGS_REPORT");
            else
                printf(" / command 0x%02X", cmd);

            printf("\n");
            break;

        case 0x70:
            printf("COMMAND_CLASS_CONFIGURATION");

            if (cmd == 0x04)
                printf(" / CONFIGURATION_SET");
            else if (cmd == 0x05)
                printf(" / CONFIGURATION_GET");
            else if (cmd == 0x06)
                printf(" / CONFIGURATION_REPORT");
            else
                printf(" / command 0x%02X", cmd);

            printf("\n");
            break;

        case 0x72:
            printf("COMMAND_CLASS_MANUFACTURER_SPECIFIC");

            if (cmd == 0x04)
                printf(" / MANUFACTURER_SPECIFIC_GET");
            else if (cmd == 0x05)
                printf(" / MANUFACTURER_SPECIFIC_REPORT");
            else
                printf(" / command 0x%02X", cmd);

            printf("\n");
            break;

        case 0x86:
            printf("COMMAND_CLASS_VERSION");

            if (cmd == 0x11)
                printf(" / VERSION_GET");
            else if (cmd == 0x12)
                printf(" / VERSION_REPORT");
            else if (cmd == 0x13)
                printf(" / VERSION_COMMAND_CLASS_GET");
            else if (cmd == 0x14)
                printf(" / VERSION_COMMAND_CLASS_REPORT");
            else
                printf(" / command 0x%02X", cmd);

            printf("\n");
            break;

        default:
            printf("UNKNOWN_COMMAND_CLASS_0x%02X / command 0x%02X\n",
                   cc,
                   cmd);
            break;
        }
    }
}



/*
 * V7.2 - ZW_SEND_DATA TRANSACTION CORE
 *
 * Transporte HOST -> controlador.
 *
 * Esta funcion:
 *
 *   1. construye ZW_SEND_DATA
 *   2. envia la trama Serial API
 *   3. espera ACK del controlador
 *   4. recibe RESPONSE de FUNC_ID 0x13
 *
 * IMPORTANTE:
 *
 * Todavia NO esperamos aqui el callback final de transmision.
 * Eso sera una fase posterior de V7.2.
 */
/*
 * V7.6 forward declaration.
 *
 * La implementacion esta mas abajo, junto al parser
 * de callback V7.3/V7.4.
 */
static int zw_send_data_wait_callback(int fd,
                                      uint8_t expected_callback_id,
                                      uint8_t *tx_status);


static int zw_send_data_transaction(int fd,
                                    uint8_t node_id,
                                    const uint8_t *command,
                                    size_t command_len,
                                    uint8_t tx_options,
                                    uint8_t callback_id)
{
    uint8_t request[MAX_FRAME];
    uint8_t response[MAX_FRAME];
    uint8_t early_sof = 0;
    size_t request_len = 0;
    size_t response_len = 0;
    int ctrl;

    printf("\n");
    printf("========================================\n");
    printf(" V7.2 ZW_SEND_DATA TRANSACTION\n");
    printf("========================================\n");

    printf("[+] Node ID                : %u (0x%02X)\n",
           node_id,
           node_id);

    printf("[+] Command length         : %zu\n",
           command_len);

    if (command && command_len)
        dump_hex("Z-Wave command", command, command_len);

    printf("[+] TX options             : 0x%02X\n",
           tx_options);

    printf("[+] Callback ID            : 0x%02X\n",
           callback_id);

    if (node_id == 0 || node_id > 232) {
        printf("[-] Node ID invalido\n");
        return -1;
    }

    if (!command || command_len == 0) {
        printf("[-] Command invalido\n");
        return -1;
    }

    if (callback_id == 0) {
        printf("[-] Callback ID 0 no permitido\n");
        return -1;
    }

    if (build_zw_send_data_frame(node_id,
                                 command,
                                 command_len,
                                 tx_options,
                                 callback_id,
                                 request,
                                 sizeof(request),
                                 &request_len) < 0) {
        printf("[-] No se pudo construir ZW_SEND_DATA\n");
        return -1;
    }

    dump_hex("TX FRAME", request, request_len);

    if (write_all(fd, request, request_len) < 0) {
        printf("[-] Error enviando ZW_SEND_DATA\n");
        return -1;
    }

    ctrl = wait_request_ack(fd, &early_sof);

    if (ctrl == ACK) {
        printf("[+] ZW_SEND_DATA ACK recibido\n");

        if (receive_frame(fd,
                          response,
                          sizeof(response),
                          &response_len,
                          0) < 0) {
            printf("[-] RESPONSE ZW_SEND_DATA invalida\n");
            return -1;
        }

    } else if (ctrl == SOF && early_sof) {

        printf("[+] RESPONSE temprana detectada\n");

        if (receive_frame(fd,
                          response,
                          sizeof(response),
                          &response_len,
                          1) < 0) {
            printf("[-] RESPONSE temprana invalida\n");
            return -1;
        }

    } else {
        printf("[-] ZW_SEND_DATA no aceptado por enlace Serial API\n");
        return -1;
    }

    if (response_len < 6) {
        printf("[-] RESPONSE ZW_SEND_DATA demasiado corta\n");
        return -1;
    }

    if (response[2] != RESPONSE) {
        printf("[-] TYPE inesperado: 0x%02X\n",
               response[2]);
        return -1;
    }

    if (response[3] != 0x13) {
        printf("[-] FUNC_ID inesperado: 0x%02X\n",
               response[3]);
        return -1;
    }

    /*
     * RESPONSE de ZW_SEND_DATA:
     *
     * DATA[0] != 0 -> controlador acepta la transmision.
     *
     * Esto NO significa todavia que el nodo haya recibido
     * correctamente el comando. Para eso necesitamos el
     * callback REQUEST/FUNC_ID 0x13 posterior.
     */
    printf("[+] ZW_SEND_DATA RESPONSE  : 0x%02X\n",
           response[4]);

    if (response[4] == 0x00) {
        printf("[-] Controlador rechazo ZW_SEND_DATA\n");
        return -1;
    }

    printf("[+] Controlador acepto ZW_SEND_DATA\n");

    /*
     * V7.6:
     *
     * RESPONSE != callback.
     *
     * La transaccion solo puede considerarse completa
     * despues de recibir REQUEST/FUNC_ID 0x13 con el
     * callback_id correspondiente y TRANSMIT_COMPLETE_OK.
     */
    {
        uint8_t tx_status = 0xFF;

        printf("[+] Esperando callback TX  : 0x%02X\n",
               callback_id);

        if (zw_send_data_wait_callback(fd,
                                       callback_id,
                                       &tx_status) != 0) {
            printf("[-] ZW_SEND_DATA callback fallo\n");
            return -1;
        }

        if (tx_status != 0x00) {
            printf("[-] ZW_SEND_DATA TX status final: 0x%02X\n",
                   tx_status);
            return -1;
        }

        printf("[+] ZW_SEND_DATA transaccion completa\n");
        printf("[+] TRANSMIT_COMPLETE_OK\n");
    }

    return 0;
}




/*
 * ============================================================
 * V7.3 - ZW_SEND_DATA CALLBACK
 * ============================================================
 *
 * Procesa el callback asincrono posterior a ZW_SEND_DATA.
 *
 * Serial API:
 *
 *   TYPE    = REQUEST  (0x00)
 *   FUNC_ID = 0x13     (ZW_SEND_DATA)
 *
 * DATA:
 *
 *   [0] callback_id
 *   [1] tx_status
 *
 * Esta funcion NO lee del puerto y NO transmite nada.
 * Solo valida una trama ya recibida por receive_frame().
 */
static int zw_send_data_process_callback(const uint8_t *frame,
                                         size_t frame_len,
                                         uint8_t expected_callback_id,
                                         uint8_t *tx_status)
{
    uint8_t callback_id;
    uint8_t status;

    if (!frame || !tx_status) {
        printf("[-] V7.3 callback: argumento NULL\n");
        return -1;
    }

    /*
     * Trama minima:
     *
     * SOF LEN TYPE FUNC CALLBACK STATUS CHECKSUM
     *
     * 7 bytes.
     */
    if (frame_len < 7) {
        printf("[-] V7.3 callback demasiado corto: %zu\n",
               frame_len);
        return -1;
    }

    if (frame[0] != SOF) {
        printf("[-] V7.3 callback sin SOF\n");
        return -1;
    }

    if (frame[2] != REQUEST) {
        printf("[-] V7.3 callback TYPE inesperado: 0x%02X\n",
               frame[2]);
        return -1;
    }

    if (frame[3] != 0x13) {
        printf("[-] V7.3 callback FUNC_ID inesperado: 0x%02X\n",
               frame[3]);
        return -1;
    }

    callback_id = frame[4];
    status = frame[5];

    printf("[+] ZW_SEND_DATA callback ID : 0x%02X\n",
           callback_id);

    printf("[+] ZW_SEND_DATA TX status   : 0x%02X\n",
           status);

    if (callback_id != expected_callback_id) {
        printf("[-] Callback ID inesperado: esperado=0x%02X recibido=0x%02X\n",
               expected_callback_id,
               callback_id);
        return -1;
    }

    *tx_status = status;

    /*
     * TRANSMIT_COMPLETE_OK = 0x00
     *
     * En V7.3 mantenemos el valor raw además de interpretar
     * explícitamente el caso de éxito.
     */
    if (status == 0x00) {
        printf("[+] ZW_SEND_DATA TX COMPLETE : OK\n");
        return 0;
    }

    printf("[-] ZW_SEND_DATA TX COMPLETE : fallo status=0x%02X\n",
           status);

    return 1;
}




/*
 * ============================================================
 * V7.4 - ZW_SEND_DATA CALLBACK WAIT PATH
 * ============================================================
 *
 * Espera una trama Serial API y la entrega al parser V7.3.
 *
 * IMPORTANTE:
 *   receive_frame() valida checksum y envia el ACK de
 *   transporte al controlador.
 *
 * En esta fase la funcion NO esta cableada todavia a una
 * transaccion ZW_SEND_DATA real.
 */
static int zw_send_data_wait_callback(int fd,
                                      uint8_t expected_callback_id,
                                      uint8_t *tx_status)
{
    uint8_t frame[MAX_FRAME];
    size_t frame_len = 0;

    printf("\n");
    printf("========================================\n");
    printf(" V7.4 WAIT ZW_SEND_DATA CALLBACK\n");
    printf("========================================\n");

    if (!tx_status) {
        printf("[-] tx_status NULL\n");
        return -1;
    }

    if (expected_callback_id == 0) {
        printf("[-] expected callback ID 0 invalido\n");
        return -1;
    }

    /*
     * V7.10:
     *
     * El callback de ZW_SEND_DATA es asincrono y en hardware
     * real puede tardar bastante mas que una RESPONSE normal.
     *
     * NO modificamos receive_frame(), ya que es una primitiva
     * compartida por muchas otras rutas.
     *
     * Esperamos aqui hasta 30 segundos a que aparezca el primer
     * byte del callback. Cuando el descriptor sea legible,
     * receive_frame() conserva exactamente su comportamiento
     * original para SOF/LENGTH/DATA/checksum/ACK.
     */
    printf("[+] Esperando actividad callback hasta 30000 ms\n");

    {
        int ready = wait_readable(fd, 30000);

        if (ready < 0) {
            printf("[-] Error esperando actividad callback\n");
            return -1;
        }

        if (ready == 0) {
            printf("[-] Timeout 30000 ms esperando callback ZW_SEND_DATA\n");
            return -1;
        }
    }

    printf("[+] Actividad callback detectada\n");

    if (receive_frame(fd,
                      frame,
                      sizeof(frame),
                      &frame_len,
                      0) < 0) {
        printf("[-] No se recibio callback ZW_SEND_DATA valido\n");
        return -1;
    }

    printf("[+] Trama candidata callback recibida\n");

    return zw_send_data_process_callback(frame,
                                         frame_len,
                                         expected_callback_id,
                                         tx_status);
}


/*
 * V7.4 OFFLINE WAIT-PATH SELFTEST
 *
 * socketpair() simula el enlace serie.
 *
 * Un extremo contiene una trama callback valida.
 * zw_send_data_wait_callback() usa receive_frame() REAL.
 *
 * Verificamos tambien que receive_frame() devuelve ACK.
 *
 * NO abre ttyACM0.
 * NO transmite Z-Wave.
 */
static int run_zw_send_data_wait_selftest(void)
{
    int sv[2] = {-1, -1};
    uint8_t tx_status = 0xFF;
    uint8_t ack = 0;
    int rc = 1;

    static const uint8_t callback_ok[] = {
        0x01, 0x05, 0x00, 0x13, 0x01, 0x00, 0xE8
    };

    printf("\n");
    printf("========================================\n");
    printf(" V7.4 CALLBACK WAIT-PATH SELFTEST\n");
    printf("========================================\n");
    printf("[+] OFFLINE: socketpair()\n");
    printf("[+] NO se abre ttyACM0\n");
    printf("[+] NO se transmite Z-Wave\n");

    if (socketpair(AF_UNIX, SOCK_STREAM, 0, sv) < 0) {
        perror("socketpair");
        return 1;
    }

    dump_hex("SYNTH CALLBACK",
             callback_ok,
             sizeof(callback_ok));

    if (write_all(sv[0],
                  callback_ok,
                  sizeof(callback_ok)) < 0) {
        printf("[-] No se pudo inyectar callback sintetico\n");
        goto out;
    }

    if (zw_send_data_wait_callback(sv[1],
                                   0x01,
                                   &tx_status) != 0) {
        printf("[-] Wait path rechazo callback valido\n");
        goto out;
    }

    if (tx_status != 0x00) {
        printf("[-] TX status inesperado: 0x%02X\n",
               tx_status);
        goto out;
    }

    /*
     * receive_frame() debe haber enviado ACK por sv[1].
     * Lo leemos desde el otro extremo.
     */
    if (read_byte_timeout(sv[0], &ack, 500) != 1) {
        printf("[-] No se recibio ACK de receive_frame()\n");
        goto out;
    }

    printf("[+] ACK devuelto            : 0x%02X\n",
           ack);

    if (ack != ACK) {
        printf("[-] Control inesperado: 0x%02X\n",
               ack);
        goto out;
    }

    printf("[+] Callback recibido por receive_frame REAL\n");
    printf("[+] Parser V7.3 ejecutado correctamente\n");
    printf("[+] ACK de transporte verificado\n");
    printf("[+] SELFTEST WAIT-PATH V7.4 OK\n");

    rc = 0;

out:
    if (sv[0] >= 0)
        close(sv[0]);

    if (sv[1] >= 0)
        close(sv[1]);

    return rc;
}



/*
 * ============================================================
 * V7.5 - FULL ZW_SEND_DATA TRANSACTION OFFLINE SELFTEST
 * ============================================================
 *
 * Simula un controlador Serial API mediante socketpair()+fork().
 *
 * PADRE:
 *   ejecuta zw_send_data_transaction() REAL
 *   y despues zw_send_data_wait_callback() REAL.
 *
 * HIJO:
 *   recibe la peticion ZW_SEND_DATA,
 *   devuelve ACK,
 *   devuelve RESPONSE/FUNC_ID 0x13 aceptada,
 *   espera el ACK de receive_frame(),
 *   envia CALLBACK REQUEST/FUNC_ID 0x13 con TX OK,
 *   y verifica el ACK final.
 *
 * NO abre ttyACM0.
 * NO transmite radio Z-Wave.
 */
static int run_zw_send_data_full_transaction_selftest(void)
{
    int sv[2] = {-1, -1};
    pid_t pid;
    int status = 0;
    int rc = 1;

    static const uint8_t command[] = {
        0x84, 0x05
    };

    /*
     * RESPONSE ZW_SEND_DATA:
     *
     * SOF LEN RESPONSE FUNC DATA CHECKSUM
     *
     * DATA = 0x01 -> accepted
     */
    uint8_t response[] = {
        SOF, 0x04, RESPONSE, 0x13, 0x01, 0x00
    };

    /*
     * CALLBACK:
     *
     * callback_id = 0x01
     * tx_status   = 0x00
     */
    uint8_t callback[] = {
        SOF, 0x05, REQUEST, 0x13, 0x01, 0x00, 0x00
    };

    printf("\n");
    printf("========================================\n");
    printf(" V7.5 FULL ZW_SEND_DATA SELFTEST\n");
    printf("========================================\n");
    printf("[+] OFFLINE: socketpair() + fork()\n");
    printf("[+] NO se abre ttyACM0\n");
    printf("[+] NO se transmite Z-Wave\n");

    response[sizeof(response) - 1] =
        zw_checksum(&response[1], response[1]);

    callback[sizeof(callback) - 1] =
        zw_checksum(&callback[1], callback[1]);

    if (socketpair(AF_UNIX, SOCK_STREAM, 0, sv) < 0) {
        perror("socketpair");
        return 1;
    }

    pid = fork();

    if (pid < 0) {
        perror("fork");
        close(sv[0]);
        close(sv[1]);
        return 1;
    }

    if (pid == 0) {
        uint8_t frame[MAX_FRAME];
        uint8_t b = 0;
        size_t pos = 0;
        size_t total = 0;

        close(sv[1]);

        /*
         * Recibir la peticion completa generada por
         * zw_send_data_transaction().
         *
         * Primero SOF y LEN; despues LEN bytes restantes.
         */
        if (read_byte_timeout(sv[0], &frame[0], 1000) != 1 ||
            frame[0] != SOF) {
            printf("[-] CHILD: no recibio SOF ZW_SEND_DATA\n");
            _exit(10);
        }

        if (read_byte_timeout(sv[0], &frame[1], 1000) != 1) {
            printf("[-] CHILD: no recibio LEN\n");
            _exit(11);
        }

        total = (size_t)frame[1] + 2;

        if (total > sizeof(frame) || total < 5) {
            printf("[-] CHILD: longitud invalida\n");
            _exit(12);
        }

        pos = 2;

        while (pos < total) {
            if (read_byte_timeout(sv[0],
                                  &frame[pos],
                                  1000) != 1) {
                printf("[-] CHILD: peticion incompleta\n");
                _exit(13);
            }
            pos++;
        }

        dump_hex("CHILD RX ZW_SEND_DATA", frame, total);

        if (frame[2] != REQUEST || frame[3] != 0x13) {
            printf("[-] CHILD: no es ZW_SEND_DATA\n");
            _exit(14);
        }

        /*
         * ACK de transporte a la peticion HOST.
         */
        b = ACK;

        if (write_all(sv[0], &b, 1) < 0)
            _exit(15);

        /*
         * RESPONSE aceptada.
         */
        if (write_all(sv[0],
                      response,
                      sizeof(response)) < 0)
            _exit(16);

        /*
         * El padre procesa RESPONSE mediante receive_frame(),
         * por lo que debe devolvernos ACK.
         */
        if (read_byte_timeout(sv[0], &b, 1000) != 1 ||
            b != ACK) {
            printf("[-] CHILD: falta ACK de RESPONSE\n");
            _exit(17);
        }

        /*
         * Enviamos ahora el callback asincrono TX COMPLETE.
         */
        if (write_all(sv[0],
                      callback,
                      sizeof(callback)) < 0)
            _exit(18);

        /*
         * zw_send_data_wait_callback() usa receive_frame(),
         * que debe ACKear tambien este callback.
         */
        if (read_byte_timeout(sv[0], &b, 1000) != 1 ||
            b != ACK) {
            printf("[-] CHILD: falta ACK de CALLBACK\n");
            _exit(19);
        }

        close(sv[0]);
        _exit(0);
    }

    /*
     * PADRE: ejecuta exactamente las piezas reales que
     * posteriormente utilizaremos sobre ttyACM0.
     */
    close(sv[0]);
    sv[0] = -1;

    if (zw_send_data_transaction(sv[1],
                                 2,
                                 command,
                                 sizeof(command),
                                 0x25,
                                 0x01) != 0) {
        printf("[-] V7.5 transaction fallo\n");
        goto parent_out;
    }

    /*
     * V7.6:
     *
     * zw_send_data_transaction() ya ha consumido y
     * validado RESPONSE + CALLBACK.
     */
    printf("[+] V7.6 FULL TRANSACTION aceptada\n");
    printf("[+] V7.6 CALLBACK integrado OK\n");

    rc = 0;

parent_out:
    if (sv[1] >= 0) {
        close(sv[1]);
        sv[1] = -1;
    }

    if (waitpid(pid, &status, 0) < 0) {
        perror("waitpid");
        return 1;
    }

    if (!WIFEXITED(status)) {
        printf("[-] CHILD termino anormalmente\n");
        return 1;
    }

    if (WEXITSTATUS(status) != 0) {
        printf("[-] CHILD exit=%d\n",
               WEXITSTATUS(status));
        return 1;
    }

    if (rc != 0)
        return 1;

    printf("[+] Simulador Serial API termino correctamente\n");
    printf("[+] ACK peticion verificado\n");
    printf("[+] RESPONSE ZW_SEND_DATA verificada\n");
    printf("[+] ACK RESPONSE verificado\n");
    printf("[+] CALLBACK ZW_SEND_DATA verificado\n");
    printf("[+] ACK CALLBACK verificado\n");
    printf("[+] SELFTEST FULL TRANSACTION V7.5 OK\n");
    printf("========================================\n");

    return 0;
}


/*
 * V7.3 ZW_SEND_DATA CALLBACK PARSER OFFLINE SELFTEST
 *
 * No abre ttyACM0.
 * No transmite Z-Wave.
 *
 * Las tramas ya estan completas y se entregan
 * directamente al parser V7.3.
 */
static int run_zw_send_data_callback_selftest(void)
{
    uint8_t tx_status = 0xFF;
    int rc;

    /*
     * REQUEST / FUNC_ID 0x13
     * callback_id = 0x01
     * tx_status   = 0x00
     *
     * LENGTH = TYPE + FUNC + DATA(2) + CHECKSUM = 5
     */
    uint8_t ok_frame[] = {
        SOF, 0x05, REQUEST, 0x13, 0x01, 0x00, 0x00
    };

    uint8_t bad_id_frame[] = {
        SOF, 0x05, REQUEST, 0x13, 0x02, 0x00, 0x00
    };

    uint8_t failed_tx_frame[] = {
        SOF, 0x05, REQUEST, 0x13, 0x01, 0x01, 0x00
    };

    /*
     * El parser actual no recalcula checksum porque recibe
     * una trama que conceptualmente ya ha pasado por
     * receive_frame(). Aun asi dejamos checksum correcto
     * para que las muestras sean tramas Serial API validas.
     */
    ok_frame[6] =
        zw_checksum(&ok_frame[1], sizeof(ok_frame) - 2);

    bad_id_frame[6] =
        zw_checksum(&bad_id_frame[1],
                    sizeof(bad_id_frame) - 2);

    failed_tx_frame[6] =
        zw_checksum(&failed_tx_frame[1],
                    sizeof(failed_tx_frame) - 2);

    printf("\n");
    printf("========================================\n");
    printf(" V7.3 ZW_SEND_DATA CALLBACK SELFTEST\n");
    printf("========================================\n");
    printf("[+] OFFLINE: no se abre ttyACM0\n");
    printf("[+] OFFLINE: no se transmite Z-Wave\n");

    /*
     * TEST 1:
     * callback correcto y TRANSMIT_COMPLETE_OK.
     */
    printf("\n[TEST 1] CALLBACK correcto / TX OK\n");

    dump_hex("SYNTH CALLBACK",
             ok_frame,
             sizeof(ok_frame));

    tx_status = 0xFF;

    rc = zw_send_data_process_callback(
            ok_frame,
            sizeof(ok_frame),
            0x01,
            &tx_status);

    if (rc != 0) {
        printf("[-] TEST 1 esperaba rc=0, recibido=%d\n",
               rc);
        return 1;
    }

    if (tx_status != 0x00) {
        printf("[-] TEST 1 tx_status inesperado: 0x%02X\n",
               tx_status);
        return 1;
    }

    printf("[+] TEST 1 OK\n");

    /*
     * TEST 2:
     * callback perteneciente a otra transaccion.
     */
    printf("\n[TEST 2] CALLBACK ID incorrecto\n");

    dump_hex("SYNTH CALLBACK",
             bad_id_frame,
             sizeof(bad_id_frame));

    tx_status = 0xFF;

    rc = zw_send_data_process_callback(
            bad_id_frame,
            sizeof(bad_id_frame),
            0x01,
            &tx_status);

    if (rc >= 0) {
        printf("[-] TEST 2 esperaba rechazo por Callback ID\n");
        return 1;
    }

    printf("[+] TEST 2 rechazado correctamente\n");

    /*
     * TEST 3:
     * callback correcto pero transmision fallida.
     */
    printf("\n[TEST 3] CALLBACK correcto / TX FAIL\n");

    dump_hex("SYNTH CALLBACK",
             failed_tx_frame,
             sizeof(failed_tx_frame));

    tx_status = 0xFF;

    rc = zw_send_data_process_callback(
            failed_tx_frame,
            sizeof(failed_tx_frame),
            0x01,
            &tx_status);

    if (rc != 1) {
        printf("[-] TEST 3 esperaba rc=1, recibido=%d\n",
               rc);
        return 1;
    }

    if (tx_status != 0x01) {
        printf("[-] TEST 3 tx_status inesperado: 0x%02X\n",
               tx_status);
        return 1;
    }

    printf("[+] TEST 3 fallo TX detectado correctamente\n");

    printf("\n");
    printf("[+] SELFTEST CALLBACK V7.3 OK\n");
    printf("========================================\n");

    return 0;
}


/*
 * V7.2 OFFLINE TRANSACTION CORE SELFTEST
 *
 * Invoca zw_send_data_transaction() exclusivamente
 * con argumentos invalidos.
 *
 * Todas las rutas deben terminar ANTES de write_all().
 * fd=-1 nunca debe utilizarse.
 */
static int run_zw_send_data_transaction_selftest(void)
{
    static const uint8_t command[] = {
        0x84, 0x05
    };

    int rc;

    printf("\n");
    printf("========================================\n");
    printf(" V7.2 TRANSACTION CORE OFFLINE SELFTEST\n");
    printf("========================================\n");
    printf("[+] NO se abre ttyACM0\n");
    printf("[+] NO se transmite Z-Wave\n");

    printf("\n[TEST 1] Node ID = 0\n");

    rc = zw_send_data_transaction(-1,
                                  0,
                                  command,
                                  sizeof(command),
                                  0x25,
                                  0x01);

    if (rc == 0) {
        printf("[-] Node 0 fue aceptado inesperadamente\n");
        return 1;
    }

    printf("[+] Node 0 rechazado correctamente\n");

    printf("\n[TEST 2] Command NULL\n");

    rc = zw_send_data_transaction(-1,
                                  2,
                                  NULL,
                                  0,
                                  0x25,
                                  0x01);

    if (rc == 0) {
        printf("[-] Command NULL fue aceptado\n");
        return 1;
    }

    printf("[+] Command NULL rechazado correctamente\n");

    printf("\n[TEST 3] Callback ID = 0\n");

    rc = zw_send_data_transaction(-1,
                                  2,
                                  command,
                                  sizeof(command),
                                  0x25,
                                  0x00);

    if (rc == 0) {
        printf("[-] Callback 0 fue aceptado\n");
        return 1;
    }

    printf("[+] Callback 0 rechazado correctamente\n");

    printf("\n");
    printf("[+] Todas las rutas terminaron antes de TX\n");
    printf("[+] SELFTEST V7.2 OK\n");
    printf("========================================\n");

    return 0;
}


static int run_passive_listener(int fd)
{
    uint8_t frame[MAX_FRAME];
    size_t frame_len;
    unsigned int frames = 0;
    unsigned int app_commands = 0;

    printf("\n");
    printf("========================================\n");
    printf(" V7.0 PASSIVE SERIAL API LISTENER\n");
    printf("========================================\n");
    printf("[+] SOLO RECEPCION de eventos Serial API\n");
    printf("[+] NO ZW_SEND_DATA\n");
    printf("[+] NO inclusion/exclusion\n");
    printf("[+] Ctrl-C para terminar\n");
    printf("\n");
    printf("[+] Manipula ahora el dispositivo Z-Wave.\n");
    printf("[+] Para DCH-Z110: abre/cierra el contacto,\n");
    printf("[+] pulsa tamper, etc.\n");

    for (;;) {
        int r;

        printf("\n");
        printf("----- ESPERANDO EVENTO %u -----\n",
               frames + 1);

        r = receive_frame(fd,
                          frame,
                          sizeof(frame),
                          &frame_len,
                          0);

        if (r < 0) {
            /*
             * receive_frame() tiene timeout finito.
             * En modo listener no lo consideramos fatal:
             * seguimos escuchando.
             */
            continue;
        }

        frames++;

        if (frame_len < 5) {
            printf("[!] trama demasiado corta\n");
            continue;
        }

        printf("[+] Serial API TYPE        : 0x%02X (%s)\n",
               frame[2],
               frame[2] == REQUEST ? "REQUEST" :
               frame[2] == RESPONSE ? "RESPONSE" :
                                      "UNKNOWN");

        printf("[+] Serial API FUNC_ID     : 0x%02X\n",
               frame[3]);

        if (frame[2] == REQUEST &&
            frame[3] == 0x04) {

            app_commands++;

            printf("\n");
            printf(">>> APPLICATION COMMAND #%u <<<\n",
                   app_commands);

            decode_application_command_handler(frame,
                                               frame_len);
        } else {
            printf("[+] Evento Serial API no decodificado\n");
        }

        printf("[+] Frames recibidos       : %u\n",
               frames);
        printf("[+] Application commands   : %u\n",
               app_commands);
    }

    /* No alcanzable normalmente: salida mediante Ctrl-C. */
    return 0;
}



/*
 * ============================================================
 * V7.7 - REAL ZW_SEND_DATA ARMED DRY-RUN
 * ============================================================
 *
 * Prepara exactamente la primera operacion Z-Wave real que
 * queremos realizar, pero DELIBERADAMENTE NO llama a
 * zw_send_data_transaction().
 *
 * No abre ttyACM0.
 * No transmite ningun byte.
 *
 * Parametros bloqueados:
 *
 *   Node ID     = 2
 *   Command     = 84 05
 *   TX options  = 25
 *   Callback ID = 01
 */
static int run_zw_send_data_real_armed_dry_run(void)
{
    static const uint8_t command[] = {
        0x84, 0x05
    };

    uint8_t frame[MAX_FRAME];
    size_t frame_len = 0;

    const uint8_t node_id = 2;
    const uint8_t tx_options = 0x25;
    const uint8_t callback_id = 0x01;

    printf("\n");
    printf("========================================\n");
    printf(" V7.7 REAL ZW_SEND_DATA — ARMED DRY-RUN\n");
    printf("========================================\n");

    printf("[+] ARMED pero TRANSMISION BLOQUEADA\n");
    printf("[+] NO se abre ttyACM0\n");
    printf("[+] NO se transmite Z-Wave\n");

    printf("\n");
    printf("[+] Node ID                : %u (0x%02X)\n",
           node_id,
           node_id);

    dump_hex("Z-Wave command",
             command,
             sizeof(command));

    printf("[+] TX options             : 0x%02X\n",
           tx_options);

    printf("[+] Callback ID            : 0x%02X\n",
           callback_id);

    if (build_zw_send_data_frame(node_id,
                                 command,
                                 sizeof(command),
                                 tx_options,
                                 callback_id,
                                 frame,
                                 sizeof(frame),
                                 &frame_len) < 0) {
        printf("[-] No se pudo construir frame V7.7\n");
        return 1;
    }

    dump_hex("ARMED TX FRAME",
             frame,
             frame_len);

    /*
     * Primera operacion prevista:
     *
     * 01 09 00 13 02 02 84 05 25 01 40
     */
    static const uint8_t expected[] = {
        0x01, 0x09, 0x00, 0x13,
        0x02, 0x02, 0x84, 0x05,
        0x25, 0x01, 0x40
    };

    if (frame_len != sizeof(expected) ||
        memcmp(frame, expected, sizeof(expected)) != 0) {
        printf("[-] FRAME V7.7 NO coincide con golden frame\n");
        return 1;
    }

    printf("[+] Golden frame verificado byte a byte\n");

    printf("\n");
    printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
    printf(" TRANSMISION REAL BLOQUEADA EN V7.7\n");
    printf(" NO se ha llamado zw_send_data_transaction()\n");
    printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");

    printf("[+] V7.7 ARMED DRY-RUN OK\n");

    return 0;
}



/*
 * V7.12 - OEM CMDQ ENTRY MODEL
 *
 * Reconstruido a partir del zw_center OEM:
 *
 *   byte 0     = longitud del comando Z-Wave
 *   byte 1     = Command Class
 *   byte 2     = Command
 *   byte 3...  = payload
 *
 * cmdq_add()/cmdq_remove() copian exactamente 33 bytes.
 *
 * ESTE BLOQUE ES PURAMENTE OFFLINE.
 * NO abre ttyACM0.
 * NO ejecuta ZW_SEND_DATA.
 */

#define OEM_CMDQ_ENTRY_SIZE       33U
#define OEM_CMDQ_COMMAND_MAX      32U

struct oem_cmdq_entry {
    uint8_t len;
    uint8_t command[OEM_CMDQ_COMMAND_MAX];
};

static int oem_cmdq_entry_build(struct oem_cmdq_entry *entry,
                                const uint8_t *command,
                                size_t command_len)
{
    if (entry == NULL || command == NULL)
        return -1;

    if (command_len == 0 || command_len > OEM_CMDQ_COMMAND_MAX)
        return -1;

    memset(entry, 0, sizeof(*entry));

    entry->len = (uint8_t)command_len;
    memcpy(entry->command, command, command_len);

    return 0;
}

static int run_oem_cmdq_model_selftest(void)
{
    struct oem_cmdq_entry entry;

    static const uint8_t wake_up_no_more_information[] = {
        0x84, 0x08
    };

    static const uint8_t expected_prefix[] = {
        0x02, 0x84, 0x08
    };

    const uint8_t *raw = (const uint8_t *)&entry;

    printf("========================================\n");
    printf(" V7.12 OEM CMDQ MODEL SELFTEST\n");
    printf("========================================\n");
    printf("[+] OFFLINE ONLY\n");
    printf("[+] NO ttyACM0\n");
    printf("[+] NO Serial API\n");
    printf("[+] NO ZW_SEND_DATA\n");
    printf("\n");

    printf("[+] sizeof(oem_cmdq_entry) : %zu\n",
           sizeof(entry));

    if (sizeof(entry) != OEM_CMDQ_ENTRY_SIZE) {
        printf("[-] Tamano CMDQ inesperado: %zu\n",
               sizeof(entry));
        return -1;
    }

    if (oem_cmdq_entry_build(
            &entry,
            wake_up_no_more_information,
            sizeof(wake_up_no_more_information)) < 0) {
        printf("[-] No se pudo construir CMDQ entry\n");
        return -1;
    }

    dump_hex("OEM CMDQ ENTRY", raw, sizeof(entry));

    printf("[+] entry[0] length        : %u\n", raw[0]);
    printf("[+] entry[1] Command Class : 0x%02X\n", raw[1]);
    printf("[+] entry[2] Command       : 0x%02X\n", raw[2]);

    if (memcmp(raw,
               expected_prefix,
               sizeof(expected_prefix)) != 0) {
        printf("[-] Prefix OEM CMDQ incorrecto\n");
        return -1;
    }

    for (size_t i = sizeof(expected_prefix);
         i < sizeof(entry);
         ++i) {
        if (raw[i] != 0) {
            printf("[-] Padding no nulo en offset %zu: 0x%02X\n",
                   i, raw[i]);
            return -1;
        }
    }

    /*
     * Pruebas negativas.
     */
    if (oem_cmdq_entry_build(&entry,
                             wake_up_no_more_information,
                             0) == 0) {
        printf("[-] Se acepto longitud cero\n");
        return -1;
    }

    {
        uint8_t oversized[OEM_CMDQ_COMMAND_MAX + 1];

        memset(oversized, 0xAA, sizeof(oversized));

        if (oem_cmdq_entry_build(&entry,
                                 oversized,
                                 sizeof(oversized)) == 0) {
            printf("[-] Se acepto comando >32 bytes\n");
            return -1;
        }
    }

    printf("[+] Layout 33 bytes verificado\n");
    printf("[+] length + command verificados\n");
    printf("[+] padding verificado\n");
    printf("[+] limites verificados\n");
    printf("[+] OEM CMDQ MODEL SELFTEST OK\n");
    printf("========================================\n");

    return 0;
}


/*
 * V7.12 STAGE 2B - OEM WAKE-UP DECISION MODEL
 *
 * Modelo PURAMENTE OFFLINE.
 *
 * NO abre ttyACM0.
 * NO usa Serial API.
 * NO ejecuta ZW_SEND_DATA.
 */

enum oem_wakeup_action {
    OEM_WAKEUP_ACTION_NONE = 0,
    OEM_WAKEUP_ACTION_SEND = 1
};

static int oem_wakeup_decide(uint8_t event_node,
                             uint8_t expected_node,
                             const uint8_t *event,
                             size_t event_len,
                             const struct oem_cmdq_entry *entry,
                             enum oem_wakeup_action *action)
{
    if (action == NULL)
        return -1;

    *action = OEM_WAKEUP_ACTION_NONE;

    if (event == NULL || event_len != 2)
        return -1;

    if (event_node == 0 || event_node > 232)
        return -1;

    if (expected_node == 0 || expected_node > 232)
        return -1;

    /*
     * Evento procedente de otro nodo.
     */
    if (event_node != expected_node)
        return 0;

    /*
     * Solo:
     *
     *   COMMAND_CLASS_WAKE_UP = 0x84
     *   WAKE_UP_NOTIFICATION  = 0x07
     */
    if (event[0] != 0x84 || event[1] != 0x07)
        return 0;

    /*
     * CMDQ vacia.
     */
    if (entry == NULL || entry->len == 0)
        return 0;

    if (entry->len > OEM_CMDQ_COMMAND_MAX)
        return -1;

    *action = OEM_WAKEUP_ACTION_SEND;

    return 0;
}


static int run_oem_wakeup_decision_selftest(void)
{
    static const uint8_t wake_notification[] = {
        0x84, 0x07
    };

    static const uint8_t wake_no_more_information[] = {
        0x84, 0x08
    };

    static const uint8_t battery_get[] = {
        0x80, 0x02
    };

    struct oem_cmdq_entry entry;
    enum oem_wakeup_action action;
    int rc;

    printf("========================================\n");
    printf(" V7.12 OEM WAKE-UP DECISION SELFTEST\n");
    printf("========================================\n");
    printf("[+] OFFLINE ONLY\n");
    printf("[+] NO ttyACM0\n");
    printf("[+] NO Serial API\n");
    printf("[+] NO ZW_SEND_DATA\n");
    printf("\n");

    /*
     * TEST 1
     * NODE 4 despierta y CMDQ contiene 84 08.
     */
    if (oem_cmdq_entry_build(
            &entry,
            wake_no_more_information,
            sizeof(wake_no_more_information)) < 0) {
        printf("[-] TEST1 build fallo\n");
        return -1;
    }

    rc = oem_wakeup_decide(
        4, 4,
        wake_notification,
        sizeof(wake_notification),
        &entry,
        &action);

    printf("[TEST1] NODE4 + 84 07 + CMDQ 84 08 -> ");

    if (rc != 0 || action != OEM_WAKEUP_ACTION_SEND) {
        printf("FAIL\n");
        return -1;
    }

    printf("SEND [OK]\n");


    /*
     * TEST 2
     * CMDQ vacia.
     */
    memset(&entry, 0, sizeof(entry));

    rc = oem_wakeup_decide(
        4, 4,
        wake_notification,
        sizeof(wake_notification),
        &entry,
        &action);

    printf("[TEST2] NODE4 + CMDQ vacia          -> ");

    if (rc != 0 || action != OEM_WAKEUP_ACTION_NONE) {
        printf("FAIL\n");
        return -1;
    }

    printf("NONE [OK]\n");


    /*
     * TEST 3
     * Evento procedente de otro nodo.
     */
    if (oem_cmdq_entry_build(
            &entry,
            wake_no_more_information,
            sizeof(wake_no_more_information)) < 0)
        return -1;

    rc = oem_wakeup_decide(
        3, 4,
        wake_notification,
        sizeof(wake_notification),
        &entry,
        &action);

    printf("[TEST3] NODE3 != NODE4              -> ");

    if (rc != 0 || action != OEM_WAKEUP_ACTION_NONE) {
        printf("FAIL\n");
        return -1;
    }

    printf("NONE [OK]\n");


    /*
     * TEST 4
     * RX 84 08 no es WAKE_UP_NOTIFICATION.
     */
    rc = oem_wakeup_decide(
        4, 4,
        wake_no_more_information,
        sizeof(wake_no_more_information),
        &entry,
        &action);

    printf("[TEST4] RX 84 08                    -> ");

    if (rc != 0 || action != OEM_WAKEUP_ACTION_NONE) {
        printf("FAIL\n");
        return -1;
    }

    printf("NONE [OK]\n");


    /*
     * TEST 5
     * Cualquier comando valido pendiente debe producir SEND.
     */
    if (oem_cmdq_entry_build(
            &entry,
            battery_get,
            sizeof(battery_get)) < 0)
        return -1;

    rc = oem_wakeup_decide(
        4, 4,
        wake_notification,
        sizeof(wake_notification),
        &entry,
        &action);

    printf("[TEST5] NODE4 + CMDQ 80 02           -> ");

    if (rc != 0 || action != OEM_WAKEUP_ACTION_SEND) {
        printf("FAIL\n");
        return -1;
    }

    printf("SEND [OK]\n");


    /*
     * TEST 6
     * Longitud CMDQ corrupta.
     */
    memset(&entry, 0, sizeof(entry));
    entry.len = OEM_CMDQ_COMMAND_MAX + 1;

    rc = oem_wakeup_decide(
        4, 4,
        wake_notification,
        sizeof(wake_notification),
        &entry,
        &action);

    printf("[TEST6] CMDQ length > 32             -> ");

    if (rc == 0) {
        printf("FAIL\n");
        return -1;
    }

    printf("REJECT [OK]\n");


    /*
     * TEST 7
     * Evento wake-up truncado.
     */
    rc = oem_wakeup_decide(
        4, 4,
        wake_notification,
        1,
        &entry,
        &action);

    printf("[TEST7] WAKE event truncado          -> ");

    if (rc == 0) {
        printf("FAIL\n");
        return -1;
    }

    printf("REJECT [OK]\n");

    printf("\n");
    printf("[+] OEM WAKE-UP DECISION MODEL OK\n");
    printf("[+] NINGUNA TRANSMISION REAL EJECUTADA\n");
    printf("========================================\n");

    return 0;
}



/*
 * V7.12 STAGE 3 - OEM WAKE-UP PIPELINE OFFLINE
 *
 * Integra:
 *
 *   CMDQ entry
 *        +
 *   WAKE_UP_NOTIFICATION
 *        +
 *   decision SEND/NONE
 *        +
 *   extraccion del comando pendiente
 *
 * El resultado queda preparado para entregarse al transporte
 * ZW_SEND_DATA ya validado por run_send_data_full_selftest().
 *
 * NO abre ttyACM0.
 * NO transmite Z-Wave.
 */

struct oem_wakeup_pipeline_result {
    enum oem_wakeup_action action;
    uint8_t node_id;
    uint8_t command_len;
    uint8_t command[OEM_CMDQ_COMMAND_MAX];
};

static int oem_wakeup_pipeline_run(
        uint8_t event_node,
        uint8_t expected_node,
        const uint8_t *event,
        size_t event_len,
        struct oem_cmdq_entry *entry,
        struct oem_wakeup_pipeline_result *result)
{
    enum oem_wakeup_action action;
    int rc;

    if (result == NULL)
        return -1;

    memset(result, 0, sizeof(*result));

    rc = oem_wakeup_decide(
        event_node,
        expected_node,
        event,
        event_len,
        entry,
        &action);

    if (rc != 0)
        return -1;

    result->action = action;

    if (action == OEM_WAKEUP_ACTION_NONE)
        return 0;

    if (entry == NULL)
        return -1;

    if (entry->len == 0 ||
        entry->len > OEM_CMDQ_COMMAND_MAX)
        return -1;

    result->node_id = expected_node;
    result->command_len = entry->len;

    memcpy(result->command,
           entry->command,
           entry->len);

    /*
     * Modelo de dequeue:
     *
     * una vez extraido el comando para envio,
     * la entrada queda vacia.
     */
    memset(entry, 0, sizeof(*entry));

    return 0;
}


static int run_oem_wakeup_pipeline_selftest(void)
{
    static const uint8_t wake_notification[] = {
        0x84, 0x07
    };

    static const uint8_t wake_no_more_information[] = {
        0x84, 0x08
    };

    static const uint8_t battery_get[] = {
        0x80, 0x02
    };

    struct oem_cmdq_entry entry;
    struct oem_wakeup_pipeline_result result;
    int rc;

    printf("========================================\n");
    printf(" V7.12 OEM WAKE-UP PIPELINE SELFTEST\n");
    printf("========================================\n");
    printf("[+] OFFLINE ONLY\n");
    printf("[+] NO ttyACM0\n");
    printf("[+] NO Serial API REAL\n");
    printf("[+] NO Z-WAVE REAL\n");
    printf("\n");


    /*
     * TEST 1
     *
     * CMDQ:
     *      84 08
     *
     * RX:
     *      NODE 4 / 84 07
     *
     * Esperado:
     *      SEND NODE 4 / 84 08
     *      CMDQ vacia despues del dequeue.
     */
    if (oem_cmdq_entry_build(
            &entry,
            wake_no_more_information,
            sizeof(wake_no_more_information)) != 0) {
        printf("[-] TEST1 build fallo\n");
        return -1;
    }

    printf("[TEST1] ENQUEUE CMDQ             : ");
    printf("%02X %02X\n",
           entry.command[0],
           entry.command[1]);

    rc = oem_wakeup_pipeline_run(
        4,
        4,
        wake_notification,
        sizeof(wake_notification),
        &entry,
        &result);

    printf("[TEST1] RX                      : NODE 4 / 84 07\n");

    if (rc != 0) {
        printf("[TEST1] PIPELINE                : FAIL\n");
        return -1;
    }

    if (result.action != OEM_WAKEUP_ACTION_SEND) {
        printf("[TEST1] ACTION                  : FAIL\n");
        return -1;
    }

    if (result.node_id != 4 ||
        result.command_len != 2 ||
        result.command[0] != 0x84 ||
        result.command[1] != 0x08) {
        printf("[TEST1] EXTRACT                 : FAIL\n");
        return -1;
    }

    printf("[TEST1] ACTION                  : SEND [OK]\n");
    printf("[TEST1] TX NODE                 : %u [OK]\n",
           result.node_id);
    printf("[TEST1] TX COMMAND              : %02X %02X [OK]\n",
           result.command[0],
           result.command[1]);

    if (entry.len != 0) {
        printf("[TEST1] DEQUEUE                 : FAIL\n");
        return -1;
    }

    printf("[TEST1] DEQUEUE                 : CMDQ EMPTY [OK]\n");


    /*
     * TEST 2
     *
     * CMDQ vacia + wake-up.
     * No debe producir SEND.
     */
    memset(&entry, 0, sizeof(entry));

    rc = oem_wakeup_pipeline_run(
        4,
        4,
        wake_notification,
        sizeof(wake_notification),
        &entry,
        &result);

    printf("[TEST2] WAKE + EMPTY CMDQ        : ");

    if (rc != 0 ||
        result.action != OEM_WAKEUP_ACTION_NONE) {
        printf("FAIL\n");
        return -1;
    }

    printf("NONE [OK]\n");


    /*
     * TEST 3
     *
     * Tenemos comando pendiente pero despierta NODE 3.
     * La entrada NO debe consumirse.
     */
    if (oem_cmdq_entry_build(
            &entry,
            wake_no_more_information,
            sizeof(wake_no_more_information)) != 0)
        return -1;

    rc = oem_wakeup_pipeline_run(
        3,
        4,
        wake_notification,
        sizeof(wake_notification),
        &entry,
        &result);

    printf("[TEST3] WRONG NODE               : ");

    if (rc != 0 ||
        result.action != OEM_WAKEUP_ACTION_NONE ||
        entry.len != 2) {
        printf("FAIL\n");
        return -1;
    }

    printf("NONE + CMDQ PRESERVED [OK]\n");


    /*
     * TEST 4
     *
     * Un comando distinto tambien debe poder atravesar
     * el pipeline.
     */
    if (oem_cmdq_entry_build(
            &entry,
            battery_get,
            sizeof(battery_get)) != 0)
        return -1;

    rc = oem_wakeup_pipeline_run(
        4,
        4,
        wake_notification,
        sizeof(wake_notification),
        &entry,
        &result);

    printf("[TEST4] BATTERY_GET              : ");

    if (rc != 0 ||
        result.action != OEM_WAKEUP_ACTION_SEND ||
        result.command_len != 2 ||
        result.command[0] != 0x80 ||
        result.command[1] != 0x02 ||
        entry.len != 0) {
        printf("FAIL\n");
        return -1;
    }

    printf("SEND + DEQUEUE [OK]\n");


    /*
     * TEST 5
     *
     * Evento incorrecto. La cola debe conservarse.
     */
    if (oem_cmdq_entry_build(
            &entry,
            wake_no_more_information,
            sizeof(wake_no_more_information)) != 0)
        return -1;

    rc = oem_wakeup_pipeline_run(
        4,
        4,
        wake_no_more_information,
        sizeof(wake_no_more_information),
        &entry,
        &result);

    printf("[TEST5] RX 84 08                 : ");

    if (rc != 0 ||
        result.action != OEM_WAKEUP_ACTION_NONE ||
        entry.len != 2) {
        printf("FAIL\n");
        return -1;
    }

    printf("NONE + CMDQ PRESERVED [OK]\n");


    /*
     * TEST 6
     *
     * Evento truncado debe rechazarse y no consumir CMDQ.
     */
    rc = oem_wakeup_pipeline_run(
        4,
        4,
        wake_notification,
        1,
        &entry,
        &result);

    printf("[TEST6] TRUNCATED WAKE           : ");

    if (rc == 0 || entry.len != 2) {
        printf("FAIL\n");
        return -1;
    }

    printf("REJECT + CMDQ PRESERVED [OK]\n");


    printf("\n");
    printf("===== PIPELINE RESULT =====\n");
    printf("[+] ENQUEUE              OK\n");
    printf("[+] WAKE-UP MATCH        OK\n");
    printf("[+] SEND DECISION        OK\n");
    printf("[+] COMMAND EXTRACTION   OK\n");
    printf("[+] DEQUEUE              OK\n");
    printf("[+] QUEUE PRESERVATION   OK\n");
    printf("\n");
    printf("[+] OEM WAKE-UP PIPELINE OFFLINE OK\n");
    printf("========================================\n");

    return 0;
}


static void usage(const char *prog)
{
    fprintf(stderr,
        "Uso: %s "
        "[--prepare-only|--get-version|--memory-get-id|"
        "--get-init-data|--all-safe|--controller-capabilities|"
        "--serial-capabilities|--capabilities-safe|"
        "--node-info NODE|--inventory|"
        "--add-node-dry-run|"
        "--add-node-callback-selftest|--add-node-state-selftest|"
        "--add-node-pipeline-selftest|--add-node-rx-selftest|"
        "--add-node-loop-selftest|"
        "--add-node-transaction-selftest|"
        "--add-node-failure-selftest|--add-node-real|"
        "--listen|--oem-cmdq-selftest|--oem-wakeup-selftest|--oem-wakeup-pipeline-selftest|--send-data-selftest|"
        "--send-data-transaction-selftest|"
        "--send-data-callback-selftest|"
        "--send-data-wait-selftest|"
        "--send-data-full-selftest|"
        "--send-data-real-armed|"
        "--send-data-real NODE] [dispositivo]\n",
        prog);
}

int main(int argc, char **argv)
{
    const char *dev = "/dev/ttyACM0";
    int mode = 0;
    int fd;
    int rc = 0;
    unsigned long node_id = 0;

    if (argc >= 2) {
        if (!strcmp(argv[1], "--prepare-only"))
            mode = 0;
        else if (!strcmp(argv[1], "--get-version"))
            mode = 1;
        else if (!strcmp(argv[1], "--memory-get-id"))
            mode = 2;
        else if (!strcmp(argv[1], "--get-init-data"))
            mode = 3;
        else if (!strcmp(argv[1], "--all-safe"))
            mode = 4;
        else if (!strcmp(argv[1], "--controller-capabilities"))
            mode = 5;
        else if (!strcmp(argv[1], "--serial-capabilities"))
            mode = 6;
        else if (!strcmp(argv[1], "--capabilities-safe"))
            mode = 7;
        else if (!strcmp(argv[1], "--inventory"))
            mode = 9;
        else if (!strcmp(argv[1], "--add-node-dry-run"))
            mode = 10;
        else if (!strcmp(argv[1], "--add-node-callback-selftest"))
            mode = 11;
        else if (!strcmp(argv[1], "--add-node-state-selftest"))
            mode = 12;
        else if (!strcmp(argv[1], "--add-node-pipeline-selftest"))
            mode = 13;
        else if (!strcmp(argv[1], "--add-node-rx-selftest"))
            mode = 14;
        else if (!strcmp(argv[1], "--add-node-loop-selftest"))
            mode = 15;
        else if (!strcmp(argv[1], "--add-node-transaction-selftest"))
            mode = 16;
        else if (!strcmp(argv[1], "--add-node-failure-selftest"))
            mode = 17;
        else if (!strcmp(argv[1], "--add-node-real"))
            mode = 18;
        else if (!strcmp(argv[1], "--listen"))
            mode = 19;
        else if (!strcmp(argv[1], "--oem-cmdq-selftest"))
            mode = 27;
        else if (!strcmp(argv[1], "--oem-wakeup-selftest"))
            mode = 28;
        else if (!strcmp(argv[1], "--oem-wakeup-pipeline-selftest"))
            mode = 29;
        else if (!strcmp(argv[1], "--send-data-selftest"))
            mode = 20;
        else if (!strcmp(argv[1], "--send-data-transaction-selftest"))
            mode = 21;
        else if (!strcmp(argv[1], "--send-data-callback-selftest"))
            mode = 22;
        else if (!strcmp(argv[1], "--send-data-wait-selftest"))
            mode = 23;
        else if (!strcmp(argv[1], "--send-data-full-selftest"))
            mode = 24;
        else if (!strcmp(argv[1], "--send-data-real-armed"))
            mode = 25;
        else if (!strcmp(argv[1], "--send-data-real")) {
            char *endp;

            if (argc < 3) {
                fprintf(stderr,
                        "ERROR: --send-data-real necesita NODE (1..232)\n");
                return 1;
            }

            node_id = strtoul(argv[2], &endp, 0);

            if (*endp != '\0' ||
                node_id < 1 ||
                node_id > 232) {
                fprintf(stderr,
                        "ERROR: NODE invalido: %s\n",
                        argv[2]);
                return 1;
            }

            mode = 26;
        }
        else if (!strcmp(argv[1], "--node-info")) {
            char *endp;

            if (argc < 3) {
                fprintf(stderr,
                        "ERROR: --node-info necesita NODE (1..232)\n");
                return 1;
            }

            node_id = strtoul(argv[2], &endp, 0);

            if (*endp != '\0' ||
                node_id < 1 ||
                node_id > 232) {
                fprintf(stderr,
                        "ERROR: Node ID invalido: %s (1..232)\n",
                        argv[2]);
                return 1;
            }

            mode = 8;
        }
        else {
            usage(argv[0]);
            return 1;
        }
    }

    /*
     * Modos con argumento NODE:
     *
     *   argv[2] = NODE
     *   argv[3] = dispositivo opcional
     *
     * mode 8  = ZW_GET_NODE_PROTOCOL_INFO
     * mode 26 = ZW_SEND_DATA_REAL
     *
     * Para el resto:
     *
     *   argv[2] = dispositivo opcional
     */
    if (mode == 8 || mode == 26) {
        if (argc >= 4)
            dev = argv[3];
    } else {
        if (argc >= 3)
            dev = argv[2];
    }

    printf("========================================\n");
    printf(" DCH-G020 Z-Wave probe v2\n");
    printf(" device: %s\n", dev);

    printf(" mode: %s\n",
           mode == 1 ? "GET_VERSION" :
           mode == 2 ? "MEMORY_GET_ID" :
           mode == 3 ? "SERIAL_API_GET_INIT_DATA" :
           mode == 4 ? "ALL_SAFE" :
           mode == 5 ? "ZW_GET_CONTROLLER_CAPABILITIES" :
           mode == 6 ? "SERIAL_API_GET_CAPABILITIES" :
           mode == 7 ? "CAPABILITIES_SAFE" :
           mode == 8 ? "ZW_GET_NODE_PROTOCOL_INFO" :
           mode == 9 ? "Z-WAVE_NETWORK_INVENTORY" :
           mode == 10 ? "ADD_NODE_DRY_RUN" :
           mode == 11 ? "ADD_NODE_CALLBACK_SELFTEST" :
           mode == 12 ? "ADD_NODE_STATE_SELFTEST" :
           mode == 13 ? "ADD_NODE_PIPELINE_SELFTEST" :
           mode == 14 ? "ADD_NODE_RX_PATH_SELFTEST" :
           mode == 15 ? "ADD_NODE_CALLBACK_LOOP_SELFTEST" :
           mode == 16 ? "ADD_NODE_TRANSACTION_SELFTEST" :
           mode == 17 ? "ADD_NODE_FAILURE_SELFTEST" :
           mode == 18 ? "ADD_NODE_REAL" :
           mode == 19 ? "PASSIVE_SERIAL_API_LISTENER" :
           mode == 20 ? "ZW_SEND_DATA_OFFLINE_SELFTEST" :
           mode == 21 ? "ZW_SEND_DATA_TRANSACTION_SELFTEST" :
           mode == 22 ? "ZW_SEND_DATA_CALLBACK_SELFTEST" :
           mode == 23 ? "ZW_SEND_DATA_WAIT_SELFTEST" :
           mode == 24 ? "ZW_SEND_DATA_FULL_SELFTEST" :
           mode == 25 ? "ZW_SEND_DATA_REAL_ARMED_DRY_RUN" :
           mode == 26 ? "ZW_SEND_DATA_REAL" :
                        mode == 27 ? "OEM_CMDQ_MODEL_SELFTEST" :
                        mode == 28 ? "OEM_WAKEUP_DECISION_SELFTEST" :
                        mode == 29 ? "OEM_WAKEUP_PIPELINE_SELFTEST" :
                        "PREPARE_ONLY");

    printf("========================================\n");

    /*
     * V7.12 OEM CMDQ MODEL SELFTEST.
     *
     * Debe ejecutarse antes de abrir ttyACM0.
     */
    if (mode == 27) {
        rc = run_oem_cmdq_model_selftest();

        printf("\n[+] resultado: %s\n",
               rc == 0 ? "OK" : "ERROR");

        return rc == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
    }



    /*
     * V7.12 STAGE 3 OEM WAKE-UP PIPELINE SELFTEST.
     *
     * OFFLINE y deliberadamente antes de setup_serial().
     */
    if (mode == 29) {
        rc = run_oem_wakeup_pipeline_selftest();

        printf("\n[+] resultado: %s\n",
               rc == 0 ? "OK" : "ERROR");

        return rc == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
    }

    /*
     * V7.12 STAGE 2B OEM WAKE-UP DECISION SELFTEST.
     *
     * Deliberadamente antes de setup_serial().
     */
    if (mode == 28) {
        rc = run_oem_wakeup_decision_selftest();

        printf("\n[+] resultado: %s\n",
               rc == 0 ? "OK" : "ERROR");

        return rc == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
    }

    /*
     * V7.1 ZW_SEND_DATA SELFTEST.
     *
     * Deliberadamente antes de setup_serial():
     * garantiza que el test NO abre el puerto real.
     */
    if (mode == 20) {
        rc = run_zw_send_data_selftest();

        printf("\n[+] resultado: %s\n",
               rc ? "ERROR" : "OK");

        return rc;
    }

    /*
     * V7.2 ZW_SEND_DATA TRANSACTION SELFTEST.
     *
     * Deliberadamente antes de setup_serial():
     * no abre ttyACM0 y no transmite nada.
     */
    if (mode == 21) {
        rc = run_zw_send_data_transaction_selftest();

        printf("\n[+] resultado: %s\n",
               rc ? "ERROR" : "OK");

        return rc;
    }

    /*
     * V7.5 FULL ZW_SEND_DATA TRANSACTION SELFTEST.
     *
     * Deliberadamente antes de setup_serial():
     * usa socketpair(), no ttyACM0.
     *
     * Simula el flujo completo:
     * REQUEST -> ACK -> RESPONSE -> CALLBACK.
     */
    if (mode == 24) {
        rc = run_zw_send_data_full_transaction_selftest();

        printf("\n[+] resultado: %s\n",
               rc ? "ERROR" : "OK");

        return rc;
    }

    /*
     * V7.4 ZW_SEND_DATA WAIT-PATH SELFTEST.
     *
     * Deliberadamente antes de setup_serial():
     * usa socketpair(), no ttyACM0.
     */
    if (mode == 23) {
        rc = run_zw_send_data_wait_selftest();

        printf("\n[+] resultado: %s\n",
               rc ? "ERROR" : "OK");

        return rc;
    }

    /*
     * V7.3 ZW_SEND_DATA CALLBACK SELFTEST.
     *
     * Deliberadamente antes de setup_serial():
     * no abre ttyACM0 y no transmite Z-Wave.
     */
    if (mode == 22) {
        rc = run_zw_send_data_callback_selftest();

        printf("\n[+] resultado: %s\n",
               rc ? "ERROR" : "OK");

        return rc;
    }

    /*
     * DRY RUN deliberadamente antes de setup_serial().
     *
     * De esta manera podemos garantizar que esta prueba
     * ni siquiera abre /dev/ttyACM0.
     */
    if (mode == 10) {
        rc = run_add_node_dry_run();

        printf("\n[+] resultado: %s\n",
               rc ? "ERROR" : "OK");

        return rc;
    }

    if (mode == 11) {
        rc = run_add_node_callback_selftest();

        printf("\n[+] resultado: %s\n",
               rc ? "ERROR" : "OK");

        return rc;
    }

    if (mode == 12) {
        rc = run_add_node_state_selftest();

        printf("\n[+] resultado: %s\n",
               rc ? "ERROR" : "OK");

        return rc;
    }

    if (mode == 13) {
        rc = run_add_node_pipeline_selftest();

        printf("\n[+] resultado: %s\n",
               rc ? "ERROR" : "OK");

        return rc;
    }

    if (mode == 14) {
        rc = run_add_node_rx_path_selftest();

        printf("\n[+] resultado: %s\n",
               rc ? "ERROR" : "OK");

        return rc;
    }

    if (mode == 15) {
        rc = run_add_node_callback_loop_selftest();

        printf("\n[+] resultado: %s\n",
               rc ? "ERROR" : "OK");

        return rc;
    }

    if (mode == 16) {
        rc = run_add_node_transaction_selftest();

        printf("\n[+] resultado: %s\n",
               rc ? "ERROR" : "OK");

        return rc;
    }

    if (mode == 17) {
        rc = run_add_node_failure_selftest();

        printf("\n[+] resultado: %s\n",
               rc ? "ERROR" : "OK");

        return rc;
    }


    /*
     * V7.7 REAL SEND_DATA ARMED DRY-RUN.
     *
     * Deliberadamente ANTES de setup_serial().
     * Comprueba exactamente lo que transmitiríamos,
     * pero no abre ttyACM0 ni llama a la transaction.
     */
    if (mode == 25) {
        rc = run_zw_send_data_real_armed_dry_run();

        printf("\n[+] resultado: %s\n",
               rc ? "ERROR" : "OK");

        return rc;
    }

    fd = setup_serial(dev);

    if (fd < 0)
        return 1;

    /*
     * ========================================================
     * V7.8 STAGE 1 - REAL ZW_SEND_DATA PATH
     * ========================================================
     *
     * El modo REAL ya ha atravesado:
     *
     *   parser CLI
     *       ->
     *   validacion NODE
     *       ->
     *   setup_serial()
     *
     * PERO EN STAGE 1 LA TRANSMISION SIGUE BLOQUEADA.
     *
     * Todavia NO llamamos zw_send_data_transaction().
     */
    if (mode == 26) {
        printf("\n");
        printf("========================================\n");
        printf(" V7.8 REAL ZW_SEND_DATA — STAGE 1\n");
        printf("========================================\n");

        printf("[+] ttyACM0 abierto/configurado\n");
        printf("[+] Node ID validado         : %lu (0x%02lX)\n",
               node_id,
               node_id);

        printf("[+] Command previsto         : 84 05\n");
        printf("[+] TX options previstas     : 0x25\n");
        printf("[+] Callback ID previsto     : 0x01\n");

        /*
         * V7.9 - PRIMER ZW_SEND_DATA REAL
         *
         * Parametros previamente validados:
         *
         *   NODE        = node_id
         *   COMMAND     = 84 05
         *   TX OPTIONS  = 0x25
         *   CALLBACK ID = 0x01
         *
         * zw_send_data_transaction() realiza la cadena completa:
         *
         *   REQUEST -> ACK -> RESPONSE -> CALLBACK
         *
         * y solo devuelve 0 si termina con
         * TRANSMIT_COMPLETE_OK.
         */
        {
            static const uint8_t command[] = {
                0x84, 0x05
            };

            printf("\n");
            printf("========================================\n");
            printf(" V7.9 — PRIMER ZW_SEND_DATA REAL\n");
            printf("========================================\n");

            printf("[!] A PARTIR DE AQUI HAY TRANSMISION REAL\n");
            printf("[+] Node ID                : %lu (0x%02lX)\n",
                   node_id,
                   node_id);
            printf("[+] Command                : 84 05\n");
            printf("[+] TX options             : 0x25\n");
            printf("[+] Callback ID            : 0x01\n");

            rc = zw_send_data_transaction(fd,
                                          (uint8_t)node_id,
                                          command,
                                          sizeof(command),
                                          0x25,
                                          0x01);

            close(fd);

            printf("\n[+] puerto cerrado\n");

            if (rc != 0) {
                printf("[-] V7.9 ZW_SEND_DATA REAL fallo: rc=%d\n",
                       rc);
                printf("[+] resultado: ERROR\n");
                return 1;
            }

            printf("[+] V7.9 ZW_SEND_DATA REAL completado\n");
            printf("[+] resultado: OK\n");

            return 0;
        }
    }

    /*
     * V7.0 listener pasivo.
     * El puerto ya esta abierto/configurado.
     */
    if (mode == 19) {
        rc = run_passive_listener(fd);

        close(fd);

        printf("\n[+] puerto cerrado\n");
        printf("[+] resultado: %s\n",
               rc ? "ERROR" : "OK");

        return rc;
    }

    if (mode == 18) {
        rc = run_add_node_real(fd);

    } else if (mode == 0) {
        printf("[+] PREPARE ONLY: no se ha enviado ningun byte Z-Wave.\n");

    } else if (mode == 1) {
        rc = run_query(fd, 0x15, "GET_VERSION");

    } else if (mode == 2) {
        rc = run_query(fd, 0x20, "MEMORY_GET_ID");

    } else if (mode == 3) {
        rc = run_query(fd, 0x02, "SERIAL_API_GET_INIT_DATA");

    } else if (mode == 4) {
        /*
         * Las tres operaciones ya probadas como consultas
         * de lectura. Una sola apertura del puerto permite
         * verificar que el framing permanece sincronizado.
         */
        if (run_query(fd, 0x15, "GET_VERSION"))
            rc = 1;

        if (!rc && run_query(fd, 0x20, "MEMORY_GET_ID"))
            rc = 1;

        if (!rc && run_query(fd, 0x02, "SERIAL_API_GET_INIT_DATA"))
            rc = 1;

    } else if (mode == 5) {
        rc = run_query(fd, 0x05,
                       "ZW_GET_CONTROLLER_CAPABILITIES");

    } else if (mode == 6) {
        rc = run_query(fd, 0x07,
                       "SERIAL_API_GET_CAPABILITIES");

    } else if (mode == 7) {
        /*
         * Solo consultas de capacidades.
         * No inclusión, exclusión, reset ni escritura NVM.
         */
        if (run_query(fd, 0x05,
                      "ZW_GET_CONTROLLER_CAPABILITIES"))
            rc = 1;

        if (!rc &&
            run_query(fd, 0x07,
                      "SERIAL_API_GET_CAPABILITIES"))
            rc = 1;

    } else if (mode == 8) {
        rc = run_node_info(fd, (uint8_t)node_id);

    } else if (mode == 9) {
        rc = run_inventory(fd);
    }

    close(fd);

    printf("\n[+] puerto cerrado\n");
    printf("[+] resultado: %s\n", rc ? "ERROR" : "OK");

    return rc;
}
