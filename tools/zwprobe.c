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
        "--add-node-failure-selftest|--add-node-real] [dispositivo]\n",
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

    if (mode == 8) {
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
                        "PREPARE_ONLY");

    printf("========================================\n");

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

    fd = setup_serial(dev);

    if (fd < 0)
        return 1;

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
