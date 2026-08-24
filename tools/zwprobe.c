#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <time.h>

static void dump_hex(const char *tag, const unsigned char *b, size_t n)
{
    size_t i;
    printf("%s (%zu bytes):", tag, n);
    for (i = 0; i < n; i++)
        printf(" %02X", b[i]);
    printf("\n");
}

static int setup_serial(const char *dev)
{
    struct termios tio;
    int fd, modem = 0;

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

    if (cfsetispeed(&tio, B115200) < 0)
        perror("cfsetispeed");

    if (cfsetospeed(&tio, B115200) < 0)
        perror("cfsetospeed");

    tio.c_cc[VMIN]  = 0;
    tio.c_cc[VTIME] = 10;

    tcflush(fd, TCIOFLUSH);

    if (tcsetattr(fd, TCSANOW, &tio) < 0) {
        perror("tcsetattr");
        close(fd);
        return -1;
    }

    printf("[+] termios: 115200 8N1 raw, CLOCAL+CREAD\n");

    if (ioctl(fd, TIOCMGET, &modem) == 0) {
        printf("[+] modem bits antes: 0x%08X\n", modem);

        modem &= ~(TIOCM_DTR | TIOCM_RTS);

        if (ioctl(fd, TIOCMSET, &modem) < 0)
            perror("TIOCMSET");
        else
            printf("[+] DTR=0 RTS=0\n");

        if (ioctl(fd, TIOCMGET, &modem) == 0)
            printf("[+] modem bits despues: 0x%08X\n", modem);
    } else {
        perror("TIOCMGET");
        printf("[!] El driver no permite consultar modem bits; continuamos.\n");
    }

    return fd;
}

static int send_query(int fd,
                      const unsigned char *frame,
                      size_t frame_len,
                      const char *name)
{
    unsigned char buf[256];

    printf("[+] QUERY: %s\\n", name);
    size_t used = 0;
    int rounds;

    tcflush(fd, TCIFLUSH);

    dump_hex("TX", frame, frame_len);

    ssize_t w = write(fd, frame, frame_len);
    if (w < 0) {
        perror("write");
        return 1;
    }

    printf("[+] enviados %zd bytes\n", w);

    /*
     * Varias ventanas cortas para capturar:
     * ACK 06 + respuesta Serial API.
     */
    for (rounds = 0; rounds < 20 && used < sizeof(buf); rounds++) {
        fd_set rfds;
        struct timeval tv;

        FD_ZERO(&rfds);
        FD_SET(fd, &rfds);

        tv.tv_sec = 0;
        tv.tv_usec = 250000;

        int r = select(fd + 1, &rfds, NULL, NULL, &tv);

        if (r < 0) {
            if (errno == EINTR)
                continue;
            perror("select");
            break;
        }

        if (r == 0) {
            if (used)
                break;
            continue;
        }

        ssize_t n = read(fd, buf + used, sizeof(buf) - used);

        if (n > 0) {
            used += n;
            dump_hex("RX parcial", buf, used);
        } else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
            perror("read");
            break;
        }
    }

    if (!used) {
        printf("[-] SIN RESPUESTA\n");
        return 2;
    }

    dump_hex("RX FINAL", buf, used);

    return 0;
}

int main(int argc, char **argv)
{
    const char *dev = "/dev/ttyACM0";
    int mode = 0;

    if (argc >= 2) {
        if (!strcmp(argv[1], "--prepare-only"))
            mode = 0;
        else if (!strcmp(argv[1], "--get-version"))
            mode = 1;
        else if (!strcmp(argv[1], "--memory-get-id"))
            mode = 2;
        else if (!strcmp(argv[1], "--get-init-data"))
            mode = 3;
        else {
            fprintf(stderr,
                "Uso: %s [--prepare-only|--get-version|--memory-get-id|--get-init-data] [dispositivo]\n",
                argv[0]);
            return 1;
        }
    }

    if (argc >= 3)
        dev = argv[2];

    printf("========================================\n");
    printf(" DCH-G020 Z-Wave probe\n");
    printf(" device: %s\n", dev);

    printf(" mode: %s\n",
           mode == 1 ? "GET_VERSION" :
           mode == 2 ? "MEMORY_GET_ID" :
           mode == 3 ? "SERIAL_API_GET_INIT_DATA" :
                       "PREPARE ONLY");

    printf("========================================\n");

    int fd = setup_serial(dev);
    if (fd < 0)
        return 1;

    int rc = 0;

    if (mode == 1) {

        const unsigned char frame[] = {
            0x01, 0x03, 0x00, 0x15, 0xE9
        };

        rc = send_query(fd, frame, sizeof(frame),
                        "GET_VERSION");

    } else if (mode == 2) {

        const unsigned char frame[] = {
            0x01, 0x03, 0x00, 0x20, 0xDC
        };

        rc = send_query(fd, frame, sizeof(frame),
                        "MEMORY_GET_ID");

    } else if (mode == 3) {

        /*
         * SERIAL_API_GET_INIT_DATA
         * FUNC_ID = 0x02
         *
         * Sólo lectura.
         * Devuelve capacidades y máscara de nodos.
         */
        const unsigned char frame[] = {
            0x01, 0x03, 0x00, 0x02, 0xFE
        };

        rc = send_query(fd, frame, sizeof(frame),
                        "SERIAL_API_GET_INIT_DATA");

    } else {

        printf("[+] PREPARE ONLY: no se ha enviado ningun byte Z-Wave.\n");
    }

    close(fd);
    printf("[+] puerto cerrado\n");

    return rc;
}
