#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include<errno.h>

#define DEV_PATH      "/dev/integrated_driver"  
#define RESP_BUF_SIZE 1024

static int send_cmd(int fd, const char *cmd, char *resp, size_t resp_len, int expect_response)
{
    ssize_t n;
    // Send command
    n = write(fd, cmd, strlen(cmd));
    if (n < 0) 
    {
        perror("write");
        return -1;
    }

    if (!expect_response)
        return 0;
    // Try to reset offset, but ignore ESPIPE for char devices
    if (lseek(fd, 0, SEEK_SET) < 0) 
    {
        if (errno != ESPIPE) 
	{   // include <errno.h> at top
            perror("lseek");
            return -1;
        }
        // If errno == ESPIPE, just continue; we'll rely on driver resetting ppos
    }
    n = read(fd, resp, resp_len - 1);
    if (n < 0) {
        perror("read");
        return -1;
    }

    resp[n] = '\0'; // NULL terminate
    return 0;
}
static void trim_newline(char *s)
{
    size_t len = strlen(s);
    if (len > 0 && (s[len - 1] == '\n' || s[len - 1] == '\r'))
        s[len - 1] = '\0';
}

int main(void)
{
    int fd;
    int choice;
    char resp[RESP_BUF_SIZE];
    char input_buf[512];

    fd = open(DEV_PATH, O_RDWR);
    if (fd < 0) {
        perror("open");
        fprintf(stderr, "Failed to open %s\n", DEV_PATH);
        return 1;
    }

    while (1) {
        printf("\n=== Integrated Driver Test Menu ===\n");
        printf("1. Get Temperature over I2C (PRINT_TMP)\n");
        printf("2. Get Pressure over I2C (PRINT_PRE)\n");
        printf("3. Display Temp & Pressure on OLED over I2C (PRINT_DIS)\n");
        printf("4. Write to SD Card using SPI (WRITE_SD <block> <text>)\n");
        printf("5. Read from SD Card using SPI (READ_SD <block>)\n");
        printf("6. UART Loopback check  (UART_CHECK <text>)\n");
        printf("0. Exit\n");
        printf("Enter choice: ");
        fflush(stdout);

        if (scanf("%d", &choice) != 1) {
            fprintf(stderr, "Invalid input\n");
            // clear stdin
            int ch;
            while ((ch = getchar()) != '\n' && ch != EOF);
            continue;
        }

        // clear the leftover newline in stdin
        int ch;
        while ((ch = getchar()) != '\n' && ch != EOF);

        if (choice == 0)
            break;

        switch (choice) {
        case 1: {
            // PRINT_TMP
            if (send_cmd(fd, "PRINT_TMP", resp, sizeof(resp), 1) == 0) {
                printf("\033[0;34mKernel response:\033[0m\033[0;32m%s\033[0m\n", resp);
            }
            break;
        }
        case 2: {
            // PRINT_PRE
            if (send_cmd(fd, "PRINT_PRE", resp, sizeof(resp), 1) == 0) {
                printf("\033[0;34mKernel response:\033[0m\033[0;32m%s\033[0m\n", resp);
            }
            break;
        }
        case 3: {
            // PRINT_DIS (no response needed, just display on OLED)
            if (send_cmd(fd, "PRINT_DIS", resp, sizeof(resp), 0) == 0) {
                printf("\033[0;34mCommand sent. Check your OLED display.\033[0m\n");
            }
            break;
        }
        case 4: {
            // WRITE_SD <block> <text>
            int block;
            char text[256];

            printf("\033[0;34mEnter block number:\033[0m ");
            fflush(stdout);
            if (scanf("%d", &block) != 1) {
                fprintf(stderr, "Invalid block number\n");
                while ((ch = getchar()) != '\n' && ch != EOF);
                break;
            }
            while ((ch = getchar()) != '\n' && ch != EOF); // clear newline

            printf("\033[0;34mEnter text to write (max 255 chars):\033[0m ");
            fflush(stdout);
            if (!fgets(text, sizeof(text), stdin)) {
                fprintf(stderr, "Failed to read text\n");
                break;
            }
            trim_newline(text);

            snprintf(input_buf, sizeof(input_buf),
                     "WRITE_SD %d %s", block, text);

            if (send_cmd(fd, input_buf, resp, sizeof(resp), 0) == 0) {
                printf("\033[0;34mWRITE_SD command sent for block\033[0m\033[0;32m %d.\033[0m\n", block);
            }
            break;
        }
        case 5: {
            // READ_SD <block>
            int block;

            printf("\033[0;34mEnter block number to read:\033[0m ");
            fflush(stdout);
            if (scanf("%d", &block) != 1) {
                fprintf(stderr, "Invalid block number\n");
                while ((ch = getchar()) != '\n' && ch != EOF);
                break;
            }
            while ((ch = getchar()) != '\n' && ch != EOF); // clear newline

            snprintf(input_buf, sizeof(input_buf),
                     "READ_SD %d", block);

            if (send_cmd(fd, input_buf, resp, sizeof(resp), 1) == 0) {
                printf("\033[0;34mData from block\033[0m %d:\n\033[0;32m%s\033[0m\n", block, resp);
            }
            break;
        }
        case 6: {
            // UART_CHECK <text>
            char text[256];

            printf("\033[0;34mEnter text to send over UART:\033[0m ");
            fflush(stdout);
            if (!fgets(text, sizeof(text), stdin)) {
                fprintf(stderr, "Failed to read text\n");
                break;
            }
            trim_newline(text);

            snprintf(input_buf, sizeof(input_buf),
                     "UART_CHECK %s", text);

            if (send_cmd(fd, input_buf, resp, sizeof(resp), 1) == 0) {
                printf("\033[0;34mReceived from UART (via driver):\033[0m\033[0;32m%s\033[0m\n", resp);
            }
            break;
        }
        default:
            printf("Unknown choice\n");
            break;
        }
    }

    close(fd);
    return 0;
}
