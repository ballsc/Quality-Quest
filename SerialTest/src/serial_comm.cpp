// C library headers
#include <stdio.h>
#include <string.h>
#include <chrono>
#include <cstdlib>
#include <ctime>
#include <fstream>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

// Windows Headers
// #include <windows.h>

void setBits(struct termios &tty, int BAUD_RATE) {
    tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size 
    tty.c_cflag |= CS8; // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
  // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
  // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

    tty.c_cc[VTIME] = 10; // wait until timeout after 1 second
    tty.c_cc[VMIN] = 0;

    // set baud rate to slider
    cfsetispeed(&tty, BAUD_RATE);
    cfsetospeed(&tty, BAUD_RATE);
};

void writeBufferToFile(const char* filename, const char* buffer, int size) {
    FILE* file = fopen(filename, "w");
    if (!file) {
        printf("Error opening file '%s': %s\n", filename, strerror(errno));
        return;
    }

    fwrite(buffer, sizeof(char), size, file);
    fclose(file);
}

int test(){
    int port1 = open("/dev/ttyUSB0", O_RDWR);
    int port2 = open("/dev/ttyUSB1", O_RDWR);

    if (port1 < 0) {
        printf("Error while opening port1, errno = %s\n", strerror(errno));
        perror("Something went wrong with open()");
        exit(1);
    }

    if (port2 < 0) {
        printf("Error while opening port2, errno = %s\n", strerror(errno));
        perror("Something went wrong with open()");
        exit(1);
    }

    struct termios tty0; // sender
    struct termios tty1; // reciever

    // Read in existing settings, and handle any error
    if(tcgetattr(port1, &tty0) != 0) {
        printf("Error %i from port1 tcgetattr: %s\n", errno, strerror(errno));
        return 0;
    }

    // set necessary bits and baud rate
    setBits(tty0, 9600);

    // Save tty0 settings, also checking for error
    if (tcsetattr(port1, TCSANOW, &tty0) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
        return 0;
    }


    // Read in existing settings, and handle any error
    if(tcgetattr(port2, &tty1) != 0) {
        printf("Error %i from port2 tcgetattr: %s\n", errno, strerror(errno));
        return 0;
    }

    // set necessary bits and baud rate
    setBits(tty1, 9600);

    // Save tty1 settings, also checking for error
    if (tcsetattr(port2, TCSANOW, &tty1) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
        return 0;
    }


    /*
        BEGIN TEST
    */

    char readIn [256];
    unsigned char writeOut[] = { '1', '0', '1', '0', '0', '\r' };

    write(port1, writeOut, sizeof(writeOut));

    memset(&readIn, '\0', sizeof(readIn));

    int size = read(port2, &readIn, sizeof(readIn));

    if(size < 0){
        printf("Error reading COM4: %s", strerror(errno));
        return 0; // failed, couldnt read from COM4
    }

    printf("Successfully read %d bytes: '%s'\n", size, readIn);

    writeBufferToFile("out.txt", readIn, size);

    close(port1);
    close(port2);

    return 1;
}

int main() {
    test();
}
