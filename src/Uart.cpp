/*
 * Uart.cpp
 *
 *  Created on: Oct 28, 2014
 *      Author: urs
 */

//uart.cpp - uart interaction library

#include "include/Uart.h"

Uart::Uart(int uart_no) {

    uart = uart_no;

    switch (uart) {
        case 1:
            tx_mux = string(MUX_DIR) + string(UART1_TX_MUX);
            rx_mux = string(MUX_DIR) + string(UART1_RX_MUX);
            ct_mux = string(MUX_DIR) + string(UART1_CT_MUX);
            rt_mux = string(MUX_DIR) + string(UART1_RT_MUX);
            tx_mode = UART1_MODE;
            rx_mode = UART1_MODE;
            ct_mode = UART1_MODE;
            rt_mode = UART1_MODE;
            file = UART1_FILE;
  //default:
        //    cerr << "invalid uart no.: " << uart_no << '\n';
    }

    ofstream fd;

    fd.open(tx_mux.c_str());
    fd << tx_mode;
    fd.close();

    fd.open(rx_mux.c_str());
    fd << rx_mode;
    fd.close();

    fd.open(ct_mux.c_str());
    fd << ct_mode;
    fd.close();

    fd.open(rt_mux.c_str());
    fd << rt_mode;
    fd.close();

    memset(&tio, 0, sizeof(tio));
    tio.c_iflag = 0;
    tio.c_oflag = 0;
    tio.c_cflag = CS8|CREAD|CLOCAL;
    tio.c_lflag = 0;
    tio.c_cc[VMIN] = 1;
    tio.c_cc[VTIME] = 5;

    com_fd = open(file.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);

    if (com_fd < 0) {
        cerr << "Cannot open serial port\n";
        exit(1);
    }

    cfsetospeed(&tio,B115200);
    cfsetispeed(&tio,B115200);

    tcsetattr(com_fd,TCSANOW,&tio);

    fcntl(com_fd, F_SETFL, 0);

}

Uart::~Uart(void) {

    close(com_fd);

}

bool Uart::transmit(uint8_t * message, int length) {
    cout << "bd;iagjb\n";
    int wr = write(com_fd, message, length);
    cout << "adkmnvbdjkbgajki\n";

    if (wr < 0) {
        cerr << "Error transmitting to uart" << uart << '\n';
        return false;
    }
    return true;
}

bool Uart::receive(uint8_t* message, int length) {

    int rd = read(com_fd, message, length);

    if (rd < 0) {
        cerr << "Error receiving from uart" << uart << '\n';
        return false;
    }
    return true;
}



/*
int main() {
    uint8_t buf[5] = {'h', 'e', 'l', 'l', 'o'};
    Uart test = Uart(1);
    test.transmit(buf, 5);

}*/
