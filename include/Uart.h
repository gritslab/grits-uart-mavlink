/*
 * Uart.h
 *
 *  Created on: Oct 28, 2014
 *      Author: urs
 *      adapted from: https://github.com/starsnabove/beaglebone
 */

//uart.h

#ifndef __UART_H_INCLUDED__
#define __UART_H_INCLUDED__

#include <iostream>
#include <fstream>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <stdlib.h>
#include <string>

using namespace std;

#define MUX_DIR "/sys/kernel/debug/omap_mux"
#define UART1_TX_MUX "uart1_txd"
#define UART1_RX_MUX "uart1_rxd"
#define UART1_CT_MUX "uart1_ctsn"
#define UART1_RT_MUX "uart1_rtsn"

#define UART1_MODE 0
#define UART1_FILE "/dev/ttyO1"

class Uart {

    private:

        string tx_mux;
        string rx_mux;
        string ct_mux;
        string rt_mux;
        int tx_mode;
        int rx_mode;
        int ct_mode;
        int rt_mode;

        string file;
        int com_fd;
        int uart;

        struct termios tio;


    public:

        Uart(int);
        ~Uart(void);
        bool transmit(uint8_t * message, int length);
        bool receive(uint8_t * message, int length);

};

#endif
