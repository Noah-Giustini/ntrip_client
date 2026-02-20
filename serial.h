/*
https://github.com/Noah-Giustini/ntrip_client/

MIT License

Copyright (c) 2026 Noah Giustini

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <string>

#ifndef SERIAL_H_
#define SERIAL_H_

#define TIMEOUT_SEC(buflen,baud) (buflen*20/baud+2)  
#define TIMEOUT_USEC 0 

typedef struct {
    char prompt;  //prompt after receiving data
    int  baudrate;  //baudrate
    char databit;  //data bits, 5, 6, 7, 8
    char  debug;  //debug mode, 0: none, 1: debug
    char  echo;   //echo mode, 0: none, 1: echo
    char fctl;   //flow control, 0: none, 1: hardware, 2: software
    char parity;  //parity 0: none, 1: odd, 2: even
    char stopbit;  //stop bits, 1, 2
    const int reserved; //reserved, must be zero
    std::string devpath; // full device path
} portinfo_t;

namespace serial {
    class Serial {
    public:
        Serial(const portinfo_t& portinfo);
        Serial(const std::string& devpath, int baudrate); // new constructor
        int PortOpen();
        int PortSet();
        void PortClose();
        int PortSend(void* data, int datalen);
        int PortRecv(void* data, int datalen, int baudrate);
    private:
        int convbaud(unsigned long int baudrate);
        void get_port();
        portinfo_t portinfo_;
        int fdcom_;
        std::string port_;
    };
}
#endif // SERIAL_H_