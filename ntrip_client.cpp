/*
MIT License

Copyright (c) 2025 Noah Giustini

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

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <string>
#include <list>
#include <memory>
#include <iostream>

#include "ntrip_client.h"

constexpr int buffer_size = 4096;
constexpr int socket_timeout = 50;  //100 * ms

static const std::string b = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";//=
static std::string base64_encode(const std::string &in) {
    std::string out;

    int val=0, valb=-6;
    for (u_char c : in) {
        val = (val<<8) + c;
        valb += 8;
        while (valb>=0) {
            out.push_back(b[(val>>valb)&0x3F]);
            valb-=6;
        }
    }
    if (valb>-6) out.push_back(b[((val<<8)>>(valb+8))&0x3F]);
    while (out.size()%4) out.push_back('=');
    return out;
}

NtripClient::NtripClient(const std::string& host, const std::string& port, const std::string& mountpoint, const std::string& username, const std::string& password) :
    host_(host),
    port_(port),
    mountpoint_(mountpoint),
    username_(username),
    password_(password) {
    initialized_ = true;
}

NtripClient::~NtripClient() {
    if (IsRunning()) {
        Stop();
    }
}

bool NtripClient::Init(const std::string& host, const std::string& port, const std::string& mountpoint, const std::string& username, const std::string& password) {
    host_ = host;
    port_ = port;
    mountpoint_ = mountpoint;
    username_ = username;
    password_ = password;
    initialized_ = true;
    return true;
}

bool NtripClient::Run() {
    if (IsRunning()) {
        Stop();
    }

    //set up network struct for socket connection
    struct sockaddr_in serv_addr;
    memset(&serv_addr, '0', sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(std::stoi(port_));
    serv_addr.sin_addr.s_addr = inet_addr(host_.c_str());

    //create socket
    sockfd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd_ < 0) {
        std::cerr << "Error: Could not create socket" << std::endl;
        return false;
    }

    //connect to server
    if (connect(sockfd_, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
        std::cerr << "Error: Could not connect to server" << std::endl;
        close(sockfd_);
        return false;
    }

    //set socket to non-blocking
    int flags = fcntl(sockfd_, F_GETFL);
    fcntl(sockfd_, F_SETFL, flags | O_NONBLOCK);

    //authenticate ntrip connection
    int ret = -1;
    std::string user_pass = username_ + ":" + password_;
    std::string user_pass_b64 = base64_encode(user_pass);
    std::string request = "GET /" + mountpoint_ + " HTTP/1.1\r\n";
    std::string user_agent = "User-Agent: NTRIP Client/1.0\r\n";
    std::string authorization = "Authorization: Basic " + user_pass_b64 + "\r\n";
    std::string request_end = "\r\n";
    std::string full_request = request + user_agent + authorization + request_end;
    ret = send(sockfd_, full_request.c_str(), full_request.length(), 0);
    if (ret < 0) {
        std::cerr << "Error: Could not send request to server" << std::endl;
        close(sockfd_);
        return false;
    }

    int timeout = 0;
    std::unique_ptr<char[]> buffer_(new char[buffer_size], std::default_delete<char[]>());
    while (timeout < socket_timeout){
        ret = recv(sockfd_, buffer_.get(), buffer_size, 0);
        if (ret > 0) {
            std::string result(buffer_.get(), ret);
            if ((result.find("HTTP/1.1 200 OK") != std::string::npos) ||
                (result.find("ICY 200 OK") != std::string::npos)) {
                if (!gga_buffer_.empty()) {
                    ret = send(sockfd_, gga_buffer_.c_str(), gga_buffer_.size(), 0);
                    if (ret < 0) {
                        std::cerr << "Error: Could not send GGA data to server" << std::endl;
                        close(sockfd_);
                        return false;
                    }
                    //send was successful so break out of loop
                    break;
                } else {
                    //nothing in the buffer to send. go next i guess
                }
            } else {
                std::cerr << "Error: Request result: " << result << std::endl;
            }
                
        } else if (ret == 0) {
            std::cerr << "Error: Remote socket closed" << std::endl;
            close(sockfd_);
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (timeout >= socket_timeout) {
        std::cout << "Error: NtripCaster[" << host_ << ":" << port_ << " " << username_ << " " << password_ << " " << mountpoint_ << "] access failed" << std::endl;
        close(sockfd_);
        return false;
    }

    // TCP socket keepalive.
#if defined(ENABLE_TCP_KEEPALIVE)
    int keepalive = 1;  // Enable keepalive attributes.
    int keepidle = 30;  // Time out for starting detection.
    int keepinterval = 5;  // Time interval for sending packets during detection.
    int keepcount = 3;  // Max times for sending packets during detection.
    setsockopt(sockfd_, SOL_SOCKET, SO_KEEPALIVE, &keepalive, sizeof(keepalive));
    setsockopt(sockfd_, SOL_TCP, TCP_KEEPIDLE, &keepidle, sizeof(keepidle));
    setsockopt(sockfd_, SOL_TCP, TCP_KEEPINTVL, &keepinterval, sizeof(keepinterval));
    setsockopt(sockfd_, SOL_TCP, TCP_KEEPCNT, &keepcount, sizeof(keepcount));
#endif  // defined(ENABLE_TCP_KEEPALIVE)

    //all the setup is done, start the thread
    thread_ = std::thread(&NtripClient::ThreadHandler, this);
    running_ = true;
    return true;
  
}

void NtripClient::Stop() {
    if (running_) {
        running_ = false;
        thread_.join();
        close(sockfd_);
    }
}

bool NtripClient::IsRunning() {
    return running_;
}

void NtripClient::ThreadHandler() {
    //todo: add main running body here
    
}
