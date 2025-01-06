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
constexpr int reporting_interval_ms = 1000;  //ms

static const std::string b = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";//=

/**
 * @brief Encodes a string to base64.
 * 
 * @param in The input string to encode.
 * @return The base64 encoded string.
 */
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

/**
 * @brief Creates an NtripClient object with the provided connection details.
 * 
 * @param host The NTRIP server host address.
 * @param port The NTRIP server port.
 * @param mountpoint The NTRIP server mountpoint.
 * @param username The NTRIP server username.
 * @param password The NTRIP server password.
 */
NtripClient::NtripClient(const std::string& host, const std::string& port, const std::string& mountpoint, const std::string& username, const std::string& password) :
    host_(host),
    port_(port),
    mountpoint_(mountpoint),
    username_(username),
    password_(password) {
    initialized_ = true;
}

/**
 * @brief Destroys the NtripClient object, stopping the client if it is still running.
 */
NtripClient::~NtripClient() {
    if (IsRunning()) {
        Stop();
    }
}

/**
 * @brief Initializes the NtripClient with the provided connection details.
 * 
 * @param host The NTRIP server host address.
 * @param port The NTRIP server port.
 * @param mountpoint The NTRIP server mountpoint.
 * @param username The NTRIP server username.
 * @param password The NTRIP server password.
 * @return true if the client is successfully initialized, false otherwise.
 */
bool NtripClient::Init(const std::string& host, const std::string& port, const std::string& mountpoint, const std::string& username, const std::string& password) {
    host_ = host;
    port_ = port;
    mountpoint_ = mountpoint;
    username_ = username;
    password_ = password;
    initialized_ = true;
    return true;
}

/**
 * @brief Runs the NtripClient, establishing a connection to the NTRIP server.
 * 
 * This function performs the following steps:
 * - Stops the client if it is already running.
 * - Sets up the network structure for the socket connection.
 * - Creates a socket and connects to the server.
 * - Sets the socket to non-blocking mode.
 * - Authenticates the NTRIP connection using the provided credentials.
 * - Sends GGA data if available.
 * - Configures TCP socket keepalive options if enabled.
 * - Starts the client thread to handle incoming data.
 * 
 * @return true if the client successfully connects and authenticates with the server, false otherwise.
 */
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

    std::cout << inet_addr(host_.c_str()) << std::endl;

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

    connected_ = true;

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
                authenticated_ = true;
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
                    break;
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

/**
 * @brief Stops the NtripClient, closing the socket and joining the thread.
 */
void NtripClient::Stop() {
    if (running_) {
        running_ = false;
        thread_.join();
        close(sockfd_);
    }
}

/**
 * @brief Checks if the NtripClient is currently running.
 * 
 * @return true if the client is running, false otherwise.
 */
bool NtripClient::IsRunning() {
    return running_;
}

/**
 * @brief Updates the GGA data buffer with the provided GGA message.
 * 
 * @param gga The GGA message to update the buffer with.
 */
void NtripClient::UpdateGGA(std::string gga) {
    gga_buffer_ = gga;
}

/**
 * @brief Cleans up the NtripClient, closing the socket if it is still open.
 */
void NtripClient::Cleanup() {
    if (sockfd_ > 0) {
        close(sockfd_);
        sockfd_ = -1;
    }
}

/**
 * @brief The main thread handler for the NtripClient.
 * 
 * This function is responsible for handling the main body of the NtripClient service.
 * It receives data from the NTRIP server and sends GGA data at regular intervals.
 * 
 * @return true if the thread handler completes successfully, false otherwise.
 */
bool NtripClient::ThreadHandler() {
    //confirm the module has been initialized
    if (!initialized_) {
        std::cerr << "Error: NtripClient not initialized" << std::endl;
        Cleanup();
        return false;
    }

    //if initialized, confirm the connection has been made
    if (!connected_) {
        std::cerr << "Error: NtripClient not connected" << std::endl;
        Cleanup();
        return false;
    }

    //if connected, confirm the client has been authenticated
    if (!authenticated_) {
        std::cerr << "Error: NtripClient not authenticated" << std::endl;
        Cleanup();
        return false;
    }

    //confirm the socket has not been broken or lost since the thread was created
    if (sockfd_ < 0) {
        std::cerr << "Error: Socket not created" << std::endl;
        Cleanup();
        return false;
    }

    //if all checks are ok, we can go ahead with the main body
    std::unique_ptr<char[]> buffer_(new char[buffer_size], std::default_delete<char[]>());
    int ret = 0;
    auto start_time = std::chrono::steady_clock::now();
    auto end_time = start_time;
    std::cout << "NtripClient service running..." << std::endl;
    while (running_) {
        ret = recv(sockfd_, buffer_.get(), buffer_size, 0);
        if (ret == 0) {
            std::cerr << "Remote socket closed" << std::endl;
            Cleanup();
            return false;
        } else if (ret < 0) {
            if ((errno != 0) && (errno != EAGAIN) && (errno != EWOULDBLOCK) && (errno != EINTR)) {
                std::cerr << "Remote socket error, errno=" << errno << std::endl;
                Cleanup();
                return false;
            }
        } else {
            //do something with the data
            //alternative methods can be created here to move it to a queue or whatever
            std::cout << "Data received: ";
            for (int i = 0; i < ret; i++) {
                std::cout << std::hex << (int)static_cast<uint8_t>(buffer_[i]);
            }
            std::cout << std::endl;
        }
        end_time = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count() >= reporting_interval_ms) {
            start_time = std::chrono::steady_clock::now();
            //send some data
            if (!gga_buffer_.empty()) {
                ret = send(sockfd_, gga_buffer_.c_str(), gga_buffer_.size(), 0);
                if (ret < 0) {
                    std::cerr << "Error: Could not send GGA data to server" << std::endl;
                    Cleanup();
                    return false;
                }
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    Cleanup();
    std::cout << "NtripClient service done." << std::endl;
    return true;
}
