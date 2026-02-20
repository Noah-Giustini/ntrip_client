/*
https://github.com/Noah-Giustini/ntrip_client/

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
#include <netdb.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>
#include <iostream>
#include <thread>
#include <chrono>
#include <cstring>
#include <string>
#include <list>
#include <iostream>
#include <sstream>

#include "ntrip_client.h"
#include "nmea.h"

constexpr int buffer_size = 4096;
constexpr int socket_timeout = 50;  //100 * ms
constexpr int reporting_interval_ms = 1000;  //ms
constexpr uint16_t max_connection_attempts = 10;
constexpr uint16_t max_authentication_attempts = 6;
static const std::string b = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";//=

static std::string binary_to_string(const std::vector<unsigned char> &source) {
    static char syms[] = "0123456789abcdef";
    std::stringstream ss;
    for (auto it = source.begin(); it != source.end(); it++)
      ss << syms[((*it >> 4) & 0xf)] << syms[*it & 0xf];
  
    return ss.str();
  }

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

    m_MainNtripClientThread = std::thread(&NtripClient::ThreadHandler, this);
    std::cout << "[NTRIP Client] ThreadHandler started." << std::endl;
    running_ = true;
    return true;
  
}

void NtripClient::ProcessGngaaInput(const std::string &InputStr) {
    const std::lock_guard<std::mutex> lock(m_Serial2NtripClientMutex);
    m_Serial2NtripClientMessages.push(InputStr);
    // std::cout << "Received InputStr ProcessGngaaInput: " << InputStr << std::endl;
  
    auto queue_size = m_Serial2NtripClientMessages.size();
    if (queue_size > m_MaxQueueSize) {
      m_MaxQueueSize = queue_size;
    }
  }



/**
 * @brief Stops the NtripClient, closing the socket and joining the thread.
 */
void NtripClient::Stop() {
    stop_called_ = true;
    if (running_) {
        running_ = false;
        m_MainNtripClientThread.join();
        Cleanup();
    }
    std::cout << "[NTRIP Client] NtripClient stopped successfully" << std::endl;
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
 * @brief Cleans up the NtripClient, closing the socket if it is still open.
 */
void NtripClient::Cleanup() {
    if (sockfd_ > 0) {
        close(sockfd_);
        sockfd_ = -1;
        std::cout << "[NTRIP Client] Cleaned up socket" << std::endl;
    }
    connected_ = false;
    authenticated_ = false;
    std::cout << "[NTRIP Client] Reset connection states" << std::endl;
    // Clear GGA message queue for a fresh start
    {
        const std::lock_guard<std::mutex> lock(m_Serial2NtripClientMutex);
        m_Serial2NtripClientMessages = {};
        std::cout << "[NTRIP Client] Cleared GGA queue" << std::endl;
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
    //connect to the server and authenticate in this thread to avoid blocking the main thread
    if (!ConnectToServer()){
        //failed to connect the socket, complete failure
        std::cout << "[NTRIP Client] Failed to connect a socket to the server. Failing Ntrip" << std::endl;
        return false;
    }

    if (!AuthenticateConnection()){
        //completely failed to authenticate, fail
        std::cout << "[NTRIP Client] Failed to authenticate with the server. Failing Ntrip" << std::endl;
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

    //confirm the module has been initialized
    if (!initialized_) {
        std::cerr << "[NTRIP Client] Error: NtripClient not initialized" << std::endl;
        Cleanup();
        return false;
    }

    //if initialized, confirm the connection has been made
    if (!connected_) {
        std::cerr << "[NTRIP Client] Error: NtripClient not connected" << std::endl;
        Cleanup();
        return false;
    }

    //if connected, confirm the client has been authenticated
    if (!authenticated_) {
        std::cerr << "[NTRIP Client] Error: NtripClient not authenticated" << std::endl;
        Cleanup();
        return false;
    }

    //confirm the socket has not been broken or lost since the thread was created
    if (sockfd_ < 0) {
        std::cerr << "[NTRIP Client] Error: Socket not created" << std::endl;
        Cleanup();
        return false;
    }

    //if all checks are ok, we can go ahead with the main body
    std::unique_ptr<char[]> buffer_(new char[buffer_size], std::default_delete<char[]>());
    int ret = 0;
    auto start_time = std::chrono::steady_clock::now();
    auto end_time = start_time;
    bool reconnect_needed = false;
    // int bytes_sent_to_receiver = 0;

    std::cout << "[NTRIP Client] NtripClient service running..." << std::endl;

    //send initial message to server. This cannot exist in the Run() function since it will block the main thread
    if (!SendInitialMessage()){
        //cannot proceed without this. fail
        std::cout << "[NTRIP Client] Failed to send an initial message to the server. Failing Ntrip" << std::endl;
        return false;
    }

    // Robust connection monitoring variables
    auto last_recv_time = std::chrono::steady_clock::now();
    constexpr int max_inactive_ms = 5000; // 5 seconds without data triggers reconnection
    while (running_) {
        if (reconnect_needed){
            Cleanup();
            if (!AttemptReconnect()){
                std::cerr << "[NTRIP Client] Attempted to reconnect but was unsuccessful" << std::endl;
                Cleanup();
                reconnect_needed = true;
            } else {
                std::cout << "[NTRIP Client] Successfully reconnected to the server" << std::endl;
                reconnect_needed = false;
                last_recv_time = std::chrono::steady_clock::now();
            }
        }
        ret = recv(sockfd_, buffer_.get(), buffer_size, 0);
        if (ret == 0) {
            std::cerr << "[NTRIP Client] Remote socket closed (recv=0)" << std::endl;
            reconnect_needed = true;
        } else if (ret < 0) {
            if ((errno != 0) && (errno != EAGAIN) && (errno != EWOULDBLOCK) && (errno != EINTR)) {
                std::cerr << "[NTRIP Client] Remote socket error, errno=" << errno << std::endl;
                reconnect_needed = true;
            }
        } else {
            std::vector<unsigned char> rtcm_msg(buffer_.get(), buffer_.get() + ret);
            if (rtcm_output_handler_ && !rtcm_msg.empty()) {
                rtcm_output_handler_(rtcm_msg);
            }
            last_recv_time = std::chrono::steady_clock::now();
        }

        // Detect inactivity (no data received for max_inactive_ms)
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_recv_time).count() > max_inactive_ms) {
            std::cerr << "[NTRIP Client] No data received for " << max_inactive_ms << " ms. Triggering reconnection." << std::endl;
            reconnect_needed = true;
        }

        //check the time difference since the last message was sent to the server
        end_time = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count() >= reporting_interval_ms) {
            start_time = std::chrono::steady_clock::now();
            std::string gga_msg;
            bool is_queue_empty;
            {
                const std::lock_guard<std::mutex> lock(m_Serial2NtripClientMutex);
                is_queue_empty = m_Serial2NtripClientMessages.empty();
            }
            if (!is_queue_empty) {
                {
                    const std::lock_guard<std::mutex> lock(m_Serial2NtripClientMutex);
                    gga_msg = std::move(m_Serial2NtripClientMessages.back());
                    m_Serial2NtripClientMessages = {};
                }
                ssize_t ret = send(sockfd_, gga_msg.data(), gga_msg.size(), 0);
                if (ret < 0) {
                    std::cerr << "[NTRIP Client] Error: Could not send data to server" << std::endl;
                    reconnect_needed = true;
                }
            } else {
                if (!running_){
                    break;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    Cleanup();
    std::cout << "[NTRIP Client] NtripClient service done." << std::endl;
    return true;
}

/**
 * @brief Sets up the socket connection to the server.
 * 
 * @return true on success, false on failure
 */
bool NtripClient::ConnectSocket() {
    //set up network struct for socket connection
    struct sockaddr_in serv_addr;
    memset(&serv_addr, '0', sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(std::stoi(port_));

    //using the host_ variable, resolve the ip address for the server
    addrinfo hints, *res;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    int status = getaddrinfo(host_.c_str(), port_.c_str(), &hints, &res);
    if (status != 0) {
        std::cerr << "[NTRIP Client] Error: Could not resolve host address" << std::endl;
        return false;
    }

    serv_addr.sin_addr.s_addr = inet_addr(inet_ntoa(((struct sockaddr_in *)(res->ai_addr))->sin_addr));

    //create socket
    sockfd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd_ < 0) {
        std::cerr << "[NTRIP Client] Error: Could not create socket" << std::endl;
        return false;
    }

    //connect to server
    if (connect(sockfd_, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
        std::cerr << "[NTRIP Client] Error: Could not connect to server" << std::endl;
        Cleanup();
        return false;
    }

    connected_ = true;

    //set socket to non-blocking
    int flags = fcntl(sockfd_, F_GETFL);
    fcntl(sockfd_, F_SETFL, flags | O_NONBLOCK);

    return true;
}

/**
 * @brief Attempts to create the socket connection to the server with an exponential backoff in case of failure.
 * 
 * @return true on success, false on failure
 */
bool NtripClient::ConnectToServer() {
    uint16_t connection_attempts = 0;
    while (connection_attempts < max_connection_attempts){
        bool ret = false;
        ret = ConnectSocket();
        if (ret){
            std::cout << "[NTRIP Client] connected socket to server, moving to authentication" << std::endl;
            return true;
        } else {
            connection_attempts++;

            if (connection_attempts >= max_connection_attempts){
                std::cout << "[NTRIP Client] Reached connection attempt limit. Failing Ntrip process" << std::endl;
                Cleanup();
                return false;
            }
            int delay = (1 << connection_attempts); // Exponential backoff
            std::cout << "[NTRIP Client] Waiting for " << delay << " milliseconds before next connection attempt\n";
            for (int i = 0; i < delay; i++){
                //method to allow this to stop gracefully if stop is called while in long exponential backoff
                if (stop_called_){
                    Cleanup();
                    return false;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            }
        }
    }

    if (connection_attempts >= max_connection_attempts){
        std::cout << "[NTRIP Client] Reached connection attempt limit. Failing Ntrip process" << std::endl;
        Cleanup();
        return false;
    }

    //this should be unreachable but is here just in case
    return true;
}

/**
 * @brief Attempts to authenticate with the server with an exponential backoff in case of failure. Call ConnectSocket() or ConnectToServer() before this
 * 
 * @return true on success, false on failure
 */
bool NtripClient::AuthenticateConnection() {
    //if initialized, confirm the connection has been made
    if (!connected_) {
        std::cerr << "[NTRIP Client] Error: NtripClient not connected before authentication" << std::endl;
        if (!ConnectToServer()){
            Cleanup();
            return false;
        } else {
            std::cout << "[NTRIP Client] connected to server before authentication" << std::endl;
        }
    }

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
    
    //should probably try to add some differrent error handling here in case something fixable goes wrong
    if (ret < 0) {
        std::cerr << "[NTRIP Client] Error: Could not send request to server" << std::endl;
        Cleanup();
        return false;
    }

    int timeout = 0;
    uint16_t auth_attempts = 0;
    std::unique_ptr<char[]> buffer_(new char[buffer_size], std::default_delete<char[]>());
    
    while (timeout < socket_timeout) {
        ret = recv(sockfd_, buffer_.get(), buffer_size, 0);
        // std::cout << "Received " << ret << " bytes" << std::endl;
        
        if (ret > 0) {
            // Convert the received data into a string for easier handling
            std::string result(buffer_.get(), ret);   
            if ((result.find("HTTP/1.1 200 OK") != std::string::npos) || 
                (result.find("ICY 200 OK") != std::string::npos)) {
                
                authenticated_ = true;
                std::cout << "[NTRIP Client] Authentication successful!" << std::endl;
                return true;

            } else {
                std::cerr << "[NTRIP Client] Error: Request result: " << result << std::endl;
                auth_attempts++;

                if (auth_attempts >= max_authentication_attempts){
                    std::cout << "[NTRIP Client] Reached authentication attempt limit. Failing Ntrip process" << std::endl;
                    Cleanup();
                    return false;
                }
                int delay = (1 << auth_attempts); // Exponential backoff
                std::cout << "[NTRIP Client] Waiting for " << delay << " milliseconds before next attempt\n";
                for (int i = 0; i < delay; i++){
                    //method to allow this to stop gracefully if stop is called while in long exponential backoff
                    if (stop_called_){
                        Cleanup();
                        return false;
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                }
                //reset the timeout and try again
                timeout = 0;
                ret = send(sockfd_, full_request.c_str(), full_request.length(), 0);
    
                //should probably try to add some differrent error handling here in case something fixable goes wrong
                if (ret < 0) {
                    std::cerr << "[NTRIP Client] Error: Could not send request to server" << std::endl;
                    Cleanup();
                    return false;
                }
            }

        } else if (ret == 0) {
            // Connection closed by the remote server
            std::cerr << "[NTRIP Client] Error: Remote socket closed" << std::endl;
            Cleanup();
            return false;
        } else {
            //could indicate something terribly bad going wrong. May be able to handle disconnection here
            if ((errno != 0) && (errno != EAGAIN) && (errno != EWOULDBLOCK) && (errno != EINTR)) {
                std::cerr << "[NTRIP Client] Remote socket error when trying to authenticate, errno=" << errno << std::endl;
                Cleanup();
                return false;
            }

            // Sleep to prevent busy-waiting
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            timeout++; 
        }
    }
    std::cout << "[NTRIP Client] Error: NtripCaster[" 
              << host_ << ":" << port_ << " " 
              << username_ << " " 
              << password_ << " " 
              << mountpoint_ 
              << "] authentication failed due to timeout" 
              << std::endl;
    Cleanup();
    return false;
}

/**
 * @brief Attempts to send an initial gga message to the server. Call AuthenticateConnection() before this
 * 
 * @return true on success, false on failure
 */
bool NtripClient::SendInitialMessage() {
    bool sent_message = false;
    while (!sent_message){
        std::string item;
        bool is_queue_empty;
        {
            const std::lock_guard<std::mutex> lock(m_Serial2NtripClientMutex);
            is_queue_empty = m_Serial2NtripClientMessages.empty();
        }

        if (!is_queue_empty) {
            {
                std::lock_guard<std::mutex> lock(m_Serial2NtripClientMutex);
                item = std::move(m_Serial2NtripClientMessages.back());
                m_Serial2NtripClientMessages = {};
            }

            std::cout << "[NTRIP Client] Sending: " << item << std::endl;
            ssize_t ret = send(sockfd_, item.data(), item.size(), 0);
            if (ret < 0) {
                std::cerr << "[NTRIP Client] Error: Could not send data to server" << std::endl;
                Cleanup();
                return false; // Or handle error appropriately, based on your larger context
            } else {
                std::cout << "[NTRIP Client] Sent initial GGA message to server successfully" << std::endl;
                sent_message = true;
                return true;
            }
        } else {
            //if process is trying to stop, return so we can exit the thread
            if (stop_called_){
                return false;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
    return true;
}

/**
 * @brief Function to attempt to fully re-connect the process to the server, remaking the socket and authenticating. Call Cleanup() before this
 * 
 * @return true on success, false on failure
 */
bool NtripClient::AttemptReconnect() {
    if (!ConnectSocket()){
        //failed to connect the socket, complete failure
        std::cout << "[NTRIP Client] Failed to re-connect a socket to the server. Failing Ntrip" << std::endl;
        return false;
    }

    if (!AuthenticateConnection()){
        //completely failed to authenticate, fail
        std::cout << "[NTRIP Client] Failed to re-authenticate with the server. Failing Ntrip" << std::endl;
        return false;
    }

    if (!SendInitialMessage()){
        //cannot proceed without this. fail
        std::cout << "[NTRIP Client] Failed to re-send an initial message to the server. Failing Ntrip" << std::endl;
        return false;
    }

    return true;
}
