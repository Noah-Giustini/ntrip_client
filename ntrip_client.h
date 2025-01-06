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

#include <string>
#include <thread>

class NtripClient {
public:

    /**
     * @brief Default constructor for NtripClient.
     */
    NtripClient() = default;

    /**
     * @brief Constructor for NtripClient with connection details.
     * 
     * @param host The NTRIP server host address.
     * @param port The NTRIP server port.
     * @param mountpoint The NTRIP server mountpoint.
     * @param username The NTRIP server username.
     * @param password The NTRIP server password.
     */
    NtripClient(const std::string& host, const std::string& port, const std::string& mountpoint, const std::string& username, const std::string& password);

    /**
     * @brief Destructor for NtripClient.
     */
    ~NtripClient();

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
    bool Init(const std::string& host, const std::string& port, const std::string& mountpoint, const std::string& username, const std::string& password);

    /**
     * @brief Runs the NtripClient, establishing a connection to the NTRIP server.
     * 
     * This function performs the following steps:
     * - Authenticates the NTRIP connection using the provided credentials.
     * - Sends GGA data if available.
     * - Configures TCP socket keepalive options if enabled.
     * - Starts the client thread to handle incoming data.
     * 
     * @return true if the client successfully connects and authenticates with the server, false otherwise.
     */
    bool Run();

    /**
     * @brief Stops the NtripClient, closing the socket connection.
     */
    void Stop();

    /**
     * @brief Checks if the NtripClient is currently running.
     * 
     * @return true if the client is running, false otherwise.
     */
    bool IsRunning();

    /**
     * @brief Updates the GGA data buffer with the provided GGA message.
     * 
     * @param gga The GGA message to update the buffer with.
     */
    void UpdateGGA(std::string gga);

private:

    /**
     * @brief Cleans up the NtripClient, closing the socket if it is still open.
     */
    bool ThreadHandler();

    /**
     * @brief Cleans up the NtripClient, closing the socket if it is still open.
     */
    void Cleanup();

    //connection details
    std::string host_;
    std::string port_;
    std::string mountpoint_;
    std::string username_;
    std::string password_;
    int sockfd_ = -1;

    //buffer to hold the latest gga message
    std::string gga_buffer_;

    //thread to handle the main body of the client
    std::thread thread_;

    //flags to track the state of the client
    bool initialized_ = false;
    bool connected_ = false;
    bool authenticated_ = false;
    bool running_ = false;
};
