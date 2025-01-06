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
    NtripClient() = default;
    NtripClient(const std::string& host, const std::string& port, const std::string& mountpoint, const std::string& username, const std::string& password);
    ~NtripClient();
    bool Init(const std::string& host, const std::string& port, const std::string& mountpoint, const std::string& username, const std::string& password);
    bool Run();
    void Stop();
    bool IsRunning();
    void UpdateGGA(std::string gga);

private:
    bool ThreadHandler();
    void Cleanup();
    std::string host_;
    std::string port_;
    std::string mountpoint_;
    std::string username_;
    std::string password_;
    std::string gga_buffer_;

    std::thread thread_;
    int sockfd_ = -1;

    bool initialized_ = false;
    bool connected_ = false;
    bool authenticated_ = false;
    bool running_ = false;
};
