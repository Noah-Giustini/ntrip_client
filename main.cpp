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

#include <iostream>
#include <fstream>
#include <vector>
#include <thread>
#include <atomic>
#include <memory>
#include <mutex>
#include <condition_variable>
#include <string>
#include <sstream>
#include <chrono>
#include <queue>
#include <map>
#include <csignal>
#include <filesystem>
#include <iomanip>
#include <ctime>

//linux specific headers
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

//class headers
#include "serial.h"
#include "ntrip_client.h"
#include "json.hpp"

using json = nlohmann::json;

struct SerialConfig {
    std::string port;
    int baudrate;
    bool is_gga_source;
};

struct NtripConfig {
    std::string host;
    std::string port;
    std::string mountpoint;
    std::string username;
    std::string password;
};

// Shared resources
std::atomic<bool> running{true};
std::mutex rtcm_mutex;
std::queue<std::vector<unsigned char>> rtcm_queue;
std::vector<std::unique_ptr<serial::Serial>> serial_ports;

//config related variables
std::vector<SerialConfig> serial_configs;
NtripConfig ntrip_cfg;
int gga_idx = -1;

//logging variables
std::string log_dir;
std::vector<std::ofstream> serial_log_files;
std::ofstream program_log_file;

// Device color codes (10 unique, then cycle)
const char* device_colors[10] = {
    "\033[34m", // Blue
    "\033[35m", // Magenta
    "\033[36m", // Cyan
    "\033[91m", // Bright Red
    "\033[92m", // Bright Green
    "\033[93m", // Bright Yellow
    "\033[94m", // Bright Blue
    "\033[95m", // Bright Magenta
    "\033[96m", // Bright Cyan
    "\033[90m"  // Bright Black (Gray)
};

void signal_handler(int signum) {
    if (signum == SIGINT) {
        running = false;
    }
}

// Helper to get datestamp string
std::string get_datestamp() {
    auto now = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm tm;
    localtime_r(&t, &tm);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y_%m_%d_%H_%M_%S");
    return oss.str();
}

// Helper to set up logging
void setup_logging(const std::vector<SerialConfig>& configs) {
    log_dir = "logs_" + get_datestamp();
    std::filesystem::create_directory(log_dir);
    // Open log files for each serial port
    serial_log_files.clear();
    for (const auto& cfg : configs) {
        std::string fname = log_dir + "/" + std::filesystem::path(cfg.port).filename().string() + ".txt";
        serial_log_files.emplace_back(fname, std::ios::out | std::ios::app);
    }
    // Open program log file (not used for std::cout/stderr redirection)
    program_log_file.open(log_dir + "/program.log", std::ios::out | std::ios::app);
}

bool load_config(const std::string& filename, std::vector<SerialConfig>& serials, NtripConfig& ntrip) {
    std::ifstream f(filename);
    if (!f) {
        std::cerr << "[main] Could not open config file: " << filename << std::endl;
        return false;
    }
    json j;
    f >> j;
    // Serial ports
    for (const auto& entry : j["serial_ports"]) {
        SerialConfig cfg;
        cfg.port = entry["port"];
        cfg.baudrate = entry["baudrate"];
        cfg.is_gga_source = entry.value("is_gga_source", false);
        serials.push_back(cfg);
    }
    // NTRIP config
    if (!j.contains("ntrip") ||
        !j["ntrip"].contains("host") ||
        !j["ntrip"].contains("port") ||
        !j["ntrip"].contains("mountpoint") ||
        !j["ntrip"].contains("username") ||
        !j["ntrip"].contains("password")) {
        std::cerr << "[main] NTRIP configuration missing or incomplete in grm.json. Exiting." << std::endl;
        return false;
    }
    ntrip.host = j["ntrip"]["host"];
    ntrip.port = j["ntrip"]["port"];
    ntrip.mountpoint = j["ntrip"]["mountpoint"];
    ntrip.username = j["ntrip"]["username"];
    ntrip.password = j["ntrip"]["password"];
    return true;
}


void gga_thread_func(NtripClient& ntrip) {
    std::vector<std::string> lines(serial_ports.size());
    char buf[512];
    while (running) {
        for (size_t idx = 0; idx < serial_ports.size(); ++idx) {
            int n = serial_ports[idx]->PortRecv(buf, sizeof(buf)-1, serial_configs[idx].baudrate);
            if (n > 0) {
                buf[n] = '\0';
                // Filter out non-printable/binary characters
                std::string filtered;
                for (int i = 0; i < n; ++i) {
                    unsigned char c = buf[i];
                    if ((c >= 32 && c <= 126) || c == '\r' || c == '\n' || c == '\t') {
                        filtered += c;
                    }
                }
                lines[idx] += filtered;
                // Log filtered data to file
                serial_log_files[idx] << filtered;
                serial_log_files[idx].flush();
                // Robust GGA extraction
                while (true) {
                    size_t gga_start = lines[idx].find("$GNGGA");
                    if (gga_start == std::string::npos)
                        gga_start = lines[idx].find("$GPGGA");
                    if (gga_start == std::string::npos)
                        break;
                    // Find end of sentence (newline)
                    size_t gga_end = lines[idx].find('\n', gga_start);
                    if (gga_end == std::string::npos)
                        break; // Wait for more data
                    std::string gga = lines[idx].substr(gga_start, gga_end - gga_start);
                    // Extract fix-type flag (field 6, 0-based index 5)
                    std::string fix_type_str = "Unknown";
                    int field_count = 0;
                    for (size_t i = 0; i < gga.size(); ++i) {
                        if (gga[i] == ',') {
                            ++field_count;
                            if (field_count == 6) {
                                size_t next_comma = gga.find(',', i + 1);
                                std::string fix_flag = gga.substr(i + 1, next_comma - (i + 1));
                                if (fix_flag == "1") fix_type_str = "Single-Point";
                                else if (fix_flag == "2") fix_type_str = "Differential";
                                else if (fix_flag == "4") fix_type_str = "RTK-Float";
                                else if (fix_flag == "5") fix_type_str = "RTK-Fix";
                                else fix_type_str = "Unknown";
                                break;
                            }
                        }
                    }
                    
                    std::string device_color = device_colors[idx % 10];
                    std::string fix_color;
                    if (fix_type_str == "Single-Point") fix_color = "\033[31m"; // Red
                    else if (fix_type_str == "Differential") fix_color = "\033[38;5;208m"; // Orange
                    else if (fix_type_str == "RTK-Float") fix_color = "\033[33m"; // Yellow
                    else if (fix_type_str == "RTK-Fix") fix_color = "\033[32m"; // Green
                    else fix_color = "\033[37m"; // White
                    std::cout << device_color << "[" << serial_configs[idx].port << "]" << "\033[0m "
                              << fix_color << "[Fix Type]: " << fix_type_str << "\033[0m " << gga << std::endl;
                    if (static_cast<int>(idx) == gga_idx) {
                        ntrip.ProcessGngaaInput(gga + "\r\n");
                    }
                    // Remove everything up to and including this GGA
                    lines[idx].erase(0, gga_end + 1);
                }
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void ntrip_to_serial_func() {
    while (running) {
        std::vector<unsigned char> rtcm_msg;
        {
            std::lock_guard<std::mutex> lock(rtcm_mutex);
            if (!rtcm_queue.empty()) {
                rtcm_msg = std::move(rtcm_queue.front());
                rtcm_queue.pop();
            }
        }
        if (!rtcm_msg.empty()) {
            for (size_t i = 0; i < serial_ports.size(); ++i) {
                serial_ports[i]->PortSend((void*)rtcm_msg.data(), rtcm_msg.size());
            }
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
}

int main(int argc, char* argv[]) {
    // Set up Ctrl+C handler
    std::signal(SIGINT, signal_handler);

    if (argc < 2) {
        std::cerr << "[main] Usage: " << argv[0] << " <path/to/grm.json>" << std::endl;
        return 1;
    }
    std::string config_path = argv[1];

    // Load config
    if (!load_config(config_path, serial_configs, ntrip_cfg)) {
        return 1;
    }
    setup_logging(serial_configs);
    if (serial_configs.empty()) {
        std::cerr << "[main] No serial ports configured. Exiting." << std::endl;
        return 1;
    }
    // Find GGA source port
    gga_idx = -1;
    for (size_t i = 0; i < serial_configs.size(); ++i) {
        if (serial_configs[i].is_gga_source) {
            gga_idx = static_cast<int>(i);
            break;
        }
    }
    if (gga_idx == -1) {
        std::cerr << "[main] No GGA source port configured. Exiting." << std::endl;
        return 1;
    }
    // Open all serial ports using Serial class
    for (const auto& cfg : serial_configs) {
        portinfo_t pinfo = {0, cfg.baudrate, '8', 0, 0, '0', '0', '1', 0, cfg.port};
        auto sp = std::make_unique<serial::Serial>(pinfo);
        if (sp->PortOpen() < 0) {
            std::cerr << "[main] Failed to open serial port: " << cfg.port << std::endl;
            return 1;
        }
        sp->PortSet();
        serial_ports.push_back(std::move(sp));
    }
    // NTRIP client config from grm.json
    NtripClient ntrip(ntrip_cfg.host, ntrip_cfg.port, ntrip_cfg.mountpoint, ntrip_cfg.username, ntrip_cfg.password);
    // Set handler to push RTCM/NTRIP output to queue
    ntrip.SetRtcmOutputHandler([&](const std::vector<unsigned char>& msg) {
        std::lock_guard<std::mutex> lock(rtcm_mutex);
        rtcm_queue.push(msg);
    });

    //start threads
    ntrip.Run();
    std::thread gga_thread(gga_thread_func, std::ref(ntrip));
    std::thread ntrip_to_serial_thread(ntrip_to_serial_func);

    //wait for exit signal SIGINT (Ctrl+C)
    std::cout << "[main] Press Ctrl+C to exit..." << std::endl;
    while (running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    //stop threads
    ntrip.Stop();
    gga_thread.join();
    ntrip_to_serial_thread.join();
    for (auto& sp : serial_ports) sp->PortClose();
    ntrip.Stop();
    return 0;
}
