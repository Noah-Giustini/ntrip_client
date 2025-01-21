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

#include <csignal>
#include <iostream>
#include <cstdlib>
#include <math.h>
#include "ntrip_client.h"

bool run = true;

/**
 * @brief Signal handler for SIGINT.
 * 
 * @param signal The signal number.
 */
void signal_handler(int signal) {
    std::cout << "SIGINT received, shutting down..." << std::endl;
    run = false;
}

/**
 * @brief Converts a decimal degree to a DDMM format.
 * 
 * @param degree The decimal degree to convert.
 * @return The DDMM format of the decimal degree.
 */
static double degree_to_ddmm(double const& degree) {
  int deg = static_cast<int>(floor(degree));
  double minute = degree - deg*1.0;
  return (deg*1.0 + minute*60.0/100.0);
}

/**
 * @brief Generates a GGA message from the provided latitude, longitude, and altitude.
 * 
 * @param lat The latitude in decimal degrees.
 * @param lon The longitude in decimal degrees.
 * @param alt The altitude in meters.
 * @return The GGA message.
 */
static std::string generage_gga_message (double lat, double lon, double alt){
    char buffer[256];
    time_t now = time(0);
    struct tm tstruct;
    tstruct = *gmtime(&now);
    char utc_time[20];
    strftime(utc_time, sizeof(utc_time), "%H%M%S", &tstruct);

    double lat_ddmm = degree_to_ddmm(lat);
    double lon_ddmm = degree_to_ddmm(lon);

    snprintf(buffer, sizeof(buffer),
             "$GPGGA,%s,%.4f,%c,%.4f,%c,1,08,0.9,%.1f,M,0.0,M,,",
             utc_time,
             fabs(lat_ddmm), (lat >= 0) ? 'N' : 'S',
             fabs(lon_ddmm), (lon >= 0) ? 'E' : 'W',
             alt);

    int checksum = 0;
    for (int i = 1; buffer[i] != '\0'; i++) {
        checksum ^= buffer[i];
    }

    char checksum_str[10];
    snprintf(checksum_str, sizeof(checksum_str), "*%02X", checksum);
    std::string gga_message = std::string(buffer) + std::string(checksum_str) + "\r\n";

    return gga_message;
}

/**
 * @brief Main function for the NtripClient.
 * 
 * @return 0 if the program exits successfully.
 */
int main() {
    std::string gga_message = generage_gga_message(51.05011, -114.08529, 1045);
    NtripClient client;
    client.Init("rtk2go.com", "2101", "axiombase1", "", "");
    client.UpdateGGA(gga_message);
    client.Run();
    std::signal(SIGINT, signal_handler);
    std::cout << "NtripClient is running. Press Ctrl+C to stop." << std::endl;
    std::cout << gga_message << std::endl;
    while (run) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        client.UpdateGGA(gga_message);
    }
    client.Stop();
    return 0;
}