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

#include "nmea.h"

// Calculate NMEA checksum (returns two uppercase hex digits as string)
std::string nmea_checksum(const std::string& sentence) {
    unsigned char cksum = 0;
    // skip initial '$' if present
    size_t start = (sentence[0] == '$') ? 1 : 0;
    size_t end = sentence.find('*');
    if (end == std::string::npos) end = sentence.size();
    for (size_t i = start; i < end; ++i) {
        cksum ^= static_cast<unsigned char>(sentence[i]);
    }
    char buf[3];
    snprintf(buf, sizeof(buf), "%02X", cksum);
    return std::string(buf);
}

// Check if a NMEA message has a valid checksum
bool nmea_check(const std::string& sentence) {
    size_t star = sentence.find('*');
    if (star == std::string::npos || star + 2 >= sentence.size()) return false;
    std::string expected = nmea_checksum(sentence.substr(0, star));
    std::string actual = sentence.substr(star + 1, 2);
    for (auto& c : actual) c = toupper(c);
    return expected == actual;
}

// Format a string as a NMEA sentence with checksum
std::string nmea_format(const std::string& body) {
    std::string sentence = body;
    if (!sentence.empty() && sentence[0] != '$') sentence = "$" + sentence;
    std::string cksum = nmea_checksum(sentence);
    return sentence + "*" + cksum + "\r\n";
}