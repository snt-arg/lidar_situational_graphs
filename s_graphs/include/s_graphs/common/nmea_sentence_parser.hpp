/*
Copyright (c) 2023, University of Luxembourg
All rights reserved.

Redistributions and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 'AS IS'
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*/



#ifndef NMEA_SENTENCE_PARSER_HPP
#define NMEA_SENTENCE_PARSER_HPP

#include <boost/algorithm/string.hpp>
#include <cmath>
#include <numeric>
#include <string>
#include <vector>

namespace s_graphs {

/**
 * @brief
 */
struct GPRMC {
 public:
  /**
   * @brief Contructor of class GPRMC
   */
  GPRMC() { status = 'V'; }

  /**
   * @brief Contructor of class GPRMC
   *
   * @param tokens
   */
  GPRMC(const std::vector<std::string>& tokens) {
    if (tokens[0] != "$GPRMC" || tokens.size() < 12) {
      status = 'V';
      return;
    }

    long time = std::stol(tokens[1]);
    hour = time / 10000;
    minute = (time % 10000) / 100;
    second = time % 100;

    status = tokens[2][0];

    latitude = degmin2deg(std::stod(tokens[3]));
    latitude = tokens[4] == "N" ? latitude : -latitude;

    longitude = degmin2deg(std::stod(tokens[5]));
    longitude = tokens[6] == "E" ? longitude : -longitude;

    speed_knots = std::stod(tokens[7]);
    track_angle_degree = std::stod(tokens[8]);

    long date = std::stol(tokens[9]);
    year = date % 100;
    month = (date / 100) % 100;
    day = (date / 10000) % 100;

    magnetic_variation = std::stod(tokens[10]);
    magnetic_variation =
        tokens[11][0] == 'E' ? magnetic_variation : -magnetic_variation;
  }

  double degmin2deg(double degmin) {
    double d = std::floor(degmin / 100.0);
    double m = (degmin - d * 100.0) / 60.0;
    return d + m;
  }

 public:
  char status;  // Status A=active or V=Void.

  int hour;  // Fix taken at 12:35:19 UTC
  int minute;
  int second;

  double latitude;  //
  double longitude;

  double speed_knots;         // Speed over the ground in knots
  double track_angle_degree;  // Track angle in degrees True

  int year;
  int month;
  int day;

  double magnetic_variation;
};

/**
 * @brief
 */
class NmeaSentenceParser {
 public:
  /**
   * @brief Contructor of class NmeaSentenceParser.
   *
   */
  NmeaSentenceParser() {}
  ~NmeaSentenceParser() {}

  /**
   * @brief
   *
   * @param sentence
   * @return Instance of GPRMC.
   */
  GPRMC parse(const std::string& sentence) const {
    int checksum_loc = sentence.find('*');
    if (checksum_loc == std::string::npos) {
      return GPRMC();
    }

    int checksum = std::stoul(sentence.substr(checksum_loc + 1), nullptr, 16);

    std::string substr = sentence.substr(1, checksum_loc - 1);
    int sum = std::accumulate(substr.begin(),
                              substr.end(),
                              static_cast<unsigned char>(0),
                              [=](unsigned char n, unsigned char c) { return n ^ c; });

    if (checksum != (sum & 0xf)) {
      std::cerr << "checksum doesn't match!!" << std::endl;
      std::cerr << sentence << " " << sum << std::endl;
      return GPRMC();
    }

    std::vector<std::string> tokens;
    boost::split(tokens, sentence, boost::is_any_of(","));

    return GPRMC(tokens);
  }
};

}  // namespace s_graphs

#endif  // NMEA_SENTENCE_PARSER_HPP
