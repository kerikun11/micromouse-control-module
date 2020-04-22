#include "accel_designer.h"

#include <fstream>
#include <iostream>

void printCsv(const std::string &filebase, const ctrl::AccelDesigner &ad) {
  logi << ad << std::endl;
  const float Ts = 0.0001f;
  std::ofstream of;
  const auto ticks = ad.getTimeStamp();
  float t = 0;
  for (size_t i = 0; i < ticks.size(); ++i) {
    of = std::ofstream(filebase + "_" + std::to_string(i) + ".csv");
    while (t + Ts < ticks[i]) {
      of << t;
      of << "," << ad.j(t);
      of << "," << ad.a(t);
      of << "," << ad.v(t);
      of << "," << ad.x(t);
      of << std::endl;
      t += Ts;
    }
  }
}

int main(void) {
  ctrl::AccelDesigner ad;
  ad.reset(100, 10, 4, 0, 2, 4);
  // ad.reset(1000, 10, 4, 0, 2, 4);
  // ad.reset(100, 1, 4, 0, 2, 4);
  // ad.reset(100, 10, 4, 0, 2, 0.4);
  // ad.reset(100, 10, 4, 0, -2, -0.4);

  // ad.reset(911459, 128550, 7344.07, 8391.93, 1995.32, 309.06);
  // ad.reset(911459, 128550, 7344.07, -8391.93, -1995.32, -309.06);
  // ad.reset(130043, 836543, 8952.48, 6185.94, 2342.21, 1237.71);
  // ad.reset(130043, 836543, 8952.48, -6185.94, -2342.21, -1237.71);
  // ad.reset(829703, 6047.83, 7056.55, 0, 7454.57, 1957.28);
  printCsv("accel", ad);

  return 0;
}
