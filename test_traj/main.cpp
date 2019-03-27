#include "AccelDesigner.h"
#include "SlalomDesigner.h"

#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>

std::ofstream of("out.csv");

int main(void) {
  signal_processing::SlalomDesigner sd;
  signal_processing::AccelDesigner ad;
  ad.reset(5400*M_PI, 90 * M_PI, 0, 4 * M_PI, 0, M_PI / 2);
  std::cout << ad << std::endl;
  ad.printCsv(of, 0.0001f);
  return 0;
}
