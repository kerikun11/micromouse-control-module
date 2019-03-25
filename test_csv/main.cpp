#include "AccelDesigner.h"
#include <cstdio>
#include <fstream>
#include <iostream>

using namespace signal_processing;

int main(void) {
  /* Parameter Settings */
  float a_max, v_start, v_sat, v_end, distance;
  std::cout << "a_max: ";
  std::cin >> a_max;
  a_max = a_max == 0 ? 9000 : a_max;
  std::cout << "v_start: ";
  std::cin >> v_start;
  v_start = v_start == 0 ? 0 : v_start;
  std::cout << "v_sat: ";
  std::cin >> v_sat;
  v_sat = v_sat == 0 ? 2400 : v_sat;
  std::cout << "v_end: ";
  std::cin >> v_end;
  v_end = v_end == 0 ? 0 : v_end;
  std::cout << "distance: ";
  std::cin >> distance;
  distance = distance == 0 ? 90 * 8 : distance;
  std::cout << "filename: ";
  std::string filename;
  std::cin >> filename;
  filename = filename == "" ? "out.csv" : filename;

  /* Design */
  AccelDesigner sd;
  sd.reset(a_max, v_start, v_sat, v_end, distance);

  /* Output */
  std::ofstream of(filename);
  sd.printCsv(of);
  std::cout << "x_end: " << sd.x_end() << std::endl;

  /* End */
  return 0;
}
