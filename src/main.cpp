#include <iostream>
#include "FrameMessage.h"

int main (int argc, char** argv) {
  std::cout << holovision::read_msg_from_file("../262754255709.bin") << std::endl;
  std::cout << holovision::read_msg_from_file("../262756114425.bin") << std::endl;
  return (0);
}
