#include <iostream>
#include <string>

#include "Dbg.h"

int main (int argc, char* argv[]) {
  std::string arg(argc > 1 ? argv[1] : "colorpts");
  if (arg.compare("colorpts") == 0) {
    holovision::colorpoints_pipeline();
  }
  else if (arg.compare("meshsocket") == 0) {
    int frames = argc > 2 ? std::stoi(argv[2]) : 4;
    holovision::meshsocket_pipeline(frames);
  }
  else {
    std::cout << "Invalid arg" << std::endl;
  }
  return (0);
}
