#include <iostream>
#include <string>
#include <boost/program_options.hpp>

#include "Dbg.h"

namespace po = boost::program_options;

int main (int argc, char* argv[]) {
  // Declare the supported options
  std::string IP_ADDRESS;
  std::string CMD;
  po::options_description opt("Allowed options");
  opt.add_options()
    (
      "ip", 
      po::value<std::string>(&IP_ADDRESS)->default_value("192.168.0.102"), 
      "IP address of hololens"
    )
    (
      "cmd",
      po::value<std::string>(&CMD)->default_value("colorpts"),
      "Pipeline to run"
    );
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, opt), vm);
  po::notify(vm);
  if (vm.count("ip"))
    IP_ADDRESS = vm["ip"].as<std::string>();
  if (vm.count("cmd"))
    CMD = vm["cmd"].as<std::string>();

  std::cout << "IP Address set to " << IP_ADDRESS << std::endl;
  std::cout << "Running pipeline " << CMD << std::endl;
  // read in command
  if (CMD.compare("colorpts") == 0) {
    holovision::colorpoints_pipeline();
  }
  else if (CMD.compare("meshsocket") == 0) {
    int frames = argc > 2 ? std::stoi(argv[2]) : 4;
    holovision::meshsocket_pipeline(frames);
  }
  else {
    std::cout << "Invalid arg " << CMD << std::endl;
  }
  return (0);
}
