#include "HybridSDNApp.hpp"
#define FD_SETSIZE 4096
#include <sys/select.h>
	
int main(int argc, char** argv){
  uint16_t port = (argc>1)? static_cast<uint16_t>(std::atoi(argv[1])) : 6633;
  HybridSDNApp::Paths paths;
   paths.graph_json = "config/NSFNET.json";
   paths.flows_csv  = "config/flows.csv";

  try {
    HybridSDNApp app(port, paths);
    app.run();                    // blocks until you externally stop / signal
  } catch (const std::exception& ex) {
    std::cerr << "[fatal] " << ex.what() << "\n";
    return 1;
  }
  return 0;
}

