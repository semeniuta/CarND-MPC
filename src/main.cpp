#include <uWS/uWS.h>
#include <iostream>
#include "Controller.h"
#include "MPC.h"
#include "client.h"

int main() {

  uWS::Hub h;

  MPCConfig conf{50, 0.2, 5.};
  conf.change_delta_penalty_ = 100;
  conf.change_a_penalty_ = 500;

  MPCController controller{conf};

  initHub(h, controller);

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();

}
