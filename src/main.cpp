#include <uWS/uWS.h>
#include <iostream>
#include "Controller.h"
#include "MPC.h"
#include "client.h"

int main() {

  uWS::Hub h;

  NewMPC controller{};

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
