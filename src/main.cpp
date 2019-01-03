#include <uWS/uWS.h>
#include <iostream>
#include "Controller.h"
#include "MPC.h"
#include "client.h"

int main() {

  uWS::Hub h;

  // 50, 0.2, 5.    <- slow, but well
  // 20, 0.05, 20.  <- faster, a bit wiggly, doesn't deal with sharp turns
  MPCConfig conf{50, 0.2, 5.};

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
