#include <uWS/uWS.h>
#include <iostream>
#include "Controller.h"
#include "MPC.h"
#include "client.h"
#include "PID.h"
#include "Localizer.h"

class PIDContoller : public Controller {

public:

  PIDContoller(const Localizer& localizer, const PID& pid)
  : pid_{pid}, localizer_{localizer} { }

  ControllerResult activate(double x, double y, double psi, double v) override {

    localizer_.activate(x, y, psi, v);

    double cte = localizer_.state_(4);
    pid_.UpdateError(cte);

    ControllerResult res{};
    res.steer_value = pid_.SteeringAngle();
    res.throttle_value = 0.2;

    return res;

  }

private:

  PID pid_;
  Localizer localizer_;
  
};

int main() {

  WaypointsMap map = readWaypoints("../data/waypoints.txt");

  uWS::Hub h;

  PID pid;
  pid.Init(0.3, 0., 20.);
  Localizer localizer{map, 10, 3};

  PIDContoller controller{localizer, pid};

  initHub(h, controller);

  //MPC mpc;
  // TODO call initHub with concrete Controller

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
