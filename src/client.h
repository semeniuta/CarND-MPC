//
// Created by Oleksandr Semeniuta on 2018-12-29.
//

#ifndef CLIENT_H
#define CLIENT_H

#include "Controller.h"
#include <string>
#include <uWS/uWS.h>

std::string hasData(std::string s);

void initHub(uWS::Hub& h, Controller& controller);

#endif //CLIENT_H
