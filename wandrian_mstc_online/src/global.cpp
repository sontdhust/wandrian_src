/*
 * global.cpp
 *
 *  Created on: Dec 15, 2015
 *      Author: manhnh
 */

#include "../include/global.hpp"

namespace wandrian {

GlobalPtr Global::instance;

Global::Global() {
}

Global::~Global() {
}

GlobalPtr Global::get_instance() {
  if (instance == NULL)
    instance = GlobalPtr(new Global());
  return instance;
}

}
