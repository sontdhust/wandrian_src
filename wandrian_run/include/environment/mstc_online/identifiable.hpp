/*
 * identifiable.hpp
 *
 *  Created on: Mar 11, 2016
 *      Author: sontd
 */

#ifndef WANDRIAN_RUN_INCLUDE_ENVIRONMENT_MSTC_ONLINE_IDENTIFIABLE_HPP_
#define WANDRIAN_RUN_INCLUDE_ENVIRONMENT_MSTC_ONLINE_IDENTIFIABLE_HPP_

#include <string>

namespace wandrian {
namespace environment {
namespace mstc_online {

class Identifiable {

public:
  Identifiable(std::string);
  ~Identifiable();

  std::string get_robot_name() const;

private:
  std::string robot_name;
};

}
}
}

#endif /* WANDRIAN_RUN_INCLUDE_ENVIRONMENT_MSTC_ONLINE_IDENTIFIABLE_HPP_ */
