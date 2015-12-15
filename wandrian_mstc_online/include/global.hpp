/*
 * global.hpp
 *
 *  Created on: Dec 15, 2015
 *      Author: manhnh
 */

#ifndef WANDRIAN_MSTC_ONLINE_INCLUDE_GLOBAL_HPP_
#define WANDRIAN_MSTC_ONLINE_INCLUDE_GLOBAL_HPP_

#include "plans/spiral_stc/cell.hpp"
#include <boost/shared_ptr.hpp>

using namespace wandrian::plans::spiral_stc;

namespace wandrian {

class Global {

public:
  std::set<CellPtr, CellComp> old_cells;

  Global();
  ~Global();
  static boost::shared_ptr<Global> get_instance();

private:
  static boost::shared_ptr<Global> instance;
};

typedef boost::shared_ptr<Global> GlobalPtr;

}

#endif /* WANDRIAN_MSTC_ONLINE_INCLUDE_GLOBAL_HPP_ */
