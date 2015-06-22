#include "../include/keyop_core/keyop_core.hpp"

using keyop_core::KeyOpCore;

int main(int argc, char **argv) {
	ros::init(argc, argv, "kobuki_keyop");
	KeyOpCore keyop;
	if (keyop.init()) {
		keyop.spin();
	} else {
		ROS_ERROR_STREAM("Couldn't initialize KeyOpCore!");
	}

	ROS_INFO_STREAM("Program exiting");
	return 0;
}
