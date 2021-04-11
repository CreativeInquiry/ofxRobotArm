#ifndef ROBOTKUKA_HPP
#define ROBOTKUKA_HPP
#include <pthread.h>
#include "models.hpp"
#include <queue>
namespace kuka
{
	class RobotKuka
	{
	public:
		RobotKuka(float CYCLE_TIME);
		RobotKuka();

		void startCommunicator(char *const ip, unsigned int port);
		void stopCommunicator();
		void move(KukaPose pose, double time);

		float CYCLE_TIME;

		std::queue<KukaPose> move_queue;

		KukaPose security_limit_speed = KukaPose(100.0, 100.0, 100.0, 60.0, 60.0, 60.0); // [mm/s] [deg/s];

	private:
		void set_communicator_running_flag(bool value);
		bool *get_communicator_running_flag();

		bool *communicator_running_flag;
	};
}
#endif // ROBOTKUKA_HPP