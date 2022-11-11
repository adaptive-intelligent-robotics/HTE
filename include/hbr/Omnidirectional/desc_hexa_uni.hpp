#ifndef ROBOT_DART_DESCRIPTOR_HEXA_UNI_HPP
#define ROBOT_DART_DESCRIPTOR_HEXA_UNI_HPP

// for size_t
#include <cstddef>

#include <algorithm>
#include <map>
#include <vector>
#include <numeric>

#include<Eigen/Core>

namespace robot_dart {
    class RobotDARTSimu;
    class Robot;

    namespace descriptor {

      struct DutyCycle:public BaseDescriptor{
      public:
	DutyCycle(RobotDARTSimu* simu, size_t desc_dump = 1):BaseDescriptor(simu,desc_dump)
	{
	  for (size_t i = 0; i<6; i++){
	    _contacts[i] = std::vector<size_t>();
	  }
	}


	virtual void operator()()
	{
	  const dart::collision::CollisionResult& col_res = _simu->world()->getLastCollisionResult();
	  for (size_t i = 0; i < 6; ++i) {
	    std::string leg_name = "leg_" + std::to_string(i) + "_3";
	    dart::dynamics::BodyNodePtr body_to_check = _simu->robots().back()->skeleton()->getBodyNode(leg_name);
	    _contacts[i].push_back(col_res.inCollision(body_to_check));
	  }
	      
	} // operator()
	
	void get(std::vector<double>& results)
	{
	  double sum = 0.0;
	  for (size_t i = 0; i < 6; i++) {
	    sum += (std::round(std::accumulate(_contacts[i].begin(), _contacts[i].end(), 0.0) / double(_contacts[i].size()) * 100.0) / 100.0);
	  }

	  results.push_back(sum/6.0);

	}
      protected:
	std::map<size_t, std::vector<size_t>> _contacts;

      };//struct dutycycle 


		
    } // namespace descriptor
} // namespace robot_dart

#endif
