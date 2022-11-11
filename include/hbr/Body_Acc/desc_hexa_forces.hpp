#ifndef ROBOT_DART_DESCRIPTOR_HEXA_FORCES_HPP
#define ROBOT_DART_DESCRIPTOR_HEXA_FORCES_HPP

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

      struct Forces:public BaseDescriptor{
      public:
	Forces(RobotDARTSimu* simu, size_t desc_dump = 1):BaseDescriptor(simu,desc_dump)
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
	
		float magnitude = body_to_check->getExternalForceGlobal().head(6).tail(3).cast <float> ().norm();
	    _contacts[i].push_back(col_res.inCollision(body_to_check));
	  }
	      
	} // operator()
	
	void get(std::vector<double>& results)
	{
	  for (size_t i = 0; i < 6; i++) {
		double contact = std::round(std::accumulate(_contacts[i].begin(), _contacts[i].end(), 0.0) / double(_contacts[i].size()) * 100.0) / 100.0;
		if(contact > 0.25)
	    	results.push_back(1.0);
		else
			results.push_back(0.0);
	  }
	}

	void reset(){
		this->_contacts.clear();
	}

	int size(){
		return this->_contacts[0].size();
	}

	std::vector<double> avg(){
		std::vector<double> avg;
		for (size_t i = 0; i < 6; i++) {
			double contact = std::round(std::accumulate(_contacts[i].begin(), _contacts[i].end(), 0.0) / double(_contacts[i].size()) * 100.0) / 100.0;
			avg.push_back(contact);
		}
		return avg;
	}

      protected:
	std::map<size_t, std::vector<size_t>> _contacts;

      };//struct forces


		
    } // namespace descriptor
} // namespace robot_dart

#endif
