#ifndef STAT_QD_TRAJ_HPP_
#define STAT_QD_TRAJ_HPP_

#include <numeric>
#include <sferes/stat/stat.hpp>
#include "matplotlibcpp.h"
#include <cmath>

namespace plt = matplotlibcpp;

namespace sferes {
    namespace stat {
        SFERES_STAT(QdMapTraj, Stat){
        public:
            template <typename E> void refresh(const E& ea)
            {
                if (ea.gen() % Params::pop::dump_period == 0){
					std::string prefix = std::string("map_traj");
					std::string fname2 = ea.res_dir() + "/" +prefix + std::string(".png");
                	write_progress(fname2, ea.pop()[10]);
				}

				// if (ea.gen() % Params::pop::dump_period == 0)
                //     ea.pop()[10]->fit().simulate(*ea.pop()[10],true,ea.res_dir() +"/hexapod"+".mp4");

				
            }


            template <typename Ind>
            void write_progress(const std::string& fname2, const Ind& indiv) const
            {
	      
	      std::cout << "writing..." << fname2 << std::endl;



		std::vector<double> x, y,t,yaw;
		
		for(size_t i=0; i < indiv->fit().traj().size(); i++)
		  {
		    x.push_back((double)indiv->fit().traj()[i][3]);
		    y.push_back((double)indiv->fit().traj()[i][4]);
			yaw.push_back((double)indiv->fit().traj()[i][2]);
			t.push_back(i);
		  }

		
		plt::figure();
		plt::figure_size(1500,1000);

		plt::subplot(3,1,1);
		plt::plot(t,x);
		// plt::title(std::string("Trajectory / Forward"));
		plt::xlabel("Time Step");
		plt::ylabel("Forward Velocity");

		plt::subplot(3,1,2);
		plt::plot(t,y);
		// plt::title(std::string("Trajectory / Lateral"));
		plt::xlabel("Time Step");
    	plt::ylabel("Lateral Velocity");


		plt::subplot(3,1,3);
		plt::plot(t,yaw);
		// plt::title(std::string("Trajectory / Yaw"));
		plt::xlabel("Time Step");
    	plt::ylabel("Yaw Velocity");

		std::map<std::string, double> keywords;
		// keywords.insert(std::make_pair("left", 0.1)); // also right, top, bottom
		keywords.insert(std::make_pair("wspace", 0.3)); // also hspace
		plt::subplots_adjust(keywords);

		plt::subplots_adjust();
		// Enable legend.
		plt::legend();
		// Save the image (file format is determined by the extension)
		plt::save(fname2.c_str());
		plt::close();
	    }

        };

    } // namespace stat
} // namespace sferes

#endif
