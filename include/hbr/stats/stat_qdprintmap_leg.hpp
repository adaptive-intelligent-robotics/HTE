#ifndef STAT_QD_PRINT_MAP_LEG_HPP_
#define STAT_QD_PRINT_MAP_LEG_HPP_

#include <numeric>
#include <sferes/stat/stat.hpp>
#include "matplotlibcpp.h"
#include <cmath>

namespace plt = matplotlibcpp;

namespace sferes {
    namespace stat {
        SFERES_STAT(QdPrintMapLeg, Stat){
        public:
            template <typename E> void refresh(const E& ea)
            {
                if (ea.gen() % Params::pop::dump_period == 0)
                    _write_progress(std::string("map"), ea);
            }
            template <typename EA>
        void _write_progress(const std::string& prefix, const EA& ea) const
            {
	      std::string fname2 = ea.res_dir() + "/" +prefix+"_"+std::to_string(ea.gen()) + std::string(".png");
	      std::cout << "writing..." << fname2 << std::endl;



		std::vector<double> x, y,z;
		
		for(size_t i=0; i < ea.pop().size(); i++)
		  {
		    auto bd = ea.pop()[i]->fit().desc();
		    x.push_back(bd[0]);
		    y.push_back(bd[1]);
		    z.push_back(ea.pop()[i]->fit().value());
		  }
		
		// to fix boudaries
		x.push_back(1.1);
		y.push_back(1.1);
		z.push_back(0);
		x.push_back(-0.1);
		y.push_back(-0.1);
		z.push_back(-1);


		
		// std::cout<<x.size()<<" "<<y.size()<<" "<<z.size()<<std::endl;
		plt::figure();
		plt::figure_size(1500,1000);
		plt::scatter(x,y,z,3.0, true);
		// plt::quiver(x,y,v,w,z);

		// Add graph title
		plt::title(std::string("Archive ")+std::to_string(ea.gen()));
		plt::xlabel("Forward Displacement");
    	plt::ylabel("Vertical Displacement");
		// Enable legend.
		plt::legend();
		// Save the image (file format is determined by the extension)
		plt::save(fname2.c_str());
		// plt::close();
		plt::close();
	    }
	      

	    std::vector<std::vector<double> > create_color( const std::vector<double>& x) const
	    {
	      double min = std::numeric_limits<double>::infinity();
	      //double max = - std::numeric_limits<double>::infinity();
	      double max = 0;
	      for(auto & val:x)
		{
		  min=std::min(val,min);
		  //max=std::max(val,max);
		  
		}
	      double diff = max-min;
	      if(diff==0)
		diff=1;
	      std::vector<std::vector<double> > color;
	      for(size_t i=0; i<x.size(); i++){
		color.push_back({(sin( (x[i]-min)/diff*2*M_PI + 1.3*M_PI) / 2.0+0.5)*0.8+0.1,
				sin((x[i]-min)/diff*2*M_PI)/2*0.6+0.60,
				1-(sin((x[i]-min)/diff*2*M_PI+1.6*M_PI)/2+0.5)});
	      }
	      return color;
	    }


        };

    } // namespace stat
} // namespace sferes

#endif
