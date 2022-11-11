#ifndef STAT_QD_LEG_GRID_HPP_
#define STAT_QD_LEG_GRID_HPP_

#include <numeric>
#include <sferes/stat/stat.hpp>
#include "matplotlibcpp.h"
#include <cmath>

namespace plt = matplotlibcpp;

namespace sferes {
    namespace stat {
        SFERES_STAT(QdMapLegGrid, Stat){
        public:
            template <typename E> void refresh(const E& ea)
            {
                if (ea.gen() % Params::pop::dump_period == 0)
                    _write_progress(std::string("grid_legs"), ea);
            }
            template <typename EA>
            void _write_progress(const std::string& prefix, const EA& ea) const
            {
	      std::string fname2 = ea.res_dir() + "/" +prefix + std::string(".png");
	      std::cout << "writing..." << fname2 << std::endl;



		std::vector<double> x, y,v,w,z;
		
		for(size_t i=0; i < ea.pop().size(); i++)
		  {
		    auto bd = ea.pop()[i]->fit().desc();
		    x.push_back(bd[0]);
		    y.push_back(bd[1]);
			// v.push_back(cos(ea.pop()[i]->fit().angle()));
			// w.push_back(sin(ea.pop()[i]->fit().angle()));
			std::bitset<6> bit; 
			// std::string bit_string = "";
			// for(size_t k=2; k < bd.size(); k++){
			for(size_t k=0; k < ea.pop()[i]->fit().forces().size(); k++){
				
				bit[k] = ea.pop()[i]->fit().forces()[k];
				// bit_string += std::to_string(bd[k]);
				
			}
			// std::cout<<bit_string<<std::endl;
			
			int mybit_int;
			mybit_int = (int)(bit.to_ulong()); 
		    z.push_back(mybit_int);
		  }
		

		

		
		size_t nb_col=64;
		std::vector<std::vector<double> > color;
		for(size_t i=0; i<nb_col; i++){
			double x= ((float)i/(float)nb_col);
			color.push_back({
					(sin(x*2*M_PI + 1.3*M_PI) / 2.0+0.5)*0.8+0.1,
					sin(x*2*M_PI)/2*0.6+0.60,
					1-(sin(x*2*M_PI+1.6*M_PI)/2+0.5)});
		}

		plt::figure();
		plt::figure_size(3000,3000);
		int row=0,col=0;

		for(int i=0; i < 64; i++){

			std::bitset<6> bit(i); 
			std::bitset<3> col_bit,row_bit;
			for(int l=0;l<3;l++){
				col_bit[l] = bit[l];
				row_bit[l] = bit[l+3];
			}
			plt::subplot(8,8,((row_bit.to_ulong())*8)+(col_bit.to_ulong())+1); //trying to find the correct position within the grid


			std::vector<double> x_cat,y_cat;
			std::vector<std::vector<double> > c_cat;

			for(size_t k=0;k<z.size();k++){
				if(z[k]==i){
					x_cat.push_back(x[k]);
					y_cat.push_back(y[k]);
					c_cat.push_back(color[i]);
				}

			}

			x_cat.push_back(1.1);
			y_cat.push_back(1.1);
			c_cat.push_back(color[i]);

			x_cat.push_back(-0.1);
			y_cat.push_back(-0.1);
			c_cat.push_back(color[i]);

			plt::scatter(x_cat,y_cat,3.0, c_cat);
			if(i%8==0){
				plt::ylabel(row_bit.to_string());
				row++;
			}
			if(i>55){
				plt::xlabel(col_bit.to_string());
				col++;
			}

		}
		// Save the image (file format is determined by the extension)
		plt::save(fname2.c_str());
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