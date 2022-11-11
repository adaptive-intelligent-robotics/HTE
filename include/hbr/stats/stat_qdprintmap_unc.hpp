#ifndef STAT_QD_PRINT_MAP_UNC_HPP_
#define STAT_QD_PRINT_MAP_UNC_HPP_

#include <numeric>
#include <sferes/stat/stat.hpp>
#include "matplotlibcpp.h"
#include <cmath>

namespace plt = matplotlibcpp;

namespace sferes {
    namespace stat {
        SFERES_STAT(QdPrintMapUnc, Stat){
        public:
            template <typename E> void refresh(const E& ea)
            {
                if (ea.gen() % Params::pop::dump_period == 0)
                    _write_progress(std::string("map_unc"), ea);
            }
            template <typename EA>
            void _write_progress(const std::string& prefix, const EA& ea) const
            {
	    	}


        };

    } // namespace stat
} // namespace sferes

#endif
