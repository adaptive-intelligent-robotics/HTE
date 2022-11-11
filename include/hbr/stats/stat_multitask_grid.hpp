#ifndef STAT_QD_MT_GRID_HPP_
#define STAT_QD_MT_GRID_HPP_

#include <numeric>
#include <sferes/stat/stat.hpp>
#include "matplotlibcpp.h"
#include <cmath>

namespace plt = matplotlibcpp;

namespace sferes {
    namespace stat {
        SFERES_STAT(QdMapMTGrid, Stat){
        public:
            template <typename E> void refresh(const E& ea)
            {
                if ((ea.gen() % (Params::pop::nb_gen-1) == 0) && (ea.gen()>0))//needs to be the same as the qd_multitask_eval.hpp file
                    _write_progress(std::string("grid_multi_task"), ea);
            }
            template <typename EA>
            void _write_progress(const std::string& prefix, const EA& ea) const
            {
				
			}
        };

    } // namespace stat
} // namespace sferes

#endif
