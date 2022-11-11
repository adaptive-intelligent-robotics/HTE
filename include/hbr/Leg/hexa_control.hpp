#ifndef ROBOT_DART_CONTROL_HEXA_LEG_CONTROL
#define ROBOT_DART_CONTROL_HEXA_LEG_CONTROL

// #include <hexapod_controller/hexapod_controller_simple.hpp>
// #include "hexapod_controller_leg.hpp"
#include "hexapod_controller_leg_simple.hpp"
// #include "hexapod_controller_leg_fixed.hpp"


#include <robot_dart/control/policy_control.hpp>

namespace robot_dart {
    namespace control {

        template <bool Fixed>
        struct HexaLegPolicy {
        
        // HexaLegPolicy (bool fixed) : _fixed(fixed){}
        public:
            void set_params(const Eigen::VectorXd& ctrl)
            {
                //We pass the leg as an parameter

                _controller.set_fixed(_fixed);    

                _controller.set_leg((int) ctrl[ctrl.size()-1]);
                Eigen::VectorXd controls;
                controls.resize(ctrl.size()-1);
                for(int i=0;i<ctrl.size()-1;i++){
                    controls[i] = ctrl[i];
        
                }
                _controller.set_parameters(controls);       
            }

            size_t output_size() const { return 18; }

            Eigen::VectorXd query(const std::shared_ptr<robot_dart::Robot>& robot, double t)
            {
                if (!_h_params_set) {
                    _dt = robot->skeleton()->getTimeStep();
                }
                auto angles = _controller.pos(t);


                Eigen::VectorXd target_positions = Eigen::VectorXd::Zero(18 + 6);
                for (size_t i = 0; i < angles.size(); i++)
                {
                    target_positions(i+6) = ((i % 3 == 1) ? 1.0 : -1.0) * angles[i];

                } 
                // std::cout<<"TIME"<<t<<"\t"<<target_positions<<"\n"<<std::endl;

                Eigen::VectorXd q = robot->skeleton()->getPositions();
                // std::cout<<q.size()<<std::endl;
                
                Eigen::VectorXd q_err = target_positions - q;

                // std::cout<<q_err.size()<<std::endl;

                double gain = 1.0 / (dart::math::constants<double>::pi() * _dt);
                Eigen::VectorXd vel = q_err * gain;
                

                return vel.tail(18);
            }

            void set_h_params(const Eigen::VectorXd& h_params)
            {
                _dt = h_params[0];
                _h_params_set = true;
            }

            Eigen::VectorXd h_params() const
            {
                return Eigen::VectorXd::Ones(1) *_dt;
            }

            void set_controller(hexapod_controller::HexaLegController &controller)
            {
               this->_controller = controller;
            }

            std::vector<double> pos(double t){
                return this->_controller.pos(t);
            }
            
        protected:
            hexapod_controller::HexaLegController _controller;

            bool _fixed=Fixed;
            double _dt;
            bool _h_params_set = false;
            // int _leg=leg;
        };
        using HexaLegControl = robot_dart::control::PolicyControl<HexaLegPolicy<false>>;
        using HexaLegControlInit = robot_dart::control::PolicyControl<HexaLegPolicy<true>>;
    } // namespace control
} // namespace robot_dart

#endif
