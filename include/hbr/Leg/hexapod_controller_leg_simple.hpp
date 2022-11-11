#ifndef __CONTROLLER_SIMPLE_HPP__
#define __CONTROLLER_SIMPLE_HPP__

// For M_PI constant
#define _USE_MATH_DEFINES

#include <array>
#include <cassert>
#include <cmath>
#include <vector>

#define ARRAY_DIM 100

namespace hexapod_controller {

  class HexaLegController
  {
  public:
          typedef std::array<double, ARRAY_DIM> array_t;

          HexaLegController() {}

        //   HexaLegController(int leg, bool fixed=false) : _leg(leg), _fixed(fixed) {}

          /**
              All parameters should have a value between 0 and 1.
           **/
          void set_parameters(const Eigen::VectorXd& ctrl)
          {
                  assert(ctrl.size() == 6);

                  _controller = ctrl;

                  // Set random movement for all the legs to prevent segmentation fault
                  array_t random_signal_sin = _sine_signal(0.0, 0.0);
                  array_t random_signal_cos = _cosine_signal(0.0, 0.0);
                  _legs0commands.clear();
                  _legs0commands.push_back(random_signal_cos);
                  _legs0commands.push_back(random_signal_sin);
                  _legs1commands.clear();
                  _legs1commands.push_back(random_signal_cos);
                  _legs1commands.push_back(random_signal_sin);
                  _legs2commands.clear();
                  _legs2commands.push_back(random_signal_cos);
                  _legs2commands.push_back(random_signal_sin);
                  _legs3commands.clear();
                  _legs3commands.push_back(random_signal_cos);
                  _legs3commands.push_back(random_signal_sin);
                  _legs4commands.clear();
                  _legs4commands.push_back(random_signal_cos);
                  _legs4commands.push_back(random_signal_sin);
                  _legs5commands.clear();
                  _legs5commands.push_back(random_signal_cos);
                  _legs5commands.push_back(random_signal_sin);

                  // Set controller for specified leg
                  switch (_leg) {
                  case 0:
                          _legs0commands.clear();
                          if (_fixed) {
                                  _legs0commands.push_back(_fixed_signal(ctrl[0], ctrl[1], ctrl[2]));
                                  _legs0commands.push_back(_fixed_signal(0,0,0));
                          } else {
                                  _legs0commands.push_back(_control_signal(ctrl[0], ctrl[1], ctrl[2]));
                                  _legs0commands.push_back(_control_signal(ctrl[3], ctrl[4], ctrl[5]));
                          }
                        //   if (std::all_of(_legs0commands[0].begin(), _legs0commands[0].end(), [](double i) { return i==0.0; }))
                        //         std::cout<<"EMPTY LEG COMMAND : "<<_leg<<(_fixed ? " true": " false")<<std::endl;
                          break;
                  case 1:
                          _legs1commands.clear();
                          if (_fixed) {
                                  _legs1commands.push_back(_fixed_signal(ctrl[0], ctrl[1], ctrl[2]));
                                  _legs1commands.push_back(_fixed_signal(0,0,0));
                          } else {
                                  _legs1commands.push_back(_control_signal(ctrl[0], ctrl[1], ctrl[2]));
                                  _legs1commands.push_back(_control_signal(ctrl[3], ctrl[4], ctrl[5]));
                          }
                        //   if (std::all_of(_legs1commands[0].begin(), _legs1commands[0].end(), [](double i) { return i==0.0; }))
                        //         std::cout<<"EMPTY LEG COMMAND : "<<_leg<<(_fixed ? " true": " false")<<std::endl;
                          break;
                  case 2:
                          _legs2commands.clear();
                          if (_fixed) {
                                  _legs2commands.push_back(_fixed_signal(ctrl[0], ctrl[1], ctrl[2]));
                                  _legs2commands.push_back(_fixed_signal(0,0,0));
                          } else {
                                  _legs2commands.push_back(_control_signal(ctrl[0], ctrl[1], ctrl[2]));
                                  _legs2commands.push_back(_control_signal(ctrl[3], ctrl[4], ctrl[5]));
                          }
                        //   if (std::all_of(_legs2commands[0].begin(), _legs2commands[0].end(), [](double i) { return i==0.0; }))
                        //         std::cout<<"EMPTY LEG COMMAND : "<<_leg<<(_fixed ? " true": " false")<<std::endl;
                          break;
                  case 3:
                          _legs3commands.clear();
                          if (_fixed) {
                                  _legs3commands.push_back(_fixed_signal(ctrl[0], ctrl[1], ctrl[2]));
                                  _legs3commands.push_back(_fixed_signal(0,0,0));
                          } else {
                                  _legs3commands.push_back(_control_signal(ctrl[0], ctrl[1], ctrl[2]));
                                  _legs3commands.push_back(_control_signal(ctrl[3], ctrl[4], ctrl[5]));
                          }
                        //   if (std::all_of(_legs3commands[1].begin(), _legs3commands[1].end(), [](double i) { return i==0.0; }))
                        //         std::cout<<"EMPTY LEG COMMAND : "<<_leg<<(_fixed ? " true": " false")<<std::endl;
                          break;
                  case 4:
                          _legs4commands.clear();
                          if (_fixed) {
                                  _legs4commands.push_back(_fixed_signal(ctrl[0], ctrl[1], ctrl[2]));
                                  _legs4commands.push_back(_fixed_signal(0,0,0));
                          } else {
                                  _legs4commands.push_back(_control_signal(ctrl[0], ctrl[1], ctrl[2]));
                                  _legs4commands.push_back(_control_signal(ctrl[3], ctrl[4], ctrl[5]));
                          }
                        //   if (std::all_of(_legs4commands[1].begin(), _legs4commands[1].end(), [](double i) { return i==0.0; }))
                        //         std::cout<<"EMPTY LEG COMMAND : "<<_leg<<(_fixed ? " true": " false")<<std::endl;
                          break;
                  case 5:
                          _legs5commands.clear();
                          if (_fixed) {
                                  _legs5commands.push_back(_fixed_signal(ctrl[0], ctrl[1], ctrl[2]));
                                  _legs5commands.push_back(_fixed_signal(0,0,0));
                          } else {
                                  _legs5commands.push_back(_control_signal(ctrl[0], ctrl[1], ctrl[2]));
                                  _legs5commands.push_back(_control_signal(ctrl[3], ctrl[4], ctrl[5]));
                                    
                          }
                        //   if (std::all_of(_legs5commands[1].begin(), _legs5commands[1].end(), [](double i) { return i==0.0; }))
                        //         std::cout<<"EMPTY LEG COMMAND : "<<_leg<<(_fixed ? " true": " false")<<std::endl;
                          break;
                  }

          }

          const Eigen::VectorXd& parameters() const
          {
                return _controller;
          }
        
          void set_broken(const std::vector<int> broken_legs)
          {
          _broken_legs = broken_legs;
          }

          const std::vector<int>& broken_legs() const
          {
          return _broken_legs;
          }

          std::vector<double> pos(double t) const
          {
                assert(_controller.size() == 6);
                std::vector<double> angles;
                int leg = 0;

                for (int leg = 0; leg < 6; leg++) {
                        
                        switch (leg) {
                        case 0:
                                angles.push_back(M_PI_4 / 2 * _legs0commands[0][(int(std::floor(t * ARRAY_DIM))) % ARRAY_DIM]);
                                angles.push_back(M_PI_4 * _legs0commands[1][(int(std::floor(t * ARRAY_DIM))) % ARRAY_DIM]);
                                angles.push_back(-M_PI_4 * _legs0commands[1][(int(std::floor(t * ARRAY_DIM))) % ARRAY_DIM]);
                                break;

                        case 1:
                                angles.push_back(M_PI_4 / 2 * _legs1commands[0][(int(std::floor(t * ARRAY_DIM))) % ARRAY_DIM]);
                                angles.push_back(M_PI_4 * _legs1commands[1][(int(std::floor(t * ARRAY_DIM))) % ARRAY_DIM]);
                                angles.push_back(-M_PI_4 * _legs1commands[1][(int(std::floor(t * ARRAY_DIM))) % ARRAY_DIM]);
                                break;

                        case 2:
                                angles.push_back(M_PI_4 / 2 * _legs2commands[0][(int(std::floor(t * ARRAY_DIM))) % ARRAY_DIM]);
                                angles.push_back(M_PI_4 * _legs2commands[1][(int(std::floor(t * ARRAY_DIM))) % ARRAY_DIM]);
                                angles.push_back(-M_PI_4 * _legs2commands[1][(int(std::floor(t * ARRAY_DIM))) % ARRAY_DIM]);
                                break;

                        case 3:
                                angles.push_back(M_PI_4 / 2 * _legs3commands[0][(int(std::floor(t * ARRAY_DIM))) % ARRAY_DIM]);
                                angles.push_back(M_PI_4 * _legs3commands[1][(int(std::floor(t * ARRAY_DIM))) % ARRAY_DIM]);
                                angles.push_back(-M_PI_4 * _legs3commands[1][(int(std::floor(t * ARRAY_DIM))) % ARRAY_DIM]);
                                break;

                        case 4:
                                angles.push_back(M_PI_4 / 2 * _legs4commands[0][(int(std::floor(t * ARRAY_DIM))) % ARRAY_DIM]);
                                angles.push_back(M_PI_4 * _legs4commands[1][(int(std::floor(t * ARRAY_DIM))) % ARRAY_DIM]);
                                angles.push_back(-M_PI_4 * _legs4commands[1][(int(std::floor(t * ARRAY_DIM))) % ARRAY_DIM]);
                                break;

                        case 5:
                                angles.push_back(M_PI_4 / 2 * _legs5commands[0][(int(std::floor(t * ARRAY_DIM))) % ARRAY_DIM]);
                                angles.push_back(M_PI_4 * _legs5commands[1][(int(std::floor(t * ARRAY_DIM))) % ARRAY_DIM]);
                                angles.push_back(-M_PI_4 * _legs5commands[1][(int(std::floor(t * ARRAY_DIM))) % ARRAY_DIM]);
                                break;
                        }
                }
        //   for(int i=0; i<angles.size();i++){
        //           if(angles[i]!=0.0){
        //              std::cout<<_leg<<"\t"<<i<<"\t"<<angles[i]<<std::endl;     
        //           }
        //   }
        //   std::cout<<"Done"<<std::endl;
                
                return angles;

          }

          void set_leg(int leg){
                this->_leg = leg;
          }
          void set_fixed(bool fixed){
                this->_fixed=fixed;
          }

  protected:
          // Defines which leg to control
          int _leg;
          // If true, fix leg to _start
          bool _fixed=false;
          std::vector<array_t> _legs0commands;
          std::vector<array_t> _legs1commands;
          std::vector<array_t> _legs2commands;
          std::vector<array_t> _legs3commands;
          std::vector<array_t> _legs4commands;
          std::vector<array_t> _legs5commands;
          Eigen::VectorXd _controller;
          std::vector<int> _broken_legs;

          array_t _sine_signal(double amplitude, double phase) const
          {
                  // Calculate phase shift
                  double shift = M_PI * phase;
                  array_t command;
                  for (unsigned i = 0; i < ARRAY_DIM; ++i) {
                          // Calculate time step
                          double t = 2 * M_PI * i / ARRAY_DIM;
                          // Follow Sine wave: a * sin(x - b)
                          command[i] = amplitude * sin(t - shift);
                  }
                  return command;
          }

          array_t _cosine_signal(double start, double amplitude) const
          {
                  array_t command;
                  for (unsigned i = 0; i < ARRAY_DIM; ++i) {
                          double t = 2 * M_PI * i / ARRAY_DIM;
                        //   command[i] = amplitude * (cos(t+(M_PI/2)) - 0) + start;
                          command[i] = amplitude * 0.5*(cos(t) - 1) + start;
                  }
                  return command;
          }

          array_t _fixed_signal(double amplitude, double phase, double duty_cycle) const
          {
                  array_t command;
                  array_t control_signal = _control_signal(amplitude,phase,duty_cycle);

                  for (unsigned i = 0; i < ARRAY_DIM; ++i) {
                        //   command[i] = position;
                        command[i] = 0.0;
                        // command[i] = control_signal[0];
                  }
                  return command;
          }
          // Clip values between -1 and 1
          double _clip(double value) const
          {
                  return std::max(-1.0, std::min(value, 1.0));
          }

        array_t _control_signal(double amplitude, double phase, double duty_cycle) const
        {
            array_t temp;
            int up_time = ARRAY_DIM * duty_cycle;
            for (int i = 0; i < up_time; i++)
                temp[i] = amplitude;
            for (int i = up_time; i < ARRAY_DIM; i++)
                temp[i] = -amplitude;

            // filtering
            int kernel_size = ARRAY_DIM / 10;

            array_t command;

            std::vector<double> kernel(2 * kernel_size + 1, 0);
            double sigma = kernel_size / 3;

            double sum = 0;
            for (int i = 0; i < int(kernel.size()); i++) {
                kernel[i] = exp(-(i - kernel_size) * (i - kernel_size) / (2 * sigma * sigma)) / (sigma * sqrt(M_PI));
                sum += kernel[i];
            }

            for (int i = 0; i < ARRAY_DIM; i++) {
                command[i] = 0;
                for (int d = 1; d <= kernel_size; d++) {
                    if (i - d < 0)
                        command[i] += temp[ARRAY_DIM + i - d] * kernel[kernel_size - d];
                    else
                        command[i] += temp[i - d] * kernel[kernel_size - d];
                }
                command[i] += temp[i] * kernel[kernel_size];
                for (int d = 1; d <= kernel_size; d++) {
                    if (i + d >= ARRAY_DIM)
                        command[i] += temp[i + d - ARRAY_DIM] * kernel[kernel_size + d];
                    else
                        command[i] += temp[i + d] * kernel[kernel_size + d];
                }

                command[i] /= sum;
            }

            // apply phase
            array_t final_command;
            int current = 0;
            int start = std::floor(ARRAY_DIM * phase);
            for (int i = start; i < ARRAY_DIM; i++) {
                final_command[current] = command[i];
                current++;
            }
            for (int i = 0; i < start; i++) {
                final_command[current] = command[i];
                current++;
            }

            return final_command;
        }
  };
}

#endif
