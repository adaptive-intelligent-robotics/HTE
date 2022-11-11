#! /usr/bin/env python
import sys
sys.path.insert(0, sys.path[0]+'/waf_tools')
print(sys.path[0])


import os
import limbo
import mcts
import ros
import hexa_control

import dart
import corrade
import magnum
import magnum_integration
import magnum_plugins
import robot_dart



from waflib.Configure import conf

# import dart 
# import robot_dart
#import hexapod_dart

def options(opt):
    opt.load('mcts')
    opt.load('hexapod_controller')
    opt.load('ros')
    opt.load('hexa_control')
    opt.load('dart')
    opt.load('robot_dart')
    opt.load('corrade')
    opt.load('magnum')
    opt.load('magnum_integration')
    opt.load('magnum_plugins')
    #opt.load('hexapod_dart')

    opt.add_option('--robot', action='store_true', help='build for real robot (no graphic, no simulator)[true/false]', dest='robot')

def configure(conf):

    print('conf exp:')
    conf.get_env()['BUILD_GRAPHIC'] = False
    conf.get_env()['BUILD_ROBOT'] = False

    conf.load('dart')
    conf.load('corrade')
    conf.load('magnum')
    conf.load('magnum_integration')
    conf.load('magnum_plugins')
    conf.load('robot_dart')
    conf.check_dart()
    print("#"*100)

    conf.check_corrade(components='Utility PluginManager', required=False)
    conf.env['magnum_dep_libs'] = 'MeshTools Primitives Shaders SceneGraph GlfwApplication Text MagnumFont'
    if conf.env['DEST_OS'] == 'darwin':
        conf.env['magnum_dep_libs'] += ' WindowlessCglApplication'
    else:
        conf.env['magnum_dep_libs'] += ' WindowlessGlxApplication'
    conf.check_magnum(components=conf.env['magnum_dep_libs'], required=False)
    conf.check_magnum_plugins(components='AssimpImporter', required=False)
    conf.check_magnum_integration(components='Dart', required=False)
    print("#"*100)
    if len(conf.env.INCLUDES_MagnumIntegration) > 0:
        conf.get_env()['BUILD_MAGNUM'] = True
        conf.env['magnum_libs'] = magnum.get_magnum_dependency_libs(conf, conf.env['magnum_dep_libs']) + magnum_integration.get_magnum_integration_dependency_libs(conf, 'Dart')
    print("#"*100)
    conf.check_robot_dart()
    print("#"*100)
    #conf.load('hexapod_dart')
    #conf.check_hexapod_dart()

    conf.load('mcts')
    conf.check_mcts()
    print("#"*100)
    if conf.options.robot:
      conf.load('ros')
      conf.load('hexa_control')
      conf.check_ros()
      conf.check_hexa_control()
      if conf.env.DEFINES_ROS:
        conf.get_env()['BUILD_ROBOT'] = True
    
    # print "Libraries", str(bld.env['magnum_libs'])
    print('done')


def build(bld):
    bld.env.LIBPATH_PYTHON = '/usr/lib/x86_64-linux-gnu/'
    bld.env.LIB_PYTHON = [ 'python3.8']
    bld.env.INCLUDES_PYTHON = '/usr/include/python3.8'

    #only needed for debugging 
    #bld.env.LIBPATH_ROBOTDART = '/workspace/lib'
    #bld.env.LIB_ROBOTDART = ['RobotDARTSimu']
    #bld.env.INCLUDES_ROBOTDART = '/workspace/include'
    libs = str(bld.env['magnum_libs'])  + 'ROBOTDART HEXAPOD_CONTROLLER DART MCTS ROS TBB EIGEN BOOST_DART BOOST ROS DYNAMIXEL LIMBO LIBCMAES NLOPT SFERES2 BOOST_CHRONO RT PTHREAD MPI PYTHON'
    graphic_libs = 'DART_GRAPHIC ' + libs
    
    robot_libs = 'HEXACONTROL ' + libs
    cxxflags = bld.get_env()['CXXFLAGS'] + ["-g"]+ ["-pg"]


## Regular RTE

    bld.program(features = 'cxx',
                source = 'include/hbr/dart_exp.cpp',
                includes = '. ../../ /git/limbo/src/',
                uselib =  bld.env['magnum_libs']  + 'ROBOTDART TBB BOOST EIGEN PTHREAD MPI DART ROBOT_DART DART_GRAPHIC PYTHON',
                use = 'sferes2',
                defines = ["BD_ACC","ASYNC","GP_USE","LEG_CONFIG","ORIENT"],
                cxxflags = ["-g"],
                target = 'hbr_training')

    bld.program(features = 'cxx',
                          source = 'hte.cpp',
                          uselib_local = '/git/limbo',
                          uselib = graphic_libs,
                          includes=". ../../src ../../ ../ ./include /git/limbo/src/",
                          cxxflags = cxxflags,
                          defines = ["HBR","GP_USE","ASYNC","LEG_CONFIG","BD_ACC"],
                          target = 'hte')

    bld.program(features = 'cxx',
                          source = 'hte.cpp',
                          uselib_local = '/git/limbo',
                          uselib = graphic_libs,
                          includes=". ../../src ../../ ../ ./include /git/limbo/src/",
                          cxxflags = cxxflags,
                          defines = ["HBR","GRAPHIC","GP_USE","ASYNC","LEG_CONFIG","BD_ACC"],
                          target = 'hte_graphic')


    # bld.program(features = 'cxx',
    #                       source = 'hte.cpp',
    #                       uselib_local = '/git/limbo',
    #                       uselib = graphic_libs,
    #                       includes=". ../../src ../../ ../ ./include /git/limbo/src/",
    #                       cxxflags = cxxflags,
    #                       defines = ["GRAPHIC"],
    #                       target = 'high_dim_graphic')
                          
    # bld.program(features = 'cxx',
    #                       source = 'hte.cpp',
    #                       uselib_local = '/git/limbo',
    #                       uselib = graphic_libs,
    #                       includes=". ../../src ../../ ../ ./include /git/limbo/src/",
    #                       cxxflags = cxxflags,
    #                       defines = ["LOW_DIM","GRAPHIC"],
    #                       target = 'low_dim_graphic')

## APROL


    # bld.program(features = 'cxx',
    #                       source = 'aprol.cpp',
    #                       uselib_local = '/git/limbo',
    #                       uselib = graphic_libs,
    #                       includes=". ../../src ../../ ../ ./include /git/limbo/src/",
    #                       cxxflags = cxxflags,
    #                       defines = ["LOW_DIM","GRAPHIC"],
    #                       target = 'aprol_graphic')

    # bld.program(features = 'cxx',
    #         source = 'include/hbr_src/dart_exp.cpp',
    #         includes = '. ../../ /git/limbo/src/',
    #         uselib =  bld.env['magnum_libs']  + 'ROBOTDART TBB BOOST EIGEN PTHREAD MPI DART ROBOT_DART DART_GRAPHIC PYTHON',
    #         use = 'sferes2',
    #         defines = ["BD_ACC","ASYNC","GP_USE","LEG_CONFIG"],
    #         cxxflags = ["-g"],
    #         target = 'acceleration_gp_legs')





    
    
    
    
    
    
    
