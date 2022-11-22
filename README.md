# Hierarchical Trial and Error Algorithm

This repository contains the code for the [Hierarchical Trial and Error algorithm ](https://doi.org/10.1145/3512290.3528751) presented at GECCO 2022. The implementation builds on top of the RTE algorithm from the paper Chatzilygeroudis, K., Vassiliades, V., & Mouret, J.-B. (2018). Reset-free Trial-and-Error Learning for Robot Damage Recovery. Robotics and Autonomous Systems. All the code uses the [Sferesv2](https://github.com/sferes2/sferes2) framework to run the hierarchical repertoires, so if you would like to familiarise yourself with the framework it will help to have a look at their examples first.
The link to the original repository can be found [here](https://github.com/resibots/chatzilygeroudis_2018_rte). This repository implements the Hierarchical Trial and Error algorithm (HTE) on top of the RTE code and uses the robot_dart wrapper instead of the hexapod_dart wrapper. This phase of the RTE algorithm consists of the adaptation and path planning phase which uses: 

1. limbo  
limbo is the library used for the Gaussian Processes to update the model (BD update)
2. MCTS (Monte Carlo Tree Search)  
An [MCTS library](https://github.com/resibots/chatzilygeroudis_2018_rte/tree/master/mcts) was used and installed as part of this program.
3. A*  
A* is used to guide MCTS as MCTS samples actions randomnly. This can be found in the 'include' directory. 


## Dependencies
- Ubuntu 
- Sferesv2 (QD Branch) https://github.com/sferes2/sferes2/tree/qd/sferes
- DART simulator, http://dartsim.github.io/ (release-6.1 branch)
- Eigen 3, http://eigen.tuxfamily.org/
- Boost  
Note: If you are using the singularity images/containers in the singularity directory, all the required dependancies and libraries have already been pre-installed and the experiment can be readily run.

## Repository Structure

This repository consists of two separate steps:

1. Generating Hierarchical Behavioural Repertoires
2. Using HTE to plan and adapt with Hierarchical Behavioural Repertoires

The directory to train and use hierarchical repertoires is in the include directory under [`hbr`](include/hbr). For our paper, we used three layers which all correspond to an individual Evolutionary Algorithm Class called `QD_Layer` which is defined in the [`qd_layer.hpp` file](include/hbr/HBR/qd_layer.hpp). Hierarchical Repertoires are in turn composed of multiple layers which are stored in the class `HBR_Seq` in the [file `HBR_Seq.hpp`](include/hbr/HBR/HBR_Seq.hpp).

In each layer directory, you will find a main file with the `QD_Layer` definition, the descriptors (to extract features from the simulators) and the hexapod controllers. Here are the different layer folder:

1. [Bottom Layer](include/hbr/Leg/)
- The main file is the [`fit_hexa_leg.hpp` file](include/hbr/Leg/fit_hexa_leg.hpp). This layer directly calls the Hexapod in the simulation and extracts the Behavioural Descriptors from it.
2. [Middle Layer](include/hbr/Body_Acc/)
- The main file is the [`fit_hexa_body_gp.hpp` file](include/hbr/Body_Acc/fit_hexa_body_gp.hpp). This layer calls six behaviours from the Bottom Layer which are in turn then executed on the robot. 
3. [Top Layer](include/hbr/Omnidirectional/)
- The main file is the [`fit_mt_me.hpp` file](include/hbr/Omnidirectional/fit_mt_me.hpp). This layer calls three behaviours from the Middle Layer which cascades down the commands to the Bottom Layer and finally executes it on the robot.

## Training HBRs
Before executing HTE, it is necessary to train the Hierarchical Repertoires. This is done with the [`dart_exp.cpp` file](include/hbr/dart_exp.cpp). The following section explains how to train and run the code within a singularity container. The singluarity container is defined in the [`singularity.def` file](singularity/singularity.def).

## Run HTE Experiments in Sandbox Mode  
This section details the steps to quickly run a demo and observe how this algorihtm works on a hexapod robot in simulation. This demo works for a pre-defined archive and pre-defined map. This is to avoid complications for new users to the environment and C++. Details for how to self-experiment with the code and change various parameters can be found in the following section. 

1. Clone the repository on the local device using git.
```
git clone https://github.com/adaptive-intelligent-robotics/HTE.git
```

2. Enter the repository and navigate to the 'singularity' directory
``` 
cd hte/singularity
```

3. Run the start_container.sh script which automatically builds the singularity image with all the necessary dependencies. 
```
./start_container.sh
```
4. Once you have built your image, you should be in the home directory of the Singularity Container.

5. Compile the Script by using the [`setup.sh` file](singularity/resources/setup.sh). That file is located in the `git/sferes` directory when you created the singularity container.
Example: 
```
cd /git/sferes2
./setup.sh
```
6. The command above should compile the code that is defined in the wscript that is contained within your folder (which is bound to the singularity container).

```
/git/sferes2/exp/HTE/wscript
```

Every file that is changed locally, will directly changed within the container now. This makes it easier to develop code. Compiling all the programs can take some time so feel free to comment out the blocks of code that you don't need within the [wscript](wscript).
If you would like to run the code without visuals , please remove the keyword `GRAPHIC` from the wscript build definitions.

7. Among the compiled programs, we have :
    - `hbr_training`: This trains and saves Hierarchical Behavioural Repertoires on the Hexapod Omnidirectional Task as noted in the paper. To define the target folder for the results, define the argument `-d` with the path.
    - `hte` : This runs the HTE version. The archives (called `gen` files) that will be created bys the `hbr_training` should be put into the directory `exp/resources/hbr_repertoires/hbr_repertoire`. This includes the binary files of each layer (`gen` file) as well as the repertoire content `archive_x.dat` of the **last** layer where `x` is the number of iterations you have used. **NOTE** Rename the binary files of each layer by appending the suffix of the layer : `gen_l1`,`gen_l2` and `gen_l3` . We have included the original trained repertoires as `.zip` files which have been used in the paper and put them into the [resources folder](resources/):
        - [aprol_repertoires](resources/aprol_repertoires/)
        - [flat_repertoires](resources/flat_repertoires/) (both 2D and 8D)
        - [hbr_repertoires ](resources/hbr_repertoires)
    In the [singularity definition](singularity/singularity.def) we unzip the files and put it into the correct folders. You don't have to do anything if you have already started the container as noted in step 3.
    - `hte_graphic` : Same as the previous algorithm but with a graphical interface. However you will have to export a DISPLAY variable to match the visual server you have on your machine.
    - `low_dim_graphic` : This runs the classical RTE version with a 2D archive. The archives for this version are located in [flat_2D_repertoires](resources/flat_repertoires/flat_2D_repertoire.zip).
    - `high_dim_graphic` : This runs the classical RTE version with a 8D archive. The archives for this version are located in [flat_8D_repertoires](resources/flat_repertoires/flat_8D_repertoire.zip).

    They can then be executed by running the command (for example): 
    ```
    build/exp/HTE/hte -r 1 -l 0 2>/dev/null
    ```

    **NOTE**
    Sometimes the graphical interface fails and we needed to do the following to get it to work with a graphical interface.
    ```
    unset DISPLAY
    export DISPLAY=:1
    export LC_ALL=C; unset LANGUAGE
    ```
    where the Display number is your display

8. The compiled codes for the HTE (`build/exp/HTE/hte`) take the following arguments:
    - `-r` : repertoire number x that should be used (1 to 5)
    - `-l` : leg that should be damaged (0 to 5) and double damage (both central legs) is 6

    When using damage, DART produces some Warning that are currently silenced with `2>/dev/null`.

    **Note** Tuning the number of [MCTS iterations](hte.cpp#L2090) has an impact of the runtime. Start with a lower number if you want to experiment quickly.

    For Example:
    ```
     build/exp/HTE/hte -r 1 -l 0 2>/dev/null
    ```
    If you run the example in the singularity container, the results can be found in the singularity file under `/HTE.sif/git/sferes2/build/exp/HTE` if you do not specify any results directory.
    

## Where to find stuff?
### 1. The building blocks for the map
Currently we only use one single map and the building blocks for this are in the `hte.cpp` file in the init_simu_world function.

### 2.HBR Training 
All the layers are defined in the [hbr](include/hbr) folder as noted above.

### 3. A* library 
The A* library used was written by Chatzilygeroudis, K. and was taken from [here](https://github.com/resibots/chatzilygeroudis_2018_rte). It can be found in the include directory in this repository. 

### 4. MCTS library
The MCTS library is preinstalled in the container. But the source code can be obtained from [here](https://github.com/resibots/chatzilygeroudis_2018_rte/tree/master/mcts).


## Citation
If you find this useful and used the code or paper in your research please cite our paper:

_Maxime Allard, Simón C. Smith, Konstantinos Chatzilygeroudis, and Antoine Cully. 2022. Hierarchical quality-diversity for online damage recovery. In Proceedings of the Genetic and Evolutionary Computation Conference (GECCO '22). Association for Computing Machinery, New York, NY, USA, 58–67. https://doi.org/10.1145/3512290.3528751_

```
@inproceedings{Allard2022,
author = {Allard, Maxime and Smith, Sim\'{o}n C. and Chatzilygeroudis, Konstantinos and Cully, Antoine},
title = {Hierarchical Quality-Diversity for Online Damage Recovery},
year = {2022},
isbn = {9781450392372},
publisher = {Association for Computing Machinery},
address = {New York, NY, USA},
url = {https://doi.org/10.1145/3512290.3528751},
doi = {10.1145/3512290.3528751},
booktitle = {Proceedings of the Genetic and Evolutionary Computation Conference},
pages = {58–67},
numpages = {10},
keywords = {quality-diversity, hierarchical learning, robotics},
location = {Boston, Massachusetts},
series = {GECCO '22}
}
```
## Contact
Please contact Maxime Allard (maxime.allard@imperial.ac.uk) for any inquires or difficulties that you might encounter.



