# Robust Drone Racing based on Imitation Learning and Modularization
In this repository, we modularize the whole navigation drone system, and utilize imitation learning to train the perception module.
This kind of modularazation enables us to combine the robustness provided by data-based approaches and the precision provided by model-based approaches. 

Our navigation system was only trained using synthetic textures but is able to navigate in environment with randomly-chosen photorealistic textures.

Video: [YouTube](https://youtu.be/8ws1VSvASHc)

<p align="center">
  <img src="./docs/sim2real.gif" alt="ddr">
</p>

## Installation

### Requirements

The code was tested with Ubuntu 18.04 and ROS Melodic (Python2.7).
Different OS and ROS versions are possible but with the possibility of potential conflict.

### Step-by-Step Procedure

Use the following commands to create a new catkin workspace and a virtual environment with all the required dependencies.

```bash
# first install Anaconda in your computer for using virtual environments
# then create a new environment with python2.7 option
conda create -n virtual_env python=2.7.17
# activate this environment
conda activate virtual_env

# install required packages
pip install -r python_dependencies.txt
# install tensoflow
conda install tensorflow-gpu==1.12.0

#create a new ros workpace
cd ~
mkdir -p drone_racing_worksapce/src
cd ~/drone_racing_worksapce/src
catkin init
catkin build

# download the ros packages included in this repository
cd ~/drone_racing_worksapce/src
git clone https://github.com/tianqi-wang1996/DeepRobustDroneRacing.git

# Build and re-source the workspace
catkin build
source ../devel/setup.bash
```


## Let's Race

Once you have installed the dependencies, you will be able to fly in simulation with our pre-trained checkpoint. You don't need GPU for execution. Note that if the network can't run at least at 10Hz, you won't be able to fly successfully.

Open a terminal and type:
```bash
cd drone_racing_ws
. ./catkin_ddr/devel/setup.bash
. ./droneflow/bin/activate
export CUDA_VISIBLE_DEVICES=''
roslaunch deep_drone_racing_learning  net_controller_launch.launch

```

Open an other terminal and type:
```bash
cd drone_racing_ws
. ./catkin_ddr/devel/setup.bash
. ./droneflow/bin/activate
roslaunch test_racing test_racing.launch

```

## Train your own Sim2Real model

You can use the following commands to generate data in simulation and train your model on it. The trained checkpoint can then be used to control a physical platform on a race track.

### Generate data

```bash
cd drone_racing_ws
. ./catkin_ddr/devel/setup.bash
. ./droneflow/bin/activate
roscd drone_racing/resources/scripts
python collect_data.py

```

It is possible to change parameters (number of iteration per background/ gate texture/ etc. ) in the above script.
Defaults should be good. Optionally, you can use the data we have already collected, available at [this link](http://rpg.ifi.uzh.ch/datasets/sim2real_ddr/simulation_training_data.zip).


### Train the Network

```bash
roscd deep_drone_racing_learner/src/ddr_learner

```

Modify the file [train\_model.sh](./learning/deep_drone_racing_learner/src/ddr_learner/train_model.sh) to add the path of validation data collected in the real world, which you can download from [this link](http://rpg.ifi.uzh.ch/datasets/sim2real_ddr/validation_real_data.zip).
Then, run the following command to train the model.

```bash
./train_model.sh

```

### Test the Network

Edit the following file to use the checkpoint you just trained

```bash
rosed deep_drone_racing_learning net_controller_launch.launch

```

The trained network can now be tested in an environment which was never observed at training time.

Open a terminal and run:

```bash
cd drone_racing_ws
. ./catkin_ddr/devel/setup.sh
. ./droneflow/bin/activate
export CUDA_VISIBLE_DEVICES=''
roslaunch deep_drone_racing_learning  net_controller_launch.launch

```

Open another terminal and run:

```bash
cd drone_racing_ws
. ./catkin_ddr/devel/setup.sh
. ./droneflow/bin/activate
roslaunch test_racing test_racing.launch

```
