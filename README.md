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

### Installation Procedure

Use the following commands to create a new catkin workspace and a virtual environment with all the required dependencies.

```bash
# first install Anaconda in your computer for using virtual environments
# then create a new environment with python2.7 option
conda create -n virtual_env python=2.7.17
# activate this environment
conda activate virtual_env

# install required packages
pip install -r python_dependencies.txt
# install tensoflow using anaconda
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


## Race with Trained Network

We have provided our final trained checkpoint file in this repository.
If you want to change the checkpoint file used for testing once you have trained a new one, go to [net_controller_launch.launch](./sim2real_drone_racing/learning/deep_drone_racing_learning_node/launch/net_controller_launch.launch) sim2real_drone_racing/learning/deep_drone_racing_learning_node/launch/net_controller_launch.launch
and change the this line:
```bash
<arg name="ckpt_file" default="$(find deep_drone_racing_learner)/src/ddr_learner/results/best_model_without_warmup_1.5/model_latest"/>
```

Open a terminal and type:
```bash
conda activate virtual_env
roslaunch deep_drone_racing_learning  net_controller_launch.launch

```

Open an other terminal and type:
```bash
conda activate virtual_env
roslaunch test_racing test_racing.launch

```

## Train your own Drone Racing Model
You can also generate data in simulation and train your model on it. 

In this paper, we use a customized DAgger policy to alleviate the drawback of pure imitation learning. If you want skip the time-consuming procedure of data collection and intermediate training process, our final colllected data is shared in [this link](https://drive.google.com/file/d/1o8MM5zCbC3CgiHmArsvOy51HmqmRe46R/view?usp=sharing). Unzip the files into folder ./sim2real_drone_racing/learning/deep_drone_racing_learner/data and then train the network
```bash
conda activate virtual_env
roscd deep_drone_racing_learner/src/ddr_learner
# under this folder, first modify the training data directory in the training bash file - train_model.sh, change this line:
train_data=../../data/Training_final_test
# then, run the training bash file
./train_model.sh

```


### DAgger Step1: Generate data

```bash
cd drone_racing_ws
. ./catkin_ddr/devel/setup.bash
. ./droneflow/bin/activate
roscd drone_racing/resources/scripts
python collect_data.py

```

It is possible to change parameters (number of iteration per background/ gate texture/ etc. ) in the above script.
Defaults should be good. Optionally, you can use the data we have already collected, available at [this link](http://rpg.ifi.uzh.ch/datasets/sim2real_ddr/simulation_training_data.zip).


### DAgger Step2: Train the Network

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
