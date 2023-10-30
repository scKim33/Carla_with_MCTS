# Carla with Monte Carlo Tree Search(MCTS)

## Prerequisites
- heightmap package
- carla server
- roscore

## Code configuration
![alt text](https://github.com/sckim33/Carla_with_MCTS/fig/1.jpg?raw=true)

## Structure
```
vanilla_mcts
    ├── carla_simulator
    │   ├── carla-0.9.12-py3.8-linux-x86_64_with_full_seg.egg
    │   └── test_scenario.py
    ├── CMakeLists.txt
    ├── config
    │   └── config.yaml
    ├── include
    │   ├── mcts_random.h
    │   └── utils.h
    ├── launch
    │   └── mcts_random.launch
    ├── package.xml
    ├── README.md
    ├── rviz
    │   └── rviz.rviz
    └── src
        ├── main.cpp
        ├── mcts_random.cpp
        └── utils.cpp
```
- test_scenario.py : create carla actor
- config.yaml : set the parameters of mcts search
- mcts_random.cpp : mcts loop executed, called by main.cpp

## How to use
1. ROS master
```
roscore
```
2. Carla Server
```
./CarlaUE4.sh
```
3. Carla Actor
```
cd carla_simulator
python3 test_scenario.py
```
4. Run MCTS using roslaunch command
```
roslaunch vanilla_mcts mcts_random.launch
```
