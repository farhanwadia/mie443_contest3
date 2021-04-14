# MIE443 Contest 3: Finding and Interacting with Emotional People in an Unknown Enviroment - Group 18
## Group Members
**Farhan Wadia - 1003012606**

**Henry Cueva Barnuevo - 1003585122**

**Yilin Huang - 1003145232**

## Execution Commands
1. Place this repository in the `home/turtlebot/catkin_ws/src` folder of the file system.

2. Launch the simulated world in Gazebo:
    
    a) `cd` to the `mie443_contest3` folder and activate the conda environment:
    ```bash
    conda activate mie443
    ```
    b) Launch Gazebo
    ```bash
    roslaunch mie443_contest3 turtlebot_world.launch world:=practice
    ```
3. Launch GMapping:
    ```bash
    roslaunch mie443_contest3 gmapping.launch
    ```
4. Run the Victim Detector:
    a) `cd` to the `mie443_contest3` folder and activate the conda environment:
    ```bash
    conda activate mie443
    ```
    b) `cd` to the `mie443_contest3/src` folder and run:
    ```bash
    python victimLocator.py
    ```
5. Run the Emotion Classifier:   
    a) `cd` to the `mie443_contest3` folder and activate the conda environment:
    ```bash
    conda activate mie443
    ```
    b) `cd` to the `mie443_contest3/src` folder and run:
    ```bash
    python emotionClassifier.py
    ```
6. Launch RVIZ:
    ```bash
    roslaunch turtlebot_rviz_launchers view_navigation.launch
    ```
7. Launch the contest 3 code:
    ```bash
    roslaunch mie443_contest3 contest3.launch
    ```
8. Run sound_play. Use `conda deactivate` as needed to make sure no conda environment is running before doing the below command:
    ```bash
    rosrun sound_play soundplay_node.py
    ```
## Discussion of exploration strategy

