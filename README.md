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
Our strategy to try and resolve the issues with the provided frontier-based exploration strategy is as follows:
1. We subscribe to `contest3/frontiers` and in our callback function `markerCallback()`, we maintain a vector of vectors called `redFrontiers`. Each vector within the larger vector corresponds to all the points within a particular red frontier. We also maintain a boolean variable to check if every single frontier is currently red: `allFrontiersRed`
2. In our experience, the provided frontier-based exploration strategy worked fairly well so long as not every frontier was red. However, we found the timeout for frontiers to get blacklisted and turn red was too long, so we shortened `progress_timeout` in `contest3.launch` to 4s so the frontiers would get blacklisted quicker if no progress was being made so that the below steps could be executed earlier.
3. The strategy from here is broken up into cases depending on how many red frontiers exist. As soon as a single blue frontier exists after any step, the strategy can be stopped.
	
    a) **Only one frontier exists and is red**
    
    > i) Determine the point in the frontier closest to the current TurtleBot position. This is done using helper function `getClosestPoint()`
       
    > ii) Use helper function `navigateNearby()` to try and navigate to this point. Note that `navigateNearby()` uses `"move_base/NavfnROS/make_plan"` to check if a plan is valid before executing it, similar to Contest 2. If a valid plan could not be found to navigate directly to the closest point at an angle of 0, we attempt to navigate to 36 adjacent points until we get a valid plan. These points are defined by radii 1.5m, 1m, 2m, 2.5m, 3m, and 0.5m away from the closest point at angles 0, π/3, -π/3, 2π/3, -2π/3 and π. Note that the radii have intentionally not been defined in a sequential order so we can check starting from the median then contract/expand. The navigation commands are sent using `move_base`, similar to Contest 2, except we now use a feedback callback in the call to `ac.sendGoal()` so the frontier colours continuously update.
     
    > iii) Completing up to step 3. a) ii) typically resolves the issue, but if it didn't make the frontier blue, spin in place between π/12 and π/3 in a pseudo-random direction chosen by `chooseAngular()` which we developed for Contest 1
    
    > iv) If the frontier is still red, repeat step 3. a) ii), but using the furthest point in the frontier calculated with helper function `getFurthestPoint()`
     
    > v) If none of the above has worked, resort to strategies from Contest 1. Set `linear = 0.25`, then try moving forward 0.85m at 0.15 m/s. If the frontier is still red then spin in place between π/12 and π/4 in a pseudo-random direction chosen by `chooseAngular()`. Note that our bumper handling methods from Contest 1 have been implemented in case of a bumper hit during this step.
    
    > vi) Repeat from step 3. a) i) if not resolved
      
    b) **Multiple frontiers exist and all are red**
    
    > i) Sort the frontiers in order of shortest to largest distance between the TurtleBot and the closest point to each individual frontier. This is done with helper function `orderIndices()`.
     
    > ii) Repeat steps 3. a) i) to 3. a) iv) for each individual frontier in the sorted order
      
    > iii) Repeat step 3. a) v) if not resolved
     
    > iv) Repeat from beginning of step 3 if not resolved.

