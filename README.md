# KTH formula student

# Exercise 1
## Docker
first pull the kthfsdv docker:
```
docker pull kthfsdv/ros-intro
```
and run it:
```
docker run -it kthfsdv/ros-intro
```
Then start two new terminal windows and respectively run:
````
docker ps
````
to get the image name, and then run:
````
docker exec -it <image name> /bin/bash
````
and then insde the container:
```
source /ros_entrypoint.sh
cd ~/kthfsdv/
```
You should now have three terminals in the container.
## Get code
In one of the terminals get the content of exc1 by doing:
```
source /opt/ros/melodic/setup.bash
mkdir -p ~/kthfsdv/src
cd ~/kthfsdv/src/
git clone https://github.com/markusjonek/formula_student.git
mv formula_student/exc1 ./
rm -r formula_student
catkin build
```
## Start roscore and nodes
In one of the terminals start the roscore:
```
source ~/kthfsdv/devel/setup.bash
roscore
```
In another terminal start checking the topic "kthfs/result":
```
source ~/kthfsdv/devel/setup.bash
rostopic echo kthfs/result
```
In the third terminal launch the nodes with:
```
source ~/kthfsdv/devel/setup.bash
cd ~/kthfsdv/src/exc1
roslaunch launch.launch
```
Then check the "rostopic echo" window if it is working.


Here is a screenshot of PlotJuggler:
![exercise1_plot](https://user-images.githubusercontent.com/17691221/163716334-0ec68367-8a8b-4977-9a72-8f2e1e72ebe0.png)
___
# Exercise 2
To run the plotting you will need matplotlib and numpy
````
pip3 install numpy
pip3 install matplotlib
````
Then to run the code do:
````
python3 plot.py "function" "color"
````
Choose "live" or "simple" as "function" and a valid matplotlib-color for "color". 



For example:
````
python3 plot.py live blue
# or
python3 plot.py simple green
````





Here are screenshots of plot.py:

````
python3 plot.py live blue
````
![figure3](https://user-images.githubusercontent.com/17691221/163729055-20b1663d-f542-49a2-a643-dfc49b821de5.png)


___

````
python3 plot.py simple red
````
![figure2](https://user-images.githubusercontent.com/17691221/163728309-4fb8025b-9999-40b3-8a0d-dab1e5d5c1bc.png)
