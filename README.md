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
and then:
```
source /opt/ros/melodic/setup.bash
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
```
You should now have three terminals in the container.
## Get code
In one of the terminals get the content of exc1 by doing:
```
mkdir -p ~/kthfsdv/src
cd ~/kthfsdv/src/
git clone https://github.com/markusjonek/formula_student.git
mv formula_student/exc1 ./
rm -r formula_student
cd ~/kthfsdv
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
To run the plotting you will need matplotlib, numpy and tikzplotlib:
````
pip3 install numpy
pip3 install matplotlib
pip3 install tikzplotlib
````
Then to run the code do:
````
python3 plot.py
````
You will have the ability to change line color, add a grid, start/stop, scroll and save the figure as .tex and .png.


Here is a screenshot of plot.py:


![figure1](https://user-images.githubusercontent.com/17691221/163770054-80a46d23-5a3e-4cbc-9d23-cf9997f22a04.png)
