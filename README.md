# KTH formula student

# Exercise 1
first pull the kthfsdv docker:
```
docker pull kthfs/ros-intro
```
and run it:
```
docker run -it kthfs/ros-intro
```
Then do:
```
source /opt/ros/melodic/setup.bash
mkdir -p ~/kthfsdv/src
cd ~/kthfsdv/
catkin build
```
___
Start two new terminal windows and respectively run:
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
___
You should now have three terminals in the container at ~/kthfsdv/
___

In one of the terminals get this content of exc1 by doing:
```
cd /src/
git clone https://github.com/markusjonek/formula_student.git
mv formula_student/exc1 ./
rm -r formula_student
cd ..
catkin build
source devel/setup.bash
```
___
In one of the terminals start the roscore:
```
source devel/setup.bash
roscore
```
___
In another terminal start checking the topic "kthfs/result":
```
source devel/setup.bash
rostopic echo kthfs/result
```
___
In the third terminal launch the nodes with:
```
cd /root/kthfsdv/src/exc1
roslaunch launch.launch
```
___
Then check the rostopic echo window if it is working.
