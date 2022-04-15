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

