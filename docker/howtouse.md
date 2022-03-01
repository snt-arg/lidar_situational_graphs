# s_graphs

Original repository: https://github.com/koide3/s_graphs


## Build
```bash
cd s_graphs/docker
./build.sh
```

## Run

### On host:
```bash
roscore
```

```bash
rosparam set use_sim_time true

cd s_graphs/rviz
rviz -d s_graphs.rviz
```

```bash
rosbag play --clock hdl_400.bag
```
http://www.aisl.cs.tut.ac.jp/databases/s_graphs/hdl_400.bag.tar.gz

### On docker image:
```bash
cd s_graphs/docker
./run.sh

roslaunch s_graphs s_graphs_400.launch
```


![s_graphs](https://user-images.githubusercontent.com/31344317/98347836-4fed5a00-205b-11eb-931c-158f6cd056bf.gif)
