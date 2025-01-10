## 安装方式

clone本仓库代码到ros工作空间中

编译

```bash
catkin_make
source devel/setup.sh
```

运行

```bash
roslaunch astar_path_planner astar_planner.launch
```

当编译无报错、rviz显示随机生成圆柱障碍物和绿色直线初始路径时表明编译安装成功

## 第三方库
osqp（安装需要指定版本以免报错），OsqpEigen

git clone --recursive -b release-0.6.3 https://github.com/oxfordcontrol/osqp.git
cd osqp
mkdir build && cd build
cmake .. 
make
sudo make install

参考原文链接：https://blog.csdn.net/tugepaopaoo/article/details/131178584
