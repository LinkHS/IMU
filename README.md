# IMU位姿计算

位置姿态的相关计算函数均位于`utilities.py`中  
- 姿态计算：`calc_orient`函数，包含了`Integrate`, `Mahony`, `Madgwick`三种计算方法。  
- 位置计算：`calc_posvel`函数，包含了零偏校正、高通/低通滤波、加速度阈值等功能。

## 用法示例
```bash
python3 test.py
```

## jupyter notebook
- Quat.ipynb  
  包含了四元数定义、计算等方法与示例
- IMU.ipynb  
  包含了IMU位姿计算的示例
- imu_position_estimation.ipynb
  Austin的位姿计算示例

## ROS相关（需安装ROS环境）
- recoder.py
  记录连接的IMU发布的话题
- read_bag.py
  使用python读取rosbag数据

