# Others
## Use ros-python in vscode
载入ros环境后（`source catkin_ws/devel/setup.bash`），查看ros-python环境（`echo PYTHONPATH`），将类似下面内容添加到`$WORKSPACE/.env`中

```bash
PYTHONPATH=/home/algo/catkin_ws/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages
```