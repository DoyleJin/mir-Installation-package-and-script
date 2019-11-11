# Mir-UR5相关ROS包安装

> **install_script_1.0**为完整版，包括ros，Mir，Kinect，UR5,气动夹爪以及ORBSlam包
> **install_script_2.0**为精简版，只安装Mir和Kinect包

## 安装步骤

对于全新的操作系统，将安装包解压至任意路径下，在该路径下打开终端

在运行前请将软件安装源修改为清华源

1. 修改**.sh**文件权限

```shell
$ chmod +x install_script_1.0.sh
```

2. 运行**.sh**文件

```shell
$ ./ininstall_script_1.0.sh
```

`ps:`

- 若想改变安装位置等信息，修改**.sh**文件中地址相关信息即可。

- **Mir-UR5**移动平台的底层集成开发工作均由北京智科特有限公司[I Quotient Robotics](https://github.com/I-Quotient-Robotics)完成，特此鸣谢杨飞潺、[田野](https://github.com/flymaxty)以及 [闫磊](https://github.com/QuartzYan) 等多位工程师给予的宝贵技术支持