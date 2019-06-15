# xm_controllers

## Create Date: 2017.9.1

## Function: 
晓萌控制器代码

## Package:
1.base_simple_controller:底盘控制器，实现速度的更新和里程计数据的更新
2.gripper_controller:爪子控制器，主要是为moveit提供接口，但是如果moveit爪子不作为一个规划组，这个控制器也不是非要不可。。。
  不过这个控制器是我按照ros官方控制器改来的一个简化版本，如果想实现对末端执行器更加精确的控制，如加入力矩控制等，可以参考一下
3.motor_controller:这个控制器针对升降电机写的，因为相同的控制器接口，使用机械臂的控制器，会导致升降抖动得很厉害，所以去掉了速度平滑，重新实现了一个
  控制器

## Summary:
  所有控制器都以插件的形式存在，最后由control_manager统一更新

## Use:
  详见handsfree_hw启动文件
