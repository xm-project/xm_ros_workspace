#xm_smach

Create Date: 2017.9.1

Function: 
晓萌控制策略代码，通过statemachine来组织晓萌的行为

Folder:
  1.launch:启动文件
  2.scripts:状态机代码
  3.smach_lib:状态机类
  4.smach_log:状态机编写流程
  5.tests:测试用状态机
Summary:
  对状态机代码中常用的类实现了封装，方便以后使用。

Use:
  roscd xm_smach/launch
  whoiswho:
    bash whoiswho.bash
  gpsr:
    bash gpsr.bash
