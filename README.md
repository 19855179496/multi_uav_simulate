# 多机仿真操作指南
# 1.px4\_autopilot、Fast\_Auto\_Mav的安装
   1. 按照54所那个文件安装
<img width="107" height="140" alt="图片" src="https://github.com/user-attachments/assets/39a8006a-eb3e-43d2-aecb-3489d42310d5" />

# 2.px4里的launch文件的修改，修改成多机的形式
   1. 多机启动，包含mavros通信部分的启动multi\_kp880.launch，它会调用single\_vehicle\_spawn.launch文件（这个文件里面调用的无人机模型已经被我改成了调用模板sdf）
      1. 重要：端口号要严格执行，否则没数据
<img width="349" height="132" alt="图片" src="https://github.com/user-attachments/assets/6cf2459d-8bd9-4aea-ad97-22eeedfa0742" />

   2. 修改kp880.sdf变成模板类型kp880.sdf.jina，这样的话可以针对不同无人机修改相应参数
<img width="331" height="49" alt="图片" src="https://github.com/user-attachments/assets/015423e7-7cca-4310-8ff3-e101ba34083b" />

# 3.使用指令使得mavlink通信频率提高，提升imu频率
```yaml
cd /home/hr/aaworkspace/simulate/Fast_Auto_Mav
source ./devel/setup.zsh
sh raise_imu_rate.sh
```
<img width="334" height="59" alt="图片" src="https://github.com/user-attachments/assets/dd8f7bfc-3dbe-47ec-86e8-5c7b2f30e74e" />
