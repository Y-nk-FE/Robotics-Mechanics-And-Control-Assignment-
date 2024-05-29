%% Solution.m
%
%  文件说明(1-6行)：
%  文件一共分为两个部分，分别使第一部分、第二部分
%  第一部分(7-45行)：函数的调用，对对应的题目进行求解
%  第二部分(46-1323行)：函数的定义，对第一部分需要使用到的所有进行定义
%% Part 1.函数的调用，对对应的题目进行求解
% 清除工作区和命令行窗口
clc;
clear;
% Solve Qustion 2
% 计算并且绘制出关节的位置、速度、加速度曲线（关节空间下）
% Plot_Posotion_Velocity_AcceleratedVelocity_Curve()

% Solve Question 3
% 绘制机械臂末端的位置和姿态
% Get_Pose_Trace_of_the_end_of_Robot_R()

% Solve Question 4
% 绘制机械臂末端位置轨迹（笛卡尔空间下）
% Plot_End_Position_InXYZ()
% 绘制机械臂末端速度轨迹（笛卡尔空间下）
% Plot_End_velocity_InXYZ()

% Solve Question 5
% PD控制器进行轨迹跟踪（正运动学）
% PD_Controllor_For_Trace_Forward()

% Solve Qustion 6
% PD控制器进行轨迹跟踪（逆运动学）
% PD_Controllor_For_Trace_Reverse()

% Solve Question 7
% 求解函数参数
% Solve_For_Function2()
% Trajectory_Planning_2()
% Plot_For_Trajectory_Planning_2
% 阻抗控制
% Impedance_Control();

% Solve Question 8
% Get Disturb Data
% Get_Disturb_Data()
Impedance_Control_With_Disturb()

%% Part 2.函数的定义-各种功能函数的定义
%--------------------------Function Definition-----------------------------
% function 1：计算位置坐标
function Data_Position=Calculate_Robot_Position()
    time=0:0.001:10;
    Data_Position=zeros(10001,4);
    index = 1;
    for t=time
        % Position(t)
        [q1,q2,q3]=Position_Calculate_Function(t);
        Data_Position(index,:)=[t,q1,q2,q3];
        index = index + 1;
    end
end

% function 2: 绘制关节空间下的位置、速度、加速度关于时间t的函数图像
function []=Plot_Posotion_Velocity_AcceleratedVelocity_Curve() 
    Joint_Data = Get_Joint_Data()
    % ---------------subplot绘图-----------------

    % ----------绘制位置--------
    figure(1)
    hold on
    title("第1关节位置-q1")
    ylabel("旋转角度/°")
    xlabel("时间/s")
    plot(Joint_Data(:,1),Joint_Data(:,2))
    hold off

    figure(2)
    hold on
    title("第2关节位置-q2")
    ylabel("旋转角度/°")
    xlabel("时间/s")
    plot(Joint_Data(:,1),Joint_Data(:,3))
     hold off

    figure(3)
    hold on
    title("第2关节位置-q2")
    ylabel("平移距离/m")
    xlabel("时间/s")
    plot(Joint_Data(:,1),Joint_Data(:,4))
    hold off

    % ----------------绘制速度-----------------------
    figure(4)
    hold on
    title("第1关节速度-v1")
    ylabel("旋转角度/(°/s)")
    xlabel("时间/s")
    plot(Joint_Data(:,1),Joint_Data(:,5))
    hold off

    figure(5)
    hold on
    title("第2关节速度-v2")
    ylabel("旋转角度/(°/s)")
    xlabel("时间/s")
    plot(Joint_Data(:,1),Joint_Data(:,6))
    hold off

    figure(6)
    hold on
    title("第2关节速度-v3")
    ylabel("平移距离/(m/s)")
    xlabel("时间/s")
    plot(Joint_Data(:,1),Joint_Data(:,7))
    hold off

    % ---------------绘制加速度----------------------

    figure(7)
    hold on
    title("第1关节加速度-a1")
    ylabel("旋转角度/(°/s^2)")
    xlabel("时间/s")
    plot(Joint_Data(:,1),Joint_Data(:,8))
    hold off

    figure(8)
    hold on
    title("第2关节加速度-a2")
    ylabel("旋转角度/(°/s^2)")
    xlabel("时间/s")
    plot(Joint_Data(:,1),Joint_Data(:,9))
    hold off

    figure(9)
    hold on
    title("第2关节加速度-a3")
    ylabel("平移距离/(m/s^2)")
    xlabel("时间/s")
    plot(Joint_Data(:,1),Joint_Data(:,10))
    hold off
    
end

% function 3: 计算位置的函数
function [q1,q2,q3]=Position_Calculate_Function(t)
    if t<=5
        q1=(12/5)*t^2-(8/25)*t^3;
        q2=(24/5)*t^2-(16/25)*t^3;
        q3=(3/250)*t^2-(1/625)*t^3;
    else
        q1=(6/5)*(t-5)^2-(4/25)*(t-5)^3+20;
        q2=(12/5)*(t-5)^2-(8/25)*(t-5)^3+40;
        q3=(3/500)*(t-5)^2-(1/1250)*(t-5)^3+0.1;
    end     
end

% function 4: 计算速度的函数
function [v1,v2,v3]=Velocity_Calculate_Function(t)
    if t<=5
        v1=(24/5)*t-(24/25)*t^2;
        v2=(48/5)*t-(48/25)*t^2;
        v3=(3/125)*t-(3/625)*t^2;
        
    else
        v1=(12/5)*(t-5)-(12/25)*(t-5)^2;
        v2=(24/5)*(t-5)-(24/25)*(t-5)^2;
        v3=(3/250)*(t-5)-(3/1250)*(t-5)^2;
    end
end

% function 5: 计算加速度
function [a1,a2,a3]=Accelerated_Velocity_Calculate_Function(t)
    if t<=5
        a1=(24/5)-(48/25)*t;
        a2=(48/5)-(96/25)*t;
        a3=(3/125)-(6/625)*t;
    else
        a1=(12/5)-(24/25)*(t-5);
        a2=(24/5)-(48/25)*(t-5);
        a3=(3/250)-(3/625)*(t-5);
    end
end

% function 6: 绘制机械臂末端的位姿T
function []=Get_Pose_Trace_of_the_end_of_Robot_T()
    % robot base data
    Joint_Position = Calculate_Robot_Position();
    % 旋转关节-角度制
    EndPose = zeros(4,4,1000);
    index=1;
    for i=1:10:10001
        theta1=Joint_Position(i,2);
        theta2=Joint_Position(i,3);
        d3=Joint_Position(i,4);
        T = Get_T_Matrix(theta1,theta2,d3);
        EndPose(:,:,index)=T;
        index=index+1;
    end
    view(3)
    tranimate(EndPose,'axis',[0,250,0,200,-100,0])
end

% function 7:计算矩阵T
function T03=Get_T_Matrix(theta1,theta2,d3)
    % 旋转关节-转为弧度制
    theta1 = pi*(theta1/180);
    theta2 = pi*(theta2/180);
    % 平移关节
    d3=1*d3;
    % 连杆长度
    l1=100;
    l2=120;
    T03 = [cos(theta1+theta2) sin(theta1+theta2) 0 cos(theta1+theta2)*l2+cos(theta1)*l1;
            sin(theta1+theta2) -cos(theta1-theta2) 0 sin(theta1+theta2)*l2+sin(theta1)*l1;
            0 0 -1 -1*d3;
            0 0 0 1];
end

% function 8：计算矩阵R
function R03=Get_R_Matrix(theta1,theta2,d3)% 旋转关节-转为弧度制
    theta1 = pi*(theta1/180);
    theta2 = pi*(theta2/180);
    % 平移关节
    d3=1*d3;
    % 连杆长度
    l1=100;
    l2=120;
    R03 = [cos(theta1+theta2) sin(theta1+theta2) 0 ;
            sin(theta1+theta2) -cos(theta1-theta2) 0 ;
            0 0 -1 ];
end

% function 9:绘制机械臂末端的位姿R
function []=Get_Pose_Trace_of_the_end_of_Robot_R()
    % robot base data
    Joint_Position = Calculate_Robot_Position();
    % 旋转关节-角度制
    EndPose = zeros(3,3,100);
    index=1;
    for i=1:10:1000
        theta1=Joint_Position(i,2);
        theta2=Joint_Position(i,3);
        d3=Joint_Position(i,4);
        R = Get_R_Matrix(theta1,theta2,d3);
        EndPose(:,:,index)=R;
        index=index+1;
    end
    view(3)
    tranimate(EndPose)
end

% function 10:绘制机械臂末端的轨迹曲线（基于笛卡尔坐标系）
function []=Plot_End_Position_InXYZ()
    clc;
    clear;
    Joint_Position = Calculate_Robot_Position();
    % 连杆长度
    l1=100;
    l2=120;
    PosXYZ=zeros(10001,4);
    dt=0.001;
    for i=1:1:10001
        % 获取数据
        theta1=Joint_Position(i,2);
        theta2=Joint_Position(i,3);
        d3=Joint_Position(i,4);
        % 转为弧度制
        theta1 = pi*(theta1/180);
        theta2 = pi*(theta2/180);
        x=cos(theta1+theta2)*l2+cos(theta1)*l1;
        y=sin(theta1+theta2)*l2+sin(theta1)*l1;
        z=-d3;
        PosXYZ(i,:)=[dt*(i-1),x,y,z];
    end
    subplot(3,1,1)
    hold on
    xlabel('时间/s')
    ylabel('X/m')
    title('X轴')
    plot(PosXYZ(:,1),PosXYZ(:,2));
    hold off

    subplot(3,1,2)
    hold on
    xlabel('时间/s')
    ylabel('Y/m')
    title('Y轴')
    plot(PosXYZ(:,1),PosXYZ(:,3));
    hold off

    subplot(3,1,3)
    hold on
    xlabel('时间/s')
    ylabel('Z/m')
    title('Z轴')
    plot(PosXYZ(:,1),PosXYZ(:,4));
    hold off
end

% function 11:绘制机械臂末端的速度曲线（基于笛卡尔坐标系）
function []=Plot_End_velocity_InXYZ()
    clc;
    clear;
    Joint_Position = Get_Joint_Data(); 
    % 连杆长度
    l1=100;
    l2=120;
    VeloXYZ=zeros(1001,4);
    for i=2:1:1001
        % 获取数据
        theta1=Joint_Position(i,2);theta1_fore=Joint_Position(i-1,2);
        theta2=Joint_Position(i,3);theta2_fore=Joint_Position(i-1,3);
        d3=Joint_Position(i,4);d3_fore=Joint_Position(i-1,4);
        % 转为弧度制
        theta1 = pi*(theta1/180);theta1_fore=pi*(theta1_fore/180);
        theta2 = pi*(theta2/180);theta2_fore=pi*(theta2_fore/180);
        % delta
        delta_theta1 = theta1-theta1_fore;
        delta_theta2 = theta2-theta2_fore;
        delta_d3 = d3-d3_fore;
        x=(-sin(theta1+theta2)*l2-sin(theta1)*l1)*delta_theta1-(sin(theta1+theta2)*l2)*delta_theta2;
        y=(cos(theta1+theta2)*l2+cos(theta1)*l1)*delta_theta1+(cos(theta1+theta2)*l2)*delta_theta2;
        z=-1*delta_d3;
        VeloXYZ(i,:)=[i/100,x,y,z];
    end
    subplot(1,3,1)
    hold on
    xlabel('时间/s')
    ylabel('X/(m/s)')
    title('X轴')
    plot(VeloXYZ(:,1),VeloXYZ(:,2));
    hold off

    subplot(1,3,2)
    hold on
    xlabel('时间/s')
    ylabel('Y/(m/s)')
    title('Y轴')
    plot(VeloXYZ(:,1),VeloXYZ(:,3));
    hold off

    subplot(1,3,3)
    hold on
    xlabel('时间/s')
    ylabel('Z/(m/s)')
    title('Z轴')
    plot(VeloXYZ(:,1),VeloXYZ(:,4));
    hold off
end

% function 12:得到关节空间的的信息
function Joint_Data = Get_Joint_Data()
    Joint_Data=zeros(10001,10);
    % 计算位置
    dt=0.001;
    for i=1:10001
        t=dt*(i-1);
        % Position(t)
        [q1,q2,q3]=Position_Calculate_Function(t);
        % Velocity(t)
        [v1,v2,v3]=Velocity_Calculate_Function(t);
        % Accelerated_Velocity(t)
        [a1,a2,a3]=Accelerated_Velocity_Calculate_Function(t);
        Joint_Data(i,:)=[t,q1,q2,q3,v1,v2,v3,a1,a2,a3];
    end
end

% function 13：获得笛卡尔坐标系下末端位置坐标的数据
function PosXYZ=Get_End_Pos_InXYZ()
    %Joint_Position = Calculate_Robot_Position();
    Joint_Position = Get_Joint_Data();
    % 连杆长度（cm）
    l1=100;
    l2=120;
    PosXYZ=zeros(10001,4);
    dt = 0.001;
    for i=1:1:10001
        % 获取数据
        theta1=Joint_Position(i,2);
        theta2=Joint_Position(i,3);
        d3=Joint_Position(i,4);
        % 转为弧度制
        theta1 = pi*(theta1/180);
        theta2 = pi*(theta2/180);
        % 计算末端位置
        x=cos(theta1+theta2)*l2+cos(theta1)*l1;
        y=sin(theta1+theta2)*l2+sin(theta1)*l1;
        z=-d3;
        % 存储末端位置
        PosXYZ(i,:)=[dt*(i-1),x,y,z];
    end
end

% function 14：PD控制器进行轨迹跟踪
function []=PD_Controllor_For_Trace_Forward()
    % 获得期望下关节空间的信息
    Exp_Joint_Data = Get_Joint_Data();
    % 获取期望的轨迹（关节空间下）
    Exp_Pos_Trace_Joint = [Exp_Joint_Data(:,1) ,Exp_Joint_Data(:,2:4)];
    % 获得期望的速度（关节空间下）
    Exp_Velocity_Joint = [Exp_Joint_Data(:,1) ,Exp_Joint_Data(:,5:7)];
    % 获得期望的加速度
    Exp_Accelerate_Velocity_Joint = [Exp_Joint_Data(:,1) ,Exp_Joint_Data(:,8:10)];

    % PD-Controllor-parameter(kp,kd,dt)
    kp = 200;
    kd = 10;
    delta_t = 0.001;
    % input_v
    current_input_v = zeros(1,3);
    next_input_v = zeros(1,3);

    % Pos_error
    delta_pos_i = zeros(1,3);
    delta_pos_i_1 = zeros(1,3);
    delta_pos_i_2 = zeros(1,3);

    % real Position
    Real_Pos_Joint = zeros(10001,4);
    Real_Velocity_Joint = zeros(10001,4);
    Real_Accelerate_Velocity_Joint = zeros(10001,4);
    % 列向量

    % trace by PD
    for i=1:10001
        % Input-Joint-v
        % Output-Joint-Pos
        % 记录速度
        Real_Velocity_Joint(i,:)=[0.001*i,current_input_v];
        % calculate the real position
        if i==1
            Real_Pos_Joint(i,1)=0;
            Real_Pos_Joint(i,2:4)=current_input_v*delta_t;
        else
            Real_Pos_Joint(i,1)=0.001*i;
            Real_Pos_Joint(i,2:4)=Real_Pos_Joint(i-1,2:4)+current_input_v*delta_t;
        end
        % Pos_delta(1,3)
        delta_pos= Exp_Pos_Trace_Joint(i,2:4)-Real_Pos_Joint(i,2:4);
        if i==1
            delta_pos_i =delta_pos;
        elseif i==2
            delta_pos_i_1 = delta_pos_i;
            delta_pos_i =delta_pos;
        else
            delta_pos_i_2 = delta_pos_i_1;
            delta_pos_i_1 = delta_pos_i;
            delta_pos_i =delta_pos;
        end

        delta_v = kp*(delta_pos_i-delta_pos_i_2)+kd*(delta_pos_i_1-2*delta_pos_i_1+delta_pos_i_2);
        
        % 计算出下一时刻的速度
        next_input_v = current_input_v + delta_v;
        % 加速度
        Real_Accelerate_Velocity_Joint(i,:)=[0.001*i,delta_v/delta_t];
        % 更新速度
        current_input_v = next_input_v;
    end
    Plot_for_PD(Exp_Pos_Trace_Joint,Real_Pos_Joint,Exp_Velocity_Joint,Real_Velocity_Joint,Exp_Accelerate_Velocity_Joint,Real_Accelerate_Velocity_Joint)
end

% function 15：绘制期望与实际的图像
function []=Plot_for_PD(Exp_Pos_Joint,Real_Pos_Joint,Exp_Velocity_Joint,Real_Velocity_Joint,Exp_Accelerate_Velocity_Joint,Real_Accelerate_Velocity_Joint)
    % Position Trace
    Plot_for_PD_inJoint_Pos(Exp_Pos_Joint,Real_Pos_Joint);
    % Velocity Trace
    Plot_for_PD_inJoint_Velocity(Exp_Velocity_Joint,Real_Velocity_Joint);
    % Accelerate  Velocity Trace
    Plot_for_PD_inJoint_Accelerate_Velocity(Exp_Accelerate_Velocity_Joint,Real_Accelerate_Velocity_Joint);
end

% function 16: 绘制期望与实际（关节空间下的位置跟踪）
function []=Plot_for_PD_inJoint_Pos(Exp_Pos_Joint,Real_Pos_Joint)
    % 绘制位置跟踪
    subplot(3,3,1)
    hold on
    plot(Exp_Pos_Joint(:,1),Exp_Pos_Joint(:,2))
    plot(Real_Pos_Joint(:,1),Real_Pos_Joint(:,2))
    title("Joint{1}-Pos")
    ylabel("旋转角度/°")
    xlabel("时间/s")
    legend('Exp','Real')
    subplot(3,3,2)
    hold on
    plot(Exp_Pos_Joint(:,1),Exp_Pos_Joint(:,3))
    plot(Real_Pos_Joint(:,1),Real_Pos_Joint(:,3))
    title("Joint{2}-Pos")
    legend('Exp','Real')
    ylabel("旋转角度/°")
    xlabel("时间/s")
    subplot(3,3,3)
    hold on
    plot(Exp_Pos_Joint(:,1),Exp_Pos_Joint(:,4))
    plot(Real_Pos_Joint(:,1),Real_Pos_Joint(:,4))
    title("Joint{3}-Pos")
    legend('Exp','Real')
    ylabel("平移距离/m")
    xlabel("时间/s")
end

% function 17: 绘制期望与实际（关节空间下的速度跟踪）
function []=Plot_for_PD_inJoint_Velocity(Exp_Velocity_Joint,Real_Velocity_Joint)
    % 绘制位置跟踪
    subplot(3,3,4)
    hold on
    plot(Exp_Velocity_Joint(:,1),Exp_Velocity_Joint(:,2))
    plot(Real_Velocity_Joint(:,1),Real_Velocity_Joint(:,2))
    title("Joint{1}-velocity")
    ylabel("旋转角速度/(°/s)")
    xlabel("时间/s")
    legend('Exp','Real')
    subplot(3,3,5)
    hold on
    plot(Exp_Velocity_Joint(:,1),Exp_Velocity_Joint(:,3))
    plot(Real_Velocity_Joint(:,1),Real_Velocity_Joint(:,3))
    title("Joint{2}-velocity")
    legend('Exp','Real')
    ylabel("旋转角速度/(°/s)")
    xlabel("时间/s")
    subplot(3,3,6)
    hold on
    plot(Exp_Velocity_Joint(:,1),Exp_Velocity_Joint(:,4))
    plot(Real_Velocity_Joint(:,1),Real_Velocity_Joint(:,4))
    title("Joint{3}-velocity")
    legend('Exp','Real')
    ylabel("平移速度/(m/s)")
    xlabel("时间/s")
end

% function 18: 绘制期望与实际（关节空间下的速度跟踪）
function []=Plot_for_PD_inJoint_Accelerate_Velocity(Exp_Accelerate_Velocity_Joint,Real_Accelerate_Velocity_Joint)
    % 绘制位置跟踪
    subplot(3,3,7)
    hold on
    plot(Exp_Accelerate_Velocity_Joint(:,1),Exp_Accelerate_Velocity_Joint(:,2))
    plot(Real_Accelerate_Velocity_Joint(:,1),Real_Accelerate_Velocity_Joint(:,2))
    title("Joint{1}-Accelerate Velocity")
    ylabel("旋转角加速度/(°/s^2)")
    xlabel("时间/s")
    legend('Exp','Real')
    subplot(3,3,8)
    hold on
    plot(Exp_Accelerate_Velocity_Joint(:,1),Exp_Accelerate_Velocity_Joint(:,3))
    plot(Real_Accelerate_Velocity_Joint(:,1),Real_Accelerate_Velocity_Joint(:,3))
    title("Joint{2}-Accelerate Velocity")
    legend('Exp','Real')
    ylabel("旋转角加速度/(°/s^2)")
    xlabel("时间/s")
    subplot(3,3,9)
    hold on
    plot(Exp_Accelerate_Velocity_Joint(:,1),Exp_Accelerate_Velocity_Joint(:,4))
    plot(Real_Accelerate_Velocity_Joint(:,1),Real_Accelerate_Velocity_Joint(:,4))
    title("Joint{3}-Accelerate Velocity")
    legend('Exp','Real')
    ylabel("平移加速度/(m/s^2)")
    xlabel("时间/s")
end

% function 19：PD控制器（逆运动学）
function []=PD_Controllor_For_Trace_Reverse()
    % 机器人的基本信息
    l1=100;
    l2=120;
    % 获得期望下关节空间的信息
    Exp_Joint_Data = Get_Joint_Data();
    % 获取期望的轨迹（关节空间下）
    Exp_Pos_Joint = [Exp_Joint_Data(:,1) ,Exp_Joint_Data(:,2:4)];
    % 获得期望的速度（关节空间下）
    Exp_Velocity_Joint = [Exp_Joint_Data(:,1) ,Exp_Joint_Data(:,5:7)];
    % 获得期望的加速度
    Exp_Accelerate_Velocity_Joint = [Exp_Joint_Data(:,1) ,Exp_Joint_Data(:,8:10)];
    % 期望机械臂末端的位置
    Exp_Pos_XYZ = Get_End_Pos_InXYZ();

    % PD-Controllor-parameter(kp,kd,dt)
    kp = 1000;
    kd = 400;
    dt = 0.001;
    % input_v
    current_joint_v = zeros(1,3);
    next_joint_v = zeros(1,3);

    % Pos_error
    delta_pos_i = zeros(1,3);
    delta_pos_i_1 = zeros(1,3);
    delta_pos_i_2 = zeros(1,3);

    % real Position
    Real_Pos_Joint = zeros(10001,4);
    Real_Velocity_Joint = zeros(10001,4);
    Real_Accelerate_Velocity_Joint = zeros(10001,4);
    Real_Pos_XYZ = zeros(10001,4);
    % 列向量

    % trace by PD
    for i=1:10001
        % Input-Joint-v
        % Output-Joint-Pos
        % 记录速度
        t = dt*(i-1);
        Real_Velocity_Joint(i,:)=[t,current_joint_v];
        
        % 关节空间下的关节位置
        if i==1
            Real_Pos_Joint(i,1)=0;
            Real_Pos_Joint(i,2:4)=current_joint_v*dt;
        else
            Real_Pos_Joint(i,1)=t;
            Real_Pos_Joint(i,2:4)=Real_Pos_Joint(i-1,2:4)+current_joint_v*dt;
        end
        
        % 逆运动学：从笛卡尔空间到关节空间
        % 计算笛卡尔空间的位置
        q1=Real_Pos_Joint(i,2);q2=Real_Pos_Joint(i,3);q3=Real_Pos_Joint(i,4);
        % 角度转弧度
        q1=pi*q1/180;q2=pi*q2/180;
        x=cos(q1+q2)*l2+cos(q1)*l1;
        y=sin(q1+q2)*l2+sin(q1)*l1;
        z=-q3;
        Real_Pos_XYZ(i,:)=[t,x,y,z];

        % Pos_delta(1,3)
        delta_pos= Exp_Pos_Joint(i,2:4)-Real_Pos_Joint(i,2:4);
        if i==1
            delta_pos_i =delta_pos;
        elseif i==2
            delta_pos_i_1 = delta_pos_i;
            delta_pos_i =delta_pos;
        else
            delta_pos_i_2 = delta_pos_i_1;
            delta_pos_i_1 = delta_pos_i;
            delta_pos_i =delta_pos;
        end

        delta_joint_v = kp*(delta_pos_i-delta_pos_i_2)+kd*(delta_pos_i_1-2*delta_pos_i_1+delta_pos_i_2);
        
        % 计算出下一时刻的速度
        next_joint_v = current_joint_v + delta_joint_v;
        % 加速度（关节）
        Real_Accelerate_Velocity_Joint(i,:)=[t,delta_joint_v/dt];
        % 更新速度（关节）
        current_joint_v = next_joint_v;
    end

    % 机械臂末端位置变化(机械臂基座的笛卡尔坐标系下)
    figure(1)
    subplot(3,1,1)
    hold on
    plot(Exp_Pos_XYZ(:,1),Exp_Pos_XYZ(:,2));
    plot(Real_Pos_XYZ(:,1),Real_Pos_XYZ(:,2));
    legend('exp','real')
    xlabel('时间/s')
    ylabel('位置/m')
    title('Pos-X')

    subplot(3,1,2)
    hold on
    plot(Exp_Pos_XYZ(:,1),Exp_Pos_XYZ(:,3));
    plot(Real_Pos_XYZ(:,1),Real_Pos_XYZ(:,3));
    legend('exp','real')
    xlabel('时间/s')
    ylabel('位置/m')
    title('Pos-Y')

    subplot(3,1,3)
    hold on
    plot(Exp_Pos_XYZ(:,1),Exp_Pos_XYZ(:,4));
    plot(Real_Pos_XYZ(:,1),Real_Pos_XYZ(:,4));
    legend('exp','real')
    xlabel('时间/s')
    ylabel('位置/m')
    title('Pos-Z')

    % 机械臂末端位置误差
    figure(2)
    subplot(3,1,1)
    hold on
    plot(Exp_Pos_XYZ(:,1),Exp_Pos_XYZ(:,2)-Real_Pos_XYZ(:,2));
    legend('error')
    xlabel('时间/s')
    ylabel('位置/m')
    title('Pos-X-Error')

    subplot(3,1,2)
    hold on
    plot(Exp_Pos_XYZ(:,1),Exp_Pos_XYZ(:,3)-Real_Pos_XYZ(:,3));
    legend('error')
    xlabel('时间/s')
    ylabel('位置/m')
    title('Pos-Y-Error')

    subplot(3,1,3)
    hold on
    plot(Exp_Pos_XYZ(:,1),Exp_Pos_XYZ(:,4)-Real_Pos_XYZ(:,4));
    legend('error')
    xlabel('时间/s')
    ylabel('位置/m')
    title('Pos-Z-Error')


    % 关节角度变化(期望与实际)
    % figure(3)
    % subplot(3,1,1)
    % hold on
    % plot(Exp_Pos_Joint(:,1),Exp_Pos_Joint(:,2));
    % plot(Real_Pos_Joint(:,1),Real_Pos_Joint(:,2));
    % plot(Real_Pos_Joint(:,1),Exp_Pos_Joint(:,2)-Real_Pos_Joint(:,2));
    % legend('exp','real','error')
    % subplot(3,1,2)
    % hold on
    % plot(Exp_Pos_Joint(:,1),Exp_Pos_Joint(:,3));
    % plot(Real_Pos_Joint(:,1),Real_Pos_Joint(:,3));
    % legend('exp','real')
    % subplot(3,1,3)
    % hold on
    % plot(Exp_Pos_Joint(:,1),Exp_Pos_Joint(:,4));
    % plot(Real_Pos_Joint(:,1),Real_Pos_Joint(:,4));
    % legend('exp','real')
    % 绘制实际角度变化
    % figure(4)
    % subplot(3,1,1)
    % plot(Real_Pos_Joint(:,1),Real_Pos_Joint(:,2));
    % subplot(3,1,2)
    % plot(Real_Pos_Joint(:,1),Real_Pos_Joint(:,3));
    % subplot(3,1,3)
    % plot(Real_Pos_Joint(:,1),Real_Pos_Joint(:,4));
    error_x=0;error_y=0;error_z=0;
    for i=1:10001
        error_x = error_x + Exp_Pos_XYZ(i,2)-Real_Pos_XYZ(i,2);
        error_y = error_y + Exp_Pos_XYZ(i,3)-Real_Pos_XYZ(i,3);
        error_z = error_z + Exp_Pos_XYZ(i,4)-Real_Pos_XYZ(i,4);
    end
    error_x=error_x/10000
    error_y=error_y/10000
    error_z=error_z/10000
end

% function 20：轨迹生成器2（两点间的轨迹规划）
function Planning_Data_PVA=Trajectory_Planning_Generator_2()
   % 规划时间：10s
   % 起始位置：q0-[0 0 0]
   % 期望位置：qd-[30 60 0.15]
   % 规划要求：初始和期望关节的速度、加速度均为0
   % 前提假设：没有最大速度限制
   % 所用模型：五次多项式 q(t) =>a0=0;a1=0;a2=0;
   % qi a0 a1 a2 a3    a4      a5
   % q1 0  0  0  3/10  -9/200  9/5000
   % …………运行function21即可得到所有参数
   Planning_Data_PVA = zeros(10001,10);
   dt = 0.001;
   for i=1:10001
       t = dt*(i-1);
       Pos_Joint = Planning_Pos_Formula(t);
       Vel_Joint = Planning_Velocity_Formula(t);
       Acc_Joint = Planning_Accelerate_Formula(t);
       % Planning_Data_PVA(i,:)=[t,Pos_Joint(1),Pos_Joint(2),Pos_Joint(3),Vel_Joint(1),Vel_Joint(2),Vel_Joint(3),Acc_Joint(1),Acc_Joint(2),Acc_Joint(3)];
       Planning_Data_PVA(i,:)=[t,Pos_Joint,Vel_Joint,Acc_Joint];
   end
end

% function 21:解方程，解析参数
function []=Solve_For_Function2()
    % 定义参数
    syms a3 a4 a5
    % 定义方程组
    eqns1 = [1000*a3+10000*a4+100000*a5==30,300*a3+4000*a4+50000*a5==0,60*a3+1200*a4+20000*a5==0];
    solve(eqns1)
    eqns2 = [1000*a3+10000*a4+100000*a5==60,300*a3+4000*a4+50000*a5==0,60*a3+1200*a4+20000*a5==0];
    solve(eqns2)
    eqns3 = [1000*a3+10000*a4+100000*a5==0.15,300*a3+4000*a4+50000*a5==0,60*a3+1200*a4+20000*a5==0];
    solve(eqns3)
end

% function 22:关节位置函数
function Real_Time_Postion=Planning_Pos_Formula(t)
    q1=(3/10)*t^3 +(-9/200)*t^4 +(9/5000)*t^5;
    q2=(3/5)*t^3 +(-9/100)*t^4 +(9/2500)*t^5;
    q3=(3/2000)*t^3 +(-9/40000)*t^4 +(9/1000000)*t^5;
    Real_Time_Postion = [q1,q2,q3];
end

% function 23：关节速度函数
function Real_Time_Velocity=Planning_Velocity_Formula(t)
    v1=(9/10)*t^2 +(-36/200)*t^3 +(45/5000)*t^4;
    v2=(9/5)*t^2 +(-36/100)*t^3 +(45/2500)*t^4;
    v3=(9/2000)*t^2 +(-36/40000)*t^3 +(45/1000000)*t^4;
    Real_Time_Velocity = [v1,v2,v3];
end

% function 24：关节加速度函数
function Real_Time_Accelerate=Planning_Accelerate_Formula(t)
    a1=(18/10)*t +(-108/200)*t^2 +(180/5000)*t^3;
    a2=(18/5)*t +(-108/100)*t^2 +(180/2500)*t^3;
    a3=(18/2000)*t +(-108/40000)*t^2 +(180/1000000)*t^3;
    Real_Time_Accelerate = [a1,a2,a3];
end

% function 25：对得到的位置、速度、加速度绘图
function []=Plot_For_Trajectory_Planning_2()
    Planning_Data_PVA = Trajectory_Planning_Generator_2();
    % Joint 1
    % Position Velocity Accelerate
    figure(1)
    subplot(3,1,1)
    hold on
    title("Joint 1 Position(t)")
    xlabel("时间t/s")
    ylabel("关节位置p/°")
    plot(Planning_Data_PVA(:,1),Planning_Data_PVA(:,2))
    subplot(3,1,2)
    hold on
    title("Joint 1 Velocity(t)")
    xlabel("时间t/s")
    ylabel("关节速度v/(°/s)")
    plot(Planning_Data_PVA(:,1),Planning_Data_PVA(:,5))
    subplot(3,1,3)
    hold on
    title("Joint 1 Accelerate(t)")
    xlabel("时间t/s")
    ylabel("关节加速度a/(°/s^2)")
    plot(Planning_Data_PVA(:,1),Planning_Data_PVA(:,8))
    
    % Joint 2
    % Position Velocity Accelerate
    figure(2)
    subplot(3,1,1)
    hold on
    title("Joint 2 Position(t)")
    xlabel("时间t/s")
    ylabel("关节位置p/°")
    plot(Planning_Data_PVA(:,1),Planning_Data_PVA(:,3))
    subplot(3,1,2)
    hold on
    title("Joint 2 Velocity(t)")
    xlabel("时间t/s")
    ylabel("关节速度v/(°/s)")
    plot(Planning_Data_PVA(:,1),Planning_Data_PVA(:,6))
    subplot(3,1,3)
    hold on
    title("Joint 2 Accelerate(t)")
    xlabel("时间t/s")
    ylabel("关节加速度a/(°/s^2)")
    plot(Planning_Data_PVA(:,1),Planning_Data_PVA(:,9))

    % Joint 3
    % Position Velocity Accelerate
    figure(3)
    subplot(3,1,1)
    hold on
    title("Joint 3 Position(t)")
    xlabel("时间t/s")
    ylabel("关节位置p/m")
    plot(Planning_Data_PVA(:,1),Planning_Data_PVA(:,4))
    subplot(3,1,2)
    hold on
    title("Joint 3 Velocity(t)")
    xlabel("时间t/s")
    ylabel("关节速度v/(m/s)")
    plot(Planning_Data_PVA(:,1),Planning_Data_PVA(:,7))
    subplot(3,1,3)
    hold on
    title("Joint 3 Accelerate(t)")
    xlabel("时间t/s")
    ylabel("关节加速度a/(m/s^2)")
    plot(Planning_Data_PVA(:,1),Planning_Data_PVA(:,10))
end


% function 26: 阻抗控制(位置&力混合控制)
% 要求：绘制出关节位置，速度，加速度，控制力矩曲线
% 目的：阻抗控制是通过控制方法使机械手末端呈现需要的刚性和阻尼
% 通常：
% 对于需要进行位置控制的自由度，要求大纲性，及表现出很硬的特性以确保位置控制精度
% 对于需要进行力控制的自由度，则要求在该方向有合适的阻抗特性，以满足接触力的要求
% 机械臂连杆的参数：
% l1=1.0m=100.0cm=1000.0mm
% l2=1.2m-120.0cm=1200.0mm
% 计算位置的时候为了计算精度，单位应该均统一为mm，但是在计算力(N)和力矩(N·m)的时候，需要转换为m
% 暂时无重力补偿
function []=Impedance_Control()
    % 机器人的物理模型参数
    m1=10;m2=10;m3=10;
    % 令惯性矩阵为m
    m = [m1 0 0;0 m2 0;0 0 m3];
    % 进行轨迹规划，获取规划信息，即期望信息（关节空间下）
    % 数据（10001，10）
    % 数据说明：[t,Px,Py,Pz,Vx,Vy,Vz,Ax,Ay,Az]
    Exp_Data_Joint = Trajectory_Planning_Generator_2();
    
    % 期望刚性KX
    K_X = [500 0 0;0 500 0;0 0 500];
    % 阻尼参数KB
    K_B = [50 0 0;0 50 0;0 0 30];
    % 时间步长
    dt = 0.001;

    % 记录控制过程中的实时数据（关节空间）
    % 数据存储格式
    % 1-t  
    % (2,3,4)-(P1,P2,P3) 
    % (5,6,7)-(V1,V2,V3)
    % (8,9,10)-(a1,a2,a3)
    % (11,12,13)-(T1,T2,T3)
    Real_Data_Joint=zeros(10001,13);

    % i时刻的位置误差
    Pos_Error_i = zeros(1,3);
    Pos_Error_i_1 = zeros(1,3);
    Pos_Error_i_2 = zeros(1,3);
    % Torque(i时刻，q1 q2 q3关节的力矩)
    Current_Torque_Joint = zeros(1,3);
    % 加速度
    Current_Accelerate_Joint=zeros(1,3);
    % 速度
    Current_Velocity_Joint= zeros(1,3);
    % 位置
    Current_Position_Joint=zeros(1,3);

    t=0;
    % 控制器
    for i=1:10001
        % 获取当前时间并保存
        t= dt*(i-1);
        Real_Data_Joint(i,1)=t;
        % 获取当前时刻，即i时刻的位置误差
        if i==1
            Pos_Error_i = Exp_Data_Joint(i,2:4);
        else
            Pos_Error_i = Exp_Data_Joint(i,2:4)-Real_Data_Joint(i-1,2:4);
        end
        % delta_torque (3,1)
        delta_torque_31 = K_X*(Pos_Error_i-Pos_Error_i_1)'+K_B*(Pos_Error_i-2*Pos_Error_i_1+Pos_Error_i_2)';
        % delta_torque (1,3)
        delta_torque_13 = delta_torque_31';
        % Current_Torque
        Current_Torque_Joint = Current_Torque_Joint + delta_torque_13;
        % Save the Current_Torque to Real_Data_Joint(i,11:13)
        Real_Data_Joint(i,11:13)=Current_Torque_Joint;

        % 计算加速度,保存加速度
        Current_Torque_31 = Current_Torque_Joint';
        Current_Accelerate_31 = inv(m) * Current_Torque_31;
        Current_Accelerate_Joint = Current_Accelerate_31';
        Real_Data_Joint(i,8:10) = Current_Accelerate_Joint;

        % 通过得到加速度计算新的速度,并保存下来
        Current_Velocity_Joint = Current_Velocity_Joint + Current_Accelerate_Joint*dt;
        Real_Data_Joint(i,5:7) = Current_Velocity_Joint;

        % 计算位移,并保存下来
        Current_Position_Joint = Current_Position_Joint + Current_Velocity_Joint * dt;
        Real_Data_Joint(i,2:4) = Current_Position_Joint;

        % 更新误差
        Pos_Error_i_2 = Pos_Error_i_1;
        Pos_Error_i_1 = Pos_Error_i;
    end
    % plot(Real_Data_joint(:,1),Real_Data_joint(:,2))
    Plot_For_Impedance_Controller_Resault(Exp_Data_Joint,Real_Data_Joint)
end

% function 27: 为阻抗控制器的结果进行图像绘制
% 内容说明：对实际运动过程的位置、速度、加速度、关节扭矩输出进行绘制，并且与期望值进行比较
% 函数调用位置：在阻抗控制器计算完成后进行绘图
% 传入参数说明：期望值数据(10001,10)、真实值数据(10001,13)
function Plot_For_Impedance_Controller_Resault(Exp_Data_Joint,Real_Data_Joint)
    % 关节一（位置、速度、加速度、力矩）
    figure(1)
    subplot(2,2,1) % 期望位置&实际位置
    hold on
    title("Joint-1 Position Compare")
    xlabel("时间/s")
    ylabel("关节位置/°")
    plot(Exp_Data_Joint(:,1),Exp_Data_Joint(:,2))
    plot(Real_Data_Joint(:,1),Real_Data_Joint(:,2))
    legend("Exp","Real")
    subplot(2,2,2)
    hold on
    title("Joint-1 Velocity Compare")
    xlabel("时间/s")
    ylabel("关节角速度/(°/s)")
    plot(Exp_Data_Joint(:,1),Exp_Data_Joint(:,5))
    plot(Real_Data_Joint(:,1),Real_Data_Joint(:,5))
    legend("Exp","Real")
    subplot(2,2,3)
    hold on
    title("Joint-1 Accelerate Compare")
    xlabel("时间/s")
    ylabel("关节角加速度/(°/s^2)")
    plot(Exp_Data_Joint(:,1),Exp_Data_Joint(:,8))
    plot(Real_Data_Joint(:,1),Real_Data_Joint(:,8))
    legend("Exp","Real")
    subplot(2,2,4)
    hold on
    title("Joint-1 Torque Output")
    xlabel("时间/s")
    ylabel("关节输出力矩/(N·m)")
    plot(Real_Data_Joint(:,1),Real_Data_Joint(:,11))
    legend("Real")

    % 关节二（位置、速度、加速度、力矩）
    figure(2)
    subplot(2,2,1)
    hold on
    title("Joint-2 Position Compare")
    xlabel("时间/s")
    ylabel("关节位置/°")
    plot(Exp_Data_Joint(:,1),Exp_Data_Joint(:,3))
    plot(Real_Data_Joint(:,1),Real_Data_Joint(:,3))
    legend("Exp","Real")
    subplot(2,2,2)
    hold on
    title("Joint-2 Velocity Compare")
    xlabel("时间/s")
    ylabel("关节角速度/(°/s)")
    plot(Exp_Data_Joint(:,1),Exp_Data_Joint(:,6))
    plot(Real_Data_Joint(:,1),Real_Data_Joint(:,6))
    legend("Exp","Real")
    subplot(2,2,3)
    hold on
    title("Joint-2 Accelerate Compare")
    xlabel("时间/s")
    ylabel("关节角加速度/(°/s^2)")
    plot(Exp_Data_Joint(:,1),Exp_Data_Joint(:,9))
    plot(Real_Data_Joint(:,1),Real_Data_Joint(:,9))
    legend("Exp","Real")
    subplot(2,2,4)
    hold on
    title("Joint-2 Torque Output")
    xlabel("时间/s")
    ylabel("关节输出力矩/(N·m)")
    plot(Real_Data_Joint(:,1),Real_Data_Joint(:,12))
    legend("Real")

    % 关节三（位置、速度、加速度、力矩）
    figure(3)
    subplot(2,2,1)
    hold on
    title("Joint-3 Position Compare")
    xlabel("时间/s")
    ylabel("关节位置/m")
    plot(Exp_Data_Joint(:,1),Exp_Data_Joint(:,4))
    plot(Real_Data_Joint(:,1),Real_Data_Joint(:,4))
    legend("Exp","Real")
    subplot(2,2,2)
    hold on
    title("Joint-2 Velocity Compare")
    xlabel("时间/s")
    ylabel("关节角速度/(m/s)")
    plot(Exp_Data_Joint(:,1),Exp_Data_Joint(:,7))
    plot(Real_Data_Joint(:,1),Real_Data_Joint(:,7))
    legend("Exp","Real")
    subplot(2,2,3)
    hold on
    title("Joint-3 Accelerate Compare")
    xlabel("时间/s")
    ylabel("关节角加速度/(m/s^2)")
    plot(Exp_Data_Joint(:,1),Exp_Data_Joint(:,10))
    plot(Real_Data_Joint(:,1),Real_Data_Joint(:,10))
    legend("Exp","Real")
    subplot(2,2,4)
    hold on
    title("Joint-3 Torque Output")
    xlabel("时间/s")
    ylabel("关节输出力矩/(N)")
    plot(Real_Data_Joint(:,1),Real_Data_Joint(:,13))
    legend("Real")
end


% function 28:系统施加外力下的阻抗控制
% 说明：系统在4~6s内添加了外力[2N·m 2N·m 3N]
% 假设：机械臂感受到的外力由每个关节上的力矩传感器感知
function []=Impedance_Control_With_Disturb()
    % 机器人的物理模型参数
    m1=10;m2=10;m3=10;
    % 令惯性矩阵为m
    m = [m1 0 0;0 m2 0;0 0 m3];
    % 进行轨迹规划，获取规划信息，即期望信息（关节空间下）
    % 数据（10001，10）
    % 数据说明：[t,Px,Py,Pz,Vx,Vy,Vz,Ax,Ay,Az]
    Exp_Data_Joint = Trajectory_Planning_Generator_2();
    
    % 期望刚性KX
    K_X = [500 0 0;0 500 0;0 0 500];
    % 阻尼参数KB
    K_B = [50 0 0;0 50 0;0 0 30];
    % 时间步长
    dt = 0.001;

    % 记录控制过程中的实时数据（关节空间）
    % 数据存储格式
    % 1-t  
    % (2,3,4)-(P1,P2,P3) 
    % (5,6,7)-(V1,V2,V3)
    % (8,9,10)-(a1,a2,a3)
    % (11,12,13)-(T1,T2,T3)
    Real_Data_Joint=zeros(10001,13);

    % i时刻的位置误差
    Pos_Error_i = zeros(1,3);
    Pos_Error_i_1 = zeros(1,3);
    Pos_Error_i_2 = zeros(1,3);
    % Torque(i时刻，q1 q2 q3关节的力矩)
    Current_Torque_Joint = zeros(1,3);
    % 加速度
    Current_Accelerate_Joint=zeros(1,3);
    % 速度
    Current_Velocity_Joint= zeros(1,3);
    % 位置
    Current_Position_Joint=zeros(1,3);

    % 内环PID需要的参数
    % 传感器感知到的力矩大小(10001,4)
    Moment_Sensor_Joint = Get_Disturb_Data();
    % 期望输出力矩
    Exp_Moment_Joint = zeros(1,3);
    % 当前输出力矩
    Current_Moment_Joint = zeros(1,3);
    % 当前传感器的数值
    Current_Sensor_Joint = zeros(1,3);
    % 当前时刻力矩的输出误差、前一时刻的误差、前两个时刻的误差
    Moment_Error_i = zeros(1,3);
    Moment_Error_i_1 = zeros(1,3);
    Moment_Error_i_2 = zeros(1,3);
    % 内环PID的参数
    Moment_Kp = [0.01 0 0;0 0.01 0;0 0 0.001];
    Moment_Kd = [0.01 0 0;0 0.01 0;0 0 0.001];

    % 控制器
    for i=1:10001
        % 获取当前时间并保存
        t= dt*(i-1);
        Real_Data_Joint(i,1)=t;
        % 获取当前时刻，即i时刻的位置误差
        if i==1
            Pos_Error_i = Exp_Data_Joint(i,2:4);
        else
            Pos_Error_i = Exp_Data_Joint(i,2:4)-Real_Data_Joint(i-1,2:4);
        end
        % 外环PID
        % delta_torque (3,1)
        delta_torque_31 = K_X*(Pos_Error_i-Pos_Error_i_1)'+K_B*(Pos_Error_i-2*Pos_Error_i_1+Pos_Error_i_2)';
        % delta_torque (1,3)
        delta_torque_13 = delta_torque_31';
        % Current_Torque
        Current_Torque_Joint = Current_Torque_Joint + delta_torque_13;


        % 内环PID
        Current_Moment_Joint = Current_Torque_Joint;
        % 传感器
        dissturb = Moment_Sensor_Joint(i,2:4);
        % 计算当前误差
        Moment_Error_i = -1* dissturb;
        % 通过PID重新调整输出力矩（采用增量式PID）
        delta_moment_31 = Moment_Kp*(Moment_Error_i - Moment_Error_i_1)'+Moment_Kd*(Moment_Error_i - 2*Moment_Error_i_1 + Moment_Error_i_2)';
        % 转置
        delta_moment_13 = delta_moment_31';
        % 更新输出力矩
        Current_Moment_Joint = Current_Moment_Joint + delta_moment_13;
        % 更新误差
        Moment_Error_i_2 = Moment_Error_i_1;
        Moment_Error_i_1 = Moment_Error_i;
        % 将内环输出传递给下一个环节
        Current_Torque_Joint = Current_Moment_Joint;

        % Save the Current_Torque to Real_Data_Joint(i,11:13)
        Real_Data_Joint(i,11:13)=Current_Torque_Joint;

        

        % 计算加速度,保存加速度
        Current_Torque_31 = Current_Torque_Joint';
        Current_Accelerate_31 = inv(m) * Current_Torque_31;
        Current_Accelerate_Joint = Current_Accelerate_31';
        Real_Data_Joint(i,8:10) = Current_Accelerate_Joint;

        % 通过得到加速度计算新的速度,并保存下来
        Current_Velocity_Joint = Current_Velocity_Joint + Current_Accelerate_Joint*dt;
        Real_Data_Joint(i,5:7) = Current_Velocity_Joint;

        % 计算位移,并保存下来
        Current_Position_Joint = Current_Position_Joint + Current_Velocity_Joint * dt;
        Real_Data_Joint(i,2:4) = Current_Position_Joint;

        % 更新误差
        Pos_Error_i_2 = Pos_Error_i_1;
        Pos_Error_i_1 = Pos_Error_i;
    end
    % plot(Real_Data_joint(:,1),Real_Data_joint(:,2))
    Plot_For_Impedance_Controller_Resault_With_Disturb(Exp_Data_Joint,Real_Data_Joint,Moment_Sensor_Joint)
end

%function 29：设置系统施加的外力
function Disturb_Data = Get_Disturb_Data()
    Disturb_Data=zeros(10001,4);
    dt=0.001;
    for i=1:10001
        t = dt*(i-1);
        Disturb_Data(i,1)=t;
        if t>=4 && t<=6
            Disturb_Data(i,2:4)=[2 2 3];
        end
    end
    % test for Disturb Data
    % figure(1)
    % hold on
    % plot(Disturb_Data(:,1),Disturb_Data(:,2))
    % plot(Disturb_Data(:,1),Disturb_Data(:,3))
    % plot(Disturb_Data(:,1),Disturb_Data(:,4))
    % legend('Joint-1-Disturb','Joint-2-Disturb','Joint-3-Disturb')
end

%function 30: 为带有外力干扰的阻抗控制器的内环进行绘图
function Plot_For_Impedance_Controller_Resault_With_Disturb(Exp_Data_Joint,Real_Data_Joint,Disturb_Data)
    % 关节一（位置、速度、加速度、力矩）
    figure(1)
    subplot(2,2,1) % 期望位置&实际位置
    hold on
    title("Joint-1 Position Compare")
    xlabel("时间/s")
    ylabel("关节位置/°")
    plot(Exp_Data_Joint(:,1),Exp_Data_Joint(:,2))
    plot(Real_Data_Joint(:,1),Real_Data_Joint(:,2))
    legend("Exp","Real")
    subplot(2,2,2)
    hold on
    title("Joint-1 Velocity Compare")
    xlabel("时间/s")
    ylabel("关节角速度/(°/s)")
    plot(Exp_Data_Joint(:,1),Exp_Data_Joint(:,5))
    plot(Real_Data_Joint(:,1),Real_Data_Joint(:,5))
    legend("Exp","Real")
    subplot(2,2,3)
    hold on
    title("Joint-1 Accelerate Compare")
    xlabel("时间/s")
    ylabel("关节角加速度/(°/s^2)")
    plot(Exp_Data_Joint(:,1),Exp_Data_Joint(:,8))
    plot(Real_Data_Joint(:,1),Real_Data_Joint(:,8))
    legend("Exp","Real")
    subplot(2,2,4)
    hold on
    title("Joint-1 Torque Output")
    xlabel("时间/s")
    ylabel("关节输出力矩/(N·m)")
    plot(Real_Data_Joint(:,1),Real_Data_Joint(:,11))
    plot(Disturb_Data(:,1),Disturb_Data(:,2))
    legend("Real","Disturb")

    % 关节二（位置、速度、加速度、力矩）
    figure(2)
    subplot(2,2,1)
    hold on
    title("Joint-2 Position Compare")
    xlabel("时间/s")
    ylabel("关节位置/°")
    plot(Exp_Data_Joint(:,1),Exp_Data_Joint(:,3))
    plot(Real_Data_Joint(:,1),Real_Data_Joint(:,3))
    legend("Exp","Real")
    subplot(2,2,2)
    hold on
    title("Joint-2 Velocity Compare")
    xlabel("时间/s")
    ylabel("关节角速度/(°/s)")
    plot(Exp_Data_Joint(:,1),Exp_Data_Joint(:,6))
    plot(Real_Data_Joint(:,1),Real_Data_Joint(:,6))
    legend("Exp","Real")
    subplot(2,2,3)
    hold on
    title("Joint-2 Accelerate Compare")
    xlabel("时间/s")
    ylabel("关节角加速度/(°/s^2)")
    plot(Exp_Data_Joint(:,1),Exp_Data_Joint(:,9))
    plot(Real_Data_Joint(:,1),Real_Data_Joint(:,9))
    legend("Exp","Real")
    subplot(2,2,4)
    hold on
    title("Joint-2 Torque Output")
    xlabel("时间/s")
    ylabel("关节输出力矩/(N·m)")
    plot(Real_Data_Joint(:,1),Real_Data_Joint(:,12))
    plot(Disturb_Data(:,1),Disturb_Data(:,3))
    legend("Real","Disturb")

    % 关节三（位置、速度、加速度、力矩）
    figure(3)
    subplot(2,2,1)
    hold on
    title("Joint-3 Position Compare")
    xlabel("时间/s")
    ylabel("关节位置/m")
    plot(Exp_Data_Joint(:,1),Exp_Data_Joint(:,4))
    plot(Real_Data_Joint(:,1),Real_Data_Joint(:,4))
    legend("Exp","Real")
    subplot(2,2,2)
    hold on
    title("Joint-2 Velocity Compare")
    xlabel("时间/s")
    ylabel("关节角速度/(m/s)")
    plot(Exp_Data_Joint(:,1),Exp_Data_Joint(:,7))
    plot(Real_Data_Joint(:,1),Real_Data_Joint(:,7))
    legend("Exp","Real")
    subplot(2,2,3)
    hold on
    title("Joint-3 Accelerate Compare")
    xlabel("时间/s")
    ylabel("关节角加速度/(m/s^2)")
    plot(Exp_Data_Joint(:,1),Exp_Data_Joint(:,10))
    plot(Real_Data_Joint(:,1),Real_Data_Joint(:,10))
    legend("Exp","Real")
    subplot(2,2,4)
    hold on
    title("Joint-3 Torque Output")
    xlabel("时间/s")
    ylabel("关节输出力矩/(N)")
    plot(Real_Data_Joint(:,1),Real_Data_Joint(:,13))
    plot(Disturb_Data(:,1),Disturb_Data(:,4))
    legend("Real","Disturb")
end

