
clc,clear,close all;

pause(2);

%% 
map=int16(im2bw(imread('bukeda.bmp'))); % 
source=[250 20]; % 起点
goal=[360 360]; % 终点
robotDirection=pi/8; %初始的运动方向
robotSize=[10 10]; %机器人的大小
robotSpeed=10; % 机器人的速度
maxRobotSpeed=10; % 最大运动速度
distanceThreshold=30; %  
maxAcceleration=10; % 
maxTurn=10*pi/180; % 
k=3; %  
attractivePotentialScaling=300000; %引力势能的放大比例
repulsivePotentialScaling=300000; % 斥力势能的放大比例
minAttractivePotential=0.5; %最小的吸引势能

%% %%% 

currentPosition=source; % 机器人中心点的起点位置
currentDirection=robotDirection; % 初始方位
robotHalfDiagonalDistance=((robotSize(1)/2)^2+(robotSize(2)/2)^2)^0.5; % 机器人的包围圆圈的半径大小
pathFound=false; %目标是否到达的bool型标志
pathCost=0; % 对走过的距离进行计数
t=1; %迭代次数
imshow(map==1);
rectangle('position',[1 1 size(map)-1],'edgecolor','k')

%绘制起点终点圆圈，（注意坐标顺序与平时不同，避免出错）
rectangle('position',[fliplr(source-10) 20 20],'facecolor',[1 .0 .0],'Curvature',[1 1]);
rectangle('position',[fliplr(goal-10) 20 20],'facecolor',[0 1.0 .0],'Curvature',[1 1]);
text(source(2)-10,source(1)-20,'起点','Color','red','FontSize',14);
text(goal(2)-10,goal(1)+20,'目标','Color','green','FontSize',14);

xlabel('x');
ylabel('y');
axis on;

pathLength=0; 
if ~plotRobot(currentPosition,currentDirection,map,robotHalfDiagonalDistance)
     error('source lies on an obstacle or outside map'); 
end
M(t)=getframe;  %获取当前图形
t=t+1;  %计数迭代次数
%   判断目标点是否在地图范围内，否则判错
if ~feasiblePoint(goal,map), error('goal lies on an obstacle or outside map'); end

tic;    %开始计时

while ~pathFound
    
    % 计算机器人正前方的障碍物距离
    i=robotSize(1)/2+1; %从机器人的包围圆的外部开始计数
    while true
        x=int16(currentPosition+i*[sin(currentDirection) cos(currentDirection)]);
        if ~feasiblePoint(x,map), break; end  %直到遇到障碍物或者超出范围停止计数
        i=i+1;
    end
    distanceFront=i-robotSize(1)/2; %减掉包围圆的半径得到距离
    
    % 计算机器人左边的障碍物距离
    i=robotSize(2)/2+1;
    while true
        x=int16(currentPosition+i*[sin(currentDirection-pi/2) cos(currentDirection-pi/2)]);
        if ~feasiblePoint(x,map), break; end
        i=i+1;
    end
    distanceLeft=i-robotSize(2)/2;  
    
    % 计算机器人右边的障碍物距离
    i=robotSize(2)/2+1;
    while true
        x=int16(currentPosition+i*[sin(currentDirection+pi/2) cos(currentDirection+pi/2)]);
        if ~feasiblePoint(x,map), break; end
        i=i+1;
    end
    distanceRight=i-robotSize(2)/2;  
    
    % 计算机器人左前方的障碍物距离
    i=robotHalfDiagonalDistance+1;
    while true
        x=int16(currentPosition+i*[sin(currentDirection-pi/4) cos(currentDirection-pi/4)]);
        if ~feasiblePoint(x,map), break; end
        i=i+1;
    end
    distanceFrontLeftDiagonal=i-robotHalfDiagonalDistance;
    
    % 计算机器人右前方的障碍物距离
    i=robotHalfDiagonalDistance+1;
    while true
        x=int16(currentPosition+i*[sin(currentDirection+pi/4) cos(currentDirection+pi/4)]);
        if ~feasiblePoint(x,map), break; end
        i=i+1;
    end
    distanceFrontRightDiagonal=i-robotHalfDiagonalDistance;
    
    % 计算目标全局方位
     angleGoal=atan2(goal(1)-currentPosition(1),goal(2)-currentPosition(2));
    
     %计算到障碍物的距离
     distanceGoal=( sqrt(sum((currentPosition-goal).^2)) );
     if distanceGoal<distanceThreshold, pathFound=true; end %如果距离小于阈值则目标已经达到
     
     % compute potentials
     % 计算斥力势能
     repulsivePotential=(1.0/distanceFront)^k*[sin(currentDirection) cos(currentDirection)] + ...
     (1.0/distanceLeft)^k*[sin(currentDirection-pi/2) cos(currentDirection-pi/2)] + ...
     (1.0/distanceRight)^k*[sin(currentDirection+pi/2) cos(currentDirection+pi/2)] + ...
     (1.0/distanceFrontLeftDiagonal)^k*[sin(currentDirection-pi/4) cos(currentDirection-pi/4)] + ...
     (1.0/distanceFrontRightDiagonal)^k*[sin(currentDirection+pi/4) cos(currentDirection+pi/4)];
     % 计算引力势能
     attractivePotential=max([(1.0/distanceGoal)^k*attractivePotentialScaling minAttractivePotential])*[sin(angleGoal) cos(angleGoal)];
     % 计算总的势能，与速度改变量相关
     totalPotential=attractivePotential-repulsivePotentialScaling*repulsivePotential;
     
     %计算偏转角度
     preferredSteer=atan2(robotSpeed*sin(currentDirection)+totalPotential(1),robotSpeed*cos(currentDirection)+totalPotential(2))-currentDirection;
     while preferredSteer>pi, preferredSteer=preferredSteer-2*pi; end % check to get the angle between -pi and pi
     while preferredSteer<-pi, preferredSteer=preferredSteer+2*pi; end % check to get the angle between -pi and pi
     preferredSteer=min([maxTurn preferredSteer]);
     preferredSteer=max([-maxTurn preferredSteer]);
     % 计算新的方向
     currentDirection=currentDirection+preferredSteer;
     
     % 计算新的速度绝对值，并进行有效性判断
     preferredSpeed=sqrt(sum((robotSpeed*[sin(currentDirection) cos(currentDirection)] + totalPotential).^2));
     preferredSpeed=min([robotSpeed+maxAcceleration preferredSpeed]);
     robotSpeed=max([robotSpeed-maxAcceleration preferredSpeed]);
     robotSpeed=min([robotSpeed maxRobotSpeed]);
     robotSpeed=max([robotSpeed 0]);
     
     if robotSpeed==0, error('robot had to stop to avoid collission'); end
     
     % 计算新的位置坐标
     newPosition=currentPosition+robotSpeed*[sin(currentDirection) cos(currentDirection)];
     pathCost=pathCost+distanceCost(newPosition,currentPosition); % 累加走过的距离
     currentPosition=newPosition;
     if ~feasiblePoint(int16(currentPosition),map), error('collission recorded'); end % 判断是否碰到障碍物
     
     % plotting robot
     if ~plotRobot(currentPosition,currentDirection,map,robotHalfDiagonalDistance)
        error('collission recorded');
     end
     M(t)=getframe;t=t+1;
end
% 输出使用的时间和路程
fprintf('processing time=%d \nPath Length=%d \n\n', toc,pathCost); 

