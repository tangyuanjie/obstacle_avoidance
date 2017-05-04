
clc,clear,close all;

pause(2);

%% 
map=int16(im2bw(imread('bukeda.bmp'))); % 
source=[250 20]; % ���
goal=[360 360]; % �յ�
robotDirection=pi/8; %��ʼ���˶�����
robotSize=[10 10]; %�����˵Ĵ�С
robotSpeed=10; % �����˵��ٶ�
maxRobotSpeed=10; % ����˶��ٶ�
distanceThreshold=30; %  
maxAcceleration=10; % 
maxTurn=10*pi/180; % 
k=3; %  
attractivePotentialScaling=300000; %�������ܵķŴ����
repulsivePotentialScaling=300000; % �������ܵķŴ����
minAttractivePotential=0.5; %��С����������

%% %%% 

currentPosition=source; % ���������ĵ�����λ��
currentDirection=robotDirection; % ��ʼ��λ
robotHalfDiagonalDistance=((robotSize(1)/2)^2+(robotSize(2)/2)^2)^0.5; % �����˵İ�ΧԲȦ�İ뾶��С
pathFound=false; %Ŀ���Ƿ񵽴��bool�ͱ�־
pathCost=0; % ���߹��ľ�����м���
t=1; %��������
imshow(map==1);
rectangle('position',[1 1 size(map)-1],'edgecolor','k')

%��������յ�ԲȦ����ע������˳����ƽʱ��ͬ���������
rectangle('position',[fliplr(source-10) 20 20],'facecolor',[1 .0 .0],'Curvature',[1 1]);
rectangle('position',[fliplr(goal-10) 20 20],'facecolor',[0 1.0 .0],'Curvature',[1 1]);
text(source(2)-10,source(1)-20,'���','Color','red','FontSize',14);
text(goal(2)-10,goal(1)+20,'Ŀ��','Color','green','FontSize',14);

xlabel('x');
ylabel('y');
axis on;

pathLength=0; 
if ~plotRobot(currentPosition,currentDirection,map,robotHalfDiagonalDistance)
     error('source lies on an obstacle or outside map'); 
end
M(t)=getframe;  %��ȡ��ǰͼ��
t=t+1;  %������������
%   �ж�Ŀ����Ƿ��ڵ�ͼ��Χ�ڣ������д�
if ~feasiblePoint(goal,map), error('goal lies on an obstacle or outside map'); end

tic;    %��ʼ��ʱ

while ~pathFound
    
    % �����������ǰ�����ϰ������
    i=robotSize(1)/2+1; %�ӻ����˵İ�ΧԲ���ⲿ��ʼ����
    while true
        x=int16(currentPosition+i*[sin(currentDirection) cos(currentDirection)]);
        if ~feasiblePoint(x,map), break; end  %ֱ�������ϰ�����߳�����Χֹͣ����
        i=i+1;
    end
    distanceFront=i-robotSize(1)/2; %������ΧԲ�İ뾶�õ�����
    
    % �����������ߵ��ϰ������
    i=robotSize(2)/2+1;
    while true
        x=int16(currentPosition+i*[sin(currentDirection-pi/2) cos(currentDirection-pi/2)]);
        if ~feasiblePoint(x,map), break; end
        i=i+1;
    end
    distanceLeft=i-robotSize(2)/2;  
    
    % ����������ұߵ��ϰ������
    i=robotSize(2)/2+1;
    while true
        x=int16(currentPosition+i*[sin(currentDirection+pi/2) cos(currentDirection+pi/2)]);
        if ~feasiblePoint(x,map), break; end
        i=i+1;
    end
    distanceRight=i-robotSize(2)/2;  
    
    % �����������ǰ�����ϰ������
    i=robotHalfDiagonalDistance+1;
    while true
        x=int16(currentPosition+i*[sin(currentDirection-pi/4) cos(currentDirection-pi/4)]);
        if ~feasiblePoint(x,map), break; end
        i=i+1;
    end
    distanceFrontLeftDiagonal=i-robotHalfDiagonalDistance;
    
    % �����������ǰ�����ϰ������
    i=robotHalfDiagonalDistance+1;
    while true
        x=int16(currentPosition+i*[sin(currentDirection+pi/4) cos(currentDirection+pi/4)]);
        if ~feasiblePoint(x,map), break; end
        i=i+1;
    end
    distanceFrontRightDiagonal=i-robotHalfDiagonalDistance;
    
    % ����Ŀ��ȫ�ַ�λ
     angleGoal=atan2(goal(1)-currentPosition(1),goal(2)-currentPosition(2));
    
     %���㵽�ϰ���ľ���
     distanceGoal=( sqrt(sum((currentPosition-goal).^2)) );
     if distanceGoal<distanceThreshold, pathFound=true; end %�������С����ֵ��Ŀ���Ѿ��ﵽ
     
     % compute potentials
     % �����������
     repulsivePotential=(1.0/distanceFront)^k*[sin(currentDirection) cos(currentDirection)] + ...
     (1.0/distanceLeft)^k*[sin(currentDirection-pi/2) cos(currentDirection-pi/2)] + ...
     (1.0/distanceRight)^k*[sin(currentDirection+pi/2) cos(currentDirection+pi/2)] + ...
     (1.0/distanceFrontLeftDiagonal)^k*[sin(currentDirection-pi/4) cos(currentDirection-pi/4)] + ...
     (1.0/distanceFrontRightDiagonal)^k*[sin(currentDirection+pi/4) cos(currentDirection+pi/4)];
     % ������������
     attractivePotential=max([(1.0/distanceGoal)^k*attractivePotentialScaling minAttractivePotential])*[sin(angleGoal) cos(angleGoal)];
     % �����ܵ����ܣ����ٶȸı������
     totalPotential=attractivePotential-repulsivePotentialScaling*repulsivePotential;
     
     %����ƫת�Ƕ�
     preferredSteer=atan2(robotSpeed*sin(currentDirection)+totalPotential(1),robotSpeed*cos(currentDirection)+totalPotential(2))-currentDirection;
     while preferredSteer>pi, preferredSteer=preferredSteer-2*pi; end % check to get the angle between -pi and pi
     while preferredSteer<-pi, preferredSteer=preferredSteer+2*pi; end % check to get the angle between -pi and pi
     preferredSteer=min([maxTurn preferredSteer]);
     preferredSteer=max([-maxTurn preferredSteer]);
     % �����µķ���
     currentDirection=currentDirection+preferredSteer;
     
     % �����µ��ٶȾ���ֵ����������Ч���ж�
     preferredSpeed=sqrt(sum((robotSpeed*[sin(currentDirection) cos(currentDirection)] + totalPotential).^2));
     preferredSpeed=min([robotSpeed+maxAcceleration preferredSpeed]);
     robotSpeed=max([robotSpeed-maxAcceleration preferredSpeed]);
     robotSpeed=min([robotSpeed maxRobotSpeed]);
     robotSpeed=max([robotSpeed 0]);
     
     if robotSpeed==0, error('robot had to stop to avoid collission'); end
     
     % �����µ�λ������
     newPosition=currentPosition+robotSpeed*[sin(currentDirection) cos(currentDirection)];
     pathCost=pathCost+distanceCost(newPosition,currentPosition); % �ۼ��߹��ľ���
     currentPosition=newPosition;
     if ~feasiblePoint(int16(currentPosition),map), error('collission recorded'); end % �ж��Ƿ������ϰ���
     
     % plotting robot
     if ~plotRobot(currentPosition,currentDirection,map,robotHalfDiagonalDistance)
        error('collission recorded');
     end
     M(t)=getframe;t=t+1;
end
% ���ʹ�õ�ʱ���·��
fprintf('processing time=%d \nPath Length=%d \n\n', toc,pathCost); 

