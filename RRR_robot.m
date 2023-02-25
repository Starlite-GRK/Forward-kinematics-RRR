%The given robot is a RRR manipulator
%Geometric Approach has been used for simulation 
%And Dh and transformation calculations for transformation matrices calculations

%R is world frame
%B, C and W are robot revolute joint frames
%T is the tool frame
%S is the table frame where object is placed
%G is the object frame

close all
close all
clc

%Link lengths and final joint angles
L1=input('Enter length of link 1:'); 
L2=input('Enter length of link 2:');
L3=input('Enter length of link 3:');
Th1=input('Enter first joint angle:');
Th2=input('Enter second joint angle:');
Th3=input('Enter third joint angle:');

%Simulation

plot([0,L1],[0,0],[L1,L2],[0,0],[L2,L3],[0,0],'linewidth',2) %Plotting the base position of the robot

theta1=linspace(0,Th1,5);
theta2=linspace(0,Th2,5);
theta3=linspace(0,Th3,5);

ct=1; % counter
for i=1:length(theta1)
THETA1=theta1(i);
   for j=1:length(theta2)
   THETA2=theta2(j);
     for k=1:length(theta3)
     THETA3=theta3(k);

     %coordinates
     x0=0;
     y0=0;
     
     x1=L1*cosd(THETA1);
     y1=L1*sind(THETA1);
     x2=x1+L2*cosd(THETA1+THETA2);
     y2=y1+L2*sind(THETA1+THETA2);
     x3=x2+L3*cosd(THETA1+THETA2+THETA3);
     y3=y2+L3*sind(THETA1+THETA2+THETA3);
     
     plot([x0,x1],[y0,y1],[x1,x2],[y1,y2],[x2,x3],[y2,y3],'linewidth',2)
     grid on;
     axis([-0.1 8 -0.1 8])
     
     m(ct)=getframe(gcf);
     ct=ct+1;
     end
   end
end  

%Calculating the final transformation matrices

%World frame to tool frame
R=[1,0,0,0;0,1,0,0;0,0,1,0;0,0,0,1]
RTB=DHmatrix(Th1,0,L1,90)
BTC=DHmatrix(Th2,0,L2,0)
CTW=DHmatrix(Th3,0,L3,0)

RTW=RTB*BTC*CTW
RTT=RotY(0)*RTW %Taking object orientation same as joint frame                                                                                                            `

%World frame to table frame
%Translations between world frame and table frame
X1=input('Enter distance in X direction between world frame and table frame:'); 
Y1=input('Enter distance in Y direction between world frame and table frame:');
Z1=input('Enter distance in Z direction between world frame and table frame:');
RTS=[1,0,0,X1;0,1,0,Y1;0,0,1,Z1;0,0,0,1]

%Table frame to object frame
%Translations between table frame and object frame
X2=input('Enter distance in X direction between table frame and object frame:'); 
Y2=input('Enter distance in Y direction between table frame and object frame:');
Z2=input('Enter distance in Z direction between table frame and object frame:');
STG=[0,1,0,X2;-1,0,0,Y2;0,0,-1,Z2;0,0,0,1]
TTG=inv(RTT)*RTS*STG

