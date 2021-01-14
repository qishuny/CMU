% Homework 4
% Problem 1c
% Qishun Yu

clear all
close all
clc

aTrajectory = matfile('aTraj.mat');
bTrajectory = matfile('bTraj.mat');
desiredStates = aTrajectory.ouputStates1a;
nominalStates = bTrajectory.ouputStates1b;

xD = desiredStates(:,2:3);
uD = desiredStates(:,1);
xN = nominalStates(:,2:3);
uN = nominalStates(:,1);

n = 100;
dt = 0.1;
T=n*dt;
time=0:dt:T-dt;

Q = 100*eye(2);
R = 1;
B = [0;1];

xInit=[0;0];
Qf = 100*eye(2);
S2Init=reshape(Qf,[],1);

trialX1 = zeros(n,5);
trialX2 = zeros(n,5);

%Plot
trajPlot = figure(1);
hold on
plot(xD(:,1),xD(:,2),'DisplayName','desired')
plot(xN(:,1),xN(:,2),'DisplayName','nominal')
hold off

for iter = 1:5
    % back propagation for s2
    [t,S]=ode45(@(t,S)ricattiEqn1(t,S,time,xN,uN,xD,uD,B,Q,R), linspace(T-dt,0,n), S2Init);
    tS2=flip(t);
    S2=flipud(S);
    
    xRef = xD - xN;
    % back propagation for s1
    S1Init=-2*Q*xRef(end,:)';
    [t,S]=ode45(@(t,S)ricattiEqn2(t,S,time,tS2,S2,xN,uN,xD,uD,B,Q,R), linspace(T-dt,0,n), S1Init);
    tS1=flip(t);
    S1=flipud(S);
    
    k1 = zeros(n,2);
    k2 = zeros(n,1);
    for i = 1:n
        s2 = reshape(S2(i,:),2,2);
        s1 = reshape(S1(i,:),2,1);
        k1(i,:) = (R^(-1)*B')*s2;
        k2(i) =(R^(-1)*B')*0.5*s1;
    end
    % forward dynamics
    [t,x] = ode45(@(t,x)dynamics(t,x,time,xN,uN,xD,uD,tS1,k1,tS2,k2),time,xInit);
    
    %update input and states
    u = zeros(n,1);
    for i=1:n
        u(i) = (uD(i)) - k1(i,:)*(x(i,:)-xN(i,:))' - k2(i);
    end   
    xN=x;
    uN=u';
  
    trialX1(:,iter) = x(:,1);
    trialX2(:,iter) = x(:,2); 
end



for i = 1:5
    figure(trajPlot)
    hold on
    char =  int2str( i );
    plot(trialX1(:,i),trialX2(:,i),'DisplayName',char);
    hold off
end
xlabel('theta');
ylabel('thetadot');
title('iLQR');

legend
function xdot=dynamics(t,x,time,xN,uN,xD,uD,tS1,k1,tS2,k2)

xNI(1)=interp1(time',xN(:,1),t);
xNI(2)=interp1(time',xN(:,2),t);
uDI=interp1(time',uD,t);
K1I(1)=interp1(tS2,k1(:,1),t);
K1I(2)=interp1(tS2,k1(:,2),t);
K2I=interp1(tS1,k2',t);
xRef=x-xNI';

u=uDI-K1I*xRef-K2I;
xdot=[x(2); 
    u - sin(x(1))-0.3*x(2)];
end

function Sdot=ricattiEqn1(t,S,time,xN,uN,xD,uD,B,Q,R)
xNI(1)=interp1(time',xN(:,1),t);
xNI(2)=interp1(time',xN(:,2),t);
A=[0, 1;
    -cos(xNI(1)), -0.3];
s2=reshape(S,2,2);
Sdot=reshape((-(Q-s2*B*R^(-1)*B'*s2+s2*A+A'*s2)),[],1);
end

function Sdot=ricattiEqn2(t,S,time,tS2,S2,xN,uN,xD,uD,B,Q,R)
xNI(1)=interp1(time',xN(:,1),t);
xNI(2)=interp1(time',xN(:,2),t);
xDI(1)=interp1(time',xD(:,1),t);
xDI(2)=interp1(time',xD(:,2),t);
uNI=interp1(time',uN,t);
uDI=interp1(time',uD,t);
S2I(1)=interp1(tS2,S2(:,1),t);
S2I(2)=interp1(tS2,S2(:,2),t);
S2I(3)=interp1(tS2,S2(:,3),t);
S2I(4)=interp1(tS2,S2(:,4),t);
A=[0 1; -cos(xNI(1)) -0.3];
xRef=xDI'-xNI';
uRef=uDI-uNI;
S1=reshape(S,2,1);
S2=reshape(S2I,2,2);
Sdot=reshape((-(-2*Q*xRef+(A'-S2*B*R^(-1)*B')*S1+2*S2*B*uRef)),[],1);
end
