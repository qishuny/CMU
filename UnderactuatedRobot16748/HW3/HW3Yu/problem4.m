% Homework 3
% Problem 4
% Qishun Yu

clear;
close all;
clc;
desiredStates = matfile('states.mat').states;
desiredU = matfile('u.mat').u;


Qf=eye(2);

Q=eye(2);
R=1;
B=[0;1];
dt = 0.025;

S = zeros(201,4);
S(201,:) = reshape(Qf,4,1);
for i = 201:-1:2
    A=[0 1; -cos(desiredStates(i,1)) 0];
    Stemp = reshape(S(i,:),2,2);
    sdot=-A'*Stemp-Stemp*A+Stemp*B*(R\B')*Stemp-Q;
    S(i-1,:) = reshape((Stemp-dt*sdot),4,1);
end


x0 = [0.5 0];

x= zeros(201,2);
x(1,:) = x0;
uBar = zeros(201,1);
for i = 1:200
    xStar = [desiredStates(i,1),desiredStates(i,2)];
    Stemp = reshape(S(i,:),[2,2]);
    K = (R\B')*Stemp;
    uBar(i) = -K*(x(i,:)-xStar).'+desiredU(i);
    xdot=[x(i,2); uBar(i) - sin(x(i,1))];
    x(i+1,1) = x(i,1)+xdot(1)*dt;
    x(i+1,2) = x(i,2)+xdot(2)*dt;
end


plot(x(:,1),x(:,2),'r');
hold on;
plot(desiredStates(:,1),desiredStates(:,2));
hold off;
xlabel('theta');
ylabel('thetadot');
legend('stablized states','open-loop states');


figure;
plot(uBar);
xlabel('t');
ylabel('u');
