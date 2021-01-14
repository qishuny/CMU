% Homework 3
% Problem 3
% Qishun Yu
% Worked with Huaiqian Shou

clear all;
close all;
clc;

n = 50;
dt = 0.1;

%initialize states and inputs
u=zeros(n,1);
x0=zeros(n,4);
states =  [u x0];


% constrains
A = [];
b = [];
Aeq = [];
beq = [];
upper = [repmat(5,n,1) inf(n,4)];
lower = [repmat(-5,n,1) -inf(n,4)];


options =optimoptions(@fmincon,'TolFun', 0.00000001,'MaxIter', 10000, ...
    'MaxFunEvals', 100000,'Display','iter', ...
    'DiffMinChange', 0.001,'Algorithm', 'sqp');
ouputStates =fmincon(@(states) states(1:n,1)'*states(1:n,1), states, A,b,Aeq,beq, lower,upper,@func,options);

figure()
plot(ouputStates(:,3),ouputStates(:,5));
xlabel('theta');
ylabel('thetadot');
title('Pole Trajectory Plot');

figure()
plot(ouputStates(:,2),ouputStates(:,4));
xlabel('x');
ylabel('xdot');
title('Cart Trajectory Plot');

T=n*dt;
time=0:dt:T-dt;
figure()
plot(time,ouputStates(:,1));
xlabel('t');
ylabel('control input');
title('Control Input Plot');

% inequality c constraints
function [C,Ceq] = func(states)
C = [];

Ci = [0-states(1,2);0-states(1,3);0-states(1,4);0-states(1,5)];
Cf = [0-states(end,2);pi-states(end,3);0-states(end,4);0-states(end,5)];
delta = defect_func(states);
Ceq = [Ci;reshape(delta,4*49,1);Cf];
end

% define constraints that enforce the system’s dynamics at collocation points
function delta = defect_func(states)
    %xk xk+1
    xP = states(1:(end-1),2:5);
    xN = states(2:(end),2:5);
    
    %fk fk+1
    xdot = dynamics(states(:,2:5),states(:,1));
    xdotP = xdot(1:(end-1),:);
    xdotN = xdot(2:(end),:);
    
    %uk uk+1
    uP = states(1:(end-1),1);
    uN = states(2:(end),1);
    
    % xmid polynomials
    xM = 1/2*(xP+xN)+1/8*(xdotN-xdotP);
    % umid linear
    uM = 1/2*(uP+uN);
    xdotM = dynamics(xM,uM);
    
    delta = xP-xN + (0.1/6)*(xdotP + 4*xdotM + xdotN);
end

% system dynamics
function xdot = dynamics(x,u)
xdot = zeros(size(x,1),4);
for i  = 1:size(x,1)
    xTemp = x(i,:);
    uTemp = u(i,1);
    xdot(i,:) = [xTemp(3),
        xTemp(4),
    (uTemp + sin(xTemp(2))*(xTemp(4)^2 + cos(xTemp(2))))/(1+sin(xTemp(2))^2),
    (-uTemp*cos(xTemp(2)) - xTemp(4)^2*cos(xTemp(2))*sin(xTemp(2)) - 2*sin(xTemp(2)))/(1+sin(xTemp(2))^2)];
end
end