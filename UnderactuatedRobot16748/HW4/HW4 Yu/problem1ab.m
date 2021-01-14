% Homework 4
% Problem 1a and 1b
% Qishun Yu

clear all;
close all;
clc;

n = 100;
dt = 0.1;

%initialize states and inputs
u=zeros(n,1);
x0=zeros(n,2);
states =  [u x0];


% constrains
A = [];
b = [];
Aeq = [];
beq = [];
upper = [repmat(1.5,n,1) inf(n,2)];
lower = [repmat(-1.5,n,1) -inf(n,2)];


options =optimoptions(@fmincon,'TolFun', 0.00000001,'MaxIter', 10000, ...
    'MaxFunEvals', 100000,'Display','iter', ...
    'DiffMinChange', 0.001,'Algorithm', 'sqp');
ouputStates1a =fmincon(@cost_func, states, A,b,Aeq,beq, lower,upper,@func,options);
ouputStates1b =fmincon(@cost_func, states, A,b,Aeq,beq, lower,upper,@func1b,options);
save('aTraj.mat','ouputStates1a');
save('bTraj.mat','ouputStates1b');

figure()
hold on
plot(ouputStates1a(:,2),ouputStates1a(:,3));
plot(ouputStates1b(:,2),ouputStates1b(:,3));
xlabel('theta');
ylabel('thetadot');
title('Pole Trajectory Plot');
hold off

T=n*dt;
time=0:dt:T-dt;
% figure()
% hold on
% plot(time,ouputStates1a(:,1));
% plot(time,ouputStates1b(:,1));
% xlabel('t');
% ylabel('control input');
% title('Control Input Plot');
% hold off
% inequality c constraints
function [C,Ceq] = func(states)
C = [];

% initial[0,0] final []
Ci = [0-states(1,2);0-states(1,3)];
Cf = [-pi-states(end,2);0-states(end,3);];
delta = defect_func1a(states);
Ceq = [Ci;reshape(delta,2*100-2,1);Cf];
end

% inequality c constraints
function [C,Ceq] = func1b(states)
C = [];

% initial[0,0] final []
Ci = [0-states(1,2);0-states(1,3)];
Cf = [-pi-states(end,2);0-states(end,3);];
delta = defect_func1b(states);
Ceq = [Ci;reshape(delta,2*100-2,1);Cf];
end

% define constraints that enforce the system’s dynamics at collocation points
function delta = defect_func1a(states)
    %xk xk+1
    xP = states(1:(end-1),2:3);
    xN = states(2:(end),2:3);
    
    %fk fk+1
    xdot = dynamics1a(states(:,2:3),states(:,1));
    xdotP = xdot(1:(end-1),:);
    xdotN = xdot(2:(end),:);
    
    %uk uk+1
    uP = states(1:(end-1),1);
    uN = states(2:(end),1);
    
    % xmid polynomials
    xM = 1/2*(xP+xN)+1/8*(xdotN-xdotP);
    % umid linear
    uM = 1/2*(uP+uN);
    xdotM = dynamics1a(xM,uM);
    
    delta = xP-xN + (0.1/6)*(xdotP + 4*xdotM + xdotN);
end

% define constraints that enforce the system’s dynamics at collocation points
function delta = defect_func1b(states)
    %xk xk+1
    xP = states(1:(end-1),2:3);
    xN = states(2:(end),2:3);
    
    %fk fk+1
    xdot = dynamics1b(states(:,2:3),states(:,1));
    xdotP = xdot(1:(end-1),:);
    xdotN = xdot(2:(end),:);
    
    %uk uk+1
    uP = states(1:(end-1),1);
    uN = states(2:(end),1);
    
    % xmid polynomials
    xM = 1/2*(xP+xN)+1/8*(xdotN-xdotP);
    % umid linear
    uM = 1/2*(uP+uN);
    xdotM = dynamics1b(xM,uM);
    
    delta = xP-xN + (0.1/6)*(xdotP + 4*xdotM + xdotN);
end


function J= cost_func(states)
    Q=eye(2);
    R=1;
    J=0;
    for iter=1:size(states,1)
        J=J+states(iter,2:3)*Q*states(iter,2:3)'+states(iter,1)'*R*states(iter,1);
    end
end
% system dynamics
function xdot = dynamics1a(x,u)
xdot = zeros(size(x,1),2);
for i = 1:size(x,1)
    xdot(i,:)=[x(i,2);
        u(i)-(((x(i,1)^2)-1)*x(i,2))-sin(x(i,1))];
end
end

function xdot = dynamics1b(x,u)
xdot = zeros(size(x,1),2);
for i = 1:size(x,1)
    xdot(i,:)=[x(i,2);
        u(i)-(0.3*x(i,2))-sin(x(i,1))];
end
end