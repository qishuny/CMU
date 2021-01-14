clear all
close all
clc

%Problem 3
%Qishun Yu
%
% initialize all possible states 
x_val = linspace(-pi,2*pi,20);
xdot_val = linspace(-3/2*pi,3/2*pi,20);
[x, xdot] = ndgrid(x_val,xdot_val);
states = [reshape(x,1,numel(x)); reshape(xdot,1,numel(xdot))];
ns = size(states,2);

% initialize all possible control u
u = linspace(-2,2,100);
nu = size(u,2);
S = repmat(states,1,nu); 
U = reshape(repmat(u,ns,1),1,ns*nu); 

dt= 0.05;
Sn = S + dynamics(S,U).*dt;
[Pi,P] = bilinear_interp(states,Sn,x_val,xdot_val);
C = reshape(cost(S,U),ns,nu);

J = zeros(ns,1); 
iter = 1; 
err = 1e6;

while (iter<=20)
        [Jnew, PI] = min(C + reshape(sum( P.*J(Pi),1),ns,nu),[],2);
        J = Jnew; 
        iter = iter+1;        
end

plot_func(J,u(PI),x_val,xdot_val);

% learn the function from https://
%ocw.mit.edu/courses/electrical-engineering-and-computer-science/
%6-832-underactuated-robotics-spring-2009/assignments/

function [Pi,P] = bilinear_interp(states,Sn,q_bins,qdot_bins)
ns = size(Sn,2);
Pi = zeros(4,ns);
P = Pi;

N = size(Sn,2);
smax = repmat([q_bins(end);qdot_bins(end)],1,N);
ind = Sn>smax;
Sn(ind) = smax(ind);
smin = repmat([q_bins(1);qdot_bins(1)],1,N); 
ind = Sn<smin;
Sn(ind) = smin(ind);

for i=1:ns
    
    ind_q = max([find(q_bins <= Sn(1,i),1,'last') 1]);
    ind_qdot = max([find(qdot_bins <= Sn(2,i),1,'last') 1]);
    
    offset = [0 0;1 0;0 1;1 1];
    if (ind_q == length(q_bins))
        offset(:,1) = -offset(:,1);
    end
    if (ind_qdot == length(qdot_bins))
        offset(:,2) = -offset(:,2);
    end
    
    totl_area = abs(q_bins(ind_q+offset(2,1))-q_bins(ind_q))*...
                abs(qdot_bins(ind_qdot+offset(3,2))-qdot_bins(ind_qdot));
    
    for j=1:4
        state = [q_bins(ind_q+offset(j,1)); qdot_bins(ind_qdot+offset(j,2))];
        Pi(j,i) = find(sum(abs(states-repmat(state,1,size(states,2))),1)==0);
        P(5-j,i) = (abs(Sn(1,i)-q_bins(ind_q+offset(j,1)))*abs(Sn(2,i)-qdot_bins(ind_qdot+offset(j,2))))/totl_area;
    end
end
end

function xdot = dynamics(x,u)
xdot = [x(2,:);-sin(x(1,:))-x(2,:)+u];
end

function C = cost(X,u)
Xdes = repmat([pi 0]',1,size(X,2));
X(2,:) = abs(X(2,:));

du= 0.5;
Q = diag([1 1]).*du;
R = 1*du;
C = dot((X - Xdes),1*Q*(X - Xdes))+R*u.^2;
end


function plot_func(J,PI,q_bins,qdot_bins)
figure();
n1 = size(q_bins,2); 
n2 = size(qdot_bins,2);
subplot(2,1,1);
imagesc(q_bins,qdot_bins,reshape(PI,n1,n2)'); 
axis xy;
xlabel('x'); 
ylabel('xdot'); 
title('optimal policy'); 
colorbar;
subplot(2,1,2);
imagesc(q_bins,qdot_bins,reshape(J,n1,n2)'); 
axis xy;
xlabel('x'); 
ylabel('xdot'); 
title('value iteration'); 
colorbar;
end