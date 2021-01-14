clear all
close all
clc
%Problem 2 a
%Qishun Yu

%Parameters
m = 1;
g = 9.8;
l = 1;


dt = 0.01;
%initialize grid & gridsize
dx = 0.1;
[theta,thetadot] = ndgrid(-2*pi:dx:2*pi,-2*g/l:dx:2*g/l);

%changeable conditions
%b = u = 0
b1 = 0;
u1 = 0;
f1 = [0;0];
output1 = basin_attraction(b1,u1,f1,theta,thetadot);

% b = 0.25 u = 0
b2 = 0.25;
u2 = 0;
f2 = [0;0];
output2 = basin_attraction(b2,u2,f2,theta,thetadot);

% b = 0.25 u = g/2l
b3 = 0.25;
u3 = g/(2*l);
f3 = [(pi/6);0];
output3 = basin_attraction(b3,u3,f3,theta,thetadot);




%Plot on three conditions using colormap
figure()
imagesc([theta(1,1) theta(end,1)],[thetadot(1,1) thetadot(1,end)],output1');
axis xy; 
colormap();
colorbar;
xlabel('theta');
ylabel('theta dot');
title('b = u = 0')

figure()
imagesc([theta(1,1) theta(end,1)],[thetadot(1,1) thetadot(1,end)],output2');
axis xy; 
colormap();
colorbar;
xlabel('theta');
ylabel('theta dot');
title('b = 0.25 u = 0')

figure()
imagesc([theta(1,1) theta(end,1)],[thetadot(1,1) thetadot(1,end)],output3');
axis xy; 
colormap();
colorbar;
xlabel('theta');
ylabel('theta dot');
title('b = 0.25 u = g/(2*l)')

function output = basin_attraction(b,u,f,theta,thetadot)
    
    %Parameters
    m = 1;
    g = 9.8;
    l = 1;

    dx = 0.1;
    dt = 0.01;
    
    % initial input and output
       
    output = zeros(size(theta));
    
    iterations = 2e4;
    for i=1:size(theta,1)
        for j=1:size(theta,2)
            x =  [theta(i,j) thetadot(i,j)]';
            for t=1:iterations
                % Dynamics and update
                xdot = [x(2); (u-m*g*l*sin(x(1))-b*x(2))./(m*l*l)];
                x = x+ xdot*dt;

                output(i,j)= 0;
                % Check converge or diverge
                if norm(x-f) <= dx
                    output(i,j) = 1;
                    break
                end
            end
            if norm(x-f)<= dx
                output(i,j) = 1;
            end
        end
    end

end

