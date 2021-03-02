clear all
close all
clc

%% RFT Modeling
% Qishun Yu
% 09/26/2020
 

%% EXAMPLE START

file_name = 'smooth_wheel_150.DXF';

% file_name = 'Rover_wheel_printable_straight.DXF';
pos_points = read_dxf(file_name);

% pos_points = makecircle(20);
[beta,area] = initial_beta(pos_points);

% angular velocity
w = 0.2*pi;

T = 8;
dt = 0.2;
simulation(T,dt,w,pos_points,beta,area)


% unused plots
% [beta, gamma] = bg_func(pos_points,vel_points);
% [u,v]= beta_plot(pos_points,vel_points,beta,gamma);

% plt_vel(pos_points,vel_points)
% plot_alphaMap()


%EXAMPLE END
%% Functions

%Simulation
function simulation(T,dt,w,pos_points,beta,area)
    tStep = 0.00;
    
    rho = 20; %????
    center = [0,0];
    
    f = figure(1);
    scatter(pos_points(:,1),pos_points(:,2),1,'blue')
    hold on 
    plot(center(1),center(2),'r*')
    
    while(tStep<T)
        tStep =tStep+dt;
        xt = w*rho*dt;
        zt = 0;
        theta = -dt*w;       
        
        [pos_points,center,beta] = rotate_update(pos_points,center,beta,theta,xt,zt);
        
        
        depth = 0;
        [pos_all,beta_all] = pos_limit(pos_points,depth,beta);
        vel_all = vel_func(pos_allz,w);
        [alphaX_all,alphaZ_all] = alpha_all(pos_all,vel_all,beta);
%         [u,v]= beta_plot(pos_points,vel_points,beta,gamma)

        % plot
        
        clf(f)
        scatter(pos_points(:,1),pos_points(:,2),1,'blue')
        hold on 
        plot(center(1),center(2),'r*')
        quiver(pos_all(:,1),pos_all(:,2),alphaX_all,alphaZ_all,'r')
        xlim([0,500])
        ylim([-60,60])
        daspect([1 1 1])
        pause(0.1)
          
    end
    hold off
end

% Solve for velocity at each point
function [vel_points] = vel_func(pos_points,w)
vel_points = zeros(size(pos_points));
for i = 1:size(pos_points,1)
    xtemp = pos_points(i,1);
    ytemp = pos_points(i,2);
    r = sqrt(xtemp^2 + ytemp^2);
    angle = atan2(ytemp,xtemp);
    vel_points(i,1) = r*w+(sin(angle)*r*w);
    vel_points(i,2) = -(cos(angle)*r*w);
end
end

% find positions of points below depth
function [pos_all,beta_all] = pos_limit(pos_points,depth,beta)
pos_all = [];
beta_all = [];
for i = 1:size(pos_points,1)
    if pos_points(i,2)<depth
        pos_all = [pos_all;pos_points(i,:)];
        beta_all = [beta_all;beta(i)];
    end
end
end

% find beta, gamma
function [beta, gamma] = bg_func(pos_points,vel_points,beta)

gamma = zeros(size(pos_points,1),1);
for i = 1:size(pos_points,1)
 
    gTemp = pi+atan2(vel_points(i,2),vel_points(i,1));
    if gTemp >pi
        gTemp =2*pi-gTemp;
    end
    gamma(i) = gTemp;
end
end

% find total alphax and alphaz with give gamma and beta
function [alphaX_all,alphaZ_all] = alpha_all(pos_points,vel_points,beta)
    alphaX_all = zeros(size(pos_points,1),1);
    alphaZ_all = zeros(size(pos_points,1),1);
    [beta, gamma] = bg_func(pos_points,vel_points,beta);

    for i = 1:size(pos_points,1)
        [alphaX_all(i),alphaZ_all(i)]= alpha_func(beta(i),gamma(i));
    end
end

% find the local alphax and alphaz with give gamma and beta
function [alphaX, alphaZ] = alpha_func(beta,gamma)
% using discrete Fourier transform fitting function
% beta [-pi,pi]
% gamma [-pi,pi]
% Fourier coefficients M
% granular medium: generic coefficient
% define (-1 as M1 for simplicity)
A00 = 0.206;
A10 = 0.169;
B11 = 0.212;
B01 = 0.358;
BM11 = 0.055;
C11 = -0.124;
C01 = 0.253;
CM11 = 0.007;
D10 = 0.088;
M = [A00, A10, B11, B01, BM11, C11, C01, CM11, D10];
% scaling factor
sf = 1;


if beta >= -pi && beta <= -pi/2
    beta = beta + pi; 
elseif beta >= pi/2 && beta <= pi
    beta = beta - pi;
end

if gamma >= -pi && gamma <= -pi/2
    alphaZ = sf*(M(1)*cos(0)+M(2)*cos(2*(-beta))+M(3)*sin(2*(-beta)+(-pi-gamma))...
        +M(4)*sin((-pi-gamma))+M(5)*sin((-2*(-beta))+(-pi-gamma)));
    alphaX = -sf*(M(6)*cos(2*(-beta)+(-pi-gamma))+M(7)*cos((-pi-gamma))...
        +M(8)*sin(-2*(-beta)+(-pi-gamma))+M(9)*sin(2*(-beta)));
elseif gamma >= pi/2 && gamma <= pi
   alphaZ = sf*(M(1)*cos(0)+M(2)*cos(2*(-beta))+M(3)*sin(2*(-beta)+(pi-gamma))...
        +M(4)*sin((pi-gamma))+M(5)*sin((-2*(-beta))+(pi-gamma)));
    alphaX = -sf*(M(6)*cos(2*(-beta)+(pi-gamma))+M(7)*cos((pi-gamma))...
        +M(8)*sin(-2*(-beta)+(pi-gamma))+M(9)*sin(2*(-beta)));
else
    alphaZ = sf*(M(1)*cos(0)+M(2)*cos(2*beta)+M(3)*sin(2*beta+gamma)...
        +M(4)*sin(gamma)+M(5)*sin((-2*beta)+gamma));
    alphaX = sf*(M(6)*cos(2*beta+gamma)+M(7)*cos(gamma)...
        +M(8)*sin(-2*beta+gamma)+M(9)*sin(2*beta));
end

end

% Read dxf file of the wheel and plot the wheel shape
function pos_points = read_dxf(file_name)
dxf = DXFtool(file_name);
h = findobj(gca,'Type','line');
x=get(h,'Xdata');
y=get(h,'Ydata');

x1 = [];
y1 = [];
for i = 1:size(x,1)
    if iscell(x(i))
        xtemp = cell2mat(x(i));
        ytemp = cell2mat(y(i));
    else
        xtemp = x;
        ytemp = y;
    end
    si = size(xtemp,2);
    if si ==50
        x1 = [x1;xtemp];
        y1 = [y1;ytemp];
    elseif si ==2
        [xList, yList]= line_func (xtemp,ytemp,50);
        x1 = [x1;xList'];
        y1 = [y1;yList'];
    end
end
x1 = reshape(x1,[],1);
y1 = reshape(y1,[],1);

pos_points = [x1,y1];
    function [xList, yList]= line_func (x2,y2,num)
        xList = zeros(num,1);
        yList = zeros(num,1);
        for j = 1:50
            xList(j) = x2(1)+j*(x2(2)-x2(1))/(num+1);
            yList(j) = y2(1)+j*(y2(2)-y2(1))/(num+1);
        end
        
    end
end

% DRAW A CIRCLE :)
function pos_points = makecircle(rho)
    n = 250;
    pos_points = zeros(n,2);
    for i = 1:n
        theta = 2*pi/n*i;
        [pos_points(i,1),pos_points(i,2)] = pol2cart(theta,rho);
    end
end

function [beta,area] = initial_beta(pos_points)
pos_copy = pos_points;

[M,~] = size(pos_points);

beta = zeros(size(pos_points,1),1);
area = zeros(size(pos_points,1),1);
for i  = 1:M
    temp = repmat([pos_points(i,1),pos_points(i,2)],M,1);
    a = pos_copy-temp;
    all_dist= vecnorm(a,2,2);
    [sorted_dist,sorted_index]=sort(all_dist,'ascend');
    prev_points = pos_points(sorted_index(2),:);
    next_points = pos_points(sorted_index(3),:);
    Atemp = (sorted_dist(2)+sorted_dist(3))/2;
    bTemp = pi-atan2((next_points(2)-prev_points(2)),(next_points(1)-prev_points(1)));
    if bTemp >pi
        bTemp = bTemp-pi;
    end
    beta(i) = bTemp;
    area(i) = Atemp;
end
end
% translation and rotation
function [pos_new,center_new,beta] = rotate_update(pos_points,center,beta,theta,xt,zt)
    pos_new = zeros(size(pos_points));
    rel_points = zeros(size(pos_points));
    
    rel_points(:,1) = pos_points(:,1)- repmat(center(1),size(pos_points,1),1);
    rel_points(:,2) = pos_points(:,2)- repmat(center(2),size(pos_points,1),1);
    
    center_new = [center(1)+xt,center(2)+zt];
    for i = 1:size(rel_points,1)
        T = [cos(theta), -sin(theta),0;
            sin(theta), cos(theta), 0;
            0,0,1];
        pTemp = T*[rel_points(i,1);rel_points(i,2);1];
        pos_new(i,:) = [pTemp(1)+center_new(1),pTemp(2)+center_new(2)];
        beta(i) = beta(i)-theta;
        if beta(i) >pi
            beta(i) = beta(i)-pi;
        end
    end
    
    
end

%% Plot functions
%Plot velocity and position profile functions
function plt_vel(pos_points,vel_points)
figure()
hold on
quiver(pos_points(:,1),pos_points(:,2),vel_points(:,1),vel_points(:,2),'r')
scatter(pos_points(:,1),pos_points(:,2),1,'blue')
title('position and velocity')
axis equal
hold off
end

% plot alpha x and alpha z as a function of beta and gamma
function plot_alphaMap()

beta = linspace(-pi,pi,500);
gamma = linspace(-pi,pi,500);
outputZ = zeros(size(beta,2),size(gamma,2));
outputX = zeros(size(beta,2),size(gamma,2));
for i = 1: size(beta,2)
    for j = 1:size(gamma,2)
        [alphaX, alphaZ] = alpha_func(beta(i),gamma(j));
        outputX(i,j)= alphaX;
        outputZ(i,j)= alphaZ;
    end
end

figure()
hold on
imagesc([beta(1,1) beta(1,end)],[gamma(1,1) gamma(1,end)],outputX);
axis xy;
colormap();
colorbar;
xlabel('gamma');
ylabel('beta');
title('alphaX')
hold off
figure()
hold on
imagesc([beta(1,1) beta(1,end)],[gamma(1,1) gamma(1,end)],outputZ);
axis xy;
colormap();
colorbar;
xlabel('gamma');
ylabel('beta');
title('alphaZ')
hold off
end

% plot beta and gamma(Test)
function [u,v]= beta_plot(pos_points,vel_points,beta,gamma)
a = size(pos_points(:,1),1);
figure()
scatter(pos_points(:,1),pos_points(:,2),'.')
hold on
for k=1:a
      b = beta(k)*180/pi;
      g = gamma(k)*180/pi;
      text(pos_points(k,1),pos_points(k,2),num2str(g), 'FontSize', 17)
end
% quiver(pos_points(:,1),pos_points(:,2),u,v,'r')
axis equal
end
