% Homework 4
% Problem 2b
% Qishun Yu

function problem2b
clear all
close all
clc
x0 = [-0.3127; 4.5]; 

threshold = 0.0001;

x= x0;
xP = x - P(x);
coeff = 0.0001;

while (norm(xP) > threshold)
    xP = x - P(x);
    dxP = eye(2) - dPdx(x)+coeff*eye(2);
    x = x - dxP\xP;

end

fixedPoint = x


J = dPdx(fixedPoint);
eig(J)

    function xNext = P(x)
        %Parameters
        m = 1; l = 1; g = 9.8; alpha = pi/8; gamma = 0.08;
        
        theta = x(1);
        thetadot = x(2);
        
        w1 = sqrt((2*g/l)*(1 - cos(gamma-alpha)));
        w2 = -sqrt((2*g/l)*(1 - cos(alpha-gamma)));
        
        if (thetadot>w1)
            thetadotNext = cos(2*alpha)*sqrt((thetadot)^2 + 4*(g/l)*sin(alpha)*sin(gamma));
        elseif (thetadot<=w1) && (thetadot>=w2)
            thetadotNext = -thetadot*cos(2*alpha);
        else
            thetadotNext = -cos(2*alpha)*sqrt((thetadot)^2 + 4*(g/l)*sin(alpha)*sin(gamma));
        end
        
        xNext = [theta; thetadotNext];
    end


    function jacobian = dPdx(x)
        %Parameters
        coeff = 1; l = 1; g = 9.8; alpha = pi/8; gamma = 0.08;
        
        theta = x(1);
        thetadot = x(2);
           
        w1 = sqrt((2*g/l)*(1 - cos(gamma-alpha)));
        w2 = -sqrt((2*g/l)*(1 - cos(alpha-gamma)));
        
        if (thetadot>w1)
            thetadotNext = (cos(2*alpha)*thetadot)/sqrt(thetadot^2 + (4*g*sin(alpha)*sin(gamma))/l);
        elseif (thetadot<=w1) && (thetadot>=w2)
            thetadotNext = -cos(2*alpha);
        else
            thetadotNext = -(cos(2*alpha)*thetadot)/sqrt(thetadot^2 + (4*g*sin(alpha)*sin(gamma))/l);
        end
        
        jacobian = [1 0;
            0 thetadotNext];
    end
end