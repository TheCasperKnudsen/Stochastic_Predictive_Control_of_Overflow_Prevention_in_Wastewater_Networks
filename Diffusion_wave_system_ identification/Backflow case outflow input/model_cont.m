function [dx, y] = model_cont(t, x, u, p1, p2, p3, z, Nx, varargin)
% Discrete time nlgreyest model for the kinematic wave level-based model. 

dx = zeros(Nx,1);
y = zeros(4,1);
%% State equation
dx(1) =  p1*u(1) - p1*p2*(((x(1)-x(2)+z)*g(x(1)))^(0.5)); 

for i = 2:Nx 
    if i == 5
        dx(i) =  p1*p2*(((x(i-1)-x(i)+z)*g(x(i-1)))^(0.5) - ((x(i)-x(i+1)+z)*g(x(i)))^(0.5)) + p1*u(2); 
    elseif i == Nx
        dx(i) = -p1*u(3) + p1*p2*(((x(i-1)-x(i)+z)*g(x(i-1)))^(0.5));
    else
        dx(i) =  p1*p2*(((x(i-1)-x(i)+z)*g(x(i-1)))^(0.5) - ((x(i)-x(i+1)+z)*g(x(i)))^(0.5));  
    end
end

%% Output equations

if Nx == 8 
    y(1) = x(2);
    y(2) = x(4);
    y(3) = x(5);
    y(4) = x(7);
    %y(5) = p2*((z*g(x(i)))^(0.5)); %p2*g(x(Nx));   
elseif Nx == 16
    y(1) = x(4);
    y(2) = x(8);
    y(3) = x(10);
    y(4) = x(14);
    %y(5) = p2*((z*g(x(i)))^(0.5));     
end

%% g(z) non-linear function 
function y = g(zz)    
    y = ((zz).^(3)) ./ (zz + p3);        
end
end