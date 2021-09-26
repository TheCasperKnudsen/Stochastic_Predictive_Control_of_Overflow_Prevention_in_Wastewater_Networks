function [dx] = dynamics_MPC_linear(x,u,d,A,B,E)
% """ Model dynamics description """
% Continuous dynamics defined in the form: dx/dt = A*x + B*u + E*d
% 
% Parameters:   [A,B,E]
% Disturbances: [d(1),d(2)]
% Inputs:       [u(1),u(2)]
% States:       [ht1,ht2 | hp1,hp2,hp3,hp4]
% """
% Nxp = 4; 
% Nxt = 2;
% dx = casadi.MX.zeros(Nxt+ Nxp,1); 

dx = A*x + B*u + E*d;
    
end