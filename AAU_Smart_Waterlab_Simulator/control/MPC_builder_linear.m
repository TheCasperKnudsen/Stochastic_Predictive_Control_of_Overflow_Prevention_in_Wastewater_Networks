%% casadi_builder.m
% """
% Builds optimization problem
% """
addpath('C:\Users\74647\OneDrive - Grundfos\MATLAB\CasAdi') 
import casadi.*
opti = casadi.Opti();                                       % Optimization problem

% Nonlinear dynamics integration
dt_MPC_lin = casadi.MX.sym('dt_lin',1);  
x_MPC_lin = casadi.MX.sym('x_lin',Nxt+ Nxp);
u_MPC_lin = casadi.MX.sym('u_lin',Nxt);
d_MPC_lin = casadi.MX.sym('d_lin',ND);                              % [d_t1, d_t2, d_p]
A_MPC_lin = casadi.MX.sym('A_lin',Nx,Nx);    
B_MPC_lin = casadi.MX.sym('B_lin',Nx,Nu);       
E_MPC_lin = casadi.MX.sym('E_lin',Nx,ND);       

% Linear dynamics integration
k1_MPC_lin = dynamics_MPC_linear(x_MPC_lin, u_MPC_lin, d_MPC_lin, A_MPC_lin, B_MPC_lin, E_MPC_lin);
k2_MPC_lin = dynamics_MPC_linear(x_MPC_lin + dt_MPC_lin / 2.0 * k1_MPC_lin, u_MPC_lin, d_MPC_lin, A_MPC_lin, B_MPC_lin, E_MPC_lin);
k3_MPC_lin = dynamics_MPC_linear(x_MPC_lin + dt_MPC_lin / 2.0 * k2_MPC_lin, u_MPC_lin, d_MPC_lin, A_MPC_lin, B_MPC_lin, E_MPC_lin);
k4_MPC_lin = dynamics_MPC_linear(x_MPC_lin + dt_MPC_lin * k3_MPC_lin, u_MPC_lin, d_MPC_lin, A_MPC_lin, B_MPC_lin, E_MPC_lin);

if intMethod == 1
    xf = x_MPC_lin + dt_MPC_lin / 6.0 * (k1_MPC_lin + 2 * k2_MPC_lin + 2 * k3_MPC_lin + k4_MPC_lin);
    F_integral_lin = casadi.Function('F_RK4_lin', {x_MPC_lin, u_MPC_lin, d_MPC_lin, A_MPC_lin, B_MPC_lin, E_MPC_lin, dt_MPC_lin}, {xf}, {'x[k]','u[k]','d[k]','A','B','E','dt'},{'x[k+1]'});
elseif intMethod == 2
    xf = x_MPC_lin + dt_MPC_lin*dynamics_MPC_linear(x_MPC_lin, u_MPC_lin, d_MPC_lin, A_MPC_lin, B_MPC_lin, E_MPC_lin);
    F_integral_lin = casadi.Function('F_EUL_lin', {x_MPC_lin, u_MPC_lin, d_MPC_lin, A_MPC_lin, B_MPC_lin, E_MPC_lin, dt_MPC_lin}, {xf}, {'x[k]','u[k]','d[k]','A','B','E','dt'},{'x[k+1]'});
end