%% Jacobians
compute_A_discrete = casadi.Function('A',{x_MPC,u_MPC,d_MPC,p_MPC,dt_MPC},{jacobian(F_integral(x_MPC,u_MPC,d_MPC,p_MPC,dt_MPC),x_MPC)}); 
compute_B_discrete = casadi.Function('B',{x_MPC,u_MPC,d_MPC,p_MPC,dt_MPC},{jacobian(F_integral(x_MPC,u_MPC,d_MPC,p_MPC,dt_MPC),u_MPC)});
compute_E_discrete = casadi.Function('E',{x_MPC,u_MPC,d_MPC,p_MPC,dt_MPC},{jacobian(F_integral(x_MPC,u_MPC,d_MPC,p_MPC,dt_MPC),d_MPC)});

%% (A,B,E) matrices
A_disc = full(compute_A_discrete(X_sim_OP,U_sim_OP,D_sim_OP,P_sim,dt_sim));
B_disc = full(compute_B_discrete(X_sim_OP,U_sim_OP,D_sim_OP,P_sim,dt_sim));
E_disc = full(compute_E_discrete(X_sim_OP,U_sim_OP,D_sim_OP,P_sim,dt_sim));

%% Jacobians continuous
compute_A_cont = casadi.Function('A_cont',{x_MPC,u_MPC,d_MPC,p_MPC},{jacobian(dynamics_MPC(x_MPC,u_MPC,d_MPC,p_MPC),x_MPC)});
compute_B_cont = casadi.Function('B_cont',{x_MPC,u_MPC,d_MPC,p_MPC},{jacobian(dynamics_MPC(x_MPC,u_MPC,d_MPC,p_MPC),u_MPC)});
compute_E_cont = casadi.Function('E_cont',{x_MPC,u_MPC,d_MPC,p_MPC},{jacobian(dynamics_MPC(x_MPC,u_MPC,d_MPC,p_MPC),d_MPC)});
%% (A,B,E) continuous matrices
A_cont = full(compute_A_cont(X_sim_OP,U_sim_OP,D_sim_OP,P_sim));
B_cont = full(compute_B_cont(X_sim_OP,U_sim_OP,D_sim_OP,P_sim));
E_cont = full(compute_E_cont(X_sim_OP,U_sim_OP,D_sim_OP,P_sim));

% %% 1-step Prediction
% if i >= 3  
%     OP_u = U_opt(:,i-1);
%     OP_x = X_sim(:,i-2);
%     OP_d = D_sim(:,(i-1)*(t_step)-(t_step-1));
%     
%     X_sim_lin(:,i) = OP_x + A_disc*(X_sim(:,i-1)-OP_x) + B_disc*(U_opt(:,i)-OP_u) + E_disc*(D_sim(:,(i)*(t_step)-(t_step-1))-OP_d);               
% end

%% Hp-step prediction
% 
% if i >= 3  
%     OP_u = U_opt(:,i-1);
%     OP_x = X_sim(:,i-2);
%     OP_d = D_sim(:,(i-1)*(t_step)-(t_step-1));
% end