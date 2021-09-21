%% Jacobians
compute_A = casadi.Function('A',{x_MPC,u_MPC,d_MPC,p_MPC,dt_MPC},{jacobian(F_integral(x_MPC,u_MPC,d_MPC,p_MPC,dt_MPC),x_MPC)}); 
compute_B = casadi.Function('B',{x_MPC,u_MPC,d_MPC,p_MPC,dt_MPC},{jacobian(F_integral(x_MPC,u_MPC,d_MPC,p_MPC,dt_MPC),u_MPC)});
compute_E = casadi.Function('E',{x_MPC,u_MPC,d_MPC,p_MPC,dt_MPC},{jacobian(F_integral(x_MPC,u_MPC,d_MPC,p_MPC,dt_MPC),d_MPC)});

%% (A,B) matrices
A = full(compute_A(X_sim(:,i),U_opt(:,i),D_sim(:,(i)*(t_step)-(t_step-1)),P_sim,dt_sim));
B = full(compute_B(X_sim(:,i),U_opt(:,i),D_sim(:,(i)*(t_step)-(t_step-1)),P_sim,dt_sim));
E = full(compute_E(X_sim(:,i),U_opt(:,i),D_sim(:,(i)*(t_step)-(t_step-1)),P_sim,dt_sim));

%% Prediction

%X_sim_lin(:,i) = X_sim(:,i-2) + A*X_sim(:,i) + B*U_opt(:,i) + E*D_sim(:,(i)*(t_step)-(t_step-1));

if i >= 3

X_sim_lin(:,i) = X_sim(:,i-2) + A*(X_sim(:,i-1) - X_sim(:,i-2)) + B*(U_opt(:,i) - U_opt(:,i-1)) + E*(D_sim(:,(i)*(t_step)-(t_step-1)) - D_sim(:,(i-1)*(t_step)-(t_step-1)));

end