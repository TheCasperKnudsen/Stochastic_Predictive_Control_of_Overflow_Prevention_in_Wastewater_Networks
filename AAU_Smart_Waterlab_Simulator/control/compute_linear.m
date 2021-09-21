%% Jacobians
compute_A = casadi.Function('A',{x_MPC,u_MPC,d_MPC,p_MPC,dt_MPC},{jacobian(F_integral(x_MPC,u_MPC,d_MPC,p_MPC,dt_MPC),x_MPC)}); 
compute_B = casadi.Function('B',{x_MPC,u_MPC,d_MPC,p_MPC,dt_MPC},{jacobian(F_integral(x_MPC,u_MPC,d_MPC,p_MPC,dt_MPC),u_MPC)});
compute_E = casadi.Function('E',{x_MPC,u_MPC,d_MPC,p_MPC,dt_MPC},{jacobian(F_integral(x_MPC,u_MPC,d_MPC,p_MPC,dt_MPC),d_MPC)});

%% (A,B,E) matrices
A = full(compute_A(X_sim(:,i),U_opt(:,i),D_sim(:,(i)*(t_step)-(t_step-1)),P_sim,dt_sim));
B = full(compute_B(X_sim(:,i),U_opt(:,i),D_sim(:,(i)*(t_step)-(t_step-1)),P_sim,dt_sim));
E = full(compute_E(X_sim(:,i),U_opt(:,i),D_sim(:,(i)*(t_step)-(t_step-1)),P_sim,dt_sim));

%% Prediction

if i >= 3
    
    X_sim_lin(:,i) = X_sim(:,i-2) + A*(X_sim(:,i-1) - X_sim(:,i-2)) + B*(U_opt(:,i) - U_opt(:,i-1)) + E*(D_sim(:,(i)*(t_step)-(t_step-1)) - D_sim(:,(i-1)*(t_step)-(t_step-1)));
    
%     X_sim_lin(1,i) =  + A(1,1)*X_sim(1,i) + B(1,1)*U_opt(1,i) + E(1,1)*D_sim(1,(i)*(t_step)-(t_step-1));
%     X_sim_lin(2:end,i) = X_sim(2:end,i-2) + A(2:end,2:end)*(X_sim(2:end,i-1) - X_sim(2:end,i-2)) + B(2:end,2:end)*(U_opt(2:end,i) - U_opt(2:end,i-1)) + E(2:end,2:end)*(D_sim(2:end,(i)*(t_step)-(t_step-1)) - D_sim(2:end,(i-1)*(t_step)-(t_step-1)));
end