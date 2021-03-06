clearvars, clc, clear path

N = 5000;%2*2600;%2600;                                                                  % length of simulation (dependent on the length of disturbance data)
controlType = 3;                                                                         % switch between on/off and MPC

%% ============================================ Control setup ======================================
specifications;
%dataLoad;                                                                               % Load when comparing lab results

%% ===================================  Build dynamics & optimization  =============================
simulator_builder;                                                                       % build simulator dynamics
if controlType == 2
    MPC_builder_PDE;
elseif controlType == 3
    MPC_builder_linear;
end

%% =====================================  Initial conditions  ======================================
X_sim(1,1) = 3.3;%x(1,1);                                                                 % init. tank1 state [m^3]
X_sim(2,1) = 3.8;%x(2,1);                                                                 % init. tank2 state [m^3]                                                                     % warm start - Lagrange multiplier initializer
X_sim(Nxt+1:Nxt+Nxp,1) = 0.01;                                                           % init. pipe states [m]

U_opt(1,1) = u1_off;
U_opt(2,1) = u2_off;

dt_sim = 0.5*t_resample/60;                                                               % sampling time [s]       

lam_g = 0;                                                                                % warm start - Lagrange multiplier initializer
x_init = 0.1;  

X_ref_design;                                                                             % Varying reference

%% ============================================  Forecasts  ========================================
load('D_sim')                                                                            % for testing reference tracking
%load('D_sim_mod')                                                                         % intensive rain events
%D_sim = D_sim_mod;

% Plot in real time
PlotType = 2;
if PlotType == 2
    U_opt = zeros(Nu,1);
    
    figure()
    subplot(2,2,1)
    x_plot{1} = plot(X_sim(1,:)');
    hold on
    plot(X_ref_sim(1,1:t_resample:end))
    subplot(2,2,2)
    xpl{2} = plot(X_sim(2,:)');
    hold on
    plot(X_ref_sim(2,1:t_resample:end))
    subplot(2,2,3)
    upl{1} = plot(U_opt(1,:)');
    subplot(2,2,4)
    upl{2} = plot(U_opt(2,:)');
end

%% ==============================================  Simulate  ======================================
disp('Simulator running')

for i = 1:1:N                                                       
      
    if controlType == 1
        onoff_control;
    elseif controlType == 2    
        tic
        [X_MPC,U_MPC,S_MPC,Y_MPC,lam_g,x_init] = OCP(X_sim(:,i), D_sim(:,(i)*(t_step)-(t_step-1):t_step:(i-1)*t_step + (Hp)*t_step-(t_step-1)), P_sim, X_ref_sim(:,(i)*(t_step)-(t_step-2):t_step:(i-1)*t_step + (Hp)*t_step-(t_step-2)), lam_g, x_init, dt_sim);
        toc
        U_opt(:,i) = full(U_MPC(:,1));
        S_opt(:,i) = full(S_MPC);
        X_opt{i} = full(X_MPC);
    elseif controlType == 3    
        tic
        % Operating point
        X_sim_OP = X_sim(:,i);      U_sim_OP = U_opt(:,i);      D_sim_OP = D_sim(:,(i)*(t_step)-(t_step-1));
        % Linearization around OP
        compute_linear;
        [X_MPC,U_MPC,S_MPC,lam_g,x_init] = OCP(X_sim(:,i), D_sim(:,(i)*(t_step)-(t_step-1):t_step:(i-1)*t_step + (Hp)*t_step-(t_step-1)), A_cont, B_cont, E_cont, X_sim_OP, U_sim_OP, D_sim_OP, X_ref_sim(:,(i)*(t_step)-(t_step-2):t_step:(i-1)*t_step + (Hp)*t_step-(t_step-2)), lam_g, x_init, dt_sim);
        toc
        U_opt(:,i+1) = full(U_MPC(:,1));
        S_opt(:,i) = full(S_MPC);
        X_opt{i} = full(X_MPC);
    end
    
    % Linear dynamics simulator
    X_sim_lin(:,i+1) = full(F_integral_lin(X_sim_lin(:,i), U_opt(:,i+1), D_sim(:,1 + (i-1)*t_resample), A_cont, B_cont, E_cont, X_sim_OP, U_sim_OP, D_sim_OP, dt_sim ));
 
    % Lab simulator
    X_sim(:,i+1) = full(F_integral_sim(X_sim(:,i), U_opt(:,i+1), D_sim(:,1 + (i-1)*t_resample), P_sim, dt_sim ));
    
    progressbar(i/N)
    
    if PlotType == 2 && mod(i,5)==0
        drawPlots;
        i
        drawnow
    end
end

%% Static plots
  plotResults;      
  
%% test predictions

t = 1:Hp+1;
figure
for i = 1:N
    plot(t,X_opt{i}(1,:))
    hold on
    t = t+1;
end




