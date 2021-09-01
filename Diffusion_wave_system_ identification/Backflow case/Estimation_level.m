clear all;            
clear path;
clc;  
%% ================================================ Prepare data ==============================================
dataLoad;                                                                       % Load simulation data 

startData = 5000;%500; 
endData = 2*12000;%size(labRes.Data,1);

Nx = 8;
Nx_meas = 4;   

% training set
h(1:Nx_meas,:) = labRes.Data(startData:endData-1,2:2+Nx_meas-1)'/100;
Q(1,:) = uConv(labRes.Data(startData:endData-1,8),'mTos');                         % Select in/outflow
Q(2,:) = hampel(smooth(labRes.Data(startData:endData-1,6)),14);
Q(3,:) = uConv(labRes.Data(startData:endData-1,12),'mTos');     

% validation set
h_v(1:Nx_meas,:) = labRes.Data(startData:end-1,2:2+Nx_meas-1)'/100;
Q_v(1,:) = uConv(labRes.Data(startData:end-1,8),'mTos');                           % Select in/outflow
Q_v(2,:) = hampel(smooth(labRes.Data(startData:end-1,6)),14);
Q_v(3,:) = uConv(labRes.Data(startData:end-1,12),'mTos');     

%% ============================================ Idata object ================================================ 
T_down = 4;%2;
Ts_data = T_down*0.5;                                                                    % [10min] in simulation/control 

% training Idata
input = [Q(1,1:T_down:end); Q(3,1:T_down:end)]';
output = [h(1:end,1:T_down:end); Q(2,1:T_down:end)]';
data = iddata(output,input,Ts_data);                                            % (y,u,Ts) order
data.TimeUnit = 'seconds';

% validation Idata
input_v = [Q_v(1,1:T_down:end); Q_v(3,1:T_down:end)]';
output_v = [h_v(1:end,1:T_down:end); Q_v(2,1:T_down:end)]';
data_v = iddata(output_v,input_v,Ts_data);                                            % (y,u,Ts) order
data_v.TimeUnit = 'seconds';

plot(input)
hold on
plot(output(:,end))

%% ===================================================== Model ============================================

modelName = 'model_cont';
Ts_model = 0;                                                                   % 0 - continuous model, 1,2,.. - discrete model 
order = [size(output,2) size(input,2) Nx];                                      % [Ny Nu Nx] order

%/p = [0.0349   9.2717   0.25  0.05]; %working
%p = [0.0349   15.2717   0.5  0.08];   %working
 p = [0.0949   15.2717   0.5  0.08];
params = [p, Nx];

initStates = 0.0001*ones(Nx, 1);                                                % assume 0 flow at t0
sys_init = idnlgrey(modelName, order, params, initStates, Ts_model);            % create nlgreyest object

sys_init.TimeUnit = 'seconds';
sys_init.Parameters(1).Name = 'p1';
sys_init.Parameters(2).Name = 'p2';
sys_init.Parameters(3).Name = 'p3';
sys_init.Parameters(4).Name = 'z';
sys_init.Parameters(5).Name = 'Nx';
sys_init.Parameters(end).Fixed = true;                                            % number of sections fixed

sys_init.SimulationOptions.AbsTol = 1e-10;
sys_init.SimulationOptions.RelTol = 1e-8;
sys_init.SimulationOptions.Solver = 'ode4';                                     % 4th order Runge-Kutte solver - fixed-step size             

sys_init.Parameters(1).Minimum = 0.001;     %sys_init.Parameters(1).Maximum = 0.5;   % parameter constraints
sys_init.Parameters(2).Minimum = 0.001;     %sys_init.Parameters(2).Maximum = 10000;
sys_init.Parameters(3).Minimum = 0.001;     %sys_init.Parameters(3).Maximum = 100;
sys_init.Parameters(4).Minimum = 0.00001;   %sys_init.Parameters(4).Maximum = 0.1;

for i = 1:Nx
sys_init.InitialStates(i).Minimum = 0.000001;                                   % States lower bound constraints
sys_init.InitialStates(i).Maximum = 1;                                          % States upper bound constraints
end

sys_init = setinit(sys_init, 'Fixed', false(Nx,1));

%% ============================================= Solver options ============================================
opt = nlgreyestOptions;
%Search methods: 'gn' | 'gna' | 'lm' | 'grad' | 'lsqnonlin' | 'auto'
opt.SearchMethod = 'gna'; 
opt.Display = 'on';
opt.SearchOption.MaxIter = 100;
opt.SearchOption.Tolerance = 1e-15; 

%% =============================================== Estimation =============================================
tic 
sys_final = nlgreyest(data,sys_init, opt)                                       % Parameter estimation START

fprintf('\n\nThe search termination condition:\n')
sys_final.Report.Termination

estParams = [sys_final.Parameters(1).Value,...
             sys_final.Parameters(2).Value,...
             sys_final.Parameters(3).Value,...
             sys_final.Parameters(4).Value];

finalStates = sys_final.Report.Parameters.X0;                                   % estimated initial states
toc

%% ========================================== Estimated model ============================================
opt_init = simOptions('InitialCondition',initStates);                           % Simulate model on training data with initial parameters
y_init = sim(sys_init,data,opt_init);

opt_final = simOptions('InitialCondition',finalStates);                         % Simulate model on training data with estimated parameters
y_final = sim(sys_final,data_v,opt_final);

%% ========================================== Post - process ============================================
estParams
EstPlotter;

%%
% saveEnabler = 0;
% if saveEnabler == 1
%     switch Nx_meas
%         case 4
%             p_grav_Nx4 = estParams;
%             save('data\p_grav_Nx4','p_grav_Nx4')
%         case 6
%             p_grav_Nx6 = estParams;
%             save('data\p_grav_Nx6','p_grav_Nx6')
%     end 
% end



            
            
            
            