
%% Initial system comparison

modelName = 'model_cont';
Ts_model = 0;                                                                   % 0 - continuous model, 1,2,.. - discrete model
order = [size(output,2) size(input,2) Nx];                                      % [Ny Nu Nx] order

%p = [0.01, 2, 0.1];
%p = [0.0949   15.2717   0.5  0.08];
%p = [0.04   10   0.5  0.05];
p = [0.05   6   2  0.05];

params = [p, Nx];

initStates = 0.0001*ones(Nx, 1);                                                % assume 0 flow at t0
sys_init = idnlgrey(modelName, order, params, initStates, Ts_model);            % create nlgreyest object

sys_init.TimeUnit = 'seconds';
sys_init.Parameters(1).Name = 'p1';
sys_init.Parameters(2).Name = 'p2';
sys_init.Parameters(3).Name = 'p3';
sys_init.Parameters(3).Name = 'z';
sys_init.Parameters(4).Name = 'Nx';
sys_init.Parameters(end).Fixed = true;                                            % number of sections fixed

sys_init.SimulationOptions.AbsTol = 1e-10;
sys_init.SimulationOptions.RelTol = 1e-8;
sys_init.SimulationOptions.Solver = 'ode4';                                     % 4th order Runge-Kutte solver - fixed-step size

sys_init.Parameters(1).Minimum = 0.001;     %sys_init.Parameters(1).Maximum = 0.5;   % parameter constraints
sys_init.Parameters(2).Minimum = 0.001;     %sys_init.Parameters(2).Maximum = 10000;
sys_init.Parameters(3).Minimum = 0.001;     %sys_init.Parameters(3).Maximum = 100;
sys_init.Parameters(4).Minimum = 0.00001;

for i = 1:Nx
sys_init.InitialStates(i).Minimum = 0.000001;                                   % States lower bound constraints
sys_init.InitialStates(i).Maximum = 1;                                          % States upper bound constraints
end

sys_init = setinit(sys_init, 'Fixed', false(Nx,1));

opt_init = simOptions('InitialCondition',initStates);                           % Simulate model on training data with initial parameters
y_init = sim(sys_init,data,opt_init);

plotEnbaler = 1;
if plotEnbaler == 1
figure
for i = 1:1:Nx_meas
ax(i) = subplot(size(output,2),1,i);
plot(output(:,i),'b','LineWidth',0.5)
hold on
plot(y_init.OutputData(:,i),'r','LineWidth',0.5)
ylabel(['$h$' num2str(i) '[$m$]'],'interpreter','latex');
leg = legend('Data','Model','Location','NorthEast');
set(leg, 'Interpreter', 'latex');
end
linkaxes(ax, 'x')
end
% Initial outlet flow comparison
% Calculate outlet flow with initial parameters
%Qout_init = params(2) *  g(y_init.OutputData(:,Nx),params(3));
% Qout_init = y_init.OutputData(:,Nx_meas+1);
% 
% figure
% plot(output(:,end),'b','LineWidth',0.5)
% hold on
% plot(Qout_init,'r','LineWidth',0.5)
% ylabel('$Q_{out}$  [$\frac{m^3}{h}$]','interpreter','latex');
% xlabel('Time [10 min]','interpreter','latex');
% leg = legend('Data','Model','Location','NorthEast');
% set(leg, 'Interpreter', 'latex');
% title('Outflow calculation with initial parameters','interpreter','latex')
% ylim([0,0.2])
% end

%% Estimated system comparison
plotEnbaler = 1;
if plotEnbaler == 1
figure
for i = 1:1:Nx_meas
ax(i) = subplot(size(output_v,2),1,i);
plot(output_v(:,i),'b','LineWidth',0.5)
hold on
plot(y_final.OutputData(:,i),'r','LineWidth',0.5)
hold on
patch([endData/T_down,length(output_v(:,1)), length(output_v(:,1)),endData/T_down],[-1,-1,1,1],'yellow','FaceAlpha',0.1,'LineStyle','--')
leg = legend('Data','Model','Location','NorthEast');
set(leg, 'Interpreter', 'latex');
ylabel(['$h$' num2str(i) '[$dm$]'],'interpreter','latex');
ylim([0, max(y_final.OutputData(:,1)) + 0.03])
xlim([0, length(output_v(:,1))])
if i == 1
    title('Water level estimation','interpreter','latex')
end
end
%xlabel('Time [s]','interpreter','latex');
end

% Estimated outlet flow comparison
% Calculate outlet flow with estimated parameters
%Qout_est = estParams(2) *  g(y_final.OutputData(:,Nx),estParams(3));
% Qout_est = y_final.OutputData(:,Nx_meas+1);
% 
% ax(Nx_meas+1) = subplot(size(output_v,2),1,Nx_meas+1);
% plot(smooth(output_v(:,end)),'b','LineWidth',0.5)
% hold on
% plot(Qout_est,'r','LineWidth',1)
% hold on
% patch([endData/T_down,length(output_v(:,1)), length(output_v(:,1)),endData/T_down],[-1,-1,1,1],'yellow','FaceAlpha',0.1,'LineStyle','--')
% ylabel('$Flow$  [$\frac{l}{s}$]','interpreter','latex');
% xlabel('Time [s]','interpreter','latex');
% leg = legend('Data','Model','Location','NorthEast');
% set(leg, 'Interpreter', 'latex');
% title('Discharge flow estimation','interpreter','latex')
% ylim([0,0.3])
% xlim([0, length(output_v(:,1))])
% 
% linkaxes(ax, 'x')

%% g(z) non-linear function 
% function y = g(z,p3)    
%     y = ((z).^(5/3)) ./ ((z + p3).^(2/3));        
% end
