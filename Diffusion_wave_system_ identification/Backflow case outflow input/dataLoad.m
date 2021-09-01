%% Simulink workspace load
% """
% 1     : tank 1 level
% 2:5   : pipe levels
% 6     : NAN (tank2 inflow) 
% 7     : tank2 level
% 8     : Gravity pipe inflow (input) 
% 9     : tank2 outflow 
% 10    : tank2 area 
% 11    : real time 
% 12    : lateral inflow (disturbance)
% 13    : inflow to tank1 
% 14:17 : [pump1_ref(inflow), pump2_ref(outflow), inflow to tank1_ref, lateral inflow_ref]
% """

addpath('data');

%load('.\data\dataSet_nominal');
load('.\data\Simulation_data_CCTA_v4');

labRes = ans;

% Outflow calculation from tank2
labRes.Data(1:end-1,6) = (2/10^6)*labRes.Data(1,10)*(labRes.Data(2:end,7) - labRes.Data(1:end-1,7)) + 1*uConv(labRes.Data(1:end-1,9),'mTos');

% Levels
    %labRes.Data(:,2:5);
% Inflow
    %labRes.Data(:,8);
% Outflow
    %hampel(smooth(labRes.Data(1:end-1,6)),10);
% disturbance
    %labRes.Data(:,12)

    %%
% figure
% ax(1) = subplot(3,1,1);
% plot(uConv(labRes.Data(1:end-1,9),'mTos'),'blue');
% hold on
% plot(hampel(smooth(labRes.Data(1:end-1,6)),10),'red');
% grid on
% ylim([-0.05,0.2])
% ax(2) = subplot(3,1,2);
% plot(smooth(labRes.Data(1:end-1,6)));
% grid on
% ylim([-0.05,0.2])
% ax(3) = subplot(3,1,3);
% plot(labRes.Data(1:end-1,7),'red');
% grid on
% linkaxes(ax,'x')

