%% Define datasets

% measurements
u_meas_DW = input_v(:,1);
d_meas_DW = input_v(:,2);
y_meas_DW = output_v(:,end);
h_meas_DW = output_v(:,1:end-1);

% model
y_model_DW = y_final.OutputData(:,end);
h_model_DW = y_final.OutputData(:,1:end-1);

%%
save('C:\Users\74647\OneDrive - Grundfos\Documentations\CCTA2021\Experimental results\Backflow case\SaveData\u_meas_DW','u_meas_DW')
save('C:\Users\74647\OneDrive - Grundfos\Documentations\CCTA2021\Experimental results\Backflow case\SaveData\d_meas_DW','d_meas_DW')
save('C:\Users\74647\OneDrive - Grundfos\Documentations\CCTA2021\Experimental results\Backflow case\SaveData\y_meas_DW','y_meas_DW')
save('C:\Users\74647\OneDrive - Grundfos\Documentations\CCTA2021\Experimental results\Backflow case\SaveData\h_meas_DW','h_meas_DW')

save('C:\Users\74647\OneDrive - Grundfos\Documentations\CCTA2021\Experimental results\Backflow case\SaveData\y_model_DW','y_model_DW')
save('C:\Users\74647\OneDrive - Grundfos\Documentations\CCTA2021\Experimental results\Backflow case\SaveData\h_model_DW','h_model_DW')
