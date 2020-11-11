clear 
close all
clc
%%
save_plots = false;

% path = '20191022/';
% path = '20191109_001_for_plots/';
% path = '20191113_001_for_plots/';
path = '/home/humberto/cam_imu_bags_Nov112020/';


estimated = readtable([path, 'cam_imu_for_plots_Nov112020_001__slash_imu_calib_result.csv'], 'Delimiter',',','ReadVariableNames',true);
truth = readtable([path, 'cam_imu_for_plots_Nov112020_001__slash_calib_truth.csv'], 'Delimiter',',','ReadVariableNames',true);
% expected_measurements = readtable([path, 'expected_measurements.csv'], 'Delimiter',',','ReadVariableNames',true);

%%

estimated_time = table2array(estimated(:,1));
truth_time = table2array(truth(:,1));
% measure_time = table2array(expected_measurements(:,1));

% estimated_time = (estimated_time - estimated_time(1)) * 1e-9;
% truth_time = (truth_time - truth_time(1))* 1e-9;


estimated_world_to_imu_pose_idx = [11:13, 15:18];
estimated_imu_to_camera_pose_idx = [21:23, 25:28];
estimated_velocity_idx = 30:32;
estimated_gyro_bias_idx = 38:40;
estimated_accel_bias_idx = 34:36;
estimated_sigma_minus_idx = 42;
estimated_sigma_plus_idx = 43;

truth_world_to_imu_pose_idx = [11:13, 15:18];
truth_velocity_idx = 30:32;
%%

sigma_plus_table = table2array(estimated(:,estimated_sigma_plus_idx));
sigma_minus_table = table2array(estimated(:,estimated_sigma_minus_idx));

estimated_sigma_plus = zeros(length(estimated_time), 21);
estimated_sigma_minus = zeros(length(estimated_time), 21);

for i = 1:length(estimated_time)
    temp = sigma_plus_table{i};
    temp(1)  =[];
    temp(end) = [];
    
    estimated_sigma_plus(i,:) = str2num(temp); 
    
    temp = sigma_minus_table{i};
    temp(1)  =[];
    temp(end) = [];
    
    estimated_sigma_minus(i,:) = str2num(temp);    
end

% % expected_measurement = zeros(length(measure_time),32);
% % exp_temp = table2array(expected_measurements(:,9));
% actual_measurement = zeros(length(measure_time),32);
% % act_temp = table2array(expected_measurements(:,10));
% measurement_std = zeros(length(measure_time),32);
% % cov_temp = table2array(expected_measurements(:,11));
% 
% for i = 1:length(measure_time)
% %     temp = exp_temp{i};
% %     expected_measurement(i,:) = str2num(temp);
%     
%     temp = act_temp{i};
%     actual_measurement(i,:) = str2num(temp);
%     
%     temp = cov_temp{i};
%     measurement_std(i,:) = sqrt (str2num(temp));   
%     
% end

%%
% 1 : Time
% 2-5 : Estimated PX,  PY,  PZ 
% 6-8 : Estimated QX,  QY,  QZ, QW
% 9-12 : True PX,  PY,  PZ 
% 13-15 : True QX,  QY,  QZ, QW
% 16-18 : Estimated  U,   V,   W
% 19-21 : True  U,   V,   W
% 22-30 : Estimated Sigma Plus (QX,  QY,  QZ,  PX,  PY,  PZ, U,   V, W)
% 31-39 : Estimated Sigma Minus (QX,  QY,  QZ,  PX,  PY,  PZ, U,   V, W)

compiled_data = zeros(length(truth_time), 39);
for i = 1:length(truth_time)
    
    t_diff = abs(estimated_time - truth_time(i));
    idx = find(t_diff == min(t_diff),1);
    compiled_data(i,:) = [truth_time(i), table2array(estimated(idx,estimated_world_to_imu_pose_idx)), ...
        table2array(truth(i,estimated_world_to_imu_pose_idx)),  table2array(estimated(idx,estimated_velocity_idx)), ...
        table2array(truth(i,estimated_velocity_idx)), estimated_sigma_plus(idx,1:9), estimated_sigma_minus(idx,1:9)];    
    
end
compiled_data(:,1) =  (compiled_data(:,1) - compiled_data(1,1)) * 1e-9;


%%

figure(1)
sgtitle('World to IMU Translation (XYZ)')
subplot(3,1,1)
h1 = plot(compiled_data(:,1),compiled_data(:,2) , 'r','LineWidth',2);
hold on
h2 = plot(compiled_data(:,1),compiled_data(:,9), 'k','LineWidth',1.5);
h3= plot(compiled_data(:,1),compiled_data(:,25) , '--b','LineWidth',1.0);
h4= plot(compiled_data(:,1),compiled_data(:,34) , '--b','LineWidth',1.0);
hold off
title('X component (m)')
legend([h1, h2, h3], {'Estimate','Truth','3\sigma'})

subplot(3,1,2)
h1 = plot(compiled_data(:,1),compiled_data(:,3) , 'r','LineWidth',2);
hold on
h2 = plot(compiled_data(:,1),compiled_data(:,10), 'k','LineWidth',1.5);
h3= plot(compiled_data(:,1),compiled_data(:,26) , '--b','LineWidth',1.0);
h4= plot(compiled_data(:,1),compiled_data(:,35) , '--b','LineWidth',1.0);
hold off
title('Y component (m)')
legend([h1, h2, h3], {'Estimate','Truth','3\sigma'})

subplot(3,1,3)
h1 = plot(compiled_data(:,1),compiled_data(:,4) , 'r','LineWidth',2);
hold on
h2 = plot(compiled_data(:,1),compiled_data(:,11), 'k','LineWidth',1.5);
h3= plot(compiled_data(:,1),compiled_data(:,27) , '--b','LineWidth',1.0);
h4= plot(compiled_data(:,1),compiled_data(:,36) , '--b','LineWidth',1.0);
hold off
title('Z component (m)')
legend([h1, h2, h3], {'Estimate','Truth','3\sigma'})
xlabel('Time in (s)')

if(save_plots)    
    fig_name = [path, 'world_to_imu_position.png'];    
    export_fig(fig_name, '-m2.5')    
end

%%

figure(2)
sgtitle('World to IMU Velocity (U,V,W)')
subplot(3,1,1)
h1 = plot(compiled_data(:,1),compiled_data(:,16) , 'r','LineWidth',2);
hold on
h2 = plot(compiled_data(:,1),compiled_data(:,19), 'k','LineWidth',1.5);
h3= plot(compiled_data(:,1),compiled_data(:,28) , '--b','LineWidth',1.0);
h4= plot(compiled_data(:,1),compiled_data(:,37) , '--b','LineWidth',1.0);
hold off
title('X component (m/s)')
legend([h1, h2, h3], {'Estimate','Truth','3\sigma'})

subplot(3,1,2)
h1 = plot(compiled_data(:,1),compiled_data(:,17) , 'r','LineWidth',2);
hold on
h2 = plot(compiled_data(:,1),compiled_data(:,20), 'k','LineWidth',1.5);
h3= plot(compiled_data(:,1),compiled_data(:,29) , '--b','LineWidth',1.0);
h4= plot(compiled_data(:,1),compiled_data(:,38) , '--b','LineWidth',1.0);
hold off
title('Y component (m/s)')
legend([h1, h2, h3], {'Estimate','Truth','3\sigma'})

subplot(3,1,3)
h1 = plot(compiled_data(:,1),compiled_data(:,18) , 'r','LineWidth',2);
hold on
h2 = plot(compiled_data(:,1),compiled_data(:,21), 'k','LineWidth',1.5);
h3= plot(compiled_data(:,1),compiled_data(:,30) , '--b','LineWidth',1.0);
h4= plot(compiled_data(:,1),compiled_data(:,39) , '--b','LineWidth',1.0);
hold off
title('Z component (m/s)')
legend([h1, h2, h3], {'Estimate','Truth','3\sigma'})
xlabel('Time in (s)')

if(save_plots)    
    fig_name = [path, 'world_to_imu_velocity.png'];        
    export_fig(fig_name, '-m2.5')    
end
%%
estimated_time = (estimated_time - estimated_time(1)) * 1e-9;
figure(3)
sgtitle('IMU to Camera Translation Component')
subplot(3,1,1)
h1 = plot(estimated_time, table2array(estimated(:,estimated_imu_to_camera_pose_idx(1))) , 'r','LineWidth',2);
hold on
h2 = plot(estimated_time, estimated_sigma_minus(:,16), '--b','LineWidth',1.0);
h2 = plot(estimated_time, estimated_sigma_plus(:,16), '--b','LineWidth',1.0);
hold off
title('Camera to IMU X component')
legend([h1, h2], {'Estimate','3\sigma'})

subplot(3,1,2)
h1 = plot(estimated_time, table2array(estimated(:,estimated_imu_to_camera_pose_idx(2))) , 'r','LineWidth',2);
hold on
h2 = plot(estimated_time, estimated_sigma_minus(:,17), '--b','LineWidth',1.0);
h2 = plot(estimated_time, estimated_sigma_plus(:,17), '--b','LineWidth',1.0);
hold off
title('Camera to IMU Y component')
legend([h1, h2], {'Estimate','3\sigma'})

subplot(3,1,3)
h1 = plot(estimated_time, table2array(estimated(:,estimated_imu_to_camera_pose_idx(3))) , 'r','LineWidth',2);
hold on
h2 = plot(estimated_time, estimated_sigma_minus(:,18), '--b','LineWidth',1.0);
h2 = plot(estimated_time, estimated_sigma_plus(:,18), '--b','LineWidth',1.0);
hold off
title('Camera to IMU Z component')
legend([h1, h2], {'Estimate','3\sigma'})

if(save_plots)    
    fig_name = [path, 'imu_to_camera_position.png'];      
    export_fig(fig_name, '-m2.5')    
end
%%

figure(4)
sgtitle('Gyro Bias Estimates')
subplot(3,1,1)
h1 = plot(estimated_time, table2array(estimated(:,estimated_gyro_bias_idx(1))) , 'r','LineWidth',2);
hold on
h2 = plot(estimated_time, estimated_sigma_minus(:,10), '--b','LineWidth',1.0);
h2 = plot(estimated_time, estimated_sigma_plus(:,10), '--b','LineWidth',1.0);
hold off
title('Gyro Bias X component (rad/s)')
legend([h1, h2], {'Estimate','3\sigma'})

subplot(3,1,2)
h1 = plot(estimated_time, table2array(estimated(:,estimated_gyro_bias_idx(2))) , 'r','LineWidth',2);
hold on
h2 = plot(estimated_time, estimated_sigma_minus(:,11), '--b','LineWidth',1.0);
h2 = plot(estimated_time, estimated_sigma_plus(:,11), '--b','LineWidth',1.0);
hold off
title('Gyro Bias Y component (rad/s)')
legend([h1, h2], {'Estimate','3\sigma'})

subplot(3,1,3)
h1 = plot(estimated_time, table2array(estimated(:,estimated_gyro_bias_idx(3))) , 'r','LineWidth',2);
hold on
h2 = plot(estimated_time, estimated_sigma_minus(:,12), '--b','LineWidth',1.0);
h2 = plot(estimated_time, estimated_sigma_plus(:,12), '--b','LineWidth',1.0);
hold off
title('Gyro Bias Z component (rad/s)')
legend([h1, h2], {'Estimate','3\sigma'})

if(save_plots)    
    fig_name = [path, 'gyro_bias.png'];    
    export_fig(fig_name, '-m2.5')    
end
%%
figure(5)
sgtitle('Accel Bias Estimates')
subplot(3,1,1)
h1 = plot(estimated_time, table2array(estimated(:,estimated_accel_bias_idx(1))) , 'r','LineWidth',2);
hold on
h2 = plot(estimated_time, estimated_sigma_minus(:,13), '--b','LineWidth',1.0);
h2 = plot(estimated_time, estimated_sigma_plus(:,13), '--b','LineWidth',1.0);
hold off
title('Accel Bias X component (m/s^2)')
legend([h1, h2], {'Estimate','3\sigma'})

subplot(3,1,2)
h1 = plot(estimated_time, table2array(estimated(:,estimated_accel_bias_idx(2))) , 'r','LineWidth',2);
hold on
h2 = plot(estimated_time, estimated_sigma_minus(:,14), '--b','LineWidth',1.0);
h2 = plot(estimated_time, estimated_sigma_plus(:,14), '--b','LineWidth',1.0);
hold off
title('Accel Bias Y component (m/s^2)')
legend([h1, h2], {'Estimate','3\sigma'})

subplot(3,1,3)
h1 = plot(estimated_time, table2array(estimated(:,estimated_accel_bias_idx(3))) , 'r','LineWidth',2);
hold on
h2 = plot(estimated_time, estimated_sigma_minus(:,15), '--b','LineWidth',1.0);
h2 = plot(estimated_time, estimated_sigma_plus(:,15), '--b','LineWidth',1.0);
hold off
title('Accel Bias Z component (m/s^2)')
legend([h1, h2], {'Estimate','3\sigma'})

if(save_plots)    
    fig_name = [path, 'accel_bias.png'];      
    export_fig(fig_name, '-m2.5')    
end

%%
% figure(6)
% sgtitle('Measurement_Uncertanity')
% for i = 1:16
%     subplot(4,4,i)
%     plot(measure_time, expected_measurement(:,i) - actual_measurement(:,i), 'r', 'LineWidth',1)
%     hold on
%     plot(measure_time, 3 * measurement_std(:,i), 'm', 'LineWidth',1)
%     plot(measure_time,  -3 * measurement_std(:,i) , 'm', 'LineWidth',1)
%     hold off
% end
% if(save_plots)    
%     fig_name = [path, 'residuals.png'];      
%     export_fig(fig_name, '-m2.5')    
% end


