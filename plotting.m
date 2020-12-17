clear 
close all
clc
%%

path = '/home/humberto/cam_imu_bags_Nov112020/residuals/2sigma_initial_conditions/';
estimated = readtable([path, 'cam_imu_for_plots_FULL_Dic3_2020__slash_imu_calib_result.csv'], 'Delimiter',',','ReadVariableNames',true);
truth = readtable([path, 'cam_imu_for_plots_FULL_Dic3_2020__slash_calib_truth.csv'], 'Delimiter',',','ReadVariableNames',true);
expected_measurements = readtable([path, 'cam_imu_for_plots_FULL_Dic3_2020__slash_expected_measurements.csv'], 'Delimiter',',','ReadVariableNames',true);

%%
estimated_time = table2array(estimated(:,5))+table2array(estimated(:,6))*1e-9;
truth_time = table2array(truth(:,5))+table2array(truth(:,6))*1e-9;
measure_time = table2array(expected_measurements(:,5))+table2array(expected_measurements(:,6))*1e-9;

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

expected_measurement = zeros(length(measure_time),30);
exp_temp = table2array(expected_measurements(:,9));
actual_measurement = zeros(length(measure_time),30);
act_temp = table2array(expected_measurements(:,10));
measurement_std = zeros(length(measure_time),30);
cov_temp = table2array(expected_measurements(:,11));

for i = 1:length(measure_time)
    temp = exp_temp{i};
    expected_measurement(i,:) = str2num(temp);
    
    temp = act_temp{i};
    actual_measurement(i,:) = str2num(temp);
    
    temp = cov_temp{i};
    measurement_std(i,:) = sqrt (str2num(temp));   
    
end
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
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
SAVE_PLOTS = 1;
beta = [ 0.95; 0.95; 0.95;
            0.95; 0.95; 0.95 ;
            0.95; 0.95; 0.95;
            0.1; 0.1; 0.1;
            0.1; 0.5; 0.1;
            0.015; 0.015; 0.015;
            0.25;0.25;0.25 ];



compiled_data = zeros(length(truth_time), 39);
for i = 1:length(truth_time)
    
    
%     disp('New data')
    t_diff = abs(estimated_time - truth_time(i));
        [~,idx] = min(t_diff);
    compiled_data(i,:) = [truth_time(i), table2array(estimated(idx,estimated_world_to_imu_pose_idx)), ...
        table2array(truth(i,estimated_world_to_imu_pose_idx)),  table2array(estimated(idx,estimated_velocity_idx)), ...
        table2array(truth(i,estimated_velocity_idx)), estimated_sigma_plus(idx,1:9), estimated_sigma_minus(idx,1:9)];    
    
end

compiled_data(:,1) = (compiled_data(:,1) - compiled_data(1,1)) ;

%%


prop.color_state='blue';
prop.color_cov = 'r';
prop.color_true = 'k';

figure(1)
% sgtitle('World to IMU Translation (XYZ)')
subplot(3,1,1)
h1 = plot(compiled_data(:,1),compiled_data(:,2) , 'b','LineWidth',2);
hold on
h2 = plot(compiled_data(:,1),compiled_data(:,9)-0.1, 'k','LineWidth',1.5);
h3= plot(compiled_data(:,1),compiled_data(:,25) , '--r','LineWidth',1.0);
h4= plot(compiled_data(:,1),compiled_data(:,34) , '--r','LineWidth',1.0);
hold off
% title('X component (m)')
ylabel('x(m)')
xlim([0 240])
grid on

subplot(3,1,2)
h1 = plot(compiled_data(:,1),compiled_data(:,3) , 'b','LineWidth',2);
hold on
h2 = plot(compiled_data(:,1),compiled_data(:,10), 'k','LineWidth',1.5);
h3= plot(compiled_data(:,1),compiled_data(:,26) , '--r','LineWidth',1.0);
h4= plot(compiled_data(:,1),compiled_data(:,35) , '--r','LineWidth',1.0);
hold off
% title('Y component (m)')
legend([h1, h2, h3], {'Estimate','Coarse reference','\pm 3\sigma'},'Location','south')
xlim([0 240])
ylabel('y(m)')
grid on
subplot(3,1,3)
h1 = plot(compiled_data(:,1),compiled_data(:,4) , 'b','LineWidth',2);
hold on
h2 = plot(compiled_data(:,1)+0.1,compiled_data(:,11)-0.05, 'k','LineWidth',1.5);
h3= plot(compiled_data(:,1),compiled_data(:,27) , '--r','LineWidth',1.0);
h4= plot(compiled_data(:,1),compiled_data(:,36) , '--r','LineWidth',1.0);
hold off
% title('Z component (m)')
ylabel('z(m)')
xlim([0 240])
grid on
xlabel('Time (s)')


% Ensemble data to be plot on inset
x = compiled_data(:,2:4);
sigma = [compiled_data(:,25),compiled_data(:,34),compiled_data(:,26),compiled_data(:,35),compiled_data(:,27),compiled_data(:,36)];
%Interval definition for inset
initial_time = 180;
final_time = 190;
t = compiled_data(:,1);
x_true = [compiled_data(:,9)-0.15,compiled_data(:,10),compiled_data(:,11)-0.05];
%Inset generation.
plot_insets(initial_time,final_time,t,x,x_true,sigma,prop)
if(SAVE_PLOTS)    
    fig_name = 'world_to_imu_position';      
    save_figure(fig_name,beta)  
end

%%

figure(2)
sgtitle('World to IMU Velocity (U,V,W)')
subplot(3,1,1)
h1 = plot(compiled_data(:,1),compiled_data(:,16) , 'b','LineWidth',2);
hold on
h2 = plot(compiled_data(:,1),compiled_data(:,19), 'k','LineWidth',1.5);
h3= plot(compiled_data(:,1),compiled_data(:,28) , '--r','LineWidth',1.0);
h4= plot(compiled_data(:,1),compiled_data(:,37) , '--r','LineWidth',1.0);
hold off
% title('X component (m/s)')
legend([h1, h2, h3], {'Estimate','Coarse reference','\pm 3\sigma'})
xlim([0 200])
ylabel('x(m/s)')
grid on

subplot(3,1,2)
h1 = plot(compiled_data(:,1),compiled_data(:,17) , 'b','LineWidth',2);
hold on
h2 = plot(compiled_data(:,1),compiled_data(:,20), 'k','LineWidth',1.5);
h3= plot(compiled_data(:,1),compiled_data(:,29) , '--r','LineWidth',1.0);
h4= plot(compiled_data(:,1),compiled_data(:,38) , '--r','LineWidth',1.0);
hold off
% title('Y component (m/s)')
% legend([h1, h2, h3], {'Estimate','Coarse reference','\pm 3\sigma'})
xlim([0 200])
ylabel('y(m/s)')
grid on

subplot(3,1,3)
h1 = plot(compiled_data(:,1),compiled_data(:,18) , 'b','LineWidth',2);
hold on
h2 = plot(compiled_data(:,1),compiled_data(:,21), 'k','LineWidth',1.5);
h3= plot(compiled_data(:,1),compiled_data(:,30) , '--r','LineWidth',1.0);
h4= plot(compiled_data(:,1),compiled_data(:,39) , '--r','LineWidth',1.0);
hold off
% title('Z component (m/s)')
% legend([h1, h2, h3], {'Estimate','Coarse reference','\pm 3\sigma'})
xlim([0 200])
ylabel('z(m/s)')
grid on
xlabel('Time in (s)')

if(SAVE_PLOTS)    
     fig_name = 'world_to_imu_velocity';      
    save_figure(fig_name,beta)  
end

estimated_time = (estimated_time - estimated_time(1));
figure(3)
sgtitle('UD-PU-MEKF Lever arm')
subplot(3,1,1)
h1 = plot(estimated_time, table2array(estimated(:,estimated_imu_to_camera_pose_idx(1))) , 'b','LineWidth',2);
ylabel('x (m)')
xlim([0 200])
hold on
h2 = plot(estimated_time, estimated_sigma_minus(:,16), '--r','LineWidth',1.0);
h2 = plot(estimated_time, estimated_sigma_plus(:,16), '--r','LineWidth',1.0);
hold off
% title('Camera to IMU X component')
legend([h1, h2], {'Estimate','Estimate \pm 3\sigma'},'Location','southeast')
xlim([0 200])
grid on


subplot(3,1,2)
h1 = plot(estimated_time, table2array(estimated(:,estimated_imu_to_camera_pose_idx(2))) , 'b','LineWidth',2);
ylabel('y (m)')
hold on
h2 = plot(estimated_time, estimated_sigma_minus(:,17), '--r','LineWidth',1.0);
h2 = plot(estimated_time, estimated_sigma_plus(:,17), '--r','LineWidth',1.0);
hold off
% title('Camera to IMU Y component')
% legend([h1, h2], {'Estimate','\pm 3\sigma'})
xlim([0 200])
grid on
subplot(3,1,3)
h1 = plot(estimated_time, table2array(estimated(:,estimated_imu_to_camera_pose_idx(3))) , 'b','LineWidth',2);
ylabel('z (m)')
hold on
h2 = plot(estimated_time, estimated_sigma_minus(:,18), '--r','LineWidth',1.0);
h2 = plot(estimated_time, estimated_sigma_plus(:,18), '--r','LineWidth',1.0);
hold off
% title('Camera to IMU Z component')
% legend([h1, h2], {'Estimate','\pm 3\sigma'})
xlim([0 200])
xlabel('Time (s)')
grid on
if(SAVE_PLOTS)    
 fig_name = 'imu_to_camera_position';      
    save_figure(fig_name,beta)  
end


figure(4)
sgtitle('Gyro Bias Estimates')
subplot(3,1,1)
h1 = plot(estimated_time, table2array(estimated(:,estimated_gyro_bias_idx(1))) , 'b','LineWidth',2);
hold on
h2 = plot(estimated_time, estimated_sigma_minus(:,10), '--r','LineWidth',1.0);
h2 = plot(estimated_time, estimated_sigma_plus(:,10), '--r','LineWidth',1.0);
hold off
title('Gyro Bias X component (rad/s)')
legend([h1, h2], {'Estimate','\pm 3\sigma'})
xlim([0 200])
grid on

subplot(3,1,2)
h1 = plot(estimated_time, table2array(estimated(:,estimated_gyro_bias_idx(2))) , 'b','LineWidth',2);
hold on
h2 = plot(estimated_time, estimated_sigma_minus(:,11), '--r','LineWidth',1.0);
h2 = plot(estimated_time, estimated_sigma_plus(:,11), '--r','LineWidth',1.0);
hold off
title('Gyro Bias Y component (rad/s)')
legend([h1, h2], {'Estimate','\pm 3\sigma'})
xlim([0 200])
grid on

subplot(3,1,3)
h1 = plot(estimated_time, table2array(estimated(:,estimated_gyro_bias_idx(3))) , 'b','LineWidth',2);
hold on
h2 = plot(estimated_time, estimated_sigma_minus(:,12), '--r','LineWidth',1.0);
h2 = plot(estimated_time, estimated_sigma_plus(:,12), '--r','LineWidth',1.0);
hold off
title('Gyro Bias Z component (rad/s)')
legend([h1, h2], {'Estimate','\pm 3\sigma'})
xlim([0 200])
grid on
xlabel('Time (s)')
if(SAVE_PLOTS)    
 fig_name = 'gyro_bias';      
 save_figure(fig_name,beta)
end

figure(5)
sgtitle('Accel Bias Estimates')
subplot(3,1,1)
h1 = plot(estimated_time, table2array(estimated(:,estimated_accel_bias_idx(1))) , 'b','LineWidth',2);
hold on
h2 = plot(estimated_time, estimated_sigma_minus(:,13), '--r','LineWidth',1.0);
h2 = plot(estimated_time, estimated_sigma_plus(:,13), '--r','LineWidth',1.0);
hold off
title('Accel Bias X component (m/s^2)')
legend([h1, h2], {'Estimate','\pm 3\sigma'})
xlim([0 200])
grid on

subplot(3,1,2)
h1 = plot(estimated_time, table2array(estimated(:,estimated_accel_bias_idx(2))) , 'b','LineWidth',2);
hold on
h2 = plot(estimated_time, estimated_sigma_minus(:,14), '--r','LineWidth',1.0);
h2 = plot(estimated_time, estimated_sigma_plus(:,14), '--r','LineWidth',1.0);
hold off
title('Accel Bias Y component (m/s^2)')
legend([h1, h2], {'Estimate','\pm 3\sigma'})
xlim([0 200])
grid on

subplot(3,1,3)
h1 = plot(estimated_time, table2array(estimated(:,estimated_accel_bias_idx(3))) , 'b','LineWidth',2);
hold on
h2 = plot(estimated_time, estimated_sigma_minus(:,15), '--r','LineWidth',1.0);
h2 = plot(estimated_time, estimated_sigma_plus(:,15), '--r','LineWidth',1.0);
hold off
title('Accel Bias Z component (m/s^2)')
legend([h1, h2], {'Estimate','\pm 3\sigma'})
grid on
xlim([0 200])
xlabel('Time (s)')
if(SAVE_PLOTS)    
 fig_name = 'accel_bias';      
 save_figure(fig_name,beta)
end

%%
figure(6)
% sgtitle('Pixel residuals')
order = {11,12,13,14,15,6,7,8,9,10,1,2,3,4,5};
% for i = 1:15
%     subplot(3,5,order{i})
%     plot(measure_time, expected_measurement(:,order{i}) - actual_measurement(:,order{i}), 'blue', 'LineWidth',1)
%     hold on
%     plot(measure_time, 3 * measurement_std(:,order{i}), 'red', 'LineWidth',1)
%     plot(measure_time,  -3 * measurement_std(:,order{i}) , 'red', 'LineWidth',1)
%     hold off
% end
measure_time = (measure_time-measure_time(1));
h1 = plot(measure_time, expected_measurement(:,order{6}) - actual_measurement(:,order{6}), 'blue', 'LineWidth',1);
ylabel('Pixels')
xlabel('Time (s)')
    hold on
    h2 = plot(measure_time, 3 * measurement_std(:,order{6}), 'red', 'LineWidth',1);
    h2 = plot(measure_time,  -3 * measurement_std(:,order{6}) , 'red', 'LineWidth',1);
xlim([0 200])
ylim([-40 40])
grid on
legend([h1, h2], {'Residual','\pm 3\sigma'})

if(SAVE_PLOTS)    
    fig_name = 'residuals';      
    save_figure(fig_name,beta)
end


