function save_figure(name,beta)
%% Saving the figure based on value of beta.
%This block decides if the name of the file is full or partial.
path_to_save = '/home/humberto/charuco_ws/src/camera_imu_calib/scitech_figs/';
if sum(beta)<21
fig=gcf;
fig.PaperUnits = 'centimeters';
if(strcmp(name , 'residuals'))
    fig.PaperPosition = [0 0 10 5];
else
fig.PaperPosition = [0 0 22 18];
end
plot_name = sprintf('%sHARDWARE_partial_%s_beta%d%d%d%d%d%d%d.eps',path_to_save,name,beta(1)*1000,beta(4)*1000,beta(7)*1000,beta(10)*1000,beta(13)*1000,beta(16)*1000,beta(19)*1000);
print('-depsc','-r300',plot_name);      %   *// 300 dpiend
else
fig=gcf;
fig.PaperUnits = 'centimeters';
fig.PaperPosition = [0 0 22 18];
plot_name = sprintf('%sHARDWARE_full_%s.eps',path_to_save,name);
print('-depsc','-r300',plot_name);      %   *// 300 dpiend

end