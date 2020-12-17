function plot_insets(initial_time,final_time,t,x,x_true,sigma,prop)
%Clearance for insets
clearance = 1.5;

%number of filters to be plot
states_to_plot = 3;
indexOfInterest = (t < final_time) & (t > initial_time); % range of t near perturbation
hold on;
for i=1:states_to_plot
number_of_figure = 1;

figure(number_of_figure)
switch (i)
    case 1
        %create a new pair of axes inside current figure
        axes('position',[.79 .77 .17 .17]);
        %Here we plot the inset
        hold on;
        box on
    case 2    
        axes('position',[.79 .47 .17 .17]);
        hold on;
        box on

    case 3
        axes('position',[.79 .17 .17 .17]);
        hold on;
        box on


    otherwise
end
%Plot estimates
plot(t(indexOfInterest),x(indexOfInterest,i),'b','LineWidth',2)
plot(t(indexOfInterest)+0.1,x_true(indexOfInterest,i), 'k','LineWidth',1.5)
plot(t(indexOfInterest),(sigma(indexOfInterest,2*i-1)),'--r','LineWidth',1.0);
plot(t(indexOfInterest),(sigma(indexOfInterest,2*i)),'--r','LineWidth',1.0);
hold on;

switch (i)
    case 1
        lim = axis;
%         ylim(lim(3:4)*clearance )
        xlim([initial_time final_time]);
        ax = gca;
        ax.LineWidth = 2;
    case 2 
       lim = axis;
       ylim([0.9 1.25])
        xlim([initial_time final_time]);
       ax = gca;
       ax.LineWidth = 2;

    case 3
        lim = axis;
        ylim([1.35 1.95])
          xlim([initial_time final_time]);
        ax = gca;
        ax.LineWidth = 2;
        
    otherwise
        
end


end