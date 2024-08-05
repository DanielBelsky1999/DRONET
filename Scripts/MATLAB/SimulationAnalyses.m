dt = 1/30;
t = 0:dt:30;
x = -15.*sin(0.23.*t) + 34;
y = 0.12.*t.*(t-30) + 50;
z = 15*atan(0.5.*t - 5) - 14.*atan(0.5.*t-7)+ 5.*atan(0.5.*(t-25)) +12 ;

time_epsilon = 10^-9;

f1 = figure(); ax1 = subplot(1,1,1, "Parent", f1); hold on;
f2 = figure(); ax2 = subplot(1,1,1, "Parent", f2); hold on;

plot_colors = [0 0.4470 0.7410; 0.8500 0.3250 0.0980; 0.9290 0.6940 0.1250];

for simulation_indx = 1:1
    clearvars -except dt t x y z time_epsilon f1 f2 ax1 ax2 plot_colors simulation_indx;
%     filename = sprintf("DataLog_section2_iter%.0f.m", simulation_indx);
%     run(filename);
    run("..//..//Logs//DataLog.m");

    Solution_times = Solution(:,1);
    time_sub_array = [];
    Distance_error_sub_array = [];
    Calculated_Position_sub_array = [];

    for time_indx_solution = 1:length(Solution_times)    
        time_indx_generated = fix((Solution_times(time_indx_solution)+time_epsilon)/dt)+1;
        
        %%%%%%%%%%%%%%%% Error calculation between path and LS:
        
        x2 = (x(time_indx_generated) - Solution(time_indx_solution,2))^2;
        y2 = (y(time_indx_generated) - Solution(time_indx_solution,3))^2;
        z2 = (z(time_indx_generated) - Solution(time_indx_solution,4))^2;
        R = sqrt(x2 + y2 + z2);     
        
        position_vec = [Solution(time_indx_solution,2), Solution(time_indx_solution,3), Solution(time_indx_solution,4)];

        distance_error_value = R;
        time_sub_array = [time_sub_array, t(time_indx_generated)];
        Distance_error_sub_array = [Distance_error_sub_array, distance_error_value];
        Calculated_Position_sub_array = [Calculated_Position_sub_array; position_vec];
        
        if (time_indx_solution < length(Solution_times)) 
            if (abs((Solution_times(time_indx_solution+1) - Solution_times(time_indx_solution))-dt) < time_epsilon)
                continue;
            end
        end
        plot(ax1, time_sub_array, Distance_error_sub_array,".-", Color=plot_colors(simulation_indx,:));
        plot3(ax2, Calculated_Position_sub_array(:,1), Calculated_Position_sub_array(:,2), Calculated_Position_sub_array(:,3),"r.--", "LineWidth",2);
        % Reset the Arrays:
        time_sub_array = [];
        Distance_error_sub_array = [];
        Calculated_Position_sub_array = [];
    end

    %%%%%%%%%%%%%%%% Error calculation between path and LS:
    for t_i = 1:length(t)-1
        est_pos = SolutionEstimated(t_i,2:4);
        pos = [x(t_i+1),y(t_i+1),z(t_i+1)];
        e = est_pos - pos;
        error_estimation(t_i) = norm(e);
    end

    plot(ax1, t(2:end), error_estimation, Color='r', LineStyle="-");
    plot3(ax2,SolutionEstimated(:,2), SolutionEstimated(:,3), SolutionEstimated(:,4), "g","LineWidth",1);
end
plot3(ax2, x,y,z,"b");
view(ax2,3);axis equal; grid on;