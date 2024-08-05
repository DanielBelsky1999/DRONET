%% Analysis

clear variables;
load("InjectedData.mat");

run("..//..//Logs//SystemLog.m");
run("..//..//Logs//DataLog.m");

time_epsilon = 10^-9;

% Calculate solution distances from origin:
Solution_times = Solution(:,1)-Solution(1,1);


f1 = figure(); ax1 = subplot(1,2,1, "Parent", f1); hold on;
f2 = figure(); ax2 = subplot(1,1,1, "Parent", f2); hold on;

time_sub_array = [];
Distance_error_sub_array = [];
Calculated_Position_sub_array = [];

for time_indx_solution = 1:length(Solution_times)    
    time_indx_generated = fix((Solution_times(time_indx_solution)+time_epsilon)/dt)+1;
    
    %%%%%%%%%%%%%%%% Actual calculation:
    
    x2 = (x(time_indx_generated) - Solution(time_indx_solution,2))^2;
    y2 = (y(time_indx_generated) - Solution(time_indx_solution,3))^2;
    z2 = (z(time_indx_generated) - Solution(time_indx_solution,4))^2;
    R = sqrt(x2 + y2 + z2);

    distance_error_value = R;

    %%%%
    position_vec = [Solution(time_indx_solution,2), Solution(time_indx_solution,3), Solution(time_indx_solution,4)];
    %%%%%%%%%%%%%%%%%

    time_sub_array = [time_sub_array, t(time_indx_generated)];
    Distance_error_sub_array = [Distance_error_sub_array, distance_error_value];
    Calculated_Position_sub_array = [Calculated_Position_sub_array; position_vec];

    if (time_indx_solution < length(Solution_times)) 
        if (abs((Solution_times(time_indx_solution+1) - Solution_times(time_indx_solution))-dt) < time_epsilon)
            continue;
        end
    end
    % Plot:
    plot(ax1, time_sub_array, Distance_error_sub_array,".-");
    p_calc = plot3(ax2, Calculated_Position_sub_array(:,1), Calculated_Position_sub_array(:,2), Calculated_Position_sub_array(:,3),"r.--", "LineWidth",2);
    % Reset the Arrays:
    time_sub_array = [];
    Distance_error_sub_array = [];
    Calculated_Position_sub_array = [];
end


xlabel(ax1, "Time [sec]");
ylabel(ax1, "Distance Error [m]");
grid(ax1, "on");
title(ax1,"Error between injected and calculated (LS) paths");

% Estimation errors:
for t_i = 1:length(t)-1
    est_pos = SolutionEstimated(t_i,2:4);
    pos = [x(t_i+1),y(t_i+1),z(t_i+1)];
    e = est_pos - pos;
    error_estimation(t_i) = norm(e);
end

ax3 = subplot(1,2,2, "Parent", f1);
plot(ax3, t(2:end),error_estimation);
xlabel(ax3, "Time [sec]");
ylabel(ax3, "Distance Error [m]");
grid(ax3, "on");
title(ax3,"Error between injected and estimated paths");
%%%%%%%%%%%%%%% 3D

p_real = plot3(ax2,x,y,z,"Color",[0 0.4470 0.7410], "LineWidth",1);
p_est = plot3(SolutionEstimated(:,2), SolutionEstimated(:,3), SolutionEstimated(:,4), "g","LineWidth",1);
axis(ax2,"equal");
legend([p_calc,p_real,p_est], "Calculated path (MLS only, no Estimator)","Injected path","Estimated path");
xlabel(ax2,"X");
ylabel(ax2,"Y");
zlabel(ax2,"Z");
view(ax2,3); grid on;


%%%%%%%%%%%%%%%%%%%%%% Draw stations to compare

figure(); hold on;
plot3(x,y,z,".-");


% Draw Stations
for i = 1:length(cams_pos)
    pos = cams_pos(i,:);
    scatter3(pos(1),pos(2),pos(3), 40,"MarkerFaceColor",'r', "Marker",'diamond');
end
DrawCamsFOV(gca(), cams_pos, camera_angles, Vertical_halfAngle_deg, Horizontal_halfAngle_deg, depth_m);

% Draw Stations from simulation:
VEC_LEN = 10;
for station_i = 1:length(Stations.position)
    pos = Stations.position(station_i,:);
    mat = reshape(Stations.Origin2StationDCM(station_i,:,:), 3,3)' .* VEC_LEN;

    quiver3(pos(1),pos(2),pos(3),mat(1,1),mat(2,1),mat(3,1),"off", "Color","r","LineWidth",1);
    quiver3(pos(1),pos(2),pos(3),mat(1,2),mat(2,2),mat(3,2),"off", "Color","g","LineWidth",1);
    quiver3(pos(1),pos(2),pos(3),mat(1,3),mat(2,3),mat(3,3),"off", "Color","b","LineWidth",1);
end

axis equal;
xlim([-5, 55]);
ylim([-5, 55]);
zlim([-10, 55]);
xlabel("X");
ylabel("Y");
zlabel("Z");
view(3); grid on;
