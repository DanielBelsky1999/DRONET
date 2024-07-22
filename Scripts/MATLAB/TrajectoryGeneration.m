% close all;
clear variables;

t = 0:1/30:30;

% x = 35+5*sin(t);
% y = 35+5*cos(t);
% z = 7+0.5*t;
% 
x = -15.*sin(0.23.*t) + 34;
y = 0.12.*t.*(t-30) + 50;
z = 15*atan(0.5.*t - 5) - 14.*atan(0.5.*t-7)+ 5.*atan(0.5.*(t-25)) +12 ;
% 
% dx = diff(x);
% dy = diff(y);
% dz = diff(z);
% dt = 1/30;
% v = sqrt((dx/dt).^2 + (dy/dt).^2 + (dz/dt).^2);
% figure(); plot(t(1:end-1),v);

% comet3(x(1:2:end),y(1:2:end),z(1:2:end));

figure(); hold on;
% subplot(1,2,1);
plot3(x,y,z,".-");

% comet3(x,y,z);

%%%
% origin pos   = 32     , 35     , 240
% station1 pos = 32.0001, 35     , 240
% station2 pos = 32.0002, 35     , 240
% station3 pos = 32.0003, 35     , 240
% station4 pos = 32     , 34.9997, 240
%%%

cams_pos = [1.1089099908e+01 -3.3595082272e-10 -9.6773529616e-06;
            2.2178199991e+01 -7.1217698400e-11 -3.8707022709e-05;
            3.3267300250e+01 -1.9819523799e-10 -8.7093016935e-05;
            3.9329437112e-05  2.8349008236e+01 -6.2940256218e-05;];

Rz = @(psi) [cos(psi), sin(psi), 0;
            -sin(psi), cos(psi), 0;
            0        , 0        ,1];

Ry = @(theta) [cos(theta), 0    , -sin(theta);
               0         , 1    , 0          ;
               sin(theta), 0    , cos(theta)];

Rx = @(phi) [1, 0       , 0       ;
             0, cos(phi), sin(phi);
             0, -sin(phi), cos(phi)];

DCM_origin2camera = @(psi, theta, phi) Rx(deg2rad(phi))*Ry(deg2rad(theta))*Rz(deg2rad(psi));


camera_angles = [70, -15, 0.0000;
                90, -15, 0;
                80, -15, 0;
                0, -15, 0.0000;];

% camera_DCMs 

for i = 1:length(camera_angles)
    camera_DCMs(i,:,:) = DCM_origin2camera(camera_angles(i,1),camera_angles(i,2),camera_angles(i,3));
end 



% Draw Origin System
quiver3(0,0,0,50,0,0,"off", "Color","k","LineWidth",1);
quiver3(0,0,0,0,50,0,"off", "Color","k","LineWidth",1);
quiver3(0,0,0,0,0,50,"off", "Color","k","LineWidth",1);

% Draw Stations
for i = 1:length(cams_pos)
    pos = cams_pos(i,:);
    scatter3(pos(1),pos(2),pos(3), 40,"MarkerFaceColor",'r', "Marker",'diamond');
end

Vertical_halfAngle_deg = 26.1;
Horizontal_halfAngle_deg = 47;
depth_m = 10;
DrawCamsFOV(gca(), cams_pos, camera_angles, Vertical_halfAngle_deg, Horizontal_halfAngle_deg, depth_m);

axis equal;
xlim([-5, 55]);
ylim([-5, 55]);
zlim([-10, 55]);
xlabel("X");
ylabel("Y");
zlabel("Z");
view(3); grid on;


%% Azimuth elevation calc

syms theta psi or2cam

cam2drone = Ry(theta)*Rz(psi);
disp(transpose(cam2drone)*[1;0;0]);

for cam_i = 1:length(camera_DCMs)
    for point_i = 1:length(x)
        Vec = [x(point_i);y(point_i);z(point_i)] - [cams_pos(cam_i,1);cams_pos(cam_i,2);cams_pos(cam_i,3)];
        Test_Length(cam_i, point_i) = norm(Vec);
        Vec = Vec./norm(Vec);

        DCM_or2cam = DCM_origin2camera(camera_angles(cam_i,1),camera_angles(cam_i,2),camera_angles(cam_i,3));
        Vec_CAM = DCM_or2cam*Vec;

        theta = -asin(Vec_CAM(3));
        azimuth = atan(Vec_CAM(2)/Vec_CAM(1));
        
        Trajectory_elev_azim(cam_i, point_i, :) = [theta, azimuth];

       
    end
end


%% Test

% for cam_i = 1:length(camera_DCMs)
%     for point_i = 1:length(x)
%         tehta_azimuth_vec = reshape(Trajectory_elev_azim(cam_i, point_i,:),1,2);
%         pos = cams_pos(cam_i,:);
%         vector_cam2drone_CAM = (Ry(Trajectory_elev_azim(cam_i,point_i,1))*Rz(Trajectory_elev_azim(cam_i, point_i,2)))'*[1;0;0]*Test_Length(cam_i, point_i);
%         DCM_or2cam = DCM_origin2camera(camera_angles(cam_i,1),camera_angles(cam_i,2),camera_angles(cam_i,3));
%         vector_cam2drone_ORIGIN = DCM_or2cam'*vector_cam2drone_CAM;
% 
%         drone_position(cam_i, point_i,:) = pos' + vector_cam2drone_ORIGIN;
%     end
% end
% 
% plot3(drone_position(1,:,1),drone_position(1,:,2),drone_position(1,:,3), "r--");
% plot3(drone_position(2,:,1),drone_position(2,:,2),drone_position(2,:,3), "g--");
% plot3(drone_position(3,:,1),drone_position(3,:,2),drone_position(3,:,3), "k--");
% plot3(drone_position(4,:,1),drone_position(4,:,2),drone_position(4,:,3), "y--");
% 
% disp("DONE WITH TESTING");

%% DROP TO FILE

A = (1920/2)/tand(47);
B = (1080/2)/tand(26.1);


FILE = fopen("injection_data.csv", "w");

for point_i = 1:length(Trajectory_elev_azim(1,:,1))
    for cams_i = 1:length(camera_angles)
        
        az = Trajectory_elev_azim(cams_i, point_i, 2);
        el = Trajectory_elev_azim(cams_i, point_i, 1);

%            THIS IS THE GEOMETRIC WAY TO SOLVE IT:
%            pix_1 = 1920/2 - A*tan(az);
%            pix_2 = 1080/2 + B/cos(az)*tan(el);
       
        
        camera_num = 1;
        option_num = 1;
        [pix_1, pix_2] = camera_calibration_final_inverse(camera_num, option_num, -rad2deg(az), -rad2deg(el));

        if (pix_1 > 1920 || pix_1 < 0)
            continue;
        end
        if (pix_2 > 1080 || pix_2 < 0)
            continue;
        end
        str_write = sprintf("%.0f,%.10f,%.10f,%.10f\n", cams_i,round(pix_1), round(pix_2), t(point_i));
%         str_write = [cams_i, pix_1, pix_2];
        fprintf(FILE,str_write);
    end
end
fclose(FILE);
disp("DONE WITH CREATING INJECTION FILE");

return;
%% Analysis

clear Solution Stations;
run("..//..//Logs//SystemLog.m");
run("..//..//Logs//DataLog.m");

time_epsilon = 10^-9;
dt = 1/30;

% Calculate original distances from origin:
Original_times = t;
r_squared= x.*x + y.*y + z.*z;
Original_r = sqrt(r_squared);

% Calculate solution distances from origin:
Solution_times = Solution(:,1)-Solution(1,1);


f1 = figure(); ax1 = subplot(1,1,1, "Parent", f1); hold on;
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
    plot3(ax2, Calculated_Position_sub_array(:,1), Calculated_Position_sub_array(:,2), Calculated_Position_sub_array(:,3),"r.--", "LineWidth",2);
    % Reset the Arrays:
    time_sub_array = [];
    Distance_error_sub_array = [];
    Calculated_Position_sub_array = [];
end


xlabel(ax1, "Time [sec]");
ylabel(ax1, "Distance Error [m]");
grid(ax1, "on");
title(ax1,"Error between injected and calculated paths");

plot3(ax2,x,y,z,"Color",[0 0.4470 0.7410], "LineWidth",1);
axis(ax2,"equal");
legend("Calculated path (MLS only, no Estimator)","Injected Path");
xlabel(ax2,"X");
ylabel(ax2,"Y");
zlabel(ax2,"Z");
view(ax2,3); grid on;


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

