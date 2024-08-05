close all;
try
    iteratzia_NO = iteratzia_NO + 1;
    clearvars -except iteratzia_NO
catch
    iteratzia_NO = 1;
end
%% Errors
mu_err = 0;


% % % 1
% % section_NO = 1;
% % position_sigma = 0;
% % angle_sigma = 0;
% % pixel_sigma = 0; % trust me
% % 
% 2
section_NO = 2;
position_sigma = 0.5;
angle_sigma = 0;
pixel_sigma = 0; % trust me

% 3
% section_NO = 3;
% position_sigma = 0;
% angle_sigma = 0.2;
% pixel_sigma = 0; % trust me

% % % 4 
% % section_NO = 4;
% % position_sigma = 0;
% % angle_sigma = 0;
% % pixel_sigma = 7; % trust me
% % 
% % % 5 
% % section_NO = 5;
% % position_sigma = 0.5;
% % angle_sigma = 0.2;
% % pixel_sigma = 7; % trust me


%%

disp("Creating File...");

dt = 1/30;

% trajectory 1
trajectory_NO = 1;
t = 0:dt:30;
x = -15.*sin(0.23.*t) + 34;
y = 0.12.*t.*(t-30) + 50;
z = 15*atan(0.5.*t - 5) - 14.*atan(0.5.*t-7)+ 5.*atan(0.5.*(t-25)) +12 ;

% trajectory 2
% % trajectory_NO = 2;
% % t = 0:dt:50;
% % x = 35+10*sin(0.13*t);
% % y = 36+26*sin(0.13*t).*cos(0.13*t);
% % z = 18+10*sin(0.08*t);


% % velocities
% dx = diff(x);
% dy = diff(y);
% dz = diff(z);
% dt = 1/30;
% v = sqrt((dx/dt).^2 + (dy/dt).^2 + (dz/dt).^2);
% figure(); plot(t(1:end-1),v);

% comet3(x(1:2:end),y(1:2:end),z(1:2:end));

figure(); hold on;
plot3(x,y,z,".-");

%%%
% ORIGIN pos   = 32      , 35     , 240   ,   0,  0,0
% station1 pos = 32.00008, 35     , 240   ,   70, 25,0 
% station2 pos = 32.00022, 35     , 240   ,   90, 25,0
% station3 pos = 32.00040 ,35     , 240   ,   110,25,0
% station4 pos = 32      , 34.9999, 240   ,   30,25,0
% station4 pos = 32      , 34.99975, 240  ,   0,25,0
% station6 pos = 32      , 34.9995, 240   ,   -30,25,0
%%%

cams_pos = [8.8712799126e+00  3.7249758834e-11 -6.1934059024e-06;
            2.4396020029e+01 -4.6201353853e-10 -4.6835781419e-05;
            4.4356400681e+01 -1.0593836917e-10 -1.5483184173e-04;
            4.3700543668e-06  9.4496694118e+00 -6.9935488966e-06;
            2.7311974954e-05  2.3624173530e+01 -4.3708296574e-05;
            1.0924804709e-04  4.7248347060e+01 -1.7483342198e-04];

% Add errors
for i = 1:6
    for j = 1:3
        cams_pos(i,j) = cams_pos(i,j) + normrnd(mu_err, position_sigma);
    end
end

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


camera_angles = [70, -25, 0.0000;
                90, -25, 0;
                110, -25, 0;
                30, -25, 0;
                0, -25, 0;
                -30,-25,0];

% Add errors
for i = 1:6
    for j = 1:3
        camera_angles(i,j) = camera_angles(i,j) + normrnd(mu_err, angle_sigma);
    end
end


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

% cam2drone = Ry(theta)*Rz(psi);
% disp(transpose(cam2drone)*[1;0;0]);

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

if ~exist("InjectedData", 'dir')
   mkdir("InjectedData");
   disp("Created Directory 'InjectedData\'");
end

FILE = fopen("InjectedData\injection_data.csv", "w");

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
        
        pix_1 = pix_1 + round(normrnd(mu_err, pixel_sigma));
        pix_2 = pix_2 + round(normrnd(mu_err, pixel_sigma));

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
disp("DONE WITH CREATING INJECTION FILE injection_data.csv");

save(sprintf("InjectedData\\injected_dat_section%.0f_iter%.0f.mat",section_NO,iteratzia_NO), ...
    'x', ...
    'y', ...
    'z', ...
    't', ...
    'dt', ...
    'cams_pos', ...
    'camera_angles', ...
    'Vertical_halfAngle_deg', ...
    'Horizontal_halfAngle_deg', ...
    'depth_m');

disp("Saved injection parameters to 'injected_data.mat'");