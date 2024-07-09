close all;
clear variables;

t = 0:1/30:15;

x = 25+5*sin(5.*t);
y = 25+5*cos(3.*t +10);
z = 20+10*sin(t);

% figure(Units = "normalized", Position=[0.0563 0.1991 0.8339 0.6157]);
figure(); hold on;
% subplot(1,2,1);
plot3(x,y,z);


% 
% figure();
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
            3.9329437112e-05  2.8349008236e+01 -6.2940256218e-05];

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


camera_angles = [70, -20, 0.0000;
                90, -20, 0;
                110, -20, 0;
                0, -20, 0.0000];

% camera_DCMs 

for i = 1:length(camera_angles)
    camera_DCMs(i,:,:) = DCM_origin2camera(camera_angles(i,1),camera_angles(i,2),camera_angles(i,3));
end 



VEC_LEN = 5;

% Draw Origin System
quiver3(0,0,0,50,0,0,"off", "Color","k","LineWidth",1);
quiver3(0,0,0,0,50,0,"off", "Color","k","LineWidth",1);
quiver3(0,0,0,0,0,50,"off", "Color","k","LineWidth",1);

% Draw Stations
for i = 1:length(cams_pos)
    pos = cams_pos(i,:);
    mat = reshape(camera_DCMs(i,:,:), 3,3)' .* VEC_LEN;
% 
%     quiver3(pos(1),pos(2),pos(3),mat(1,1),mat(2,1),mat(3,1),"off", "Color","r","LineWidth",1);
%     quiver3(pos(1),pos(2),pos(3),mat(1,2),mat(2,2),mat(3,2),"off", "Color","g","LineWidth",1);
%     quiver3(pos(1),pos(2),pos(3),mat(1,3),mat(2,3),mat(3,3),"off", "Color","b","LineWidth",1);

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

for cam_i = 1:length(camera_DCMs)
    for point_i = 1:length(x)
        tehta_azimuth_vec = reshape(Trajectory_elev_azim(cam_i, point_i,:),1,2);
        pos = cams_pos(cam_i,:);
        vector_cam2drone_CAM = (Ry(Trajectory_elev_azim(cam_i,point_i,1))*Rz(Trajectory_elev_azim(cam_i, point_i,2)))'*[1;0;0]*Test_Length(cam_i, point_i);
        DCM_or2cam = DCM_origin2camera(camera_angles(cam_i,1),camera_angles(cam_i,2),camera_angles(cam_i,3));
        vector_cam2drone_ORIGIN = DCM_or2cam'*vector_cam2drone_CAM;

        drone_position(cam_i, point_i,:) = pos' + vector_cam2drone_ORIGIN;
    end
end

plot3(drone_position(1,:,1),drone_position(1,:,2),drone_position(1,:,3), "r--");
plot3(drone_position(2,:,1),drone_position(2,:,2),drone_position(2,:,3), "g--");
plot3(drone_position(3,:,1),drone_position(3,:,2),drone_position(3,:,3), "k--");
plot3(drone_position(4,:,1),drone_position(4,:,2),drone_position(4,:,3), "y--");

disp("DONE WITH TESTING");

%% DROP TO FILE

A = (1920/2)/tand(47);
B = (1080/2)/tand(26.1);


FILE = fopen("injection_data.csv", "w");

for point_i = 1:length(Trajectory_elev_azim(1,:,1))
    for cams_i = 1:length(camera_angles)

        pix_1 = 1920/2 - A*tan(Trajectory_elev_azim(cams_i, point_i, 2));
        pix_2 = 1080/2 + B*tan(Trajectory_elev_azim(cams_i, point_i, 1));
        
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

clear Solution;
run("..//..//Logs//DataLog.m");

figure(); hold on;

% Draw Origin System
% quiver3(0,0,0,50,0,0,"off", "Color","k","LineWidth",1);
% quiver3(0,0,0,0,50,0,"off", "Color","k","LineWidth",1);
% quiver3(0,0,0,0,0,50,"off", "Color","k","LineWidth",1);
% Original Trajectory:
p_true = plot3(x,y,z);
% Calculated Trajectory:
p_calc = plot3(Solution(:,2),Solution(:,3),Solution(:,4), "r");
legend([p_true, p_calc],"Injected", "Calculated Position");


xlabel("X");
ylabel("Y");
zlabel("Z");
axis equal;
% xlim([-20, 70]);
% ylim([-20, 70]);
% zlim([-20, 70]);
view(3); grid on;


%%
clear Solution;
run("..//..//Logs//DataLog.m");

% Calculate original distances from origin:
Original_times = t;
r_squared= x.*x + y.*y + z.*z;
Original_r = sqrt(r_squared);

% Calculate solution distances from origin:
Solution_times = Solution(:,1)-Solution(1,1);
a = Solution(:,2:4).*Solution(:,2:4);
b= a(:,1) + a(:,2) + a(:,3);
Solution_r = sqrt(b);

figure();
plot(Solution_times);
xlabel("Index");
ylabel("Time [sec]");
grid on;

figure(); hold on;
% Plot
plot(Original_times,Original_r);
plot(Solution_times,Solution_r);
legend("Original", "Solution");
xlabel("Time [sec]");
ylabel("Distance From origin");
grid on;

%% HELPER FUNCS 


function DrawCamsFOV(ax, cams_pos, camera_angles, Vertical_halfAngle_deg, Horizontal_halfAngle_deg, depth_m)
    Rz = @(psi) [cos(psi), sin(psi), 0;
            -sin(psi), cos(psi), 0;
            0        , 0        ,1];

    Ry = @(theta) [cos(theta), 0    , -sin(theta);
                   0         , 1    , 0          ;
                   sin(theta), 0    , cos(theta)];
   
    Rx = @(phi) [1, 0       , 0       ;
                 0, cos(phi), sin(phi);
                 0, -sin(phi), cos(phi)];

    DCM_cam2vec = @(psi, theta) Ry(deg2rad(theta))*Rz(deg2rad(psi));
    DCM_origin2camera = @(psi, theta, phi) Rx(deg2rad(phi))*Ry(deg2rad(theta))*Rz(deg2rad(psi));
    
    v1 = DCM_cam2vec(Horizontal_halfAngle_deg, -Vertical_halfAngle_deg)'*[1;0;0];
    v2 = DCM_cam2vec(-Horizontal_halfAngle_deg, -Vertical_halfAngle_deg)'*[1;0;0];
    v3 = DCM_cam2vec(-Horizontal_halfAngle_deg, Vertical_halfAngle_deg)'*[1;0;0];
    v4 = DCM_cam2vec(Horizontal_halfAngle_deg, Vertical_halfAngle_deg)'*[1;0;0];
    
    % Scaling up the depth
    v_len = depth_m/([1,0,0]*v1);
    
    colors = ['r', 'g', 'b', 'y', "#4DBEEE", "#D95319"];
    mat_size = size(cams_pos);
    for camera_i = 1:mat_size(1)
        psi = camera_angles(camera_i,1);
        theta = camera_angles(camera_i,2);
        phi = camera_angles(camera_i,3);
        
        DCM_ori2cam = DCM_origin2camera(psi,theta,phi);
        v1_cam = DCM_ori2cam'*v1.*v_len;
        v2_cam = DCM_ori2cam'*v2.*v_len;
        v3_cam = DCM_ori2cam'*v3.*v_len;
        v4_cam = DCM_ori2cam'*v4.*v_len;
        
        points(1,:) = cams_pos(camera_i,:) + v1_cam';
        points(2,:) = cams_pos(camera_i,:) + v2_cam';
        points(3,:) = cams_pos(camera_i,:) + v3_cam';
        points(4,:) = cams_pos(camera_i,:) + v4_cam';
        points(5,:) = cams_pos(camera_i,:);

        face_indx_sides = [2,5,3,2;
                         3,5,4,3;
                         4,5,1,4;
                         1,5,2,1]';
        face_indx_bottom = [1,2,3,4,1]';
        
        xc = points(:,1);
        yc = points(:,2);
        zc = points(:,3);
        patch(ax, xc(face_indx_sides), yc(face_indx_sides), zc(face_indx_sides), colors(camera_i), 'facealpha', 0.06);
        patch(ax, xc(face_indx_bottom), yc(face_indx_bottom), zc(face_indx_bottom), colors(camera_i), 'facealpha', 0.06);
    end
end