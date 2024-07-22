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
    
    % Calculation of corner elevation given image-canter elevation:
    A = 1/tand(Vertical_halfAngle_deg);
    A_star = A/cosd(Horizontal_halfAngle_deg);
    corner_elevation = atand(1/A_star);

    v1 = DCM_cam2vec(Horizontal_halfAngle_deg, -corner_elevation)'*[1;0;0];
    v2 = DCM_cam2vec(-Horizontal_halfAngle_deg, -corner_elevation)'*[1;0;0];
    v3 = DCM_cam2vec(-Horizontal_halfAngle_deg, corner_elevation)'*[1;0;0];
    v4 = DCM_cam2vec(Horizontal_halfAngle_deg, corner_elevation)'*[1;0;0];
    
    % Scaling up the vector fo the desired depth
    v_len = depth_m/([1,0,0]*v1);
    
    colors = ['r', 'g', 'b', 'y', 'k', "m"];
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
    
        %%%%%
        
%         vg = DCM_cam2vec(0, -Vertical_halfAngle_deg)'*[1;0;0];
%         v_len_g = depth_m/([1,0,0]*vg);
%         vg_cam = DCM_ori2cam'*vg.*v_len_g;
%         point = cams_pos(camera_i,:) + vg_cam';
%         s=scatter3(point(1),point(2),point(3), 40,"MarkerEdgeColor",'k', "Marker",'.');
% 
%         vg = DCM_cam2vec(Horizontal_halfAngle_deg, 0)'*[1;0;0];
%         v_len_g = depth_m/([1,0,0]*vg);
%         vg_cam = DCM_ori2cam'*vg.*v_len_g;
%         point = cams_pos(camera_i,:) + vg_cam';
%         s=scatter3(point(1),point(2),point(3), 40,"MarkerEdgeColor",'k', "Marker",'.');
        %%%%%
    
    end
end