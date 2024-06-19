clear variables;
close all;


%%  Load System Log
run("..//Logs//SystemLog.m");
run("..//Logs//DataLog.m");

%% 
close all;
figure();
hold on;

VEC_LEN = 8;

% Draw Origin System
quiver3(0,0,0,50,0,0,"off", "Color","k","LineWidth",1);
quiver3(0,0,0,0,50,0,"off", "Color","k","LineWidth",1);
quiver3(0,0,0,0,0,50,"off", "Color","k","LineWidth",1);

% Draw Stations
for i = 1:length(Stations.position(:,1))
    pos = Stations.position(i,:);
    mat = reshape(Stations.Origin2StationDCM(i,:,:), 3,3)' .* VEC_LEN;

    quiver3(pos(1),pos(2),pos(3),mat(1,1),mat(2,1),mat(3,1),"off", "Color","r","LineWidth",0.8);
    quiver3(pos(1),pos(2),pos(3),mat(1,2),mat(2,2),mat(3,2),"off", "Color","g","LineWidth",0.8);
    quiver3(pos(1),pos(2),pos(3),mat(1,3),mat(2,3),mat(3,3),"off", "Color","b","LineWidth",0.8);
end

% Draw LOS
% For every Station in recording:
for station_i = 1:4
    pos = Stations.position(station_i,:);
    % For every recorded LOS
    for los_i = 1:length(LOS(station_i,:,1))
        vec = reshape(LOS(station_i,los_i,:),1,3).*VEC_LEN*20;
        quiver3(pos(1),pos(2),pos(3),vec(1),vec(2),vec(3),"off", "Color","y","LineWidth",0.8);
    end
end
scatter3(Solution(1,1),Solution(1,2),Solution(1,3))


view(3);
grid on;
axis equal;
xlim([-10 50]); xlabel("X");
ylim([-10 50]); ylabel("Y");
zlim([-10 50]); zlabel("Z");