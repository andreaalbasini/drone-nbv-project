function [vicon, currentFilter] = parseData(logFile)

f = importdata(logFile);
t = f(:,1);
vicon.t = t;
vicon.x = f(:,2);
vicon.y = f(:,3);
vicon.z = f(:,4);
vicon.qw = f(:,5);
vicon.qx = f(:,6);
vicon.qy = f(:,7);
vicon.qz = f(:,8);
[vicon.yaw, vicon.pitch, vicon.roll] = ...
    quat2angle([vicon.qw vicon.qx vicon.qy vicon.qz]);
vicon.occluded = zeros(size(t));

for i=2:length(t)
    if vicon.x(i) == 0
%         vicon.x(i) = vicon.x(i-1);
        vicon.occluded(i) = 1;
    end
    if vicon.y(i) == 0
%         vicon.x(i) = vicon.x(i-1);
        vicon.occluded(i) = 1;
    end
    if vicon.z(i) == 0
%         vicon.x(i) = vicon.x(i-1);
        vicon.occluded(i) = 1;
    end
end

currentFilter.t = t;
currentFilter.x = f(:,9);
currentFilter.y = f(:,10);
currentFilter.z = f(:,11);
currentFilter.qw = f(:,12);
currentFilter.qx = f(:,13);
currentFilter.qy = f(:,14);
currentFilter.qz = f(:,15);
currentFilter.dx = f(:,16);
currentFilter.dy = f(:,17);
currentFilter.dz = f(:,18);
currentFilter.p = f(:,19);
currentFilter.q = f(:,20);
currentFilter.r = f(:,21);
[currentFilter.yaw, currentFilter.pitch, currentFilter.roll] = ...
quat2angle([currentFilter.qw currentFilter.qx currentFilter.qy currentFilter.qz]);
