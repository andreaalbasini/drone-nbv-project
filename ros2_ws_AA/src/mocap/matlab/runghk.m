% runFilters.m

clc; clear all;

% get raw vicon data and filtered data
data = importdata('raw_measurement.csv');
v.x = data.data(:,1);
v.y = data.data(:,2);
v.z = data.data(:,3);
v.qw = data.data(:,4);
v.qx = data.data(:,5);
v.qy = data.data(:,6);
v.qz = data.data(:,7);
v.t = str2double(data.textdata(2:end,1));
v.t = v.t - v.t(1);
v.t = v.t/1000000000;
[v.yaw, v.pitch, v.roll] = ...
    quat2angle([v.qw v.qx v.qy v.qz]);

v.occluded = zeros(size(v.t));

for i=2:length(v.t)
    if v.x(i) == 0
%         vicon.x(i) = vicon.x(i-1);
        v.occluded(i) = 1;
    end
    if v.y(i) == 0
%         vicon.x(i) = vicon.x(i-1);
        v.occluded(i) = 1;
    end
    if v.z(i) == 0
%         vicon.x(i) = vicon.x(i-1);
        v.occluded(i) = 1;
    end
end


%% Position/Velocity/Acceleration Static GHK Filter
% User inputs
theta = 0.8;
[pos,vel,acc] = posGHK(v,theta);

% get x velocity estimates from raw vicon data
dt = diff(v.t);
dx = diff(v.x)./dt;
dy = diff(v.y)./dt;
dz = diff(v.z)./dt;

%% Attitude/Rates EKF 
% User inputs
useRK4 = true; % if true, use rk4 integration, otherwise, use ode45
Wc=1;       % process noise
Rc=1e-7;    % measurement noise
[att, rates] = attEkf(v,Wc,Rc,useRK4);


%% Plot data
fignum = 1;
fh(fignum)=figure(fignum); clf; fignum = fignum+1;
subplot(3,1,1);
plot(v.t,v.x, v.t, pos.x)
legend('Vicon raw','GHK Filter')
ylabel('x (m)');
subplot(3,1,2);
plot(v.t,v.y, v.t, pos.y)
legend('Vicon raw','GHK Filter')
ylabel('y (m)');
subplot(3,1,3);
plot(v.t,v.z, v.t, pos.z)
legend('Vicon raw','GHK Filter')
ylabel('z (m)');
xlabel('t (s)');

fh(fignum)=figure(fignum); clf; fignum = fignum+1;
subplot(3,1,1);
plot(v.t,v.x)
ylabel('x (m)');
subplot(3,1,2);
plot(v.t,v.y)
ylabel('y (m)');
subplot(3,1,3);
plot(v.t,v.z)
ylabel('z (m)');
xlabel('t (s)');

fh(fignum)=figure(fignum); clf; fignum = fignum+1;
subplot(3,1,1);
plot(v.t(2:end),dx, v.t, vel.x)
axis([-inf inf -4 4]);
legend('Vicon raw','GHK Filter')
ylabel('dx (m/s)');
subplot(3,1,2);
plot(v.t(2:end),dy, v.t, vel.y)
axis([-inf inf -4 4]);
legend('Vicon raw','GHK Filter')
ylabel('dy (m/s)');
subplot(3,1,3);
plot(v.t(2:end),dz, v.t, vel.z)
axis([-inf inf -4 4]);
legend('Vicon raw','GHK Filter')
ylabel('dz (m/s)');
xlabel('t (s)');

fh(fignum)=figure(fignum); clf; fignum = fignum+1;
subplot(3,1,1)
plot(v.t, v.roll, v.t, att.roll)
legend('Vicon raw','New EKF')
ylabel('pitch (rad)');
subplot(3,1,2)
plot(v.t, v.pitch, v.t,att.pitch)
legend('Vicon raw','New EKF')
ylabel('pitch (rad)');
subplot(3,1,3)
plot(v.t, v.yaw, v.t, att.yaw)
legend('Vicon raw','New EKF')
ylabel('yaw (rad)');
xlabel('t (s)');

fh(fignum)=figure(fignum); clf; fignum = fignum+1;
subplot(3,1,1)
plot(v.t, rates.p(2:end))
legend('New EKF')
ylabel('p (rad/s)');
subplot(3,1,2)
plot(v.t,rates.q(2:end))
legend('New EKF')
ylabel('q (rad/s)');
subplot(3,1,3)
plot(rates.r(2:end))
legend('New EKF')
ylabel('r (rad/s)');
xlabel('t (s)');

% Link x axes
ax = [];
for ii=1:length(fh)
    ax = [ax; get(fh(ii),'children')];
end
linkaxes(ax,'x');
