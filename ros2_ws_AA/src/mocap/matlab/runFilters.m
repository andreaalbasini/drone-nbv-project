% runFilters.m

% clc; clear all;

[v, f] = parseData('viconLog1.txt'); % get raw vicon data and filtered data

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
plot(v.t,v.x,f.t,f.x, v.t, pos.x)
legend('Vicon raw','Current filter','GHK Filter')
ylabel('x (m)');
subplot(3,1,2);
plot(v.t,v.y,f.t,f.y, v.t, pos.y)
legend('Vicon raw','Current filter','GHK Filter')
ylabel('y (m)');
subplot(3,1,3);
plot(v.t,v.z,f.t,f.z, v.t, pos.z)
legend('Vicon raw','Current filter','GHK Filter')
ylabel('z (m)');
xlabel('t (s)');

fh(fignum)=figure(fignum); clf; fignum = fignum+1;
subplot(3,1,1);
plot(v.t(2:end),dx,f.t,f.dx, v.t, vel.x)
axis([-inf inf -4 4]);
legend('Vicon raw','Current filter','GHK Filter')
ylabel('dx (m/s)');
subplot(3,1,2);
plot(v.t(2:end),dy,f.t,f.dy, v.t, vel.y)
axis([-inf inf -4 4]);
legend('Vicon raw','Current filter','GHK Filter')
ylabel('dy (m/s)');
subplot(3,1,3);
plot(v.t(2:end),dz,f.t,f.dz, v.t, vel.z)
axis([-inf inf -4 4]);
legend('Vicon raw','Current filter','GHK Filter')
ylabel('dz (m/s)');
xlabel('t (s)');

fh(fignum)=figure(fignum); clf; fignum = fignum+1;
subplot(3,1,1)
plot(v.t, v.roll, f.t, f.roll, v.t, att.roll)
legend('Vicon raw','Current filter','New EKF')
ylabel('pitch (rad)');
subplot(3,1,2)
plot(v.t, v.pitch, f.t, f.pitch, v.t,att.pitch)
legend('Vicon raw','Current filter','New EKF')
ylabel('pitch (rad)');
subplot(3,1,3)
plot(v.t, v.yaw, f.t, f.yaw, v.t, att.yaw)
legend('Vicon raw','Current filter','New EKF')
ylabel('yaw (rad)');
xlabel('t (s)');

fh(fignum)=figure(fignum); clf; fignum = fignum+1;
subplot(3,1,1)
plot(f.t, f.p, v.t, rates.p(2:end))
legend('Current filter','New EKF')
ylabel('p (rad/s)');
subplot(3,1,2)
plot(f.t, f.q, v.t,rates.q(2:end))
legend('Current filter','New EKF')
ylabel('q (rad/s)');
subplot(3,1,3)
plot(f.t, f.r, v.t, rates.r(2:end))
legend('Current filter','New EKF')
ylabel('r (rad/s)');
xlabel('t (s)');

fh(fignum)=figure(fignum); clf; fignum = fignum+1;
subplot(3,1,1)
plot(v.t, f.roll-v.roll, v.t, att.roll-v.roll);
legend('Current filter','New EKF')
ylabel('roll error (rad)');
subplot(3,1,2)
plot(v.t, f.pitch-v.pitch, v.t, att.pitch-v.pitch);
legend('Current filter','New EKF')
ylabel('pitch error (rad)');
subplot(3,1,3)
plot(v.t, f.yaw-v.yaw, v.t, att.yaw-v.yaw);
legend('Current filter','New EKF')
ylabel('yaw error (rad)');
xlabel('t (s)');

% Link x axes
ax = [];
for ii=1:length(fh)
    ax = [ax; get(fh(ii),'children')];
end
linkaxes(ax,'x');
