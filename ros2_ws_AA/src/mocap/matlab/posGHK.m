function [pos,vel,acc] = posGHK(v,theta)

% get dt
dt = diff(v.t);

% compute gains
g=1-theta^3;
h=1.5*(1-theta^2)*(1-theta);
k=0.5*(1-theta)^3;

% set up and call filter
xk = [1 0 0]';            % initial state (position)
vk = [0 0 0]';            % initial dState/dt (velocity)
ak = [0 0 0]';            % initial d2State/dt2 (acceleration)
XM = zeros(length(v.t),3); XK = zeros(length(v.t),3);
VK = zeros(length(v.t),3); AK = zeros(length(v.t),3); RK = zeros(length(v.t),3);
for t = 1:length(v.t)-1
    xm = [v.x(t) v.y(t) v.z(t)]';
    XM(t,:) = xm;
    [xk,vk,ak,rk] = ghkFilter(xm, dt(t), xk, vk, ak, g, h, k, v.occluded(t));
    XK(t,:) = xk; VK(t,:) = vk; AK(t,:) = ak; RK(t,:) = rk;
end

% assign outputs
pos.x = XK(:,1);
pos.y = XK(:,2);
pos.z = XK(:,3);
vel.x = VK(:,1);
vel.y = VK(:,2);
vel.z = VK(:,3);
acc.x = AK(:,1);
acc.y = AK(:,2);
acc.z = AK(:,3);