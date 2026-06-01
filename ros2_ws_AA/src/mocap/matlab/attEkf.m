% attEkf.m

% plotData.m

function [att,rates] = attEkf(v,Wc,Rc,useRK4)
% clc; clear all;

N = length(v.t);
x0 = [0 0 0 0 0 0 1];
Q0 = eye(7);

wa=0.0; % fict noise
Wc=diag([Wc;Wc;Wc;wa;wa;wa;wa]);
Rd=diag([Rc;Rc;Rc;Rc])/0.005;

nA=7;
nC = 4;
xhatm=zeros(nA,N+1);
xhatp=zeros(nA,N+1);
L=zeros(nA,nC,N+1);

Qm=Q0;
xhatm(:,1)=x0;
% Qxb=zeros(nA,N+1);

C = [0 0 0 1 0 0 0;
     0 0 0 0 1 0 0;
     0 0 0 0 0 1 0;
     0 0 0 0 0 0 1];
ATT = zeros(length(v.t),3);
flip = 1;
for ii=1:N;
    if mod(ii,100) == 0
        display(ii);
    end
    if v.occluded(ii) == 0
        Qp=Qm-Qm*C'*inv(C*Qm*C'+Rd)*C*Qm;
        L(:,:,ii)=Qp*C'/Rd;
        meas = [v.qx(ii); v.qy(ii); v.qz(ii); v.qw(ii)];
        if ii > 1
            dotprod = v.qx(ii)*v.qx(ii-1) + v.qy(ii)*v.qy(ii-1) + v.qz(ii)*v.qz(ii-1) + v.qw(ii)*v.qw(ii-1);
            if (dotprod < 0.0) 
                flip = -flip;
            end
        end
        meas = flip*meas;
        xhatp(:,ii)=xhatm(:,ii)+ L(:,:,ii)*(meas-C*xhatm(:,ii));
%         Qxb(:,ii)=sqrt(diag(Qp));
    else
        xhatp(:,ii) = xhatm(:,ii);
    end
    [ATT(ii,3), ATT(ii,2), ATT(ii,1)] = quat2angle([xhatp(7,ii),xhatp(4,ii),xhatp(5,ii),xhatp(6,ii)]);
    
    if (ii==N), break; end
    if useRK4
        [~,xx]=rk4(@(t,x)meanvardot(t,x,Wc),v.t(ii),v.t(ii+1),[xhatp(:,ii);vec(Qp)],3);
        xx = xx(:,end);
        xhatm(:,ii+1) = xx(1:nA);
    else
        [~,xx]=ode45(@(t,x)meanvardot(t,x,Wc),[v.t(ii) v.t(ii+1)],[xhatp(:,ii);vec(Qp)]);
        xhatm(:,ii+1)=xx(end,1:nA)';
        Qm=unvec(xx(end,nA+1:end));  
    end

    Qm = unvec(xx(nA+1:end));
  
end

% assign output variables
att.roll = ATT(:,1);
att.pitch = ATT(:,2);
att.yaw = ATT(:,3);

rates.p = xhatp(1,:)';
rates.q = xhatp(2,:)';
rates.r = xhatp(3,:)';

return

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function xQdot=meanvardot(t,xQ,Wc)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
nA=7;
x=xQ(1:nA);
Q=unvec(xQ(nA+1:end));

xdot=processModel(t,x);

A=transitionMatrix(x);
Qdot=A*Q+Q*A'+Wc;
Qdot=(Qdot+Qdot')/2;
Qvecdot=vec(Qdot);

xQdot=[xdot;Qvecdot];

return

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function y=vec(P)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
y=[];
for ii=1:length(P);
    y=[y;P(ii,ii:end)'];
end
return

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function P=unvec(y)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
persistent N
if isempty(N)
    N=max(roots([1 1 -2*length(y)]));
end
P=[];kk=N;kk0=1;
for ii=1:N;
    P(ii,ii:N)=[y(kk0+[0:kk-1])]';
    kk0=kk0+kk;
    kk=kk-1;
end
P=(P+P')-diag(diag(P));
return
