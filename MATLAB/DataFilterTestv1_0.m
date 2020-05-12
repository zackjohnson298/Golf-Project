clear all
close all
clc

% Raw Data
data = getData();
rddot = data(1:3,:).*9.81;
omega = data(4:6,:);
t = data(7,:);

% Filter Data
M = [20,20,20,20,20,20];
FilteredData = filterData(data,M);
newrddot = FilteredData(1:3,:).*9.81;
newomega = FilteredData(4:6,:);

% Define Calibration Period and Data
tc = 2; %Calibration time in seconds
ii = 1;
while t(ii) <= (t(1) + tc)
    ii = ii + 1;
end
CalData = FilteredData(:,1:ii);

G = (mean(CalData(1:3,:).').*9.80665).';
angleError = mean(CalData(4:6,:).').';
rddot = FilteredData(1:3,ii:end);
w = deg2rad(FilteredData(4:6,ii:end) - angleError);
t = t(ii:end);
T = eye(3);
r(:,1) = [0 0 0].';
rdot(:,1) = [0 0 0].';
Ir(:,1) = [0 0 0].';
gamma(:,1) = [0 0 0].';

for ii = 2:length(t)
    dt = t(ii) - t(ii-1);
    dgamma = w(:,ii)*dt;
    gamma(:,ii) = gamma(:,ii-1) + dgamma;
    Tii = rotz(dgamma(1))*roty(dgamma(2))*rotx(dgamma(3));
    T = T*Tii;
    rddot(:,ii) = rddot(:,ii) - T.'*G;
    rdot(:,ii) = (rddot(:,ii)*dt);% - rddot(:,ii-1))*dt;
    r(:,ii) = (rdot(:,ii)*dt);% - rdot(:,ii-1))*dt;
    Ir(:,ii) = Ir(:,ii-1) + T*r(:,ii);
end

Ir = Ir(:,3:end);
gamma = rad2deg(gamma(:,3:end));
t = t(3:end);

figure(1)
plot(t,Ir(1,:))
figure(2)
plot(t,Ir(2,:))
figure(3)
plot(t,Ir(3,:))
figure(4)
plot(t,gamma(1,:))
figure(5)
plot(t,gamma(2,:))
figure(6)
plot(t,gamma(3,:))


% 
% % Plot Accelerations
% figure(1)
% plot(t,rddot(1,:),t,newrddot(1,:))
% titlestr = ['X Acceleration vs. Time, M = ',num2str(M(1))];
% title(titlestr)
% xlabel('Time t [s]')
% ylabel('Acceleration xddot [m/s^2]')
% legend('Raw Data','Filered Data','Location','best')
% figure(2)
% plot(t,rddot(2,:),t,newrddot(2,:))
% titlestr = ['Y Acceleration vs. Time, M = ',num2str(M(2))];
% title(titlestr)
% xlabel('Time t [s]')
% ylabel('Acceleration yddot [m/s^2]')
% legend('Raw Data','Filered Data','Location','best')
% figure(3)
% plot(t,rddot(3,:),t,newrddot(3,:))
% titlestr = ['Z Acceleration vs. Time, M = ',num2str(M(3))];
% title(titlestr)
% xlabel('Time t [s]')
% ylabel('Acceleration zddot [m/s^2]')
% legend('Raw Data','Filered Data','Location','best')
% figure(4)
% plot(t,omega(1,:),t,newomega(1,:))
% titlestr = ['Angular Velocity about X axis vs. Time, M = ',num2str(M(4))];
% title(titlestr)
% xlabel('Time t [s]')
% ylabel('Angular Velocity \Phidot [deg/s]')
% legend('Raw Data','Filered Data','Location','best')
% figure(5)
% plot(t,omega(2,:),t,newomega(2,:))
% titlestr = ['Angular Velocity about Y axis vs. Time, M = ',num2str(M(5))];
% title(titlestr)
% xlabel('Time t [s]')
% ylabel('Angular Velocity \Thetadot [deg/s]')
% legend('Raw Data','Filered Data','Location','best')
% figure(6)
% plot(t,omega(3,:),t,newomega(3,:))
% titlestr = ['Angular Velocity about Z axis vs. Time, M = ',num2str(M(6))];
% title(titlestr)
% xlabel('Time t [s]')
% ylabel('Angular Velocity \Psidot [deg/s]')
% legend('Raw Data','Filered Data','Location','best')

function newData = filterData(data,M)
    newData = data;
    [m,n] = size(data);
    for ii = 1:(m-1)
        newData(ii,:) = rollingAverageFilter(data(ii,:),M(ii));
    end
end

function filteredY = rollingAverageFilter(y,M)

    filteredY = y;
    for ii = 1:(length(y) - M)
        filteredY(ii) = sum(y(ii:(ii+M-1)))/M;
    end
        
end

function data = getData()
    fid = fopen('A-Gdata.txt','r');
    formatSpec = '%f %f %f %f %f %f %f';
    sizeA = [7 Inf];
    A = fscanf(fid,formatSpec,sizeA);
    data = A;
% 
%     ii = 2;
%     while ii < length(data)
%         if data(ii,7) == data(ii-1,7)
%             data(ii:(end-1),:) = data((ii+1):end,:);
%             data = data(1:(end-1),:);
%         end
%         ii = ii + 1;
%     end

end
