clear all
close all
clc

addpath('Functions')
addpath('Animation Stuff')
data = getData('OrientationTestv1.txt',10);
Sa = data(1:3,1:end-10);
Sw = [zeros(1,length(data)-10);data(4:6,1:end-10)].';
Sm = [zeros(1,length(data)-10);data(7:9,1:end-10)].';
t = data(10,1:end-10);
% 
for ii = 1:3
    Saf(ii,:) = rollingAverageFilter(Sa(ii,:),10);
end

for ii = 2:4
    Swf(:,ii) = rollingAverageFilter(Sw(:,ii),10);
end

for ii = 2:4
    Smf(:,ii) = rollingAverageFilter(Sm(:,ii),10);
end

Sa(:,1) = -Sa(:,1);
Sw(:,2) = -Sw(:,2);

q(1,:) = [1 0 0 0];
wErr(1,:) = [0 0 0 0];
dt(1) = 0;
[B,C] = calibrate(Sw,t,5);
%B = 5;

for ii = 2:length(t) 
    dt(ii) = t(ii) - t(ii - 1);
    [q(ii,:),wErr] = OrientationUpdate(q(ii-1,:),Sa(:,ii),Sm(ii,:),Sw(ii,:),B,C,dt,wErr);
end

eul = rad2deg(quatern2euler(q));

% for ii = 1:3
%     eul(:,ii) = rollingAverageFilter(eul(:,ii),20);
% end

figure(1)
plot(t,eul(:,1))
figure(2)
plot(t,eul(:,2))
figure(3)
plot(t,eul(:,3))

% for ii = 1:9
%     figure(ii)
%     plot(data(10,:),data(ii,:))
% end

% figure(4)
% for ii = 20:length(q)
%     R = quatern2rotMat(q(ii,:));
%     Arduino_draw(R);
%     pause(0.0001)
%     
% end
% figure(1)
% plot(t,Sw(:,2),t,Swf(:,2))
% figure(2)
% plot(t,Sw(:,3),t,Swf(:,3))
% figure(3)
% plot(t,Sw(:,4),t,Swf(:,4))

function [B,C] = calibrate(w,t,calTime)

    for ii = 1:length(t)
        if t(ii) >= (t(1) + calTime)
            break
        end
    end
    B = norm(mean(w))*sqrt(3/4);
    C = 0;

end

function filteredY = rollingAverageFilter(y,M)

    filteredY = y;
    for ii = 1:(length(y) - M)
        filteredY(ii) = sum(y(ii:(ii+M-1)))/M;
    end
        
end

function data = getData(filename,col)
    for ii = 1:col
        if ii < col
            formatSpec{ii} = '%lf ';
        else
            formatSpec{ii} = '%lf';
        end
    end
    formatSpec = cell2mat(formatSpec);
    fid = fopen(filename,'r');
    sizeA = [col Inf];
    data = fscanf(fid,formatSpec,sizeA);
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