clear all
close all
clc

dt = 1/952;
x1 = 0:dt:5;
x2 = 5:dt:(5+2*pi);
x = [x1,x2];

nA = .6;
threshold = .02;
noise = nA*rand(size(x)) - nA/2;


yDes = [zeros(size(x1)),cos(x2-5)-1];
yMeas = yDes + noise;
yCal = yMeas(1:length(x1));
[yFil,M] = Filter(yCal,yMeas,threshold);
plot(x,yMeas,'g',x(1:length(yFil))+dt*M/2,yFil,'r',x,yDes,'b')
titleStr = ['Rolling Average Filter with Threshold = ',num2str(threshold),', # of Averaging Points = ',num2str(M)];
title(titleStr)
legend('Noisy Motion','Filtered Motion','Actual Motion','Location','best')

function [newY,M] = Filter(yC,y,threshold)
    
    M = 1;
    Mmax = length(y)/10;
    while M < Mmax
        yt = rollingAverageFilter(yC,M);
        if (max(yt) <= threshold) && (min(yt) >= -threshold)
            break
        end
        M = M + 1;
    end
    if M < Mmax
        newY = rollingAverageFilter(y,M);
    else
        newY = y;
        fprintf('Error during Calibration')
    end
    
end

function filteredY = rollingAverageFilter(y,M)

    for ii = 1:length(y) - M
        filteredY(ii) = sum(y(ii:(ii+M)))/M;
    end
        
end
 