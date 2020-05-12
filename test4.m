clear
close all
clc

%% Collecting Data
g = 9.79211;
%Data = Calibrate(getData(),1);
Data = getData();
xacc = Data(:,1);
yacc = Data(:,2);
zacc = Data(:,3);
phidot = Data(:,4);
thetadot = Data(:,5);
psidot = Data(:,6);
xmag = Data(:,7);
ymag = Data(:,8);
zmag = Data(:,9);
t = Data(:,10);

%% Plotting Raw Data
plot(t,xacc)
xlabel('time (s)')
title(' Acceleration Profile x-axis ')
figure
plot(t,yacc)
xlabel('time (s)')
title(' Acceleration Profile y-axis ')
figure
plot(t,zacc)
xlabel('time (s)')
title(' Acceleration Profile z-axis ')
figure
plot(t,phidot)
xlabel('time (s)')
title(' Gyro Profile x-axis ')
figure
plot(t,thetadot)
xlabel('time (s)')
title(' Gyro Profile y-axis ')
figure
plot(t,psidot)
xlabel('time (s)')
title(' Gyro Profile z-axis ')
figure
plot(t,xmag)
xlabel('time (s)')
title(' Magnetometer Profile x-axis ')
figure
plot(t,ymag)
xlabel('time (s)')
title(' Magnetometer Profile y-axis ')
figure
plot(t,zmag)
xlabel('time (s)')
title(' Magnetometer z-axis ')

%% Finding Rotation Angles
thetaAcc = atan2d(xacc,sqrt(yacc.^2+zacc.^2));
phiAcc = atan2d(yacc,sqrt(zacc.^2+xacc.^2));
thetaGyro = zeros(size(thetaAcc));
phiGyro = zeros(size(phiAcc));
psiGyro = zeros(size(phiAcc));

thetaGyro(1) = 0;
phiGyro(1) = 0;
psiGyro(1) = 0;

for ii = 2:length(thetadot)
    dt = t(ii)-t(ii-1);
    thetaGyro(ii) = thetaGyro(ii-1)+dt*thetadot(ii);
    phiGyro(ii) = phiGyro(ii-1) + dt*phidot(ii);
    psiGyro(ii) = psiGyro(ii-1) + dt*psidot(ii);
    
end

theta = thetaGyro + (0*(thetaAcc - thetaGyro));
phi = phiGyro + (0*(phiAcc - phiGyro));
psi = psiGyro;

%% Plotting Angles
% thetap = polyfit(1:length(theta),theta.',10);
% phip = polyfit(1:length(phi),phi.',10);
% psip = polyfit(1:length(psi),psi.',10);
% 
% 
% theta = polyval(thetap,1:length(theta));
% phi = polyval(phip,1:length(phi));
% psi = polyval(psip,1:length(psi));

figure
plot(t,phi);
xlabel('time (s)')
ylabel('(\circ)')
title(' \Phi ')
figure
plot(t, theta);
xlabel('time (s)')
ylabel('\circ')
title(' \theta ')
figure
plot(t,psi);
xlabel('time (s)')
ylabel('(\circ)')
title(' \Psi ')

% for ii = 1:length(theta)
%     w = [phi(ii),theta(ii),psi(ii)];
%     Arduino_draw(w)
%     pause(.001)
% end

function Data = Calibrate(Data, time)

    for ii = 10:length(Data)
        if Data(ii,7)>= time
            break
        end 
    end 
    
    offsets = mean(Data(1:ii,1:6));
    std(Data(:,1:6))
    Data = Data(ii:end,:) - [offsets,time];

end


%% Data

function Data = getData()
    
    Data = [0.000000,0.000000,0.000000,2.086029,-27.596893,1.801910,0.268555,26.904297,-50.231934,0.000000;
0.045422,-0.148220,9.530334,-2.691650,1.704712,0.082245,0.476074,26.892090,-49.902344,0.050000;
0.047813,-0.166150,9.483716,-2.706604,2.272949,0.381317,-1.110840,26.660156,-49.206543,0.100000;
0.058571,-0.160174,9.483716,-0.157013,1.323395,-0.014954,0.378418,26.611328,-49.340820,0.150000;
0.049008,-0.155392,9.493279,-1.966400,2.287903,0.381317,0.158691,27.551270,-49.633789,0.200000;
0.050204,-0.155392,9.486107,-2.235565,1.614990,-0.022430,0.134277,26.855469,-49.108887,0.250000;
0.051399,-0.151806,9.502841,-2.706604,2.287903,0.157013,-0.134277,26.904297,-50.329590,0.310000;
0.056180,-0.164955,9.502841,-2.018738,1.951447,0.328979,0.476074,26.342773,-49.230957,0.360000;
0.064548,-0.163760,9.494473,-0.927124,1.981354,0.463562,0.354004,26.379395,-49.584961,0.410000;
0.062157,-0.162564,9.499255,-2.863617,2.153320,0.186920,-0.305176,26.525879,-49.987793,0.460000;
0.053790,-0.154197,9.504037,-0.485992,1.674805,-0.014954,-0.354004,26.513672,-49.841309,0.510000;
0.051399,-0.155392,9.523162,-0.994415,1.420593,0.014954,0.329590,26.708984,-49.230957,0.570000;
0.040641,-0.156588,9.529139,-2.751465,2.115936,0.044861,-0.061035,26.708984,-49.584961,0.620000;
0.056180,-0.154197,9.527943,-2.773895,1.899109,-0.029907,0.256348,26.806641,-49.719238,0.670000;
0.050204,-0.148220,9.525553,-1.188812,1.906586,0.343933,-0.292969,26.635742,-49.768066,0.720000;
0.050204,-0.155392,9.538701,-2.609406,1.943970,0.209351,0.329590,27.038574,-50.073242,0.770000;
0.046618,-0.154197,9.538701,-2.519684,3.678589,0.059814,0.256348,26.782227,-49.182129,0.820000;
0.059766,-0.167346,9.548264,-1.727142,1.861725,-0.052338,0.061035,27.197266,-50.183105,0.880000;
0.077696,-0.166150,9.548264,-1.069183,1.353302,0.074768,-0.683594,27.148437,-49.743652,0.930000;
0.050204,-0.160174,9.544678,-2.564545,1.757050,0.037384,-0.585937,27.087402,-49.584961,0.980000;
0.056180,-0.164955,9.562608,-2.669220,2.138367,0.179443,-0.170898,26.452637,-49.353027,1.030000;
0.052594,-0.158978,9.555435,-2.527161,1.816864,0.037384,0.390625,26.892090,-49.890137,1.090000;
0.039446,-0.161369,9.572170,-2.549591,2.078552,0.164490,-0.305176,26.977539,-49.853516,1.140000;
0.063352,-0.157783,9.555435,-1.054230,1.637421,-0.089722,0.329590,27.380371,-49.157715,1.190000;
0.057376,-0.158978,9.562608,-1.069183,1.390686,-0.044861,0.195312,27.636719,-48.767090,1.240000;
0.068134,-0.155392,9.560217,-1.689758,1.585083,-0.014954,-0.354004,26.745605,-49.108887,1.290000;
0.059766,-0.150611,9.576951,0.194397,1.824341,0.336456,-0.280762,27.734375,-49.133301,1.350000;
0.054985,-0.163760,9.575756,-0.239258,1.555176,0.179443,0.317383,27.331543,-49.572754,1.400000;
0.049008,-0.154197,9.579342,-2.818756,2.071075,0.351410,-0.244141,26.989746,-49.829102,1.450000;
0.062157,-0.158978,9.582928,-0.216827,1.450500,-0.067291,-0.329590,27.429199,-50.744629,1.500000;
0.054985,-0.149416,9.580537,-1.293488,2.205658,0.314026,-0.158691,27.099609,-49.743652,1.550000;
0.056180,-0.157783,9.579342,-0.328979,1.502838,-0.007477,-0.378418,27.478027,-49.462891,1.600000;
0.053790,-0.157783,9.602054,0.530853,1.742096,0.246735,-0.183105,27.136230,-49.902344,1.660000;
0.049008,-0.160174,9.596077,-2.370148,1.921539,0.014954,-0.122070,27.148437,-49.475098,1.710000;
0.056180,-0.149416,9.591295,-0.620575,1.330872,-0.014954,-0.366211,27.453613,-49.841309,1.760000;
0.063352,-0.163760,9.591295,-1.233673,1.585083,0.000000,0.549316,27.246094,-49.707031,1.810000;
0.051399,-0.154197,9.587709,-0.957031,1.958923,0.321503,-0.158691,27.014160,-49.291992,1.870000;
0.051399,-0.149416,9.608029,-2.586975,1.891632,0.052338,-0.622559,27.111816,-49.841309,1.920000;
0.051399,-0.155392,9.593686,-2.841187,2.033691,0.224304,-0.170898,26.623535,-48.632812,1.970000;
0.060962,-0.155392,9.603249,0.358887,1.659851,0.082245,-0.305176,27.392578,-48.791504,2.020000;
0.054985,-0.156588,9.593686,-0.643005,1.353302,-0.029907,-0.341797,27.502441,-50.500488,2.070000;
0.053790,-0.157783,9.608029,-2.295380,2.168274,0.396271,-0.402832,27.062988,-48.889160,2.120000;
0.054985,-0.156588,9.609225,-1.682281,1.712189,-0.067291,-0.256348,27.380371,-49.963379,2.180000;
0.062157,-0.154197,9.609225,-0.508423,1.704712,0.388794,-0.793457,26.953125,-49.560547,2.230000;
0.056180,-0.149416,9.614006,-2.654266,2.295380,0.261688,-0.573730,27.294922,-49.230957,2.280000;
0.049008,-0.151806,9.619983,-2.370148,2.138367,0.284119,-1.147461,27.087402,-49.267578,2.330000;
0.064548,-0.164955,9.604444,0.246735,1.188812,0.000000,-0.952148,27.111816,-50.439453,2.380000;
0.047813,-0.155392,9.627155,0.351410,1.884155,0.299072,-0.329590,27.062988,-49.401855,2.440000;
0.052594,-0.154197,9.628350,-2.220612,2.138367,0.276642,-0.451660,27.075195,-49.255371,2.490000;
0.064548,-0.164955,9.611616,-0.037384,1.689758,0.029907,-0.122070,27.038574,-50.073242,2.540000;
0.056180,-0.154197,9.622374,-0.523376,1.517792,0.022430,-0.390625,26.562500,-49.890137,2.590000;
0.059766,-0.156588,9.627155,-2.056122,1.786957,0.007477,-0.659180,27.148437,-50.097656,2.640000;
0.052594,-0.153002,9.618788,0.164490,1.532745,0.239258,0.048828,26.916504,-48.486328,2.700000;
0.052594,-0.150611,9.623569,-2.392578,1.772003,-0.007477,-0.048828,26.953125,-48.803711,2.750000;
0.051399,-0.151806,9.611616,-2.489777,2.123413,0.231781,-0.024414,27.111816,-49.267578,2.800000;
0.051399,-0.149416,9.627155,-2.549591,1.831818,0.074768,-0.097656,27.795410,-48.181152,2.850000;
0.051399,-0.158978,9.625959,-2.145844,2.175751,0.261688,-0.817871,26.892090,-49.316406,2.900000;
0.052594,-0.156588,9.628350,-2.153320,2.003784,0.171967,-0.598145,27.062988,-48.767090,2.960000;
0.056180,-0.151806,9.609225,0.089722,1.652374,0.044861,-0.341797,26.550293,-49.438477,3.010000;
0.066938,-0.157783,9.628350,-2.190704,1.764526,-0.067291,-0.598145,26.501465,-49.072266,3.060000;
0.054985,-0.160174,9.634327,0.127106,1.689758,0.358887,-0.610352,27.136230,-49.072266,3.110000;
0.054985,-0.157783,9.619983,-2.661743,1.779480,0.179443,-0.695801,27.038574,-49.096680,3.160000;
0.057376,-0.154197,9.629545,-0.089722,1.413116,0.022430,-0.427246,26.940918,-49.682617,3.220000;
0.053790,-0.153002,9.619983,0.306549,1.659851,0.134583,-0.708008,26.782227,-49.475098,3.270000;
0.052594,-0.160174,9.634327,-2.721558,2.302856,0.321503,0.097656,27.429199,-49.987793,3.320000;
0.047813,-0.155392,9.637913,-2.886047,2.018738,0.022430,-0.451660,27.099609,-49.768066,3.370000;
0.051399,-0.156588,9.628350,0.299072,1.622467,0.052338,-0.451660,27.136230,-49.414062,3.420000;
0.052594,-0.150611,9.631936,-2.295380,2.385101,0.276642,-0.683594,26.782227,-48.937988,3.480000;
0.056180,-0.160174,9.615202,-1.891632,1.704712,-0.074768,-0.329590,27.124023,-50.012207,3.530000;
0.053790,-0.149416,9.631936,-1.577606,2.018738,0.493469,-0.561523,26.855469,-50.329590,3.580000;
0.058571,-0.155392,9.635522,-2.422485,2.026215,0.142059,-1.025391,27.392578,-48.803711,3.630000;
0.058571,-0.155392,9.627155,-1.585083,1.757050,-0.037384,-0.439453,27.087402,-48.339844,3.680000;
0.052594,-0.154197,9.627155,0.321503,1.749573,0.097198,-0.329590,27.380371,-48.913574,3.740000;
0.057376,-0.157783,9.633132,0.441132,1.644897,0.119629,-0.842285,27.551270,-49.548340,3.790000;
0.066938,-0.156588,9.629545,-1.046753,1.614990,-0.014954,-0.402832,27.185059,-48.693848,3.840000;
0.064548,-0.162564,9.633132,0.104675,1.585083,0.074768,-0.646973,26.696777,-48.876953,3.890000;
0.058571,-0.154197,9.645085,-1.570129,2.115936,0.373840,0.195312,26.855469,-49.890137,3.940000;
0.065743,-0.164955,9.627155,-1.652374,1.570129,-0.007477,0.231934,26.879883,-49.682617,3.990000;
0.047813,-0.157783,9.647475,-2.220612,2.145844,0.261688,0.061035,27.148437,-49.731445,4.050000;
0.062157,-0.158978,9.631936,-2.228088,0.037384,0.000000,-0.305176,27.722168,-48.376465,4.100000;
0.045422,-0.151806,9.645085,-2.138367,2.086029,0.014954,-0.317383,27.050781,-49.304199,4.150000;
0.050204,-0.155392,9.633132,-2.766418,2.257996,0.194397,0.378418,27.246094,-49.426270,4.200000;
0.051399,-0.157783,9.651062,-0.388794,1.405640,0.097198,-0.317383,27.209473,-48.815918,4.250000;
0.057376,-0.157783,9.660625,-2.803802,2.056122,0.164490,-0.354004,27.258301,-49.267578,4.310000;
0.063352,-0.156588,9.653452,-2.340240,1.981354,0.127106,-0.183105,27.087402,-48.229980,4.360000;
0.050204,-0.150611,9.640304,-0.261688,1.824341,0.433655,-0.305176,27.233887,-49.218750,4.410000;
0.051399,-0.164955,9.637913,-0.598145,1.487885,-0.074768,0.219727,27.075195,-48.400879,4.460000;
0.043032,-0.153002,9.648671,-2.063599,2.138367,0.373840,0.366211,27.331543,-48.229980,4.520000;
0.058571,-0.150611,9.645085,-1.368256,1.637421,0.007477,0.073242,27.319336,-49.450684,4.570000;
0.059766,-0.151806,9.631936,0.336456,1.405640,0.067291,-0.329590,27.270508,-49.230957,4.620000;
0.054985,-0.153002,9.639109,-2.190704,1.936493,0.149536,-0.524902,26.757812,-48.999023,4.670000;
0.051399,-0.158978,9.642694,0.388794,1.629944,0.044861,-0.732422,27.124023,-49.707031,4.720000;
0.043032,-0.147025,9.648671,-1.689758,1.936493,0.321503,-0.146484,27.465820,-50.207520,4.770000;
0.062157,-0.150611,9.651062,-2.003784,1.854248,0.000000,0.036621,27.319336,-48.876953,4.830000;
0.064548,-0.161369,9.651062,-1.607513,1.644897,0.104675,-0.341797,27.661133,-50.195312,4.880000;
0.054985,-0.161369,9.655843,-2.295380,2.108459,0.014954,-0.671387,27.563477,-48.706055,4.930000;
0.050204,-0.161369,9.639109,-1.996307,1.846771,0.336456,-0.244141,27.075195,-48.779297,4.980000;
0.041836,-0.148220,9.653452,-1.106567,2.130890,0.358887,0.134277,27.124023,-48.901367,5.030000;
0.052594,-0.149416,9.634327,-1.884155,1.973877,0.456085,-0.122070,27.050781,-49.792480,5.090000;
0.054985,-0.154197,9.658234,0.844879,1.592560,0.261688,-0.549316,26.672363,-49.230957,5.140000;
0.049008,-0.151806,9.651062,-3.072968,2.332764,0.314026,-0.305176,27.050781,-49.328613,5.190000;
0.053790,-0.151806,9.637913,0.343933,1.689758,0.239258,-0.354004,27.429199,-49.377441,5.240000;
0.047813,-0.161369,9.649866,-3.087921,2.228088,0.142059,0.170898,27.343750,-49.340820,5.300000;
0.065743,-0.158978,9.645085,-2.340240,1.779480,0.014954,-0.427246,26.770020,-48.754883,5.350000;
0.050204,-0.151806,9.648671,-1.809387,2.100983,0.508423,-0.378418,26.708984,-49.206543,5.400000;
0.049008,-0.155392,9.645085,0.201874,1.734619,0.082245,-0.561523,27.441406,-48.986816,5.450000;
0.047813,-0.161369,9.640304,0.209351,1.540222,0.044861,-0.207520,27.478027,-49.316406,5.500000;
0.057376,-0.164955,9.641499,-2.639313,1.719666,0.067291,0.122070,27.111816,-50.146484,5.550000;
0.060962,-0.153002,9.648671,-1.547699,1.779480,-0.022430,-0.439453,27.246094,-48.937988,5.610000;
0.050204,-0.158978,9.643889,0.269165,1.450500,0.231781,-0.537109,27.197266,-49.719238,5.660000;
0.057376,-0.157783,9.643889,-2.788849,2.048645,0.261688,-1.196289,26.989746,-49.987793,5.710000;
0.054985,-0.157783,9.645085,-0.628052,1.936493,0.336456,-0.695801,26.745605,-49.755859,5.760000;
0.051399,-0.151806,9.659429,-2.018738,1.906586,0.134583,-0.378418,27.050781,-49.707031,5.810000;
0.058571,-0.156588,9.649866,0.358887,1.390686,0.022430,-0.012207,27.075195,-49.194336,5.870000;
0.051399,-0.148220,9.658234,-1.428070,2.093506,0.358887,-0.402832,26.928711,-49.511719,5.920000;
0.060962,-0.148220,9.648671,-2.489777,2.145844,0.022430,-0.183105,26.550293,-50.024414,5.970000;
0.057376,-0.149416,9.655843,-1.674805,1.981354,0.373840,-0.439453,27.111816,-48.547363,6.020000;
0.050204,-0.156588,9.641499,-0.254211,1.278534,-0.074768,-0.695801,26.989746,-49.133301,6.070000;
0.047813,-0.153002,9.652257,0.112152,1.809387,0.209351,-0.732422,27.258301,-50.268555,6.130000;
0.045422,-0.151806,9.665405,0.314026,1.600037,0.157013,-0.695801,26.354980,-48.327637,6.180000;
0.060962,-0.154197,9.652257,-1.555176,1.929016,0.209351,-0.781250,26.501465,-49.401855,6.230000;
0.056180,-0.154197,9.657038,-2.422485,1.996307,0.089722,-0.268555,27.185059,-48.095703,6.280000;
0.063352,-0.168541,9.651062,-0.837402,1.614990,-0.029907,-0.524902,27.075195,-49.572754,6.330000;
0.049008,-0.153002,9.658234,-2.482300,2.265472,0.164490,-0.756836,27.075195,-49.499512,6.390000;
0.049008,-0.154197,9.661819,-2.295380,2.168274,0.231781,-0.781250,26.904297,-50.207520,6.440000;
0.063352,-0.156588,9.646280,-1.315918,1.831818,-0.044861,-0.744629,26.879883,-49.694824,6.490000;
0.060962,-0.157783,9.641499,0.112152,1.345825,0.104675,-0.793457,26.831055,-49.304199,6.540000;
0.052594,-0.149416,9.655843,-2.467346,1.869202,0.134583,-0.598145,26.489258,-49.255371,6.590000;
0.045422,-0.151806,9.661819,-1.293488,1.839294,0.142059,-0.341797,27.368164,-49.316406,6.650000;
0.060962,-0.160174,9.655843,-2.183228,2.071075,0.216827,-0.427246,27.612305,-49.731445,6.700000;
0.053790,-0.169736,9.663014,-1.585083,1.869202,0.007477,-0.817871,27.490234,-49.401855,6.750000;
0.062157,-0.157783,9.653452,-2.549591,2.130890,0.209351,-0.439453,26.660156,-48.181152,6.800000;
0.041836,-0.149416,9.660625,-3.775787,2.056122,0.157013,-0.549316,27.453613,-48.535156,6.850000;
0.059766,-0.149416,9.648671,-1.884155,1.816864,-0.007477,-0.378418,27.026367,-48.718262,6.910000;
0.052594,-0.158978,9.672578,-1.405640,2.048645,0.343933,0.109863,26.989746,-49.157715,6.960000;
0.037055,-0.150611,9.657038,-1.764526,2.130890,0.471039,-0.341797,27.050781,-48.742676,7.010000;
0.077696,-0.163760,9.643889,-0.411224,1.674805,0.014954,-0.463867,27.380371,-48.828125,7.060000;
0.050204,-0.148220,9.661819,-0.463562,1.360779,0.052338,-0.537109,26.879883,-49.133301,7.110000;
0.052594,-0.143439,9.655843,-1.502838,1.428070,-0.067291,-0.878906,27.331543,-48.596191,7.170000;
0.054985,-0.153002,9.663014,-2.317810,1.921539,0.224304,0.097656,26.721191,-48.937988,7.220000;
0.045422,-0.160174,9.649866,0.000000,1.457977,0.246735,0.085449,27.453613,-49.414062,7.270000;
0.075306,-0.158978,9.649866,0.261688,1.652374,0.104675,-0.012207,27.368164,-49.609375,7.320000;
0.027492,-0.149416,9.673773,-2.512207,2.220612,0.493469,-0.231934,27.185059,-47.912598,7.370000;
0.054985,-0.147025,9.670187,-2.235565,2.063599,0.269165,0.036621,27.160645,-50.024414,7.430000;
0.066938,-0.157783,9.659429,0.381317,1.465454,0.037384,-0.170898,27.270508,-48.815918,7.480000;
0.033469,-0.150611,9.676164,-1.151428,1.824341,0.269165,-0.109863,27.380371,-48.864746,7.530000;
0.066938,-0.154197,9.660625,-2.549591,2.228088,0.209351,-0.439453,26.953125,-49.438477,7.580000;
0.070524,-0.167346,9.648671,-1.540222,1.532745,1.899109,-0.439453,27.404785,-49.218750,7.630000;
0.013149,-0.136267,9.686921,-2.564545,1.816864,0.059814,-0.878906,27.026367,-48.925781,7.690000;
0.076501,-0.164955,9.649866,-1.667328,1.652374,-0.082245,-0.231934,27.282715,-48.999023,7.740000;
0.057376,-0.162564,9.642694,-1.884155,1.517792,-0.022430,-0.695801,27.331543,-49.499512,7.790000;
0.031078,-0.147025,9.661819,-2.654266,1.831818,0.119629,-0.512695,27.136230,-49.157715,7.840000;
0.064548,-0.148220,9.649866,0.433655,1.398163,-0.022430,-0.708008,27.661133,-48.742676,7.890000;
0.039446,-0.167346,9.647475,-0.007477,1.390686,-0.029907,-0.781250,26.782227,-49.169922,7.950000;
0.051399,-0.149416,9.670187,-2.519684,1.869202,0.142059,-0.305176,27.478027,-49.377441,8.000000;
0.054985,-0.151806,9.651062,-2.818756,2.250519,0.239258,-0.903320,26.843262,-49.511719,8.050000;
0.050204,-0.155392,9.654648,-1.188812,1.884155,0.299072,-0.427246,27.038574,-49.169922,8.100000;
0.043032,-0.150611,9.661819,-2.385101,1.899109,0.426178,-1.074219,26.928711,-49.304199,8.150000;
0.047813,-0.142244,9.673773,-1.188812,1.637421,-0.089722,-0.341797,27.172852,-48.681641,8.200000;
0.059766,-0.156588,9.643889,-1.981354,1.779480,0.022430,-0.598145,27.014160,-48.156738,8.260000;
0.053790,-0.155392,9.668991,-1.076660,1.936493,0.381317,-0.988770,27.038574,-49.450684,8.310000;
0.044227,-0.151806,9.659429,-2.504730,2.018738,0.142059,-0.744629,27.270508,-48.986816,8.360000;
0.063352,-0.153002,9.657038,-2.228088,2.183228,0.403748,-0.573730,26.831055,-48.730469,8.410000;
0.028688,-0.155392,9.682140,-2.886047,2.123413,0.097198,-0.634766,26.989746,-48.840332,8.470000;
0.053790,-0.155392,9.654648,-2.340240,1.996307,0.171967,-0.036621,27.148437,-48.791504,8.520000;
0.045422,-0.154197,9.679749,-1.659851,1.674805,-0.029907,-0.500488,26.354980,-49.340820,8.570000;
0.040641,-0.147025,9.654648,-0.844879,1.846771,0.388794,-0.549316,26.831055,-49.536133,8.620000;
0.080087,-0.167346,9.654648,-2.721558,1.914063,0.157013,-0.109863,27.197266,-49.621582,8.670000;
0.028688,-0.150611,9.683335,-1.487885,1.689758,-0.029907,-0.646973,26.855469,-48.632812,8.730000;
0.044227,-0.132681,9.678554,-0.628052,1.876678,0.224304,-0.598145,26.708984,-48.510742,8.780000;
0.063352,-0.166150,9.654648,-0.964508,1.786957,-0.029907,-0.280762,27.014160,-49.768066,8.830000;
0.057376,-0.150611,9.660625,-2.160797,1.831818,-0.014954,-0.366211,27.380371,-49.597168,8.880000;
0.075306,-0.158978,9.649866,-2.175751,1.659851,0.037384,-0.744629,26.904297,-48.352051,8.930000;
0.045422,-0.145830,9.667796,-2.714081,1.973877,0.029907,-0.671387,26.989746,-48.608398,8.990000;
0.026297,-0.141048,9.676164,-0.777588,1.996307,0.343933,-0.695801,27.624512,-48.486328,9.040000;
0.072915,-0.148220,9.653452,-2.183228,3.715973,0.022430,-0.634766,27.087402,-49.841309,9.090000;
0.057376,-0.156588,9.646280,0.216827,1.644897,0.029907,-0.427246,27.233887,-49.084473,9.140000;
0.062157,-0.155392,9.661819,0.224304,1.689758,0.104675,-0.720215,27.758789,-48.583984,9.190000;
0.046618,-0.155392,9.659429,0.029907,1.824341,0.142059,-0.695801,27.404785,-48.669434,9.240000;
0.052594,-0.157783,9.654648,-1.502838,1.570129,-0.007477,-0.854492,27.282715,-50.146484,9.300000;
0.044227,-0.139853,9.660625,0.471039,1.704712,0.044861,-0.634766,27.368164,-48.437500,9.350000;
0.056180,-0.170932,9.663014,-2.340240,2.429962,0.201874,-0.891113,27.148437,-49.291992,9.400000;
0.062157,-0.157783,9.659429,0.328979,1.652374,0.112152,-0.622559,26.940918,-48.461914,9.450000;
0.049008,-0.151806,9.685726,-2.213135,1.936493,0.029907,-0.439453,26.928711,-49.304199,9.500000;
0.058571,-0.155392,9.666601,0.239258,1.570129,0.037384,-0.488281,27.490234,-48.510742,9.560000;
0.040641,-0.136267,9.658234,0.239258,1.749573,0.164490,-0.500488,26.818848,-49.438477,9.610000;
0.049008,-0.155392,9.672578,0.583191,1.644897,0.089722,-0.659180,26.818848,-48.742676,9.660000;
0.051399,-0.160174,9.670187,0.388794,1.712189,0.067291,-0.439453,27.124023,-49.597168,9.710000;
0.051399,-0.160174,9.655843,0.149536,1.727142,0.082245,-0.500488,27.124023,-49.108887,9.760000;
0.059766,-0.151806,9.660625,-0.157013,1.323395,-1.899109,-0.573730,26.733398,-49.133301,9.820000;
0.058571,-0.160174,9.671382,-0.403748,1.480408,0.007477,-0.463867,27.343750,-48.559570,9.870000;
0.049008,-0.151806,9.663014,-1.614990,1.749573,-0.074768,-0.146484,27.160645,-48.791504,9.920000;
0.065743,-0.170932,9.664210,-1.846771,1.839294,0.044861,-0.122070,27.380371,-48.815918,9.970000;
0.057376,-0.161369,9.664210,-2.631836,1.996307,0.216827,-0.537109,26.843262,-48.645020,10.020000;
0.041836,-0.141048,9.676164,-2.886047,1.996307,-0.089722,-0.585937,27.392578,-50.183105,10.080000;
0.050204,-0.160174,9.676164,-2.766418,2.130890,0.306549,-0.634766,26.708984,-48.339844,10.130000;
0.056180,-0.156588,9.664210,-2.594452,1.996307,0.186920,-0.573730,27.136230,-48.791504,10.180000;
0.051399,-0.151806,9.655843,0.112152,1.876678,0.097198,-0.146484,27.685547,-48.352051,10.230000;
0.074110,-0.162564,9.660625,0.538330,1.547699,0.007477,-0.195312,27.661133,-49.694824,10.280000;
0.035860,-0.141048,9.673773,-2.355194,2.100983,-0.037384,-0.671387,27.526855,-49.243164,10.340000;
0.049008,-0.154197,9.670187,0.665436,1.764526,0.261688,-1.318359,27.685547,-49.145508,10.390000;
0.068134,-0.173322,9.648671,-2.407532,1.667328,-0.037384,-0.952148,28.051758,-49.072266,10.440000;
0.058571,-0.145830,9.671382,-0.373840,1.420593,-0.059814,-0.927734,27.209473,-48.498535,10.490000;
0.051399,-0.151806,9.671382,-2.736511,2.145844,0.186920,-0.280762,27.124023,-48.815918,10.540000;
0.052594,-0.155392,9.657038,-0.523376,1.405640,-0.134583,-0.317383,26.940918,-48.876953,10.600000;
0.053790,-0.149416,9.658234,0.164490,1.674805,0.134583,-0.622559,26.757812,-48.925781,10.650000;
0.050204,-0.151806,9.660625,-0.261688,1.532745,-0.007477,-1.098633,27.038574,-48.754883,10.700000;
0.056180,-0.153002,9.665405,-2.362671,2.003784,0.381317,-0.744629,27.307129,-48.657227,10.750000;
0.054985,-0.154197,9.672578,-0.717773,1.532745,-0.164490,-1.025391,27.185059,-49.609375,10.800000;
0.046618,-0.144634,9.668991,-1.757050,1.794434,0.104675,-0.732422,26.953125,-48.486328,10.860000;
0.058571,-0.157783,9.660625,0.358887,1.629944,0.381317,-1.074219,27.026367,-49.011230,10.910000;
0.051399,-0.148220,9.671382,-1.749573,1.532745,-0.007477,-0.708008,26.965332,-48.889160,10.960000;
0.051399,-0.160174,9.657038,0.770111,1.450500,0.127106,-0.891113,27.404785,-48.974609,11.010000;
0.051399,-0.151806,9.676164,-2.504730,1.914063,0.059814,-1.171875,27.416992,-48.107910,11.060000;
0.041836,-0.149416,9.676164,-2.586975,1.801910,0.007477,-0.988770,27.014160,-48.803711,11.120000;
0.054985,-0.156588,9.672578,-0.456085,1.943970,0.261688,-0.512695,26.794434,-49.475098,11.170000;
0.038250,-0.150611,9.674968,-0.942078,1.487885,-0.059814,-0.329590,27.209473,-48.986816,11.220000;
0.050204,-0.154197,9.671382,-0.994415,1.697235,-0.029907,-0.524902,27.441406,-49.389648,11.270000;
0.056180,-0.157783,9.671382,-2.243042,1.824341,-0.022430,-0.610352,27.380371,-49.365234,11.320000;
0.041836,-0.148220,9.668991,-0.082245,1.846771,0.358887,-0.524902,27.380371,-49.206543,11.380000;
0.000000,-1.128388,9.100017,57.496643,-10.160980,11.446991,-0.646973,26.684570,-48.950195,11.430000;
0.026297,-0.221135,9.759836,-1.188812,1.801910,-0.164490,0.024414,26.477051,-49.121094,11.480000;
1.567072,1.087747,9.947502,0.373840,2.616882,-2.370148,0.756836,25.158691,-49.890137,11.530000;
0.245042,1.634011,10.229599,-0.807495,0.418701,-34.916687,0.634766,25.305176,-49.047852,11.580000;
0.016735,-0.245042,10.302514,-0.889740,1.629944,-0.411224,0.842285,25.146484,-48.889160,11.640000;
0.010758,-0.243846,10.410093,-0.912170,1.592560,-0.179443,0.463867,25.634766,-47.729492,11.690000;
0.003586,-0.242651,10.412484,-0.448608,1.757050,-0.186920,0.708008,25.402832,-49.328613,11.740000;
0.037055,-0.261776,10.358694,-0.807495,1.480408,-0.239258,1.062012,25.683594,-48.815918,11.790000;
-0.002391,-0.273730,10.325226,-4.299164,0.986938,-0.209351,-0.537109,26.696777,-48.815918,11.840000;
-0.272534,0.241456,10.388577,0.994415,0.957031,3.536530,-0.732422,27.819824,-48.474121,11.890000;
0.041836,-0.219940,10.199717,-1.443024,2.071075,0.328979,-0.793457,28.344727,-48.620605,11.950000;
0.044227,-0.202010,10.148317,-1.914063,1.682281,0.164490,-1.281738,27.661133,-48.632812,12.000000;
0.014344,-0.248628,10.106481,-0.822449,2.325287,-0.194397,-0.817871,27.319336,-49.511719,12.050000;];


end