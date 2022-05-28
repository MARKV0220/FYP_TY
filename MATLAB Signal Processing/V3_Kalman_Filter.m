close all;
clear all;
clc;

TotalDistance = 0;
AverageSpeed = 0;

%fileID = fopen('BAF5.txt','r');
fileID = fopen('Grass1.txt','r');
%fileID = fopen('Square3F.txt','r');
formatSpec = '%f';
sizeA = [10 Inf];
AGM = fscanf(fileID,formatSpec,sizeA);      %accelerometer, gyroscope, magnetic x,y,z
AGM = AGM';
fclose(fileID);

AccelX = AGM(:,1);
AccelY = AGM(:,2);
AccelZ = AGM(:,3);
GyroX = AGM(:,4);
GyroY = AGM(:,5);
GyroZ = AGM(:,6);
Bearing = AGM(:,10);

% calibrate the direction of acceleration
AccelX = AccelX*(-1);

% remove the gravity from three axes
NumMea = size(AccelX,1);                   %number of measurements, total row number
% AccelX = AccelX - sum(AccelX)/NumMea;
% AccelY = AccelY - sum(AccelY)/NumMea;
% AccelZ = AccelZ - sum(AccelZ)/NumMea;
AccelX = detrend(AccelX,'linear');
AccelY = detrend(AccelY,'linear');
AccelZ = detrend(AccelZ,'linear');

%transform acceleration into m/s^2---------g = 9.78 m/s^2
g=9.78;
accel_range = 2;                           %+-2g 
AccelX = AccelX/(2^15-1)*g*accel_range;
AccelY = AccelY/(2^15-1)*g*accel_range;
AccelZ = AccelZ/(2^15-1)*g*accel_range;

%tranform rotational speed into deg/s
gyro_range = 250;                           %+-250 degrees/s
GyroX = GyroX/(2^15-1)*gyro_range;
GyroY = GyroY/(2^15-1)*gyro_range;
GyroZ = GyroZ/(2^15-1)*gyro_range;

Fs = 50;
%t = (0:NumMea-1)/Fs;
t = 1:NumMea;
AccelX_New = kalman_filter(AccelX,1e-6,4e-4,0,1);
% figure('Name','Kalman Filter + Original');
% plot(t,AccelX,t,AccelX_New);
% title('Acceleration on X axis before and after Kalman filtering');
% xlabel('Sample');
% ylabel('Acceleration (m/s^2)');
% legend('Original','After filtering');
AccelX = AccelX_New;


%remove the rotational acceleration
% Rotating_Radius = 0;                     %radius = 5 cm
% y2 = AccelX - (GyroY/180*pi).^2 * Rotating_Radius;              %w^2*r = a
% figure;
% plot(t,AccelX,t,y2);
% AccelX = y2;

TrjX = zeros(NumMea,1);
TrjY = zeros(NumMea,1);
v= zeros(NumMea,1);
DeltaT = 0.02;              %sampling time interval is 0.02s (50Hz)

for i=1:NumMea-1
   
    v(i+1,1) = v(i,1)+(AccelX(i+1,1)+AccelX(i,1))/2*DeltaT;             %trapezoidal integration
    v(i+1,1)= abs(v(i+1,1));
    DeltaD = (v(i+1,1)+v(i,1))/2*DeltaT;                                 %the magnitude of each displacement
    TotalDistance = TotalDistance + DeltaD;
    TrjX(i+1,1) = TrjX(i,1) + DeltaD * cos((90-Bearing(i,1))/180*pi);
    TrjY(i+1,1) = TrjY(i,1) + DeltaD * sin((90-Bearing(i,1))/180*pi);
    
end

AverageSpeed = TotalDistance / (NumMea/50);

figure;
scatter(TrjX,TrjY);
%title('Reconstruced Trajectory of Kalman Filter + Double Integration (BAF)');
title('Reconstruced Trajectory around a Building');
title('Reconstruced Trajectory around a Grassland---Kalman Filter');
xlabel('W<-----------------------(meter)--------------------------->E');
ylabel('S<-----------------------(meter)--------------------------->N');
PlotInt = 10;
yval = ylim(gca);
set(gca,'YTick',yval(1):PlotInt:yval(2),"YTickLabel",yval(1):PlotInt:yval(2));
xval = xlim(gca);
set(gca,'XTick',xval(1):PlotInt:xval(2),"XTickLabel",xval(1):PlotInt:xval(2));

figure;
subplot(3,1,1);
plot(AccelX);
title('Acceleration of X axis');
xlabel('Sample');
ylabel('Acceleration (m/s^2)');

subplot(3,1,2);
plot(Bearing);
title('Compass Bearing');
xlabel('Sample');
ylabel('Degree');

subplot(3,1,3);
plot(v);
title('Integrated Velocity');
xlabel('Sample');
ylabel('Velocity (m/s)');


% figure
% subplot(3,1,1);
% plot(GyroX);
% title('Rotational Speed X');
% subplot(3,1,2);
% plot(GyroY);
% title('Rotational Speed Y');
% subplot(3,1,3);
% plot(GyroZ);
% title('Rotational Speed Z');


function X = kalman_filter(data,Q,R,x0,P0)
N = length(data);

K = zeros(N,1);
X = zeros(N,1);
P = zeros(N,1);

X(1) = x0;
P(1) = P0;

for i = 2:N
    K(i) = P(i-1) / (P(i-1) + R);
    X(i) = X(i-1) + K(i) * (data(i) - X(i-1));
    P(i) = P(i-1) - K(i) * P(i-1) + Q;
end

end



