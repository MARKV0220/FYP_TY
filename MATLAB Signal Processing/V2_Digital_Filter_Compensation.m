close all;
clear all;
clc;

TotalDistance = 0;
AverageSpeed = 0;

%fileID = fopen('BAF5.txt','r');
%fileID = fopen('Square3F.txt','r');
fileID = fopen('Grass1.txt','r');
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


% The FFT process to check the frequency component
% figure;
% AccelXFFT = fft(AccelX);
% L = 838;
Fs = 50;
% P2 = abs(AccelXFFT/L);
% P1 = P2(1:L/2+1);
% P1(2:end-1) = 2*P1(2:end-1);
% f = Fs*(0:(L/2))/L;
% plot(f,P1) ;

% Digtal filtering process for multiple variable
% Digital filter for AccelX ---- cut off 3 Hz
D = 35;
%t = (0:NumMea-1)/Fs;
t = 1:NumMea;
Fnorm1 = 3/(Fs/2);
df1 = designfilt('lowpassfir','FilterOrder',70,'CutoffFrequency',Fnorm1);
%fvtool(df1);
y1 = filter(df1,[AccelX; zeros(D,1)]); 
y1 = y1(D+1:end); 
% figure;
% plot(t,AccelX,t,y1);
% title('Acceleration on X axis before and after 70^t^h order digital filtering');
% xlabel('Sample');
% ylabel('Acceleration (m/s^2)');
% legend('Original','After filtering');
AccelX = y1;

% Digital filter for GyroY ---- cut off 1 Hz
Fnorm2 = 1/(Fs/2);
df2 = designfilt('lowpassfir','FilterOrder',70,'CutoffFrequency',Fnorm2);
y2 = filter(df2,[GyroY; zeros(D,1)]); 
y2 = y2(D+1:end); 
GyroY = y2;

% Digital filter for AccelY ---- cut off 1 Hz
y3 = filter(df2,[AccelY; zeros(D,1)]); 
y3 = y3(D+1:end); 
AccelY = y3;

% Digital filter for GyroZ ---- cut off 1 Hz
y4 = filter(df2,[GyroZ; zeros(D,1)]); 
y4 = y4(D+1:end); 
GyroZ = y4;

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
%title('Reconstruced Trajectory of Digital Filter + Double Integration (BAF)');
%title('Reconstruced Trajectory around a Building');
title('Reconstruced Trajectory around a Grassland---Digital Filter');
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







