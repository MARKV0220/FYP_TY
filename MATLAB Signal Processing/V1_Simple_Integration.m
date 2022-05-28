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
Bearing = AGM(:,10);

% calibrate the direction of acceleration
AccelX = AccelX*(-1);

% remove the gravity from three axes
NumMea = size(AccelX,1);                   %number of measurements, total row number
AccelX = AccelX - sum(AccelX)/NumMea;
AccelY = AccelY - sum(AccelY)/NumMea;
AccelZ = AccelZ - sum(AccelZ)/NumMea;


%transform into m/s^2---------g = 9.78 m/s^2
g=9.78;
AccelX = AccelX/(2^15-1)*g*2;
AccelY = AccelY/(2^15-1)*g*2;
AccelZ = AccelZ/(2^15-1)*g*2;


TrjX = zeros(NumMea,1);
TrjY = zeros(NumMea,1);
v= zeros(NumMea,1);
DeltaT = 0.02;              %sampling time interval is 0.02s (50Hz)

for i=1:NumMea-1
   
    v(i+1,1) = v(i,1)+(AccelX(i+1,1)+AccelX(i,1))/2*DeltaT;             %trapezoidal integration
    %v(i+1,1)= abs(v(i+1,1));
    DeltaD = (v(i+1,1)+v(i,1))/2*DeltaT;                                 %the magnitude of each displacement
    TotalDistance = TotalDistance + DeltaD;
    TrjX(i+1,1) = TrjX(i,1) + DeltaD * cos((90-Bearing(i,1))/180*pi);
    TrjY(i+1,1) = TrjY(i,1) + DeltaD * sin((90-Bearing(i,1))/180*pi);
    
end

AverageSpeed = TotalDistance / (NumMea/50);

figure;
scatter(TrjX,TrjY);
%title('Reconstruced Trajectory of Basic Double Integration (BAF)');
%title('Reconstruced Trajectory around a Building');
title('Reconstruced Trajectory around a Grassland---Basic Method');
xlabel('W<-----------------------(meter)--------------------------->E');
ylabel('S<-----------------------(meter)--------------------------->N');
PlotInt = 30;
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




