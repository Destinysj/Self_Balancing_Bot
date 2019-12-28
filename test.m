global A = csvread('csv_matter.csv');  #do not change this line
global O = csvread('csv_output.csv');  #do not change this line
E_roll = O(:,2);
E_pitch = O(:,1);

# global variable for accelerometer

axh=A(:,1);
axl=A(:,2);
ayh=A(:,3);
ayl=A(:,4);
azh=A(:,5);
azl=A(:,6);

# global variable for gyro

gxh=A(:,7);
gxl=A(:,8);
gyh=A(:,9);
gyl=A(:,10);
gzh=A(:,11);
gzl=A(:,12);

#bit shifting of acc

ax = bitshift(int16(axh),8);#
ax = ax + axl;
axd = (double(ax)/16384);# ????

ay = bitshift(int16(ayh),8);
ay = ay + ayl;
ayd = (double(ay)/16384);

az= bitshift(int16(azh),8);
az = az + azl;
azd = (double(az)/16384);
##bit shifting of gyro

gx = bitshift(int16(gxh),8);
gx = gx + gxl;
gxd = (double(gx)/131);

gy = bitshift(int16(gyh),8);
gy = gy + gyl;
gyd = (double(gy)/131);

gz= bitshift(int16(gzh),8);
gz = gz + gzl;
gzd = (double(gz)/131);

#low pass filter


f_cut = 5;
dT= 0.01;
tau = 1/(2*pi*f_cut);
alpha = 0.600;##tau/(tau + dT);
yax = zeros(size(axd,1),1);
yay = zeros(size(ayd,1),1);
yaz = zeros(size(azd,1),1);
for i = 2:1000,
  
  prev_A1 =  yax(i-1,:);
  xn = axd(i,:);
  yax(i,:)= (xn + alpha*(prev_A1-xn));#e yantra
  #yax(i,:)=(xn +(1-alpha)*(prev_A1-xn));#wiki
  
  yay(1,1) = 0;#ayd(1,1);
  prev_A2 = yay(i-1,:);
  yn = ayd(i,:);
  yay(i,:)= (yn + alpha*(prev_A2-yn));#e yantra
  #yay(i,:)=(yn +(1-alpha)*(prev_A2-yn));#wiki
  #yay(i,:)=yn*alpha +(yn*(1-alpha));
  
  
  prev_A3 = yaz(i-1,:);
  zn = azd(i,:);
  #yax(i,:)=(zn +(1-alpha)*(prev_A3-zn));#my try
  yaz(i,:)= (zn + alpha*(prev_A3-zn));#eyntra
  
endfor

#high pass filter

ygx = zeros(size(gxd,1),1);
ygy = zeros(size(gyd,1),1);
ygz = zeros(size(gzd,1),1);

for i = 2:1000,
  
  prev_y = ygx(i-1,:);
  prev_x = gxd(i-1,:);
  xn = gxd(i,:);
  ygx(i,:)= ((1-alpha)*prev_y + (1-alpha)*(xn - prev_x));#given
  #ygx(i,:)=(alpha * prev_y + alpha *(xn - prev_x));#wikipedia
  
  #ygx_mean(i,:) = sum(ygx(i,:));
  #ygx(i,:) = ygx(i,:) - ygx_mean(i,:);
  
  prev_y = ygy(i-1,:);
  prev_x = gyd(i-1,:);
  xn = gyd(i,:);
  ygy(i,:)= ((1-alpha)*prev_y + (1-alpha)*(yn - prev_x));#given
  #ygx(i,:)=(alpha * prev_y + alpha *(xn-prev_x));#wiki
  #ygy_mean(i,:) = sum(ygx(i,:));
  #ygy(i,:) = ygx(i,:) - ygx_mean(i,:);
  
endfor

# pitch and roll for acc
AnglepitchAcc = (atan2d(yax, sqrt(yay.^2 + yaz.^2)));# atan2d is used to directly convert it radian to degree
AnglerollAcc  = (atan2d(yay,sqrt(yax.^2 + yaz.^2)));

#complementary filter initilization
alpha2 = 0.02; 

anglePitch(1,1) = (1-alpha2)*(ygx(1,1)*dT) + (alpha2) * (AnglepitchAcc(1,1));
angleRoll(1,1) = (1-alpha2)*(ygy(1,1)*dT) + (alpha2) * (AnglerollAcc(1,1));
#complementary filter

for i = 2:1000,
  
anglePitch(i,:) = (1-alpha2)*(anglePitch(i-1,:)+ygx(i,:)*dT) + (alpha2)*(AnglepitchAcc(i,:));# second part is low pass portion acting on acelerometer and first half is integration of gyroscopes's data and 
angleRoll(i,:) = (1-alpha2)*(angleRoll(i-1,:) + ygy(i,:)*dT) + (alpha2) * (AnglerollAcc(i,:));

endfor