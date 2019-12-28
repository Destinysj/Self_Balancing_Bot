#dead line Monday, November 18, 2019, 11:59 pm

global A = csvread('csv_matter.csv');  #do not change this line
global O = csvread('csv_output.csv');  #do not change this line

#taxh = 241;
#taxl = 88;
E_pitch = O(:,1);


#tax = bitshift(int16(taxh),8);
#tax = tax + taxl

f_cut = 5;
axh = A(:,1);
axl = A(:,2);
ax = bitshift(int16(axh),8);
ax = ax + axl;
axd = double(ax/16384); # scaling
 
ayh = A(:,3);
ayl = A(:,4);
ay = bitshift(int16(ayh),8);
ay = ay + ayl;
ayd = double(ay/16384); #scaling

azh = A(:,5);
azl = A(:,6);
az = bitshift(int16(azh),8);
az = az+azl;
azd = double(az/16384); #scaling


#low pass filter for accelerometer
dT = 0.01;  #time in seconds ( readings are taken at an interval of 10ms)
Tau= 1/(2*f_cut*pi);
alpha = Tau/(Tau+dT);

yaz = zeros(size(azd,1),1); #creating a matrix of zeros of size=rows of azd X 1
yaz(1,1) = (1-alpha)*azd(1,1);
for i = 2:8000
  prev = yaz(i-1,:);
  xn = azd(i,:);
  yaz(i,:) = xn + alpha*(prev - xn);
#yax(i) = ax(i) + alpha*(yax(i-1) - ax(i));
endfor

yay = zeros(size(ayd,1),1); #creating a matrix of zeros of size=rows of azd X 1
yay(1,1) = (1-alpha)*ayd(1,1);
for i = 2:8000
  prev = yay(i-1,:);
  xn = ayd(i,:);
  yay(i,:) = xn + alpha*(prev - xn);
#yax(i) = ax(i) + alpha*(yax(i-1) - ax(i));
endfor

yax = zeros(size(axd,1),1); #creating a matrix of zeros of size=rows of azd X 1
yax(1,1) = (1-alpha)*axd(1,1);
for i = 2:8000
  prev = yax(i-1,:);
  xn = axd(i,:);
  yax(i,:) = xn + alpha*(prev - xn);
#yax(i) = ax(i) + alpha*(yax(i-1) - ax(i));
endfor

#gyroscope readings

gxh = A(:,7);
gxl = A(:,8);
gx = bitshift(int16(gxh),8);
gx = gx + gxl;
gxd = double(gx/131); # scaling exact value 131.072

gyh = A(:,9);
gyl = A(:,10);
gy = bitshift(int16(gyh),8);
gy = gy + gyl;
gyd = double(gy/131); # scaling exact value 131.072

gzh = A(:,11);
gzl = A(:,12);
gz = bitshift(int16(gzh),8);
gz = gz + gzl;
gzd = double(gz/131); # scaling exact value 131.072

#high pass filter for gyroscope

ygx = zeros(size(gxd,1),1); #creating a matrix of zeros of size=rows of azd X 1
ygx(1,1) = (1-alpha)*gxd(1,1);
for i = 2:8000
  prevy = ygx(i-1,:);
  prevx = gxd(i-1,:);
  xn = gxd(i,:);
  ygx(i,:) = (1-alpha)*(prevy + xn - prevx);
endfor

ygy = zeros(size(gyd,1),1); #creating a matrix of zeros of size=rows of azd X 1
ygy(1,1) = (1-alpha)*gyd(1,1);
for i = 2:8000
  prevy = ygy(i-1,:);
  prevx = gyd(i-1,:);
  xn = gyd(i,:);
  ygy(i,:) = (1-alpha)*(prevy + xn - prevx);
endfor

ygz = zeros(size(gzd,1),1); #creating a matrix of zeros of size=rows of azd X 1
ygz(1,1) = (1-alpha)*gzd(1,1);
for i = 2:8000
  prevy = ygz(i-1,:);
  prevx = gzd(i-1,:);
  xn = gzd(i,:);
  ygz(i,:) = (1-alpha)*(prevy + xn - prevx);
endfor

# yax yay yaz and ygx ygy ygz are filtered values 

#calculating roll and pitch angles with accelerometer
for i = 1:8000
   anglePitchAcc(i,:) = atan2(yax(i,:), sqrt((yay(i,:) * yay(i,:)) + (yaz(i,:) * yaz(i,:))));
   #anglePitchAcc = (atan2(yax, sqrt(yay.^2 + yaz.^2))*180.0)/pi;
   #angleRollAcc(i,:) = atan2(-yay(i,:), sqrt((yax(i,:) * yax(i,:)) + (yaz(i,:) * yaz(i,:)))) * 180 / pi;
endfor

#complementary filter
alpha2 = 0.03; 
#anglePitch(1,:) = alpha2* (ygx(1,:)*dT) + (1 - alpha2) * (anglePitchAcc(1,:));
#angleRoll(1,:) = alpha2* (ygy(1,:)*dT) + (1 - alpha2) * (angleRollAcc(1,:));
for i = 2:8000
   #anglePitch(i,:) = ((alpha2*ygx(i,:)*dT)/(1-alpha2) + (1 - alpha2) * (anglePitchAcc(i,:)));
  anglePitch(i,:) = alpha2*(anglePitch(i-1,:) + ygx(i,:)*dT) + (1 - alpha2) * (anglePitchAcc(i,:));
  #angleRoll(i,:) = alpha2*(angleRoll(i-1,:) + ygy(i,:)*dT) + (1 - alpha2) * (angleRollAcc(i,:));
endfor

