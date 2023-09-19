clc;
clear;

x0=0;
y0=0;
theta0=5*pi/180;
r_w=0.2; %20 cm
L=1; %1 m

xd=20; %in meter
yd=15;
thetad=45*pi/180;

xe0=xd-x0;
ye0=yd-y0;

rho0= (xe0^2+ye0^2)^(1/2);
alpha0=atan2(ye0,xe0);
beta0=-theta0-alpha0;

ini=[rho0; alpha0; beta0];