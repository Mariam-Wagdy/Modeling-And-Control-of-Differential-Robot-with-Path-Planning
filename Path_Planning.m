clc;
clear;

x0=0;
y0=0;
theta0=5*pi/180;
r_w=0.2; %20 cm
L=1; %30 cm

xd=6; %in meter
yd=7;
thetad=45*pi/180;

xe0=xd-x0;
ye0=yd-y0;

rho0= (xe0^2+ye0^2)^(1/2);
alpha0=atan2(ye0,xe0);
beta0=-theta0-alpha0;

ini=[rho0; alpha0; beta0];


%% setting map
%%map from 0 to 20 in x and y
I = imread('map5.png');
BW = im2bw(I,0.99); %% convert to b&w
BW = imresize(BW,1/8); %%compress image

viz= Visualizer2D;
viz.mapName='map';

grid = binaryOccupancyMap(BW);

%figure;
%imshow(BW)
[columns, rows] = size(BW);

xt=(xd-x0)/rows;
yt=(yd-y0)/columns;

x= x0:xt:xd; % discritizing domain
y= y0:yt:yd;

map=zeros(length(y),length(x));


rep2=zeros(length(y),length(x));

n=1;

% Loop over all locations  and deine obstacles
for col = 1:columns
    for row = 1:rows
        % color(row,col)=impixel( BW , row , col);
        if norm(impixel(BW , row , col) - [0,0,0])<1.5
            map(columns-col,row)=1;
            %index(n,:)=[col, row];
            obsi(n,:)=[columns-col,row];
            n=n+1;
        end
    end
end

map_v=map;

%% establishing fields

Katt=10;
Krep=0.1;
rlim=L;

[Uatt,Urep]=APF(columns, rows, x, y, Katt, Krep, obsi,rlim,1);
Nmap=Uatt+Urep;

% figure;
% surf(x,y,Urep)
% title("Repulsive Field Map");
% xlabel('x (m)');
% ylabel('y (m)');
% zlabel('Repulsive Force');
% 
% figure;
% surf(x,y,Nmap)
% title("Augmented Field Map");
% xlabel('x (m)');
% ylabel('y (m)');
% zlabel('Attraction Force');


[gx,gy] = gradient(Nmap);
figure;
surf(Nmap)
figure
quiver(-gx,-gy)
%surf(Nmap)
%% path
y1=y0;
x1=x0;
path(1,:)=[y1,x1]; %[3.7,1.6]; %(columns+2-col,row) (y,x)

i=1;
%% plot(path(:,2),path(:,1),'-o')
while i>0
    %% get pathhhhh
    
    
    py=floor(abs(path(i,1)/yt))+1;
    px=floor(abs(path(i,2)/xt))+1;
    
    G= [gy(py,px),gx(py,px)]; 
    G= G/norm(G); %%normalise the gradient
    
    a=xt*5; % speed, step length
    k=1;
    c=0;
    
    while k>0
        c=c+1;
        pos_new=path(i,:)-a*G;
        
        %% dont fall for gradients
        if abs(pos_new(1)-yd)<0.2 %any(pos_new<0)
            pos_new(1)=yd;
            
        elseif abs(pos_new(1)-yd)>0.2
            a=a*0.9;
        end
        
        if abs(pos_new(2)-xd)<0.2
            pos_new(2)=xd;
            
        elseif abs(pos_new(2)-xd)>0.2
            a=a*0.9;
        end
        
        %% check positivty
        if pos_new(1)<0
            a=a*0.9;
            continue;
        end
        
        if pos_new(2)<0
            a=a*0.9;
            continue;
        end
        
        pos_new_i=[floor(abs(pos_new(1)/yt))+1; floor(abs(pos_new(2)/xt))+1];
        
        %% check remaining in domain
        if pos_new_i(1)<=columns+1 && pos_new_i(2) <=rows+1
            k=-1;
        end
    end
    
    path(i+1,:)=pos_new;
    rd=((path(i+1,1)-yd)^2+(path(i+1,2)-xd)^2)^0.5; %% don't get stuck near goal
    
    %% dont get stuck 
    if i>10 && abs(norm(path(i+1,:)-path(i+1-10,:))/norm(path(i+1-10)))<0.1 && rd>1
        u=1;
        for col=-u:1:u %columns
            for row=-u:1:u %rows
                obsi(length(obsi)+1,:)=[col+pos_new_i(1),row+pos_new_i(2)];
                map_v(col+pos_new_i(1),row+ pos_new_i(2))=1;
            end
        end

        %% repeat defining Urep and Nmap
        i=0;
        [Uatt, Urep]=APF(columns, rows, x, y, Katt, Krep, obsi,rlim,i);
        Nmap=Uatt+Urep;
        [gx,gy] = gradient(Nmap);  
        

         surf(x,y,Nmap)
        
    end
        
    if rd<0.1
        i=-100;
    end
    
    i=i+1;
    
end

%% Format Data to Simulink
for i =1: length(path)-1
    y1= path(i,1);
    x1= path(i,2);
    y2= path(i+1,1);
    x2= path(i+1,2);
    path(i,3)=atan2((y2-y1),(x2-x1));
end

%path(1,3)=theta0;

Ts=0.001;
path(length(path),3)=thetad;

time=(0:length(path)-1)/10;
xfun=[time' path(:,2)];
yfun=[time' path(:,1)];
thetafun=[time' path(:,3) ];

%% display REsults
if xt<0 
    x=flip(x);
end
if yt<0 
   y=flip(y);
end

figure;
hold on
quiver(x,y,-gx,-gy)
plot(path(:,2),path(:,1),'->')
title("Vector Field");
xlabel('x (m)');
ylabel('y (m)');
hold off

figure;
surf(x,y,map);
title("Map with Obstacles");
xlabel('x (m)');
ylabel('y (m)');
zlabel('Attraction Force');

figure;
surf(x,y,Uatt)
title("Attractive Field Map");
xlabel('x (m)');
ylabel('y (m)');
zlabel('Attraction Force');

figure;
surf(x,y,Urep)
title("Repulsive Field Map");
xlabel('x (m)');
ylabel('y (m)');
zlabel('Repulsive Force');

figure;
surf(x,y,Nmap)
title("Augmented Field Map with Virtual Obstacles");
xlabel('x (m)');
ylabel('y (m)');
zlabel('Attraction Force');


