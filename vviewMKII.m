clear all
clc

%%
%initialize geometry
[Geometry, Mass]=get_Geometry('span_160', '6s_2500mAh'); %change also in function aerodynamics_hover and nominal_thrust

%supported airplane types:
%   'span_160'
%   'span_195_payload_bay'
%   'span_240'

%supported battery types:
%   '6s_2500mAh'
%   '6s_3700mAh'

%inertias and center of gravity coordinates.
[ inertia, center_of_gravity ] = get_inertia( Geometry, Mass );

Total_Mass=sum(Mass.mass_vector);       %total mass of airplane
kGravity=9.8067;        %[m/s^2] http://en.wikipedia.org/wiki/Gravity_of_Earth

%Function computes air properties depending of meters above sea level
kMeterAboveSeaLevel=2000; %[m]
[ AirProperty ] = get_AirProperty(kMeterAboveSeaLevel);

%Best fit through origin
load('fit_origin.mat')

kVelocity_hover_persegment=13*[1,1,1,1];    % speed of air per wingsegment (constant)

Geometry_arm_temp=[Geometry.large_arm Geometry.small_arm -Geometry.small_arm -Geometry.large_arm];
aerodynamics=Geometry.span/8*AirProperty.rho*kVelocity_hover_persegment.^2*Geometry.chord;
%%
%Calculation with B Matrix
B_raw=[ zeros(1,8); ...                                 %calculates Toqrues
    0, 0, 0, 0, Geometry_arm_temp;...
    zeros(1,8);...
    (aerodynamics*Geometry.chord*fit_origin_cm+aerodynamics*(1/4*Geometry.chord-center_of_gravity(1,1))*fit_origin_cl).*ones(1,4), 0, 0, 0, 0;...
    zeros(1,8); ...
    aerodynamics*fit_origin_cl.*Geometry_arm_temp, 0, 0, 0, 0];
temp=[1;inertia(1,1);1;inertia(2,2);1;inertia(3,3)];
B=B_raw./repmat(temp,1,8);                  %calculates rotational acceleration


%Calculation of A Matrix
A=[0 1 0 0 0 0;...
    0 0 0 0 0 0;...
    0 0 0 1 0 0;...
    0 0 0 0 0 0;...
    0 0 0 0 0 1;
    0 0 0 0 0 0];

C=eye(6);
D=zeros(6,8); 

%%
%---- L-M-N-plane -----
%{
B_temp=[B_raw(2,:);B_raw(4,:);B_raw(6,:)]*diag([1 1 1 1 1 1 1 1]); %change diag(...) if failure --> set 0 instead of 1  -->mind motor&flap-failure correlation
B=B_temp;
plim=[-30*pi/180 30*pi/180;     % [rad]
    -30*pi/180 30*pi/180;       % [rad]
    -30*pi/180 30*pi/180;       % [rad]
    -30*pi/180 30*pi/180;       % [rad]
    0 34.323;                   % [N]
    0 34.323;                   % [N]
    0 34.323;                   % [N]
    0 34.323];                  % [N]

disp('---------------------------------------------------------------')
  disp(' ');
  disp(' Control effectiveness matrix: B ='),disp(' '),disp(B)
  disp(' Position limits: [umin umax]'' ='),disp(' '),disp(plim')
  disp(' Blue (outer) set: { v : v = B*u, umin < u < umax }')
  disp('  Feasible virtual control set with constrained allocation')
  disp(' ')
  disp('  L:roll M:pitch N:yaw')
  disp(' ')
  disp('---------------------------------------------------------------')

radius= vview(B,plim);
%}

%---- L-M-plane -----
%{

B_temp=[B_raw(2,:);B_raw(4,:)]*diag([1 1 1 1 1 1 1 1]); %change diag(...) if failure --> set 0 instead of 1  -->mind motor&flap-failure correlation
B=B_temp;
plim=[-30*pi/180 30*pi/180;     % [rad]
    -30*pi/180 30*pi/180;       % [rad]
    -30*pi/180 30*pi/180;       % [rad]
    -30*pi/180 30*pi/180;       % [rad]
    0 34.323;                   % [N]
    0 34.323;                   % [N]
    0 34.323;                   % [N]
    0 34.323];                  % [N]

disp('---------------------------------------------------------------')
  disp(' ');
  disp(' Control effectiveness matrix: B ='),disp(' '),disp(B)
  disp(' Position limits: [umin umax]'' ='),disp(' '),disp(plim')
  disp(' Blue (outer) set: { v : v = B*u, umin < u < umax }')
  disp('  Feasible virtual control set with constrained allocation')
  disp(' ')
  disp('  L:roll M:pitch N:yaw')
  disp(' ')
  disp('---------------------------------------------------------------')

%{.
%nominal plot
B_temp=[B_raw(2,:);B_raw(4,:)]*diag([1 1 1 1 1 1 1 1]); %change diag(...) if failure --> set 0 instead of 1  -->mind motor&flap-failure correlation
B=B_temp;
[x_origin_circle,y_origin_circle,radius]=vview(B,plim,'LM','b');
radius % [Nm]
 
%motor2 failure
B_temp=[B_raw(2,:);B_raw(4,:)]*diag([1 1 1 1 1 0 1 1]); %change diag(...) if failure --> set 0 instead of 1  -->mind motor&flap-failure correlation
B=B_temp;
[x_origin_circle,y_origin_circle,radius]=vview(B,plim,'LM','g');
radius % [Nm]

%motor1 failure
B_temp=[B_raw(2,:);B_raw(4,:)]*diag([1 1 1 1 0 1 1 1]); %change diag(...) if failure --> set 0 instead of 1  -->mind motor&flap-failure correlation
B=B_temp;
[x_origin_circle,y_origin_circle,radius]=vview(B,plim,'LM','r');
radius % [Nm]
%}


%{
%nominal plot
B_temp=[B_raw(2,:);B_raw(4,:)]*diag([1 1 1 1 1 1 1 1]); %change diag(...) if failure --> set 0 instead of 1  -->mind motor&flap-failure correlation
B=B_temp;
[x_origin_circle,y_origin_circle,radius]=vview(B,plim,'LM','b');
radius % [Nm]
 
%flap2 failure
B_temp=[B_raw(2,:);B_raw(4,:)]*diag([1 0 1 1 1 1 1 1]); %change diag(...) if failure --> set 0 instead of 1  -->mind motor&flap-failure correlation
B=B_temp;
[x_origin_circle,y_origin_circle,radius]=vview(B,plim,'LM','r');
radius % [Nm]
%}

%{
%nominal plot
B_temp=[B_raw(2,:);B_raw(4,:)]*diag([1 1 1 1 1 1 1 1]); %change diag(...) if failure --> set 0 instead of 1  -->mind motor&flap-failure correlation
B=B_temp;
[x_origin_circle,y_origin_circle,radius]=vview(B,plim,'LM','b');
radius % [Nm]
 
%flap1 failure
B_temp=[B_raw(2,:);B_raw(4,:)]*diag([0 1 1 1 1 1 1 1]); %change diag(...) if failure --> set 0 instead of 1  -->mind motor&flap-failure correlation
B=B_temp;
[x_origin_circle,y_origin_circle,radius]=vview(B,plim,'LM','g');
radius % [Nm]
%}

%}

%---- L-N-plane -----
%{
B_temp=[B_raw(2,:);B_raw(6,:)]*diag([1 1 1 1 1 1 1 1]); %change diag(...) if failure --> set 0 instead of 1 -->mind motor&flap-failure correlation
B=B_temp;
plim=[-30*pi/180 30*pi/180;     % [rad]
    -30*pi/180 30*pi/180;       % [rad]
    -30*pi/180 30*pi/180;       % [rad]
    -30*pi/180 30*pi/180;       % [rad]
    0 34.323;                   % [N]
    0 34.323;                   % [N]
    0 34.323;                   % [N]
    0 34.323];                  % [N]

disp('---------------------------------------------------------------')
  disp(' ');
  disp(' Control effectiveness matrix: B ='),disp(' '),disp(B)
  disp(' Position limits: [umin umax]'' ='),disp(' '),disp(plim')
  disp(' Blue (outer) set: { v : v = B*u, umin < u < umax }')
  disp('  Feasible virtual control set with constrained allocation')
  disp(' ')
  disp('  L:roll M:pitch N:yaw')
  disp(' ')
  disp('---------------------------------------------------------------')
  
%{
%nominal plot
B_temp=[B_raw(2,:);B_raw(6,:)]*diag([1 1 1 1 1 1 1 1]); %change diag(...) if failure --> set 0 instead of 1 -->mind motor&flap-failure correlation
B=B_temp;
[x_origin_circle,y_origin_circle,radius]=vview(B,plim,'LN','b');
radius % [Nm]

%flap2 failure
B_temp=[B_raw(2,:);B_raw(6,:)]*diag([1 0 1 1 1 1 1 1]); %change diag(...) if failure --> set 0 instead of 1 -->mind motor&flap-failure correlation
B=B_temp;
[x_origin_circle,y_origin_circle,radius]=vview(B,plim,'LN','r');
radius % [Nm]

%flap1 failure
B_temp=[B_raw(2,:);B_raw(6,:)]*diag([0 1 1 1 1 1 1 1]); %change diag(...) if failure --> set 0 instead of 1 -->mind motor&flap-failure correlation
B=B_temp;
[x_origin_circle,y_origin_circle,radius]=vview(B,plim,'LN','g');
radius % [Nm]
%}

%{
%nominal plot
B_temp=[B_raw(2,:);B_raw(6,:)]*diag([1 1 1 1 1 1 1 1]); %change diag(...) if failure --> set 0 instead of 1 -->mind motor&flap-failure correlation
B=B_temp;
[x_origin_circle,y_origin_circle,radius]=vview(B,plim,'LN','b');
radius % [Nm]

%motor2 failure
B_temp=[B_raw(2,:);B_raw(6,:)]*diag([1 1 1 1 1 0 1 1]); %change diag(...) if failure --> set 0 instead of 1 -->mind motor&flap-failure correlation
B=B_temp;
[x_origin_circle,y_origin_circle,radius]=vview(B,plim,'LN','g');
radius % [Nm]

%motor1 failure
B_temp=[B_raw(2,:);B_raw(6,:)]*diag([1 1 1 1 0 1 1 1]); %change diag(...) if failure --> set 0 instead of 1 -->mind motor&flap-failure correlation
B=B_temp;
[x_origin_circle,y_origin_circle,radius]=vview(B,plim,'LN','r');
radius % [Nm]
%}
%}

%---- M-N-plane -----
%{.

B_temp=[B_raw(4,:);B_raw(6,:)]*diag([1 1 1 1 1 1 1 1]);    %change diag(...) if failure --> set 0 instead of 1 -->mind motor&flap-failure correlation
B=B_temp;
plim=[-30 30;     % [rad]
    -30 30;       % [rad]
    -30 30;       % [rad]
    -30 30;       % [rad]
    0 34.323;                   % [N]
    0 34.323;                   % [N]
    0 34.323;                   % [N]
    0 34.323];                  % [N]

  disp('---------------------------------------------------------------')
  disp(' ');
  disp(' Control effectiveness matrix: B ='),disp(' '),disp(B)
  disp(' Position limits: [umin umax]'' ='),disp(' '),disp(plim')
  disp(' Blue (outer) set: { v : v = B*u, umin < u < umax }')
  disp('  Feasible virtual control set with constrained allocation')
  disp(' ')
  disp('  L:roll M:pitch N:yaw')
  disp(' ')
  disp('---------------------------------------------------------------')

%nominal plot  
[x_origin_circle,y_origin_circle,radius]=vview(B,plim,'MN','b');
radius % [Nm]

%flap2 failure
B_temp=[B_raw(4,:);B_raw(6,:)]*diag([1 0 1 1 1 1 1 1]);    %change diag(...) if failure --> set 0 instead of 1 -->mind motor&flap-failure correlation
B=B_temp;
[x_origin_circle,y_origin_circle,radius]=vview(B,plim,'MN','m');
radius % [Nm]

%flap3 failure
B_temp=[B_raw(4,:);B_raw(6,:)]*diag([1 1 0 1 1 1 1 1]);    %change diag(...) if failure --> set 0 instead of 1 -->mind motor&flap-failure correlation
B=B_temp;
[x_origin_circle,y_origin_circle,radius]=vview(B,plim,'MN','r');
radius % [Nm]

%flap1 failure
B_temp=[B_raw(4,:);B_raw(6,:)]*diag([0 1 1 1 1 1 1 1]);    %change diag(...) if failure --> set 0 instead of 1 -->mind motor&flap-failure correlation
B=B_temp;
[x_origin_circle,y_origin_circle,radius]=vview(B,plim,'MN','y');
radius % [Nm]

%flap1 failure
B_temp=[B_raw(4,:);B_raw(6,:)]*diag([1 1 1 0 1 1 1 1]);    %change diag(...) if failure --> set 0 instead of 1 -->mind motor&flap-failure correlation
B=B_temp;
[x_origin_circle,y_origin_circle,radius]=vview(B,plim,'MN','g');
radius % [Nm]

%}


