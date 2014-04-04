function [ Geometry, Mass ] = get_Geometry( airplane_type, battery_type )
%This function accepts a string of the current airplane types and returns a
%struct with airplane geometry and a struct with masses. 
%supported airplane types:
%   'span_160'
%   'span_195_payload_bay'
%   'span_240'

%supported battery types:
%   '6s_2500mAh'
%   '6s_3700mAh'

%Numbering convention of wing segments

%               wing seen from bottom
%
%      Mot1    Mot2                   Mot3     Mot4
%   -------------------leading edge------------------
%   |       |       |       |       |       |       |
%   |   1   |   2   |   3   |  (4)  |  (5)  |  (6)  |
%   |       |       |       |       |       |       |
%   ---Flap----Flap--------------------Flap----Flap--
%
% numbering for servos, batteries, motor_controllers follows numbering of motors

% Convention for vectors
% [wing Mot1 Mot2 Mot3 Mot4 Battery1 Battery2 Battery3 Battery4...
%   motor_controller1 motor_controller2 motor_controller3 motor_controller4...
%   servo1 servo2 servo3 servo4]

% Convention for coordinate system: 
% wing seen from bottom
% The coordinate system is right handed
% x-axis is in wing plane and points in flight direction
% y-axis is colinear with leading edge
% z-axis comes out screen
% origin is on leading edge in the middle of the wing           
%                
%
%               wing seen from bottom
%           
%                          x/\    
%      Mot1    Mot2         ??          Mot3     Mot4
%   ----leading edge--  y<--*z   --------------------
%   |       |       |       |       |       |       |
%   |   1   |   2   |   3   |  (4)  |  (5)  |  (6)  |
%   |       |       |       |       |       |       |
%   ---Flap----Flap--------------------Flap----Flap--

switch battery_type
	case '6s_2500mAh'
		Mass.battery=0.360;
		Mass.x_battery=-0.071*ones(1,4);
        Mass.z_battery=[-0.045 0.04 0.04 -0.045];
	case '6s_3700mAh'
		Mass.battery=0.530;
		Mass.x_battery=-0.071*ones(1,4);
        Mass.z_battery=[-0.04 0.035 0.035 -0.04];
	otherwise
		error('Battery type currently not supported. Add it in function getGeometry.')
end

switch airplane_type
    case 'span_160'
        %wing properties 
        Geometry.span=1.6;
        Geometry.chord=0.45;
        Geometry.wing_segment=[0.4 0.4 0 0 0.4 0.4];
        Geometry.large_arm=Geometry.wing_segment(1)/2+Geometry.wing_segment(2)+Geometry.wing_segment(3);
        Geometry.small_arm=Geometry.wing_segment(2)/2+Geometry.wing_segment(3);

        %Point masses
        kDensityPerWingSurface=1.2;  %wing mass is estimated to be 1.2kg per m^2 of wing surface
        Mass.wing=Geometry.chord*Geometry.span*kDensityPerWingSurface;
        Mass.motor=0.22; 
        Mass.servo=0.028; %kg
        Mass.motor_controller=0.05;
        Mass.mass_vector=[Mass.wing ones(1,4)*Mass.motor ones(1,4)*Mass.battery ones(1,4)*Mass.motor_controller,...
                    ones(1,4)*Mass.servo];

        %Coordinates of point masses
        Mass.x_wing=-Geometry.chord/2;
        Mass.x_motor=0.085*ones(1,4);
        Mass.x_motor_controller=0.03*ones(1,4);
        Mass.x_servo=-0.25*ones(1,4);

    case 'span_195_payload_bay'
        %wing properties
    	Geometry.span=1.95;
    	Geometry.chord=0.45;
    	Geometry.wing_segment=[0.39 0.39 0.187 0.187 0.39 0.39];
        Geometry.large_arm=Geometry.wing_segment(1)/2+Geometry.wing_segment(2)+Geometry.wing_segment(3);
        Geometry.small_arm=Geometry.wing_segment(2)/2+Geometry.wing_segment(3);

        %Point masses
        kDensityPerWingSurface=1.2;  %wing mass is estimated to be 1.2kg per m^2 of wing surface
    	Mass.wing=Geometry.chord*Geometry.span*kDensityPerWingSurface;
    	Mass.motor=0.22; 
    	Mass.servo=0.028; %kg
    	Mass.motor_controller=0.05;
        Mass.mass_vector=[Mass.wing ones(1,4)*Mass.motor ones(1,4)*Mass.battery ones(1,4)*Mass.motor_controller,...
                    ones(1,4)*Mass.servo];

        %Coordinates of point masses
    	Mass.x_wing=-Geometry.chord/2;
    	Mass.x_motor=0.085*ones(1,4);
    	Mass.x_motor_controller=0.03*ones(1,4);
    	Mass.x_servo=-0.25*ones(1,4);

    case 'span_240'
        %wing properties 
    	Geometry.span=2.4;
    	Geometry.chord=0.45;
    	Geometry.wing_segment=[0.6 0.6 0 0 0.6 0.6];
        Geometry.large_arm=Geometry.wing_segment(1)/2+Geometry.wing_segment(2)+Geometry.wing_segment(3);
        Geometry.small_arm=Geometry.wing_segment(2)/2+Geometry.wing_segment(3);

        %Point masses
        kDensityPerWingSurface=1.2;  %wing mass is estimated to be 1.2kg per m^2 of wing surface
    	Mass.wing=Geometry.chord*Geometry.span*kDensityPerWingSurface;
    	Mass.motor=0.15; 
    	Mass.servo=0.028; %kg
    	Mass.motor_controller=0.05;
        Mass.mass_vector=[Mass.wing ones(1,4)*Mass.motor ones(1,4)*Mass.battery ones(1,4)*Mass.motor_controller,...
                    ones(1,4)*Mass.servo];

        %Coordinates of point masses
    	Mass.x_wing=-Geometry.chord/2;
    	Mass.x_motor=0.08*ones(1,4);
    	Mass.x_motor_controller=0.03*ones(1,4);
    	Mass.x_servo=-0.25*ones(1,4);

    otherwise
        error('The selected airplane type is not yet supported. Add it in the function getGeometry')
end

%Create dependent coordinates from above geometry
Mass.y_wing=0;
Mass.y_motor=[-Geometry.large_arm, -Geometry.small_arm, Geometry.small_arm, Geometry.large_arm];
Mass.y_battery=Mass.y_motor;
Mass.y_motor_controller=Mass.y_motor;
Mass.y_servo=Mass.y_motor;

Mass.xyz=zeros(3,length(Mass.mass_vector));
Mass.xyz(1,:)=[Mass.x_wing Mass.x_motor Mass.x_battery Mass.x_motor_controller Mass.x_servo];
Mass.xyz(2,:)=[Mass.y_wing Mass.y_motor Mass.y_battery Mass.y_motor_controller Mass.y_servo];
Mass.xyz(3,6:9)=Mass.z_battery;

end

