function [ inertia center_of_gravity ] = get_inertia( Geometry, Mass )
%function accepts structs for airplane Geometry and Mass. It returns
%inertias and center of gravity coordinates. The wing is approximated as a
%flat plate

%   Members of Geometry and Mass are best described in the function
%   getGeometry.m

% Convention for coordinate system: 
% The coordinate system is right handed
% x-axis is in wing plane and points in flight direction
% y-axis is colinear with leading edge
% z-axis comes out screen
% origin is on leading edge in the middle of the wing           
%                           
%
%                wing seen from bottom
%
%                          x/\    
%      Mot1    Mot2         ¦          Mot3     Mot4
%   ----leading edge--  y<--*z   --------------------
%   |       |       |       |       |       |       |
%   |   1   |   2   |   3   |  (4)  |  (5)  |  (6)  |
%   |       |       |       |       |       |       |
%   ---Flap----Flap--------------------Flap----Flap--


% compute center of gravity
total_mass=sum(Mass.mass_vector);
center_of_gravity(1,1)=sum(Mass.xyz(1,:).*Mass.mass_vector)/total_mass;
center_of_gravity(2,1)=sum(Mass.xyz(2,:).*Mass.mass_vector)/total_mass;
center_of_gravity(3,1)=sum(Mass.xyz(3,:).*Mass.mass_vector)/total_mass;

%compute intertia of flat plate
kCoefficientPlate=1/12;
inertia_plate=[kCoefficientPlate*Mass.wing*Geometry.span^2 0 0;
				0 kCoefficientPlate*Mass.wing*Geometry.chord^2 0;
				0 0 kCoefficientPlate*Mass.wing*(Geometry.span^2+Geometry.chord^2)];

inertia_plate=inertia_plate+Mass.wing*center_of_gravity*center_of_gravity';

%Transform all coordinates into new coordinate system with origin at center of gravity
Mass.xyz(1,:)=Mass.xyz(1,:)-center_of_gravity(1,1);
Mass.xyz(2,:)=Mass.xyz(2,:)-center_of_gravity(2,1);
Mass.xyz(3,:)=Mass.xyz(3,:)-center_of_gravity(3,1);

%Compute inertias of points. Integrals over mass elements become simple multiplications because of point masses
inertia_point=zeros(3,3);

for i=2:length(Mass.mass_vector) %start at i=2 because i=1 is the mass associated with the wing which is already modelled as plate
    inertia_point=inertia_point+Mass.mass_vector(i)*[Mass.xyz(2,i)^2+Mass.xyz(3,i)^2 , -Mass.xyz(1,i)*Mass.xyz(2,i), -Mass.xyz(1,i)*Mass.xyz(3,i);...
        -Mass.xyz(2,i)*Mass.xyz(1,i),Mass.xyz(1,i)^2+Mass.xyz(3,i)^2, -Mass.xyz(2,i)*Mass.xyz(3,i);...
        -Mass.xyz(3,i)*Mass.xyz(1,i), -Mass.xyz(3,i)*Mass.xyz(2,i), Mass.xyz(1,i)^2+Mass.xyz(2,i)^2];   
end

inertia=inertia_plate+inertia_point;

end