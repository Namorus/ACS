function [ AirProperty ] = get_AirProperty( kMeterAboveSeaLevel )
%Function computes air properties depending of meters above sea level using
%standard atmosphere model
%   Detailed explanation goes here


%Compute properties of Air depending on height over sea
kTZero=288.15; %Kelvin
kTempLapseRate=0.0065; %K/m
AirProperty.temperature=kTZero-kTempLapseRate*kMeterAboveSeaLevel; %[K] temperature according to international standard atmosphere, see http://en.wikipedia.org/wiki/Density_of_air

kPZero=101325; %Pa
kGravity=9.81;
kMolarMassAir=0.0289644;
kIdealGasConstant=8.31447;
AirProperty.pressure=kPZero*(1-kTempLapseRate*kMeterAboveSeaLevel/kTZero)^(kGravity*kMolarMassAir/(kIdealGasConstant*kTempLapseRate)); %[Pa] pressure according to int. stand. armosp.
AirProperty.rho=AirProperty.pressure*kMolarMassAir/(kIdealGasConstant*AirProperty.temperature); %[kg/m^3] density according to int. stand. armosp.

kCOne=1.458e-6; %Sutherlands coefficient C1
kS=110.4; %Sutherlands coefficent S
kSutherlandExponent=3/2;
AirProperty.mu=kCOne*AirProperty.temperature^kSutherlandExponent/(AirProperty.temperature+kS); %[kg/(m*s)] dynamic viscosity according to Sutherland's law, see http://www.cfd-online.com/Wiki/Sutherland's_law

kGammaAir=1.4;
AirProperty.speed_of_sound=sqrt(kGammaAir*AirProperty.pressure/AirProperty.rho); %[m/s] speed of sound according to Fluid II skript, p.93

end

