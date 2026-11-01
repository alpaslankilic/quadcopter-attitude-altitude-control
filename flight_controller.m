function [rotor,dz,attitude,thrust] = flight_controller(x,z,input,p)
%FLIGHT_CONTROLLER Position PD, altitude PID, angle P, rate PI and rotor allocation.
error=input(1:3)-x(1:3);velocityError=input(4:6)-x(4:6);
acceleration=input(7:9)+p.positionKp.*error+p.positionKd.*velocityError;
yaw=input(10);
attitude=[(acceleration(1)*sin(yaw)-acceleration(2)*cos(yaw))/p.gravity; ...
    (acceleration(1)*cos(yaw)+acceleration(2)*sin(yaw))/p.gravity;yaw];
attitude(1:2)=max(-p.tiltLimit,min(p.tiltLimit,attitude(1:2)));
angleError=attitude-x(7:9);angleError(3)=atan2(sin(angleError(3)),cos(angleError(3)));
rateRequest=max(-3,min(3,p.angleGain*angleError));rateError=rateRequest-x(10:12);
rawTorque=p.inertia.*p.rateKp.*rateError+z(2:4)+cross(x(10:12),p.inertia.*x(10:12));
tilt=max(.5,cos(x(7))*cos(x(8)));
rawThrust=(p.mass*(p.gravity+acceleration(3))+z(1))/tilt;
request=[max(0,rawThrust);rawTorque];squared=p.mixer\request;
rotor=sqrt(max(0,min(p.maximumRotorSpeed^2,squared)));
allocated=p.mixer*(rotor.^2);thrust=allocated(1);
dz=[p.mass*p.altitudeKi*error(3)+p.altitudeTracking*(thrust-rawThrust)*tilt; ...
    p.inertia.*p.rateKi.*rateError+p.rateTracking*(allocated(2:4)-rawTorque)];
end
