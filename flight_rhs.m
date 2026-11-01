function dx = flight_rhs(x,rotor,gust,p)
%FLIGHT_RHS Six-degree-of-freedom rigid-body motion with four first-order motors.
phi=x(7);theta=x(8);psi=x(9);w=x(10:12);
direction=[cos(psi)*sin(theta)*cos(phi)+sin(psi)*sin(phi); ...
    sin(psi)*sin(theta)*cos(phi)-cos(psi)*sin(phi);cos(theta)*cos(phi)];
euler=[1,sin(phi)*tan(theta),cos(phi)*tan(theta);0,cos(phi),-sin(phi); ...
    0,sin(phi)/cos(theta),cos(phi)/cos(theta)];
wrench=p.mixer*(x(13:16).^2);
acceleration=direction*wrench(1)/p.mass-[0;0;p.gravity]-p.linearDrag*x(4:6)/p.mass+gust(1:3)/p.mass;
angular=(wrench(2:4)+gust(4:6)-cross(w,p.inertia.*w))./p.inertia;
dx=[x(4:6);acceleration;euler*w;angular;(rotor-x(13:16))/p.rotorTimeConstant];
end
