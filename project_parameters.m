function p = project_parameters()
%PROJECT_PARAMETERS X quadrotor; world and body z axes point upward in hover.
p.mass=1.2;p.gravity=9.81;p.inertia=[.012;.014;.023];p.arm=.23;
p.thrustCoefficient=1.1e-5;p.dragCoefficient=1.8e-7;p.rotorTimeConstant=.025;
p.maximumRotorSpeed=900;p.linearDrag=.15;
d=p.arm/sqrt(2);position=[d d;-d d;-d -d;d -d];spin=[1 -1 1 -1];
p.mixer=[p.thrustCoefficient*ones(1,4); ...
    p.thrustCoefficient*position(:,2).';-p.thrustCoefficient*position(:,1).';p.dragCoefficient*spin];
p.hoverSpeed=sqrt(p.mass*p.gravity/(4*p.thrustCoefficient));
p.positionKp=[6.25;6.25;12];p.positionKd=[5;5;6];p.altitudeKi=8;
p.angleGain=8;p.rateKp=30;p.rateKi=120;p.rateTracking=15;p.altitudeTracking=4;
p.goal=[1;.5;1.5];p.yawGoal=deg2rad(20);p.tiltLimit=.35;
p.takeoffTime=.1;p.takeoffDuration=1;p.moveTime=Inf;p.moveDuration=1.2;
p.gustTime=Inf;p.gustDuration=.2;p.gustForce=[.8;-.5;.4];p.gustMoment=[.02;-.01;.005];
p.sampleTime=.002;p.sizingDuration=12;p.stopTime=12;
p.initialState=[zeros(12,1);p.hoverSpeed*ones(4,1)];p.initialControlState=zeros(4,1);
p.modelName='quadcopter_control';
end
