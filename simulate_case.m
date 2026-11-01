function [t,y,control] = simulate_case(p,stopTime)
%SIMULATE_CASE Piecewise ODE reference with exact trajectory and gust boundaries.
t=(0:p.sampleTime:stopTime).';y=zeros(numel(t),20);state=[p.initialState;p.initialControlState];
knots=unique([0,p.takeoffTime,p.takeoffTime+p.takeoffDuration,p.moveTime, ...
    p.moveTime+p.moveDuration,p.gustTime,p.gustTime+p.gustDuration,stopTime]);
knots=knots(isfinite(knots)&knots<=stopTime);options=odeset('RelTol',1e-8,'AbsTol',1e-10,'MaxStep',p.sampleTime);
for k=1:numel(knots)-1
    left=knots(k);right=knots(k+1);
    solution=ode45(@(time,x) closed_loop(min(right-1e-10,max(left,time)),x,p),[left,right],state,options);
    selected=t>=left & t<=right;y(selected,:)=deval(solution,t(selected)).';state=solution.y(:,end);
end
control=zeros(numel(t),12);
for k=1:numel(t)
    [rotor,dz,attitude,thrust]=flight_controller(y(k,1:16).',y(k,17:20).',experiment_input(t(k),p),p);
    control(k,:)=[rotor;dz;attitude;thrust].';
end
end

function dy=closed_loop(t,y,p)
input=experiment_input(t,p);[rotor,dz]=flight_controller(y(1:16),y(17:20),input(1:10),p);
dy=[flight_rhs(y(1:16),rotor,input(11:16),p);dz];
end
