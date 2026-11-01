function report = run_demo(showFigures)
%RUN_DEMO Verify nonlinear flight, actuator bounds, final recovery and body energy.
if nargin==0,showFigures=true;end
root=fileparts(mfilename('fullpath'));previous=pwd;cleanup=onCleanup(@() cd(previous));cd(root);
clear sl_controller sl_plant
p=project_parameters();[p,timing]=plan_experiment(p);[t,state,control,error]=run_simulation(p);
input=zeros(numel(t),16);for k=1:numel(t),input(k,:)=experiment_input(t(k),p).';end
assert(rank(p.mixer)==4,'Rotor mixer is singular.');
assert(min(state(:,13:16),[],'all')>=0 && max(state(:,13:16),[],'all')<=p.maximumRotorSpeed+1e-6,'Rotor limit failed.');
assert(max(abs(state(:,7:8)),[],'all')<pi/4,'Flight exceeded the modeled attitude envelope.');
target=[p.goal.',zeros(1,3),0,0,p.yawGoal,zeros(1,3),p.hoverSpeed*ones(1,4)];
assert(all(abs(state(end,:)-target)<timing.state_tolerance),'Gust recovery was truncated.');
power=zeros(numel(t),1);
for k=1:numel(t)
    phi=state(k,7);theta=state(k,8);psi=state(k,9);
    direction=[cos(psi)*sin(theta)*cos(phi)+sin(psi)*sin(phi); ...
        sin(psi)*sin(theta)*cos(phi)-cos(psi)*sin(phi);cos(theta)*cos(phi)];
    assert(abs(norm(direction)-1)<1e-12,'Rotation direction is not unit length.');
    wrench=p.mixer*(state(k,13:16).'.^2);velocity=state(k,4:6).';rate=state(k,10:12).';
    power(k)=(direction*wrench(1)+input(k,11:13).')'*velocity+ ...
        (wrench(2:4)+input(k,14:16).')'*rate-p.linearDrag*(velocity.'*velocity);
end
energy=.5*p.mass*sum(state(:,4:6).^2,2)+.5*sum(state(:,10:12).^2.*p.inertia.',2)+p.mass*p.gravity*state(:,3);
residual=trapz(t,power)-(energy(end)-energy(1));
assert(abs(residual)<.005*max(1,trapz(t,abs(power))),'Rigid-body energy balance failed.');
report=struct('verification','PASS','scaled_ode_max_error',error,'rigid_body_energy_residual_J',residual, ...
    'final_position_error_m',norm(state(end,1:3)-p.goal.'),'peak_tilt_deg',rad2deg(max(abs(state(:,7:8)),[],'all')), ...
    'stop_time_s',p.stopTime);
names=[{'time_s'},compose('state_%02d',1:16),compose('input_%02d',1:16),compose('control_%02d',1:12)];
writetable(array2table([t,state,input,control],'VariableNames',names),'results/response.csv');
write_json('results/parameters.json',p);write_json('results/timing.json',timing);write_json('results/metrics.json',report);
plot_results(t,state,control,input,p,showFigures);fprintf('QUADCOPTER verification PASS; scaled error %.3g.\n',error);
end
