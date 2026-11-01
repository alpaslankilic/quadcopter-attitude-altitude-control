function [p,timing] = plan_experiment(p)
%PLAN_EXPERIMENT Finish takeoff, finish the move, then apply and recover from a gust.
[t,y]=simulate_case(p,p.sizingDuration);
[first,~]=settled_at(t,y,p.takeoffTime+p.takeoffDuration,[0;0;p.goal(3)],0,[],p);
p.moveTime=first+p.sampleTime;
[t,y]=simulate_case(p,p.moveTime+p.sizingDuration);
[second,tolerance]=settled_at(t,y,p.moveTime+p.moveDuration,p.goal,p.yawGoal,[],p);
p.gustTime=second+p.sampleTime;
[t,y]=simulate_case(p,p.gustTime+p.gustDuration+p.sizingDuration);
[last,~]=settled_at(t,y,p.gustTime+p.gustDuration,p.goal,p.yawGoal,tolerance,p);
recovery=last-p.gustTime-p.gustDuration;
p.stopTime=ceil((last+.1*recovery)/p.sampleTime)*p.sampleTime;
timing=struct('takeoff_settled_s',first,'move_start_s',p.moveTime,'move_settled_s',second, ...
    'gust_start_s',p.gustTime,'gust_end_s',p.gustTime+p.gustDuration, ...
    'final_settled_s',last,'stop_time_s',p.stopTime,'state_tolerance',tolerance,'tail_fraction',.1);
timing.criteria='All 16 physical states; 2% maneuver excursions with declared floors for initially unexcited axes';
end

function [time,tolerance]=settled_at(t,y,start,position,yaw,tolerance,p)
target=[position;zeros(3,1);0;0;yaw;zeros(3,1);p.hoverSpeed*ones(4,1)].';
deviation=y(:,1:16)-target;
if isempty(tolerance)
    floor=[.02*p.goal.',.005*ones(1,3),.002*ones(1,3),.005*ones(1,3),2*ones(1,4)];
    tolerance=max(.02*max(abs(deviation),[],1),floor);
end
selected=find(t>=start);last=find(any(abs(deviation(selected,:))>tolerance,2),1,'last');
if isempty(last),time=start;return;end
assert(last<numel(selected)-10,'Quadcopter sizing interval did not capture settling.');time=t(selected(last+1));
end
