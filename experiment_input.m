function value = experiment_input(t,p)
%EXPERIMENT_INPUT Smooth takeoff and translation, followed by a finite gust.
[sz,vz,az]=profile(t-p.takeoffTime,p.takeoffDuration);
[sh,vh,ah]=profile(t-p.moveTime,p.moveDuration);
position=p.goal.*[sh;sh;sz];velocity=p.goal.*[vh;vh;vz];acceleration=p.goal.*[ah;ah;az];
active=t>=p.gustTime && t<p.gustTime+p.gustDuration;
value=[position;velocity;acceleration;p.yawGoal*sh;p.gustForce*active;p.gustMoment*active];
end

function [position,velocity,acceleration]=profile(t,duration)
s=min(1,max(0,t/duration));
position=10*s^3-15*s^4+6*s^5;
velocity=(30*s^2-60*s^3+30*s^4)/duration;
acceleration=(60*s-180*s^2+120*s^3)/duration^2;
end
