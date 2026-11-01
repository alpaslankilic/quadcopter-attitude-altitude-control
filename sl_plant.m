function dx = sl_plant(u)
%SL_PLANT Sixteen states, four motor requests, force and moment disturbance.
persistent p
if isempty(p),p=project_parameters();end
dx=flight_rhs(u(1:16),u(17:20),u(21:26),p);
end
