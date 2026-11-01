function output = sl_controller(u)
%SL_CONTROLLER Plant states, four PI states, then ten trajectory channels.
persistent p
if isempty(p),p=project_parameters();end
[rotor,dz,attitude,thrust]=flight_controller(u(1:16),u(17:20),u(21:30),p);
output=[rotor;dz;attitude;thrust];
end
