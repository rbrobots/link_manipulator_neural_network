function trajectory = returnPVA(start_pos, end_pos, duration)
% Returns a cubic trajectory of [position, velocity, acceleration] samples
% from start_pos to end_pos over `duration` seconds, sampled every 0.1 s.

trajectory = [];

for time_step = 0:0.1:duration
    pva_point  = calculatePVA(start_pos, end_pos, time_step, duration);
    trajectory = vertcat(trajectory, pva_point);
end
