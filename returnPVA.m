function output = returnPVA(p1, p2, t)
% Returns cubic trajectory samples [position, velocity, acceleration]
% from p1 to p2 over t seconds, sampled every 0.1 s.

pva = [];

for i = 0:0.1:t
    X   = calculatePVA(p1, p2, i, t);
    pva = vertcat(pva, X);
end

output = pva;
