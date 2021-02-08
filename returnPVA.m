function output= returnPVA(p1,p2,t)
%%position, velocity and acceleration between q and q'

pva=[];
for i=0:0.1:t
    X=calculatePVA(p1,p2,i,t);
    pva=vertcat(pva,X);

end

output=pva;
