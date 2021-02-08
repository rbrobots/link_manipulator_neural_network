function output= returnPVA(p1,p2,t)
%%position, velocity and acceleration between q and q'
%%To generate the profiles - uncomment the code below
%%position profile
pva=[];
for i=0:0.1:t
    X=calculatePVA(p1,p2,i,t);
    pva=vertcat(pva,X);
    %hold on
    %figure(1)
    %plot(i,X(1,1),'.'); 
    %axis([0 t,p2 p1])
    %xlabel('t (time)') ; ylabel('\theta (position)'); 
end
%%velocity profile
% for i=0:0.1:t
%     figure(2)
%     X=calculatePVA(p1,p2,i,t);
%     hold on
%     plot(i,X(1,2),'.'); 
%     axis([0 t,-1 1])
%     xlabel('t (time)'); ylabel('$\dot{\theta}$ (velocity)', 'Interpreter','latex');
% end
% 
%%acceleration profile
% for i=0:0.1:t
%     figure(3)
%     X=calculatePVA(p1,p2,i,t);
%     hold on
%     plot(i,X(1,3),'.'); 
%     axis([0 t,-1 1])
%     xlabel('t (time)') ; ylabel('$\dot{\dot{\theta}}$ (acceleration)', 'Interpreter','latex');
% end
% 
output=pva;