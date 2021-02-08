function x = neural_network(q2t,q3t,q4t)
%% Initialisation of Variables
%input a set of angles for q2,q3,q4 as q2=[5 2 1 3]'*pi/180 for example

net=zeros(3,3);%neural network
dl=zeros(3,3);%backprop to store delta

%%initialise weights (which were first randomised)
w_1=[0.6948    0.3171    0.9502    0.0344    0.4387    0.3816    0.7655    0.7952    0.1869]';
w_2=[0.4898    0.4456    0.6463    0.7094    0.7547    0.2760    0.6797    0.6551    0.1626]';    
w=[w_1 w_2];

b1=1;%bias for hidden
b2=1;%bias for output
p=0.1;%learning rate

%get set of input from cubic trajectory for q2,q3,q4 
%with angles used in FK

q2u=[];
q3u=[];
q4u=[];

for n=1:size(q2t,1)-1
    q2s=returnPVA(q2t(n,1),q2t(n+1,1),3);
    q2u=vertcat(q2u,q2s(:,1));
    
    q3s=returnPVA(q3t(n,1),q3t(n+1,1),3);
    q3u=vertcat(q3u,q3s(:,1));
    
    q4s=returnPVA(q4t(n,1),q4t(n+1,1),3);
    q4u=vertcat(q4u,q4s(:,1));
end

q=[q2u q3u q4u];%expected output

%use FK to get end effector positions
T0E=returnTransformation(6);

syms theta1 theta2 theta3 theta4 theta5;
%Replace angles in FK by those defined above
%to get input for neural network
xt=double(subs(T0E(1,1),{theta1 theta2 theta3 theta4 theta5},{-90 q2u q3u q4u 0}));
yt=double(subs(T0E(1,2),{theta1 theta2 theta3 theta4 theta5},{-90 q2u q3u q4u 0}));
zt=double(subs(T0E(1,3),{theta1 theta2 theta3 theta4 theta5},{-90 q2u q3u q4u 0}));

i=[xt yt zt];%input values for neural network
co=1;%counter
%% Training Network
while(co~=size(q,1))%iterate however many times specified by input
   net(1,1)=i(co,1);
   net(2,1)=i(co,2);
   net(3,1)=i(co,3);

    %feedforward
    for m=1:size(net,2)%rows
       for n=1:size(net,1)%columns
           if(m==2)%if hidden layer
               w1=[w(n,m-1) w(n+3,m-1) w(n+6,m-1)]';%3,6,9
               a=sum((net(:,1).*w1))+b1;%activation this neuron equals sum(w*i)+b
               net(n,m)=1/(1+exp(-a));%neuron equals output calculated via sigmoid function
               
           elseif(m==3)% if output layer
               w2=[w(n,m-1) w(n+3,m-1) w(n+6,m-1)]';
               a=sum(net(:,2).*w2)+b2;%activation - takes output from hidden layer as input
               net(n,m)=1/(1+exp(-a));%store sigmoid in network
           end
       end
    end
    
    %backpropagate
    for m=size(net,2):-1:1%rows
        for n=size(net,1):-1:1%columns
            if(m==size(net,2))  %output layer to hidden
                o=net(n,m);%output
                if(n==1)%target is q2
                %t=do(n,1);%target output
                    t=q(co,1);
                elseif(n==2)%target is q3
                    t=q(co,2);
                elseif(n==3)%target is q4
                    t=q(co,3);
                end
                del(n,m)=o*(1-o)*(t-o);%error gradient

                %update weights. 3 different weights all the same delta 
               w1=[w(n,m-1) w(n+3,m-1) w(n+6,m-1)]';% 3 6 9            
               
               w(n,m-1)=w(n,m-1)+p*del(n,m)*net(1,m-1);%1st weight in hidden layer
               w(n+3,m-1)=w(n+3,m-1)+p*del(n,m)*net(2,m-1);%2nd weight in hidden layer
               w(n+6,m-1)=w(n+6,m-1)+p*del(n,m)*net(3,m-1);%3rd weight in hidden layer
               
            elseif(m==size(net,2)-1)%hidden layer to input
                w1=[w(n*3-2,m) w(n*3-1,m) w(n*3,m)]';
                o=net(n,m);%output
                del(n,m)=o*(1-o)*sum(del(:,m+1).*w1);%previous delta 
                
               w(n,m-1)=w(n,m-1)+p*del(n,m)*net(1,m-1);%1st weight in input layer
               w(n+3,m-1)=w(n+3,m-1)+p*del(n,m)*net(2,m-1);%2nd weight in input layer
               w(n+6,m-1)=w(n+6,m-1)+p*del(n,m)*net(3,m-1);%3rd weight in input layer
               
            end
        end
    end 
    co=co+1;%increment counter
end

disp('Training Complete')
%% Testing Network
%store difference between output and expected output

%get values used in assignment

%angles used in assignment for testing
co=1;%counter
MSE_q2=0;%mean squared error intialised
MSE_q3=0;%mean squared error intialised
MSE_q4=0;%mean squared error intialised

q2 = [40 90 45 40 45 20 30 45 90 45 ]'*pi/180;
q3 = [-20 -70 -45 -50 -47 -60 -50 -45 -70 -20 ]'*pi/180;
q4 = [20 5 10 0 10 -40 0 10 5 20]'*pi/180;

%retrieve x,y,z for q2,q3,q4
q2u=[];
q3u=[];
q4u=[];
%get set of input from cubic trajectory for q2,q3,q4 
%with EE positions from FK
%lock q1 at -90
for n=1:size(q2,1)-1
    q2s=returnPVA(q2(n,1),q2(n+1,1),3);
    q2u=vertcat(q2u,q2s(:,1));
    
    q3s=returnPVA(q3(n,1),q3(n+1,1),3);
    q3u=vertcat(q3u,q3s(:,1));
    
    q4s=returnPVA(q4(n,1),q4(n+1,1),3);
    q4u=vertcat(q4u,q4s(:,1));
end

q=[q2u q3u q4u];%expected output

%use FK to get end effector positions
T0E=returnTransformation(6);

syms theta1 theta2 theta3 theta4 theta5;
%Replace angles in FK by those defined above
%to get input for neural network
xt=double(subs(T0E(1,1),{theta1 theta2 theta3 theta4 theta5},{-90 q2u q3u q4u 0}));
yt=double(subs(T0E(1,2),{theta1 theta2 theta3 theta4 theta5},{-90 q2u q3u q4u 0}));
zt=double(subs(T0E(1,3),{theta1 theta2 theta3 theta4 theta5},{-90 q2u q3u q4u 0}));

i=[xt yt zt];%input values for neural network
%while loop. 
while(co~=size(q,1))%iterate 
   net(1,1)=i(co,1);
   net(2,1)=i(co,2);
   net(3,1)=i(co,3);

    %feedforward
    for m=1:size(net,2)%rows
       for n=1:size(net,1)%columns
           if(m==2)%if hidden layer
               w1=[w(n,m-1) w(n+3,m-1) w(n+6,m-1)]';%3,6,9
               a=sum((net(:,1).*w1))+b1;%activation this neuron equals sum(w*i)+b
               net(n,m)=1/(1+exp(-a));%neuron equals output
               
           elseif(m==3)% if output layer
               w2=[w(n,m-1) w(n+3,m-1) w(n+6,m-1)]';
               a=sum(net(:,2).*w2)+b2;%activation - takes output from hidden layer as input
               net(n,m)=1/(1+exp(-a));%store sigmoid in network
           end
       end
    end
   %compare output values by MSE
    MSE_q2=MSE_q2+(q(co,1)-net(1,3))^2;%(q2-q2')^2
    MSE_q3=MSE_q3+(q(co,2)-net(2,3))^2;
    MSE_q4=MSE_q4+(q(co,3)-net(3,3))^2;
   
   co=co+1;%increment counter 
end
    MSE_q2=(1/size(q,1)*MSE_q2)
    MSE_q3=(1/size(q,1)*MSE_q3)
    MSE_q4=(1/size(q,1)*MSE_q4)
    
disp('Testing Complete')

