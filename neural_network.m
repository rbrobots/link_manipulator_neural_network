function neural_network(q2t, q3t, q4t)
% Trains and tests a 3-layer backpropagation neural network for inverse kinematics.
% Maps end-effector (x, y, z) positions to joint angles (q2, q3, q4).
% Input angle waypoints as column vectors in radians, e.g. q2t = [5 2 1 3]' * pi/180

%% Initialisation
net = zeros(3, 3); % 3x3: each column is a layer (input | hidden | output)

% w is 9x2: col 1 = input->hidden weights, col 2 = hidden->output weights.
% For neuron n, weights from the 3 previous neurons sit at rows n, n+3, n+6.
w_1 = [0.6948 0.3171 0.9502 0.0344 0.4387 0.3816 0.7655 0.7952 0.1869]';
w_2 = [0.4898 0.4456 0.6463 0.7094 0.7547 0.2760 0.6797 0.6551 0.1626]';
w   = [w_1, w_2];

b1 = 1;   % hidden bias
b2 = 1;   % output bias
p  = 0.1; % learning rate

%% Build cubic trajectory for training angles
q2u = [];
q3u = [];
q4u = [];

for n = 1:size(q2t, 1) - 1
    q2u = vertcat(q2u, returnPVA(q2t(n,1), q2t(n+1,1), 3));
    q3u = vertcat(q3u, returnPVA(q3t(n,1), q3t(n+1,1), 3));
    q4u = vertcat(q4u, returnPVA(q4t(n,1), q4t(n+1,1), 3));
end

q = [q2u q3u q4u]; % expected output

%% Get end-effector positions via FK — used as network inputs
T0E = returnTransformation(6);

syms theta1 theta2 theta3 theta4 theta5;
xt = double(subs(T0E(1,1), {theta1 theta2 theta3 theta4 theta5}, {-90 q2u q3u q4u 0}));
yt = double(subs(T0E(1,2), {theta1 theta2 theta3 theta4 theta5}, {-90 q2u q3u q4u 0}));
zt = double(subs(T0E(1,3), {theta1 theta2 theta3 theta4 theta5}, {-90 q2u q3u q4u 0}));

i  = [xt yt zt]; % network inputs: end-effector positions
co = 1;

%% Training
while co ~= size(q, 1)

    net(:, 1) = i(co, :)'; % load sample into input layer

    % Feedforward
    for m = 1:size(net, 2)
        for n = 1:size(net, 1)
            if m == 2
                w1 = [w(n,m-1); w(n+3,m-1); w(n+6,m-1)];
                a  = sum(net(:,1) .* w1) + b1;
                net(n,m) = sigmoid(a);
            elseif m == 3
                w1 = [w(n,m-1); w(n+3,m-1); w(n+6,m-1)];
                a  = sum(net(:,2) .* w1) + b2;
                net(n,m) = sigmoid(a);
            end
        end
    end

    % Backpropagation
    del = zeros(size(net));

    for m = size(net, 2):-1:1
        for n = size(net, 1):-1:1

            if m == size(net, 2) % output layer
                o       = net(n, m);
                t       = q(co, n);
                del(n,m) = o * (1-o) * (t - o);

                w(n,   m-1) = w(n,   m-1) + p * del(n,m) * net(1, m-1);
                w(n+3, m-1) = w(n+3, m-1) + p * del(n,m) * net(2, m-1);
                w(n+6, m-1) = w(n+6, m-1) + p * del(n,m) * net(3, m-1);

            elseif m == size(net, 2) - 1 % hidden layer
                w1      = [w(n*3-2, m+1); w(n*3-1, m+1); w(n*3, m+1)]; % weights to output neurons
                o       = net(n, m);
                del(n,m) = o * (1-o) * sum(del(:, m+1) .* w1);

                w(n,   m-1) = w(n,   m-1) + p * del(n,m) * net(1, m-1);
                w(n+3, m-1) = w(n+3, m-1) + p * del(n,m) * net(2, m-1);
                w(n+6, m-1) = w(n+6, m-1) + p * del(n,m) * net(3, m-1);
            end
        end
    end

    co = co + 1;
end

disp('Training Complete')

%% Testing
q2 = [40 90 45 40 45 20 30 45 90 45]'   * pi/180;
q3 = [-20 -70 -45 -50 -47 -60 -50 -45 -70 -20]' * pi/180;
q4 = [20 5 10 0 10 -40 0 10 5 20]'      * pi/180;

q2u = [];
q3u = [];
q4u = [];

for n = 1:size(q2, 1) - 1
    q2u = vertcat(q2u, returnPVA(q2(n,1), q2(n+1,1), 3));
    q3u = vertcat(q3u, returnPVA(q3(n,1), q3(n+1,1), 3));
    q4u = vertcat(q4u, returnPVA(q4(n,1), q4(n+1,1), 3));
end

q   = [q2u q3u q4u];
T0E = returnTransformation(6);

syms theta1 theta2 theta3 theta4 theta5;
xt = double(subs(T0E(1,1), {theta1 theta2 theta3 theta4 theta5}, {-90 q2u q3u q4u 0}));
yt = double(subs(T0E(1,2), {theta1 theta2 theta3 theta4 theta5}, {-90 q2u q3u q4u 0}));
zt = double(subs(T0E(1,3), {theta1 theta2 theta3 theta4 theta5}, {-90 q2u q3u q4u 0}));

i      = [xt yt zt];
co     = 1;
MSE_q2 = 0;
MSE_q3 = 0;
MSE_q4 = 0;

while co ~= size(q, 1)

    net(:, 1) = i(co, :)';

    % Feedforward only — no weight updates during testing
    for m = 1:size(net, 2)
        for n = 1:size(net, 1)
            if m == 2
                w1 = [w(n,m-1); w(n+3,m-1); w(n+6,m-1)];
                a  = sum(net(:,1) .* w1) + b1;
                net(n,m) = sigmoid(a);
            elseif m == 3
                w1 = [w(n,m-1); w(n+3,m-1); w(n+6,m-1)];
                a  = sum(net(:,2) .* w1) + b2;
                net(n,m) = sigmoid(a);
            end
        end
    end

    MSE_q2 = MSE_q2 + (q(co,1) - net(1,3))^2;
    MSE_q3 = MSE_q3 + (q(co,2) - net(2,3))^2;
    MSE_q4 = MSE_q4 + (q(co,3) - net(3,3))^2;

    co = co + 1;
end

MSE_q2 = (1/size(q,1)) * MSE_q2
MSE_q3 = (1/size(q,1)) * MSE_q3
MSE_q4 = (1/size(q,1)) * MSE_q4

disp('Testing Complete')
end

function y = sigmoid(x)
    y = 1 / (1 + exp(-x));
end
