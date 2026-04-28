function neural_network(joint2_targets, joint3_targets, joint4_targets)
% Trains and tests a 3-layer neural network for inverse kinematics.
% Maps end-effector (x, y, z) positions to joint angles (q2, q3, q4).
%
% Inputs: joint angle waypoints (in radians) as column vectors,
%         e.g. joint2_targets = [5 2 1 3]' * pi/180

%% Initialisation
% layer_activations: 3x3 matrix where each column is a network layer
%   col 1 = input layer, col 2 = hidden layer, col 3 = output layer
layer_activations = zeros(3, 3);

% weights: 9x2 matrix. Each column holds 9 weights for one layer transition.
%   Rows are interleaved: for output neuron n, its weights from input neurons
%   1/2/3 sit at rows n, n+3, n+6 in that column.
weights_col1 = [0.6948 0.3171 0.9502 0.0344 0.4387 0.3816 0.7655 0.7952 0.1869]';
weights_col2 = [0.4898 0.4456 0.6463 0.7094 0.7547 0.2760 0.6797 0.6551 0.1626]';
weights = [weights_col1, weights_col2];

hidden_bias   = 1;
output_bias   = 1;
learning_rate = 0.1;

%% Build cubic trajectory for training angles
joint2_traj = [];
joint3_traj = [];
joint4_traj = [];

for segment = 1:size(joint2_targets, 1) - 1
    joint2_traj = vertcat(joint2_traj, returnPVA(joint2_targets(segment, 1), joint2_targets(segment+1, 1), 3));
    joint3_traj = vertcat(joint3_traj, returnPVA(joint3_targets(segment, 1), joint3_targets(segment+1, 1), 3));
    joint4_traj = vertcat(joint4_traj, returnPVA(joint4_targets(segment, 1), joint4_targets(segment+1, 1), 3));
end

% target_angles: expected network outputs — one row per time step
target_angles = [joint2_traj, joint3_traj, joint4_traj];

%% Compute end-effector positions via forward kinematics (input to network)
end_effector_transform = returnTransformation(6);

syms theta1 theta2 theta3 theta4 theta5;
fixed_base_angle = -90;

ee_x = double(subs(end_effector_transform(1, 1), {theta1 theta2 theta3 theta4 theta5}, {fixed_base_angle joint2_traj joint3_traj joint4_traj 0}));
ee_y = double(subs(end_effector_transform(1, 2), {theta1 theta2 theta3 theta4 theta5}, {fixed_base_angle joint2_traj joint3_traj joint4_traj 0}));
ee_z = double(subs(end_effector_transform(1, 3), {theta1 theta2 theta3 theta4 theta5}, {fixed_base_angle joint2_traj joint3_traj joint4_traj 0}));

network_inputs = [ee_x, ee_y, ee_z];

%% Training
sample_idx = 1;

while sample_idx ~= size(target_angles, 1)
    % Load current sample into the input layer
    layer_activations(:, 1) = network_inputs(sample_idx, :)';

    % --- Feedforward pass ---
    for layer_idx = 1:size(layer_activations, 2)
        for neuron_idx = 1:size(layer_activations, 1)
            if layer_idx == 2  % hidden layer: inputs come from layer 1
                neuron_weights = [weights(neuron_idx, layer_idx-1); weights(neuron_idx+3, layer_idx-1); weights(neuron_idx+6, layer_idx-1)];
                weighted_sum   = sum(layer_activations(:, 1) .* neuron_weights) + hidden_bias;
                layer_activations(neuron_idx, layer_idx) = sigmoid(weighted_sum);

            elseif layer_idx == 3  % output layer: inputs come from hidden layer
                neuron_weights = [weights(neuron_idx, layer_idx-1); weights(neuron_idx+3, layer_idx-1); weights(neuron_idx+6, layer_idx-1)];
                weighted_sum   = sum(layer_activations(:, 2) .* neuron_weights) + output_bias;
                layer_activations(neuron_idx, layer_idx) = sigmoid(weighted_sum);
            end
        end
    end

    % --- Backpropagation pass ---
    delta = zeros(size(layer_activations));

    for layer_idx = size(layer_activations, 2):-1:1
        for neuron_idx = size(layer_activations, 1):-1:1

            if layer_idx == size(layer_activations, 2)  % output layer
                neuron_output = layer_activations(neuron_idx, layer_idx);
                target_val    = target_angles(sample_idx, neuron_idx);
                delta(neuron_idx, layer_idx) = neuron_output * (1 - neuron_output) * (target_val - neuron_output);

                % Update weights connecting hidden → this output neuron
                weights(neuron_idx,   layer_idx-1) = weights(neuron_idx,   layer_idx-1) + learning_rate * delta(neuron_idx, layer_idx) * layer_activations(1, layer_idx-1);
                weights(neuron_idx+3, layer_idx-1) = weights(neuron_idx+3, layer_idx-1) + learning_rate * delta(neuron_idx, layer_idx) * layer_activations(2, layer_idx-1);
                weights(neuron_idx+6, layer_idx-1) = weights(neuron_idx+6, layer_idx-1) + learning_rate * delta(neuron_idx, layer_idx) * layer_activations(3, layer_idx-1);

            elseif layer_idx == size(layer_activations, 2) - 1  % hidden layer
                % Weights from this hidden neuron to all output neurons
                output_weights = [weights(neuron_idx*3-2, layer_idx+1); weights(neuron_idx*3-1, layer_idx+1); weights(neuron_idx*3, layer_idx+1)];
                neuron_output = layer_activations(neuron_idx, layer_idx);
                delta(neuron_idx, layer_idx) = neuron_output * (1 - neuron_output) * sum(delta(:, layer_idx+1) .* output_weights);

                % Update weights connecting input → this hidden neuron
                weights(neuron_idx,   layer_idx-1) = weights(neuron_idx,   layer_idx-1) + learning_rate * delta(neuron_idx, layer_idx) * layer_activations(1, layer_idx-1);
                weights(neuron_idx+3, layer_idx-1) = weights(neuron_idx+3, layer_idx-1) + learning_rate * delta(neuron_idx, layer_idx) * layer_activations(2, layer_idx-1);
                weights(neuron_idx+6, layer_idx-1) = weights(neuron_idx+6, layer_idx-1) + learning_rate * delta(neuron_idx, layer_idx) * layer_activations(3, layer_idx-1);
            end
        end
    end

    sample_idx = sample_idx + 1;
end

disp('Training Complete')

%% Testing
% Evaluate the trained network on a fixed set of test angles and report MSE.

test_joint2 = [40 90 45 40 45 20 30 45 90 45]'  * pi/180;
test_joint3 = [-20 -70 -45 -50 -47 -60 -50 -45 -70 -20]' * pi/180;
test_joint4 = [20 5 10 0 10 -40 0 10 5 20]'      * pi/180;

joint2_traj = [];
joint3_traj = [];
joint4_traj = [];

for segment = 1:size(test_joint2, 1) - 1
    joint2_traj = vertcat(joint2_traj, returnPVA(test_joint2(segment, 1), test_joint2(segment+1, 1), 3));
    joint3_traj = vertcat(joint3_traj, returnPVA(test_joint3(segment, 1), test_joint3(segment+1, 1), 3));
    joint4_traj = vertcat(joint4_traj, returnPVA(test_joint4(segment, 1), test_joint4(segment+1, 1), 3));
end

target_angles = [joint2_traj, joint3_traj, joint4_traj];

end_effector_transform = returnTransformation(6);

syms theta1 theta2 theta3 theta4 theta5;
ee_x = double(subs(end_effector_transform(1, 1), {theta1 theta2 theta3 theta4 theta5}, {fixed_base_angle joint2_traj joint3_traj joint4_traj 0}));
ee_y = double(subs(end_effector_transform(1, 2), {theta1 theta2 theta3 theta4 theta5}, {fixed_base_angle joint2_traj joint3_traj joint4_traj 0}));
ee_z = double(subs(end_effector_transform(1, 3), {theta1 theta2 theta3 theta4 theta5}, {fixed_base_angle joint2_traj joint3_traj joint4_traj 0}));

network_inputs = [ee_x, ee_y, ee_z];

sample_idx = 1;
mse_joint2 = 0;
mse_joint3 = 0;
mse_joint4 = 0;

while sample_idx ~= size(target_angles, 1)
    layer_activations(:, 1) = network_inputs(sample_idx, :)';

    % Feedforward only (no weight updates during testing)
    for layer_idx = 1:size(layer_activations, 2)
        for neuron_idx = 1:size(layer_activations, 1)
            if layer_idx == 2
                neuron_weights = [weights(neuron_idx, layer_idx-1); weights(neuron_idx+3, layer_idx-1); weights(neuron_idx+6, layer_idx-1)];
                weighted_sum   = sum(layer_activations(:, 1) .* neuron_weights) + hidden_bias;
                layer_activations(neuron_idx, layer_idx) = sigmoid(weighted_sum);

            elseif layer_idx == 3
                neuron_weights = [weights(neuron_idx, layer_idx-1); weights(neuron_idx+3, layer_idx-1); weights(neuron_idx+6, layer_idx-1)];
                weighted_sum   = sum(layer_activations(:, 2) .* neuron_weights) + output_bias;
                layer_activations(neuron_idx, layer_idx) = sigmoid(weighted_sum);
            end
        end
    end

    % Accumulate squared errors for each output joint
    mse_joint2 = mse_joint2 + (target_angles(sample_idx, 1) - layer_activations(1, 3))^2;
    mse_joint3 = mse_joint3 + (target_angles(sample_idx, 2) - layer_activations(2, 3))^2;
    mse_joint4 = mse_joint4 + (target_angles(sample_idx, 3) - layer_activations(3, 3))^2;

    sample_idx = sample_idx + 1;
end

num_samples = size(target_angles, 1);
mse_joint2 = (1 / num_samples) * mse_joint2
mse_joint3 = (1 / num_samples) * mse_joint3
mse_joint4 = (1 / num_samples) * mse_joint4

disp('Testing Complete')
end

%% Helper
function y = sigmoid(x)
    y = 1 / (1 + exp(-x));
end
