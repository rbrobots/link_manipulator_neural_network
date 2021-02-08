# link_manipulator_neural_network
In ‘A study of neural network based inverse kinematics solution for three-joint robot’ (Köker 2014), the authors propose using backpropagation to retrieve the inverse kinematics for a three-joint robot. They state that the advantage is that the artificial neural network uses shorter calculations (in comparison to inverse kinematic solutions which is time consuming) and therefore less computation power. In this section, a similar network is implemented in MATLAB to minimize the error between the target and predicted angles for joints q2, q3 and q4. Adapted from this paper (Köker 2014), the following neural network has been implemented


Where x, y and z are the end effector positions used as input and q2, q3 and q4 are the output
angles where q1 is fixed to -90 and q5 is 0, b1 and b2 are the biases, wij and wjk are the weights.
The backpropagation algorithm follows three steps:
1. Feed forward – calculation of the activation function and sigmoid function. This is repeated for
each neuron excluding the input neurons. For the hidden layer equations 28 and 29 are used.
𝑎𝑖𝑗 = ∑𝑖𝑛 𝑤𝑖𝑗 + 𝑏1 (28)
𝑠 =
1
1 + 𝑒
−𝑎𝑖𝑗
(29)
The same process is applied to the output layer, but the inputs are the sigmoid functions from the
hidden layer, also 𝑤𝑗𝑘 and b2 are used instead.
2. Backpropagation – use gradient descent to find error in target output and actual output. Neurons
in output layer and hidden layer are calculated respectively in equations 30 and 31.
𝛿(𝑘) = 𝑂𝑈𝑇𝑘
(1 − 𝑂𝑈𝑇𝑘
)(𝑇𝐴𝑅𝐺𝐸𝑇𝑘 − 𝑂𝑈𝑇𝑘
) (30)
𝛿(𝑗) = 𝑂𝑈𝑇𝑗(1 − 𝑂𝑈𝑇𝑗)∑𝛿(𝑘) 𝑤𝑘𝑗 (31)
3. Update of weights – output to hidden layer and hidden layer to input layer respectively, equations
32 and 33. In contrast to (Köker 2014), momentum is not used; only n, which is the learning rate.
𝑤𝑘𝑗 𝑁𝐸𝑊 = 𝑤𝑘𝑗 𝑂𝐿𝐷 + 𝑛𝛿(𝑘)𝑂𝑈𝑇𝑘
(32)
𝑤𝑗𝑖 𝑁𝐸𝑊 = 𝑤𝑗𝑖 𝑂𝐿𝐷 + 𝑛𝛿(𝑗)𝑂𝑈𝑇𝑗
(33)
