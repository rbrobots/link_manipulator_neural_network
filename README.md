# link_manipulator_neural_network
<strong>Overview</strong>

In ‘A study of neural network based inverse kinematics solution for three-joint robot’ (Köker 2014), the authors propose using backpropagation to retrieve the inverse kinematics for a three-joint robot. They state that the advantage is that the artificial neural network uses shorter calculations (in comparison to inverse kinematic solutions which is time consuming) and therefore less computation power.

Given a 5 link manipulator, a similar network is implemented in MATLAB to minimize the error between the target and predicted angles for joints q2, q3 and q4. Adapted from this paper (Köker 2014), the following neural network has been implemented. Where joint q1 is fixed in place.

<strong>References</strong>

Köker, R., Öz, C., Çakar, T. and Ekiz, H., 2004. A study of neural network based inverse kinematics
solution for a three-joint robot. Robotics and autonomous systems, 49(3-4), pp.227-234.
