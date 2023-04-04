% brief: calcuates singularity configurations of the robot
% params:
% jacobian: geometric jacobian of the manipulator
% symbolicJointAngles: column vector of symbols representing joint angles
% in the jacobian. Eg: [t1, t2, t3]
% opt1: optional initial configuration of the robot. Random joint
% configurations are used if this is not supplied
% returns:
% singularJointAngles: nx1 one singularity configuration of the robot. Run this
% function as a loop to get multiple singularity configurations.

function singularJointAngles = singularity(jacobian, symbolicJointAngles, opt1)

% if use initial condition if supplied. Else, create random initial joint
% configuration.
if(nargin > 2)
    initialJointConfiguration = opt1;
else
    initialJointConfiguration = randi([-314, 314], size(jacobian, 2), 1)/100;
end

singularJointAngles = double(transpose(struct2array(vpasolve(det(jacobian) == 0, symbolicJointAngles, initialJointConfiguration))));

end