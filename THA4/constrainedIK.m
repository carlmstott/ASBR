% brief: calculates inverse kinematics using Newton Raphson method while
% handling singularity configurations using Damped Least Squares method
% robot: robot object consistig of the robot's kinematic properties
% defined in defineRobot.m
% currJointAngles: current robot configuration
% desiredPoseTransMat: desired EE position as a translation matrix
% maxIter: maximum number of iterations
% threshDist: stopping criteria for translations
% threshOr: stopping criteria for orientations
% plot: boolean value to plot (or not) the robot's frames at each iteration
% returns:
% currJointAngles: joint configuration to achieve desired pose
% err: error code


function [currJointAngles, allJacobians, allNormOrient, allNormTrans, err] = constrainedIK(robot, currJointAngles, desiredPoseTransMat, maxIter, threshDist, threshOr, plot)
allJacobians=[];
allNormOrient = [];
allNormTrans = [];
err = [];

desiredPoseTransMat = double(desiredPoseTransMat);
i = 0;
T_base_ee = Fk_Space_for_Kuka(robot,currJointAngles, plot);

gif('kukaAnimationDLS.gif','Delaytime',1/4,'loopcount',15)

twist_error_EE_frame = MatLog(TransInv(T_base_ee) * desiredPoseTransMat);

%calculate stopping criterion
distanceError = norm(twist_error_EE_frame(4:6));
orientationError = norm(twist_error_EE_frame(1:3));

jointLimits_min = deg2rad([-185;
    -140;
    -120;
    -350;
    -125;
    -350;]);

jointLimits_max = deg2rad([185;
    -5;
    168;
    350;
    125;
    350;]);

jointLimits_median = (jointLimits_min + jointLimits_max) / 2;

comparison = (currJointAngles >= jointLimits_min) + (currJointAngles <= jointLimits_max);

while ((i < maxIter) && ((distanceError > threshDist) || (orientationError > threshOr)))
    i = i + 1;
    allJacobians(:,:,i) = J_body(robot, currJointAngles);
    allNormTrans(i) = distanceError;
    allNormOrient(i) = orientationError;

    hold OFF;
    
    % Calculate Jacobian and Moore-Penrose Pseudoinverse
    J = J_body(robot, currJointAngles);
    J_dagger = pinv(J);

    % Calculate H function for joint limits and its gradient
    H = 0;
    jointAngles = sym('theta', [robot.numJoints 1]);

    for H_iter = 1 : robot.numJoints
        H = H + ((jointAngles(H_iter) - jointLimits_median(H_iter)) / (jointLimits_median(H_iter) - jointLimits_max(H_iter))) ^ 2;
    end
    H = H / robot.numJoints;
    grad_H = gradient(H);
    grad_H = double(subs(grad_H, jointAngles, currJointAngles));

    % Calculate G funtion and its gradient for task space constraint to
    % sphere
    radius_of_sphere = 3;
    
    FKspace_sym=FK_space(robot,jointAngles,0); 

    G = (norm(FKspace_sym(1:3,4)-desiredPoseTransMat(1:3,4))/radius_of_sphere) ^ 2;
    grad_G = gradient(G);
    grad_G = double(subs(grad_G, jointAngles, currJointAngles));



    delta_theta = 0.07 * J_dagger * twist_error_EE_frame + (eye(robot.numJoints) - J_dagger * J) * 10000* grad_H + (eye(robot.numJoints) - J_dagger * J) * 7000000000 * grad_G;
    currJointAngles = double(currJointAngles + delta_theta);

    comparison = ((currJointAngles >= jointLimits_min) + (currJointAngles <= jointLimits_max)) / 2;


    % update
    T_base_ee = double(Fk_Space_for_Kuka(robot, currJointAngles, plot));
    %gif
    twist_error_EE_frame = MatLog(TransInv(T_base_ee) * desiredPoseTransMat);
    
    text(75, 20, -24, "Joints In Range = ")
    text(90,20,-30,num2str(comparison));

    text(-63, 20, 15, "Joint Positions = ")
    text(-50,20,10,num2str(currJointAngles))
    
    %calculate stopping criterion
    distanceError = norm(twist_error_EE_frame(4:6));
    orientationError = norm(twist_error_EE_frame(1:3));

    text(75, 20, -40, "Linear error = ")
    text(85, 20, -45,num2str(distanceError));
    text(75, 20, -50, "Orientation error = ")
    text(85,20,-55,num2str(orientationError));

gif 

 


    if(i == maxIter)
        disp(" did not converge")
        err = -1;
    else
        err = 0;
    end

end