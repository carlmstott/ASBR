% brief: calculates inverse kinematics using Jacobian Transpose method
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


function [currJointAngles, allJacobians, allNormOrient, allNormTrans, err] = J_transpose_kinematics(robot, currJointAngles, desiredPoseTransMat, maxIter, threshDist, threshOr, plot)
desiredPoseTransMat = double(desiredPoseTransMat);
i = 0;
T_base_ee = Fk_Space_for_Kuka(robot,currJointAngles, plot);

gif('kukaAnimationDLS.gif','Delaytime',1/4,'loopcount',15)

twist_error_EE_frame = MatLog(TransInv(T_base_ee) * desiredPoseTransMat);

%calculate stopping criterion
distanceError = norm(twist_error_EE_frame(4:6));
orientationError = norm(twist_error_EE_frame(1:3));


while ((i < maxIter) && (distanceError > threshDist) && (orientationError > threshOr))
    i = i + 1;
    allJacobians(:,:,i) = J_body(robot, currJointAngles);
    allNormTrans(i) = distanceError;
    allNormOrient(i) = orientationError;

    hold OFF;

    currJointAngles = double(currJointAngles + 0.001 * transpose(J_body(robot, currJointAngles)) * twist_error_EE_frame);
    % update
    T_base_ee = double(Fk_Space_for_Kuka(robot, currJointAngles, plot));
    gif
    twist_error_EE_frame = MatLog(TransInv(T_base_ee) * desiredPoseTransMat);

    %calculate stopping criterion
    distanceError = norm(twist_error_EE_frame(4:6));
    orientationError = norm(twist_error_EE_frame(1:3));
    pause(0.1)

end

if(i == maxIter)
    disp(" did not converge")
    err = -1;
else
    err = 0;
end

end