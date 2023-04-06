function [currJointAngles, err] = DLS_inverse_kinematics(robot, currJointAngles, desiredPoseTransMat, maxIter, threshDist, threshOr, plot)
desiredPoseTransMat = double(desiredPoseTransMat);
i = 0;
T_base_ee = FK_space(robot,currJointAngles, plot);
twist_error_EE_frame = MatLog(TransInv(T_base_ee) * desiredPoseTransMat);

%calculate stopping criterion
distanceError = norm(twist_error_EE_frame(4:6));
orientationError = norm(twist_error_EE_frame(1:3));

while ((i < maxIter) && (distanceError > threshDist) && (orientationError > threshOr))
    i = i + 1;

    if(J_isptrophy(J_body(robot, currJointAngles)) > 1e4)
        isSingular = true;
    else
        isSingular = false;
    end


    if(~isSingular)
        % if robot is not singular, use Newton-Raphson
        currJointAngles = double(currJointAngles + pinv(J_body(robot, currJointAngles)) * twist_error_EE_frame);
    else
        %If robot is singular, use Damped Least Squares (LM method)

        % Ref: Eq 11 on Introduction to "Inverse Kinematics  with Jacobian
        % Transpose, PseudoInverse and Damped Leasst Square Methods" by
        % Samuel R. Buss et al.
        disp("using DLS")
        J_J_transpose = J_body(robot, currJointAngles) * transpose(J_body(robot, currJointAngles));
        currJointAngles = double(currJointAngles + transpose(J_body(robot, currJointAngles)) *inv((J_J_transpose + 0.5 * 0.5*eye(size(J_J_transpose)))) * twist_error_EE_frame);
    end

    % update
    T_base_ee = double(FK_space(robot, currJointAngles, plot));
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
    err = 0
end

end