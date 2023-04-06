function currJointAngles = IK(robot, currJointAngles, desiredPoseTransMat, maxIter)
desiredPoseTransMat = double(desiredPoseTransMat);
i = 0;
T_base_ee = FK_space(robot,currJointAngles, false);
twist_error_EE_frame = MatLog(TransInv(T_base_ee) * desiredPoseTransMat);

while i < maxIter
i = i + 1;
currJointAngles = double(currJointAngles + pinv(J_body(robot, currJointAngles)) * twist_error_EE_frame);
T_base_ee = double(FK_space(robot, currJointAngles, false));
twist_error_EE_frame = MatLog(TransInv(T_base_ee) * desiredPoseTransMat);

distanceError = norm(twist_error_EE_frame(4:6))

pause(0.1)

end
end