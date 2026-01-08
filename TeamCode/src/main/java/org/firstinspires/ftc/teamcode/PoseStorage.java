package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;

/**
 * Simple static class to store the robot's pose between OpModes.
 * This allows you to pass the final autonomous position to TeleOp.
 */
public class PoseStorage {
    public static Pose2d currentPose = new Pose2d(0, 0, 0);
}
