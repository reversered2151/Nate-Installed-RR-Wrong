package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RRStuff.MecanumDrive;


public class LocalizationHelper {

    /**
     * Initialize localization for TeleOp
     * Uses PoseStorage.currentPose as starting position (set by autonomous)
     *
     * @param hardwareMap The hardware map from the OpMode
     * @return Initialized MecanumDrive with localization ready
     */
    public static MecanumDrive initializeForTeleOp(HardwareMap hardwareMap) {
        return new MecanumDrive(hardwareMap, PoseStorage.currentPose);
    }

    /**
     * Initialize localization for Autonomous
     * Sets both the drive starting pose AND updates PoseStorage for TeleOp handoff
     *
     * @param hardwareMap The hardware map from the OpMode
     * @param startPose The starting position for autonomous
     * @return Initialized MecanumDrive with localization ready
     */
    public static MecanumDrive initializeForAuto(HardwareMap hardwareMap, Pose2d startPose) {
        PoseStorage.currentPose = startPose;  // Update PoseStorage for TeleOp handoff
        return new MecanumDrive(hardwareMap, startPose);
    }

    /**
     * Update localization estimate
     * Call this once per loop in your OpMode
     *
     * @param drive The MecanumDrive instance
     */
    public static void update(MecanumDrive drive) {
        drive.updatePoseEstimate();
    }

    /**
     * Update localization and display position on telemetry
     * Convenience method that combines update + telemetry display
     *
     * @param drive The MecanumDrive instance
     * @param telemetry The telemetry object from your OpMode
     * @param showDetails If true, shows detailed position info. If false, shows compact info.
     */
    public static void updateWithTelemetry(MecanumDrive drive, Telemetry telemetry, boolean showDetails) {
        drive.updatePoseEstimate();
        Pose2d pose = drive.localizer.getPose();

        if (showDetails) {
            telemetry.addLine("=== POSITION ===");
            telemetry.addData("X Position", "%.1f in", pose.position.x);
            telemetry.addData("Y Position", "%.1f in", pose.position.y);
            telemetry.addData("Heading", "%.1f deg", Math.toDegrees(pose.heading.toDouble()));
        } else {
            telemetry.addData("Position", "X:%.1f Y:%.1f H:%.1f°",
                pose.position.x,
                pose.position.y,
                Math.toDegrees(pose.heading.toDouble()));
        }
    }

    /**
     * Get current robot pose
     *
     * @param drive The MecanumDrive instance
     * @return Current pose (position + heading)
     */
    public static Pose2d getCurrentPose(MecanumDrive drive) {
        return drive.localizer.getPose();
    }

    /**
     * Reset robot position to a new pose
     * Useful for field-centric resets or manual position corrections
     *
     * @param drive The MecanumDrive instance
     * @param newPose The new pose to set
     */
    public static void resetPosition(MecanumDrive drive, Pose2d newPose) {
        drive.localizer.setPose(newPose);
        PoseStorage.currentPose = newPose;  // Keep PoseStorage in sync
    }

    /**
     * Calculate straight-line distance between two points on the field
     * Uses Pythagorean theorem
     *
     * @param x1 First point X coordinate (inches)
     * @param y1 First point Y coordinate (inches)
     * @param x2 Second point X coordinate (inches)
     * @param y2 Second point Y coordinate (inches)
     * @return Distance in inches
     */
    public static double calculateDistanceInches(double x1, double y1, double x2, double y2) {
        double dx = x2 - x1;
        double dy = y2 - y1;
        return Math.sqrt(dx * dx + dy * dy);
    }

    /**
     * Calculate straight-line distance between two points on the field
     * Uses Pythagorean theorem
     *
     * @param x1 First point X coordinate (inches)
     * @param y1 First point Y coordinate (inches)
     * @param x2 Second point X coordinate (inches)
     * @param y2 Second point Y coordinate (inches)
     * @return Distance in meters
     */
    public static double calculateDistanceMeters(double x1, double y1, double x2, double y2) {
        return calculateDistanceInches(x1, y1, x2, y2) * 0.0254;  // Convert inches to meters
    }

    /**
     * Calculate distance from current robot position to a target point
     *
     * @param drive The MecanumDrive instance
     * @param targetX Target X coordinate (inches)
     * @param targetY Target Y coordinate (inches)
     * @return Distance in inches
     */
    public static double getDistanceToTargetInches(MecanumDrive drive, double targetX, double targetY) {
        Pose2d currentPose = drive.localizer.getPose();
        return calculateDistanceInches(
            currentPose.position.x,
            currentPose.position.y,
            targetX,
            targetY
        );
    }

    /**
     * Calculate distance from current robot position to a target point
     *
     * @param drive The MecanumDrive instance
     * @param targetX Target X coordinate (inches)
     * @param targetY Target Y coordinate (inches)
     * @return Distance in meters
     */
    public static double getDistanceToTargetMeters(MecanumDrive drive, double targetX, double targetY) {
        return getDistanceToTargetInches(drive, targetX, targetY) * 0.0254;
    }

    /**
     * Save current pose to PoseStorage for autonomous→TeleOp handoff
     * Call this at the END of autonomous, before the OpMode finishes
     *
     * @param drive The MecanumDrive instance
     */
    public static void savePoseForTeleOp(MecanumDrive drive) {
        PoseStorage.currentPose = drive.localizer.getPose();
    }
}
