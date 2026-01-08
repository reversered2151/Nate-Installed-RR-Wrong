package org.firstinspires.ftc.teamcode.qualifiers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.LocalizationHelper;
import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.RRStuff.MecanumDrive;


@TeleOp(name = "TeleOp", group = "qualifiers")
public class qualifiersTeleOp extends LinearOpMode {

    // Formula: RPM = REGRESSION_SLOPE * distance_meters + REGRESSION_INTERCEPT
    private static final double REGRESSION_SLOPE = 200.0;      // TODO: Update with actual slope from regression
    private static final double REGRESSION_INTERCEPT = 1000.0; // TODO: Update with actual intercept from regression

    // Goal positions on field (in inches, measured from field center)
    private static final double BLUE_GOAL_X = -60.0;  // TODO: Update with actual blue basket position
    private static final double BLUE_GOAL_Y = -60.0;
    private static final double RED_GOAL_X = 60.0;    // TODO: Update with actual red basket position
    private static final double RED_GOAL_Y = 60.0;

    // Flywheel limits
    private static final double MIN_RPM = 500.0;
    private static final double MAX_RPM = 7000.0;
    private static final double SPEED_TOL_RPM = 100.0;  // RPM tolerance for "at speed"
    private static final double TICKS_PER_REV = 28.0;

    // Shooting timing
    private static final long SHOOT_DURATION_MS = 1500;  // Hold shooting for 1.5 seconds

    // Servo positions (from mechanisms.java)
    private static final double BLOCKER_OPEN = 0.6;
    private static final double BLOCKER_CLOSED = 0.95;
    // ========================================================================

    // Shooting state machine
    private enum ShootingState {
        IDLE,
        STOPPING_INTAKE,
        SPINNING_UP,
        SHOOTING,
        FINISHING
    }

    private ShootingState shootingState = ShootingState.IDLE;
    private long stateStartTime = 0;
    private double targetRpm = 0.0;
    private boolean targetBlueGoal = true;  // true = blue, false = red

    // Button debouncing
    private boolean prevRightBumper = false;
    private boolean prevDpadLeft = false;
    private boolean prevB = false;

    qualifiersHardwareMap hardware = new qualifiersHardwareMap();
    MecanumDrive drive;

    DcMotorEx flywheel;
    DcMotorEx intake;
    DcMotorEx uptake;

    Servo blocker;

    @Override
    public void runOpMode() throws InterruptedException{

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        hardware.init(hardwareMap);


        flywheel = hardware.flywheel;
        intake = hardware.intake;
        uptake = hardware.uptake;
        blocker = hardware.blocker;

        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);




        // Initialize Road Runner drive with starting pose from autonomous
        // If no auto was run, defaults to (0, 0, 0)
        drive = LocalizationHelper.initializeForTeleOp(hardwareMap);

        telemetry.addData("Starting Position", "X: %.2f, Y: %.2f, Heading: %.1f°",
            PoseStorage.currentPose.position.x,
            PoseStorage.currentPose.position.y,
            Math.toDegrees(PoseStorage.currentPose.heading.toDouble()));
        telemetry.addLine();
        telemetry.addLine("=== MANUAL POSITION OVERRIDE ===");
        telemetry.addLine("A: Far Red (56, -8, -225°)");
        telemetry.addLine("B: Close Red (-50, 50, -45°)");
        telemetry.addLine("X: Far Blue (56, -8, -225°)");
        telemetry.addLine("Y: Close Blue (-50, -50, 45°)");
        telemetry.addLine("DPAD_DOWN: Reset to (0, 0, 0)");
        telemetry.addLine();
        telemetry.addData("Status", "Ready - Press buttons to override position");
        telemetry.update();



        // Manual position selection during init
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.a) {
                // Far Red position
                LocalizationHelper.resetPosition(drive, new Pose2d(56, -8, Math.toRadians(-225)));
                telemetry.addData("Position Override", "Far Red (56, -8, -225°)");
                telemetry.update();
                sleep(300); // Debounce
            }
            else if (gamepad1.b) {
                // Close Red position
                LocalizationHelper.resetPosition(drive, new Pose2d(-50, 50, Math.toRadians(-45)));
                telemetry.addData("Position Override", "Close Red (-50, 50, -45°)");
                telemetry.update();
                sleep(300);
            }
            else if (gamepad1.x) {
                // Far Blue position
                LocalizationHelper.resetPosition(drive, new Pose2d(56, -8, Math.toRadians(-225)));
                telemetry.addData("Position Override", "Far Blue (56, -8, -225°)");
                telemetry.update();
                sleep(300);
            }
            else if (gamepad1.y) {
                // Close Blue position
                LocalizationHelper.resetPosition(drive, new Pose2d(-50, -50, Math.toRadians(45)));
                telemetry.addData("Position Override", "Close Blue (-50, -50, 45°)");
                telemetry.update();
                sleep(300);
            }
            else if (gamepad1.dpad_down) {
                // Reset to origin
                LocalizationHelper.resetPosition(drive, new Pose2d(0, 0, 0));
                telemetry.addData("Position Override", "Origin (0, 0, 0)");
                telemetry.update();
                sleep(300);
            }
        }


        while (opModeIsActive()){

            // Update localization - THIS IS CRITICAL!
            LocalizationHelper.update(drive);

            // Get current pose
            Pose2d currentPose = LocalizationHelper.getCurrentPose(drive);

            // Calculate distance to goal and target RPM
            double goalX = targetBlueGoal ? BLUE_GOAL_X : RED_GOAL_X;
            double goalY = targetBlueGoal ? BLUE_GOAL_Y : RED_GOAL_Y;
            double distanceToGoal = LocalizationHelper.getDistanceToTargetMeters(drive, goalX, goalY);
            double calculatedRpm = computeRPMLinearRegression(distanceToGoal);

            // ========================================================================
            // SHOOTING CONTROLS
            // ========================================================================

            boolean rightBumper = gamepad1.right_bumper;
            boolean bButton = gamepad1.b;
            boolean dpadLeft = gamepad1.dpad_left;

            // D-PAD LEFT: Toggle Blue/Red Goal
            if (dpadLeft && !prevDpadLeft) {
                targetBlueGoal = !targetBlueGoal;
            }
            prevDpadLeft = dpadLeft;

            // RIGHT BUMPER: Start shooting sequence
            if (rightBumper && !prevRightBumper && shootingState == ShootingState.IDLE) {
                shootingState = ShootingState.STOPPING_INTAKE;
                targetRpm = calculatedRpm;
                stateStartTime = System.currentTimeMillis();
            }
            prevRightBumper = rightBumper;

            // B BUTTON: Emergency stop
            if (bButton && !prevB) {
                shootingState = ShootingState.IDLE;
                stopFlywheel();
                intake.setPower(0);
                uptake.setPower(0);
                blocker.setPosition(BLOCKER_CLOSED);
            }
            prevB = bButton;

            // ========================================================================
            // SHOOTING STATE MACHINE
            // ========================================================================

            switch (shootingState) {
                case IDLE:
                    // Do nothing, waiting for trigger
                    break;

                case STOPPING_INTAKE:
                    // Step 1: Stop intake and uptake
                    intake.setPower(0);
                    uptake.setPower(0);
                    blocker.setPosition(BLOCKER_CLOSED);

                    // Move to next state immediately
                    shootingState = ShootingState.SPINNING_UP;
                    stateStartTime = System.currentTimeMillis();
                    break;

                case SPINNING_UP:
                    // Step 2: Spin flywheel to target RPM
                    spinFlywheelTo(targetRpm);

                    // Step 3: Wait until flywheel reaches target speed
                    if (isFlywheelAtSpeed(targetRpm)) {
                        shootingState = ShootingState.SHOOTING;
                        stateStartTime = System.currentTimeMillis();
                    }

                    // Timeout after 3 seconds if can't reach speed
                    if (System.currentTimeMillis() - stateStartTime > 3000) {
                        shootingState = ShootingState.FINISHING;
                        stateStartTime = System.currentTimeMillis();
                    }
                    break;

                case SHOOTING:
                    // Step 4: Open blocker servo
                    blocker.setPosition(BLOCKER_OPEN);

                    // Step 5: Restart intake and uptake
                    intake.setPower(0.8);
                    uptake.setPower(0.8);

                    // Keep flywheel at speed
                    spinFlywheelTo(targetRpm);

                    // Step 6: Hold for 1-2 seconds
                    if (System.currentTimeMillis() - stateStartTime >= SHOOT_DURATION_MS) {
                        shootingState = ShootingState.FINISHING;
                        stateStartTime = System.currentTimeMillis();
                    }
                    break;

                case FINISHING:
                    // Step 7: Stop flywheel and close blocker
                    stopFlywheel();
                    blocker.setPosition(BLOCKER_CLOSED);
                    intake.setPower(0);
                    uptake.setPower(0);

                    // Return to idle
                    shootingState = ShootingState.IDLE;
                    break;
            }

            // ========================================================================
            // TELEMETRY
            // ========================================================================

            telemetry.addLine("=== SHOOTING STATUS ===");
            telemetry.addData("State", shootingState.toString());
            telemetry.addData("Goal", targetBlueGoal ? "BLUE" : "RED");
            telemetry.addData("Distance", "%.2f m", distanceToGoal);
            telemetry.addData("Target RPM", "%.0f", calculatedRpm);
            telemetry.addData("Actual RPM", "%.0f", ticksPerSecToRpm(flywheel.getVelocity()));
            telemetry.addData("At Speed", isFlywheelAtSpeed(targetRpm) ? "YES" : "NO");
            telemetry.addLine();
            telemetry.addLine("=== POSITION ===");
            telemetry.addData("X Position", currentPose.position.x);
            telemetry.addData("Y Position", currentPose.position.y);
            telemetry.addData("Heading (deg)", Math.toDegrees(currentPose.heading.toDouble()));
            telemetry.addLine();
            telemetry.addLine("=== CONTROLS ===");
            telemetry.addLine("RB: Shoot | B: Emergency Stop");
            telemetry.addLine("D-Pad Left: Toggle Goal");
            telemetry.update();

            // ========================================================================
            // MOVEMENT - Field-Centric with Road Runner Integration
            // ========================================================================
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = (gamepad1.right_stick_x) * 0.75;

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.right_stick_button && gamepad1.left_stick_button) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Use Road Runner's setDrivePowers to maintain localization
            PoseVelocity2d driveVelocity = new PoseVelocity2d(
                new Vector2d(rotY, rotX),  // Linear velocity (forward/strafe)
                rx                          // Angular velocity (rotation)
            );
            drive.setDrivePowers(driveVelocity);
        }
    }

    // ========================================================================
    // SHOOTING HELPER METHODS
    // ========================================================================

    /**
     * Compute target RPM using linear regression
     * Formula: RPM = slope * distance + intercept
     */
    private double computeRPMLinearRegression(double distanceMeters) {
        double rpm = REGRESSION_SLOPE * distanceMeters + REGRESSION_INTERCEPT;
        return clamp(rpm, MIN_RPM, MAX_RPM);
    }

    /**
     * Command flywheel to spin at target RPM
     */
    private void spinFlywheelTo(double rpm) {
        flywheel.setVelocity(rpmToTicksPerSec(rpm));
    }

    /**
     * Stop flywheel
     */
    private void stopFlywheel() {
        flywheel.setPower(0.0);
    }

    /**
     * Check if flywheel is at target speed (within tolerance)
     */
    private boolean isFlywheelAtSpeed(double targetRpm) {
        double currentRpm = ticksPerSecToRpm(flywheel.getVelocity());
        return Math.abs(currentRpm - targetRpm) <= SPEED_TOL_RPM;
    }

    /**
     * Convert RPM to encoder ticks per second
     */
    private static double rpmToTicksPerSec(double rpm) {
        return rpm * (TICKS_PER_REV / 60.0);
    }

    /**
     * Convert encoder ticks per second to RPM
     */
    private static double ticksPerSecToRpm(double tps) {
        return tps * (60.0 / TICKS_PER_REV);
    }

    /**
     * Clamp value between min and max
     */
    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
