package org.firstinspires.ftc.teamcode.qualifiers;

import static org.firstinspires.ftc.teamcode.PoseStorage.currentPose;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.LocalizationHelper;
import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.RRStuff.MecanumDrive;


@TeleOp(name = "RedTeleOp", group = "qualifiers")
public class RedQualTeleOp extends LinearOpMode {

    // Formula: RPM = REGRESSION_SLOPE * distance_meters + REGRESSION_INTERCEPT
    private static final double REGRESSION_SLOPE = 439.42018;
    private static final double REGRESSION_INTERCEPT = 764.10156;

    // Goal positions on field (in inches, measured from field center)
    private static final double RED_GOAL_X = -70.0;
    private static final double RED_GOAL_Y = 70.0;

    // Flywheel limits
    private static final double MIN_RPM = 500.0;
    private static final double MAX_RPM = 6000.0;
    private static final double SPEED_TOL_RPM = 100.0;  // RPM tolerance for "at speed"
    private static final double TICKS_PER_REV = 28.0;

    // Shooting timing
    private static final long SHOOT_DURATION_MS = 2000;  // Hold shooting for 2 seconds

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

    // Button debouncing
    private boolean prevRightBumper = false;
    private boolean prevB = false;
    private boolean prevLeftBumper = false;
    private boolean prevY = false;

    // One-time rotation to goal
    private boolean isRotatingToGoal = false;
    private double targetRotationAngle = 0.0;
    private static final double ROTATION_KP = 0.8;  // Proportional gain for rotation
    private static final double ROTATION_TOLERANCE_RAD = Math.toRadians(2.0);  // 2 degree tolerance

    qualifiersHardwareMap hardware = new qualifiersHardwareMap();
    MecanumDrive drive;

    DcMotorEx flywheel;
    DcMotorEx intake;
    DcMotorEx uptake;
    DcMotorEx fl;
    DcMotorEx fr;
    DcMotorEx bl;
    DcMotorEx br;

    Servo blocker;

    @Override
    public void runOpMode() throws InterruptedException{

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        hardware.init(hardwareMap);

        // Road Runner drive with starting pose from autonomous
        // If no auto was run, defaults to (0, 0, 0)
        drive = LocalizationHelper.initializeForTeleOp(hardwareMap);

        flywheel = hardware.flywheel;
        intake = hardware.intake;
        uptake = hardware.uptake;
        blocker = hardware.blocker;
        // Use Road Runner's motor instances to avoid conflicts with odometry
        fl = drive.leftFront;
        fr = drive.rightFront;
        bl = drive.leftBack;
        br = drive.rightBack;



        // Manual position selection during init
        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Starting Position", "X: %.2f, Y: %.2f, Heading: %.1f°",
                    PoseStorage.currentPose.position.x,
                    PoseStorage.currentPose.position.y,
                    Math.toDegrees(PoseStorage.currentPose.heading.toDouble()));
            telemetry.addLine();
            telemetry.addLine("=== MANUAL POSITION OVERRIDE ===");
            telemetry.addLine("DPAD_UP: Red Auto");
            telemetry.addLine("DPAD_DOWN: Red Goal");
            telemetry.addLine();
            telemetry.addData("Status", "Ready - Press buttons to override position");
            telemetry.update();

            if (gamepad1.dpad_up) {
                // Blue Auto position
                LocalizationHelper.resetPosition(drive, new Pose2d(-24, 45, Math.toRadians(90)));
                telemetry.addData("Position Override", "Red Auto (-24, 45, 90)");
                telemetry.update();
                sleep(300); // Debounce
            } else if (gamepad1.dpad_down) {
                // Reset to origin
                LocalizationHelper.resetPosition(drive, new Pose2d(-65, 40, Math.toRadians(0)));
                telemetry.addData("Position Override", "Red Goal (-65, 40, 0°)");
                telemetry.update();
                sleep(300); // Debounce
            }
        }

        // Initialize teleop - close blocker and reset heading
        blocker.setPosition(BLOCKER_CLOSED);
        LocalizationHelper.resetPosition(drive, new Pose2d(currentPose.position.x, currentPose.position.y, 0));

        while (opModeIsActive()){

            // Update localization - THIS IS CRITICAL!
            LocalizationHelper.update(drive);

            // Get current pose
            Pose2d currentPose = LocalizationHelper.getCurrentPose(drive);

            if (gamepad1.dpad_up) {
                // Blue Auto position
                LocalizationHelper.resetPosition(drive, new Pose2d(0, 0, Math.toRadians(90)));
                telemetry.addData("Red Center Override", "(0, 0, 90)");
                telemetry.update();
                sleep(300); // Debounce
            }


            // Calculate distance to goal and target RPM
            double goalX = RED_GOAL_X;
            double goalY = RED_GOAL_Y;
            double distanceToGoal = LocalizationHelper.getDistanceToTargetMeters(drive, goalX, goalY);
            double calculatedRpm = computeRPMLinearRegression(distanceToGoal);

            // ========================================================================
            // SHOOTING CONTROLS
            // ========================================================================

            boolean rightBumper = gamepad1.right_bumper;
            boolean leftBumper = gamepad1.left_bumper;
            boolean bButton = gamepad1.b;
            boolean yButton = gamepad1.y;

            // Y BUTTON: Start one-time rotation to goal
            if (yButton && !prevY) {
                // Calculate the target angle to face the goal
                targetRotationAngle = LocalizationHelper.getAngleToTarget(drive, goalX, goalY);
                isRotatingToGoal = true;
            }
            prevY = yButton;

            // LEFT BUMPER: Toggle intake and uptake on/off
            if (leftBumper && !prevLeftBumper && shootingState == ShootingState.IDLE) {
                // Toggle intake/uptake
                if (intake.getPower() == 0) {
                    intake.setPower(0.8);
                    uptake.setPower(0.8);
                } else {
                    intake.setPower(0);
                    uptake.setPower(0);
                }
            }
            prevLeftBumper = leftBumper;

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
//                    spinFlywheelTo(targetRpm);
                    flywheel.setVelocity(targetRpm);

                    // Step 3: Wait until flywheel reaches target speed
                    if (System.currentTimeMillis() - stateStartTime > 1000) {
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
                    intake.setPower(0.7);
                    uptake.setPower(0.7);

                    // Keep flywheel at speed
//                    spinFlywheelTo(targetRpm);
                    flywheel.setVelocity(targetRpm);

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


                    // Return to idle
                    shootingState = ShootingState.IDLE;
                    break;
            }

            // ========================================================================
            // TELEMETRY
            // ========================================================================

            telemetry.addLine("=== SHOOTING STATUS ===");
            telemetry.addData("State", shootingState.toString());
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
            telemetry.addLine("LB: Intake Toggle | RB: Shoot");
            telemetry.addLine("B: Emergency Stop | DPAD_UP: Origin Reset");
            telemetry.addLine("Y: Heading Lock");
            telemetry.addData("Intake Status", intake.getPower() > 0 ? "ON" : "OFF");
            telemetry.addData("Rotating", isRotatingToGoal ? "YES" : "NO");
            if (isRotatingToGoal) {
                double currentHeading = currentPose.heading.toDouble();
                double angleError = targetRotationAngle - currentHeading;
                // Normalize to [-π, π]
                while (angleError > Math.PI) angleError -= 2 * Math.PI;
                while (angleError < -Math.PI) angleError += 2 * Math.PI;
                telemetry.addData("Angle Error", "%.1f deg", Math.toDegrees(angleError));
            }
            telemetry.update();

            // ========================================================================
            // MOVEMENT - Field-Centric with Road Runner Integration
            // ========================================================================
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = (gamepad1.right_stick_x)*.75;

            // One-time rotation to goal
            if (isRotatingToGoal) {
                double currentHeading = currentPose.heading.toDouble();
                double angleError = targetRotationAngle - currentHeading;

                // Normalize to [-π, π] for shortest path
                while (angleError > Math.PI) angleError -= 2 * Math.PI;
                while (angleError < -Math.PI) angleError += 2 * Math.PI;

                // If within tolerance, stop rotating
                if (Math.abs(angleError) < ROTATION_TOLERANCE_RAD) {
                    isRotatingToGoal = false;
                    rx = 0;
                } else {
                    // Apply proportional control to rotation
                    rx = angleError * ROTATION_KP;
                    // Clamp to reasonable values
                    rx = Math.max(-0.75, Math.min(0.75, rx));
                }

                // Driver can still override with right stick
                if (Math.abs(gamepad1.right_stick_x) > 0.1) {
                    isRotatingToGoal = false;
                    rx = (gamepad1.right_stick_x) * 0.75;
                }
            }

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.right_stick_button && gamepad1.left_stick_button) {
                // Reset heading to current pose's heading (effectively resets field-centric reference)
                LocalizationHelper.resetPosition(drive, new Pose2d(currentPose.position.x, currentPose.position.y, 0));
            }

            // Use Road Runner's heading for field-centric drive
            double botHeading = currentPose.heading.toDouble();

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double flPower = (rotY + rotX + rx) / denominator;
            double blPower = (rotY - rotX + rx) / denominator;
            double frPower = (rotY - rotX - rx) / denominator;
            double brPower = (rotY + rotX - rx) / denominator;

            fl.setPower(flPower);
            bl.setPower(blPower);
            fr.setPower(frPower);
            br.setPower(brPower);
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
