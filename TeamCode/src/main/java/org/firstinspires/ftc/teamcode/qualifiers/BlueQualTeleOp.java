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
import org.firstinspires.ftc.teamcode.RRStuff.MecanumDrive;


@TeleOp(name = "BlueTeleOp", group = "qualifiers")
public class BlueQualTeleOp extends LinearOpMode {

    // Formula: RPM = REGRESSION_SLOPE * distance_meters + REGRESSION_INTERCEPT
    private static final double REGRESSION_SLOPE = 439.42018;
    private static final double REGRESSION_INTERCEPT = 764.10156;

    // Goal positions on field (in inches, measured from field center)
    private static final double BLUE_GOAL_X = -50.0;
    private static final double BLUE_GOAL_Y = -50.0;

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
    private double targetVelocity = 0.0;

    // Button debouncing
    private boolean prevRightBumper = false;
    private boolean prevDpadLeft = false;
    private boolean prevB = false;
    private boolean prevLeftBumper = false;
    private boolean prevY = false;

    // One-time rotation to goal - PD Controller
    private boolean isRotatingToGoal = false;
    private double prevRotationError = 0.0;  // Previous error for derivative calculation
    private long prevRotationTime = 0;  // Previous time for derivative calculation

    private static final double ROTATION_KP = 0.5;  // Proportional gain
    private static final double ROTATION_KD = 0.8;  // Derivative gain (damping)
    private static final double ROTATION_MIN_POWER = 0.08;  // Minimum power to overcome friction
    private static final double ROTATION_TOLERANCE_RAD = Math.toRadians(6.0);  // 6 degree tolerance
    private static final double ROTATION_MAX_POWER = 0.6;  // Maximum rotation speed

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
                    currentPose.position.x,
                    currentPose.position.y,
                    Math.toDegrees(currentPose.heading.toDouble()));
            telemetry.addLine();
            telemetry.addLine("=== MANUAL POSITION OVERRIDE ===");
            telemetry.addLine("DPAD_UP: Blue Auto");
            telemetry.addLine("DPAD_DOWN: Blue Goal");
            telemetry.addLine();
            telemetry.addData("Status", "Ready - Press buttons to override position");
            telemetry.update();

            if (gamepad1.dpad_up) {
                // Blue Auto END position
                LocalizationHelper.resetPosition(drive, new Pose2d(-24, -45, Math.toRadians(-90)));
                telemetry.addData("Position Override", "Blue Auto END (-24, -45, -90)");
                telemetry.update();
                sleep(300); // Debounce
            }

            else if (gamepad1.dpad_down) {
                // Blue Goal
                LocalizationHelper.resetPosition(drive, new Pose2d(-65, -40, Math.toRadians(0)));
                telemetry.addData("Position Override", "Blue Goal (-65, -40, 0°)");
                telemetry.update();
                sleep(300);
            }

            else if (gamepad1.dpad_left) {
                // Blue Goal
                LocalizationHelper.resetPosition(drive, new Pose2d(70, -31, Math.toRadians(0)));
                telemetry.addData("Position Override", "Far Blue");
                telemetry.update();
                sleep(300);
            }
        }

        // Initialize teleop - close blocker
        // Keep the full pose from auto (including heading) for accurate field-centric drive
        blocker.setPosition(BLOCKER_CLOSED);
        LocalizationHelper.resetPosition(drive, new Pose2d(currentPose.position.x, currentPose.position.y, 0));

        LocalizationHelper.resetPosition(drive, new Pose2d(currentPose.position.x - 24, currentPose.position.y - 45, 0));

        while (opModeIsActive()){

            // Update localization - THIS IS CRITICAL!
            LocalizationHelper.update(drive);

            // Get current pose
            Pose2d currentPose = LocalizationHelper.getCurrentPose(drive);

            if (gamepad1.dpad_up) {
                // Blue Auto position
                LocalizationHelper.resetPosition(drive, new Pose2d(0, 0, Math.toRadians(0)));
                telemetry.addData("Blue Center Override", "(0, 0, 0)");
                telemetry.update();
                sleep(300); // Debounce
            }

            // Calculate distance to goal and target RPM
            double goalX = -BLUE_GOAL_X;
            double goalY = BLUE_GOAL_Y;
            double distanceToGoal = LocalizationHelper.getDistanceToTargetMeters(drive, goalX, goalY);
            double calculatedRpm = computeVelocityLinearRegression(distanceToGoal);

            // ========================================================================
            // SHOOTING CONTROLS
            // ========================================================================

            boolean rightBumper = gamepad1.right_bumper;
            boolean leftBumper = gamepad1.left_bumper;
            boolean bButton = gamepad1.b;
            boolean dpadLeft = gamepad1.dpad_left;
            boolean yButton = gamepad1.y;


            // Y BUTTON: Start one-time rotation to goal
            if (yButton && !prevY) {
                isRotatingToGoal = true;
                // Reset PD controller state
                prevRotationError = 0.0;
                prevRotationTime = System.currentTimeMillis();
            }
            prevY = yButton;

            // LEFT BUMPER: Toggle intake and uptake on/off
            if (leftBumper && !prevLeftBumper && shootingState == ShootingState.IDLE) {
                // Toggle intake/uptake
                if (intake.getPower() == 0) {
                    intake.setPower(0.8);
                    uptake.setVelocity(1000);
                } else {
                    intake.setPower(0);
                    uptake.setVelocity(0);
                }
            }
            prevLeftBumper = leftBumper;

            // RIGHT BUMPER: Start shooting sequence
            if (rightBumper && !prevRightBumper && shootingState == ShootingState.IDLE) {
                shootingState = ShootingState.STOPPING_INTAKE;
                targetVelocity = calculatedRpm;
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

                    blocker.setPosition(BLOCKER_CLOSED);

                    // Move to next state immediately
                    shootingState = ShootingState.SPINNING_UP;
                    stateStartTime = System.currentTimeMillis();
                    break;

                case SPINNING_UP:
                    // Step 2: Spin flywheel to target RPM
                    spinFlywheelTo(targetVelocity);

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
                    intake.setPower(0.8);
                    uptake.setVelocity(1000);

                    // Keep flywheel at speed
                    spinFlywheelTo(targetVelocity);

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
            // MOVEMENT - Field-Centric with Road Runner Integration
            // ========================================================================
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = (gamepad1.right_stick_x)*.75;

            // Rotation control variables for telemetry
            double rotationAngleError = 0;
            double rotationPower = 0;
            double targetAngle = 0;
            double rotationDerivative = 0;

            // One-time rotation to goal - PD Controller
            if (isRotatingToGoal) {
                // Use the helper method that calculates error correctly
                double angleError = LocalizationHelper.getAngleErrorToTarget(drive, goalX, goalY);
                targetAngle = LocalizationHelper.getAngleToTarget(drive, goalX, goalY);
                rotationAngleError = angleError;

                // Calculate derivative (rate of change of error)
                long currentTime = System.currentTimeMillis();
                double dt = (currentTime - prevRotationTime) / 1000.0;  // Convert to seconds

                if (dt > 0.001 && prevRotationTime != 0) {  // Avoid division by zero and skip first loop
                    double derivative = (angleError - prevRotationError) / dt;
                    rotationDerivative = derivative;

                    // PD Control: P term pushes toward target, D term resists fast changes (damping)
                    double pTerm = angleError * ROTATION_KP;
                    double dTerm = derivative * ROTATION_KD;
                    rx = -(pTerm + dTerm);

                    // Apply minimum power only when far from target (>3 degrees) and not moving too fast
                    if (Math.abs(angleError) > Math.toRadians(3.0) && Math.abs(rx) < ROTATION_MIN_POWER) {
                        rx = -Math.signum(angleError) * ROTATION_MIN_POWER;
                    }

                    // Clamp to max rotation power
                    rx = Math.max(-ROTATION_MAX_POWER, Math.min(ROTATION_MAX_POWER, rx));
                    rotationPower = rx;

                    // Update previous values for next loop
                    prevRotationError = angleError;
                    prevRotationTime = currentTime;
                } else {
                    // First loop or invalid dt - use pure P control
                    rx = -(angleError * ROTATION_KP);
                    rx = Math.max(-ROTATION_MAX_POWER, Math.min(ROTATION_MAX_POWER, rx));
                    rotationPower = rx;

                    prevRotationError = angleError;
                    prevRotationTime = currentTime;
                }

                // If within tolerance, stop rotating
                if (Math.abs(angleError) < ROTATION_TOLERANCE_RAD) {
                    isRotatingToGoal = false;
                    rx = 0;
                    prevRotationError = 0.0;  // Reset for next time
                }

                // Driver can still override with right stick
                if (Math.abs(gamepad1.right_stick_x) > 0.1) {
                    isRotatingToGoal = false;
                    rx = (gamepad1.right_stick_x) * 0.75;
                    prevRotationError = 0.0;  // Reset for next time
                }
            }

            // ========================================================================
            // TELEMETRY
            // ========================================================================

            telemetry.addLine("=== SHOOTING STATUS ===");
            telemetry.addData("State", shootingState.toString());
            telemetry.addData("Distance", "%.2f m", distanceToGoal);
            telemetry.addData("Target RPM", "%.0f", calculatedRpm);
            telemetry.addData("Actual RPM", "%.0f", ticksPerSecToRpm(flywheel.getVelocity()));
            telemetry.addData("Velocity", "%.0f", flywheel.getVelocity());
            telemetry.addData("At Speed", isFlywheelAtSpeedNEW(targetVelocity) ? "YES" : "NO");
            telemetry.addLine();
            telemetry.addLine("=== POSITION ===");
            telemetry.addData("X Position", currentPose.position.x);
            telemetry.addData("Y Position", currentPose.position.y);
            telemetry.addData("Heading (deg)", Math.toDegrees(currentPose.heading.toDouble()));
            telemetry.addLine();
            telemetry.addLine("=== CONTROLS ===");
            telemetry.addLine("LB: Intake Toggle | RB: Shoot");
            telemetry.addLine("B: Emergency Stop | DPAD_UP: Origin Reset");
            telemetry.addLine("Y: Rotate to Goal");
            telemetry.addData("Intake Status", intake.getPower() > 0 ? "ON" : "OFF");
            telemetry.addLine();
            telemetry.addLine("=== ROTATION DEBUG ===");
            telemetry.addData("Heading Lock", isRotatingToGoal ? "ACTIVE" : "OFF");
            telemetry.addData("Rotation Error", "%.2f deg", Math.toDegrees(rotationAngleError));
            telemetry.addData("Derivative (d/dt)", "%.3f rad/s", rotationDerivative);
            telemetry.addData("Rotation Power", "%.3f", rotationPower);
            telemetry.addData("Target Heading", "%.1f deg", Math.toDegrees(targetAngle));
            telemetry.addData("Current Heading", "%.1f deg", Math.toDegrees(currentPose.heading.toDouble()));
            telemetry.addData("Raw Heading (rad)", "%.3f", currentPose.heading.toDouble());
            telemetry.addData("Goal X", goalX);
            telemetry.addData("Goal Y", goalY);
            telemetry.update();

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

            // rotX = rotX * 1.1;  // Removed: Road Runner localization tracks actual movement, multiplier causes drift

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

    private double computeVelocityLinearRegression(double distanceMeters){
        double vel = REGRESSION_SLOPE * distanceMeters + REGRESSION_INTERCEPT;
        return clamp(vel, MIN_RPM,MAX_RPM);
    }

    /**
     * Command flywheel to spin at target RPM
     */
    private void spinFlywheelTo(double rpm) {
        flywheel.setVelocity(rpm);
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

    private boolean isFlywheelAtSpeedNEW(double targetRpm) {
        double currentRpm = flywheel.getVelocity();
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