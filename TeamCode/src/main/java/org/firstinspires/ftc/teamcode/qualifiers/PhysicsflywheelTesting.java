package org.firstinspires.ftc.teamcode.qualifiers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.LocalizationHelper;
import org.firstinspires.ftc.teamcode.RRStuff.MecanumDrive;

@Disabled
@TeleOp(name = "Flywheel Testing (Localization)", group = "qualifiers")
public class PhysicsflywheelTesting extends LinearOpMode {

    // ========================================================================
    // CONFIGURATION SECTION - ADJUST THESE VALUES FOR OUR ROBOT
    // ========================================================================
    private static final double WHEEL_DIAMETER_M = 0.090;  // 90mm (put in meters)

    //direct drive
    private static final double GEAR_RATIO_WHEEL_PER_MOTOR = 1.0;

    private static final double TICKS_PER_REV = 28.0;

    // TODO: TUNE THIS - Effectiveness coefficient (accounts for flap wheels, friction, slip)
    // Start with 0.8 and adjust based on testing:
    //   - If shots go TOO FAR: INCREAS E this value (try 0.85)
    //   - If shots go TOO SHORT: DECREASE this value (try 0.75)
    // This accounts for energy from flap wheels + energy transfer efficiency
    private static final double KEFF = 0.80;

    // Use a tape measure to measure from floor to where artifact exits shooter
    private static final double H_LAUNCH_M = 0.41;  // 30cm = 0.3m (example)

    private static final double H_GOAL_M = 0.9906;  //~39 inches (put in meters)

    // TODO: Measure your shooter angle above horizontal (in degrees)
    // Use a protractor or angle finder to measure the angle of your shooter
    // 0° = horizontal, 45° = 45 degrees up, etc.
    private static final double THETA_DEG = 70.0;

    // Measure from field center (0,0) to the center of the blue basket
    // Standard FTC field: X = forward/back, Y = left/right
    // Example positions (adjust to actual field measurements):
    private static final double BLUE_GOAL_X = -60.0;  // Adjust based on your field
    private static final double BLUE_GOAL_Y = -60.0;  // Adjust based on your field

    // Measure from field center (0,0) to the center of the red basket
    private static final double RED_GOAL_X = 60.0;   // Adjust based on your field
    private static final double RED_GOAL_Y = 60.0;   // Adjust based on your field

    private static final double MIN_RPM = 500.0;
    private static final double MAX_RPM = 7000.0;


    private static final double MIN_RANGE_M = 0.5;   // 0.5m minimum
    private static final double MAX_RANGE_M = 5.0;   // 5.0m maximum

    private static final double SPEED_TOL_RPM = 100.0;           // RPM tolerance for "at speed"
    private static final long ONE_SHOT_HOLD_MS = 2000;           // Hold at speed for 2 seconds
    private static final long ONE_SHOT_SPINUP_TIMEOUT_MS = 3000; // Timeout if can't reach speed
    private static final double MANUAL_RPM_INCREMENT = 50.0;     // RPM change per button press

    // ========================================================================
    // END CONFIGURATION SECTION
    // ========================================================================

    // Hardware
    private MecanumDrive drive;

    // Goal tracking
    private boolean targetBlueGoal = true;  // true = blue, false = red
    private double goalX = BLUE_GOAL_X;
    private double goalY = BLUE_GOAL_Y;

    // Control states
    private boolean holdMode = false;       // LB toggles hold mode
    private boolean prevLB = false;
    private boolean prevRB = false;
    private boolean prevDpadLeft = false;
    private boolean prevDpadUp = false;
    private boolean prevDpadDown = false;

    private boolean oneShotActive = false;
    private long oneShotStartMs = 0L;
    private long oneShotSpinupStartMs = 0L;

    private double targetRpm = 0.0;
    private boolean manualOverride = false;

    qualifiersHardwareMap hardware = new qualifiersHardwareMap();

    DcMotorEx flywheel;
    DcMotorEx intake;

    @Override
    public void runOpMode() {
        // Initialize telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize hardware
        hardware.init(hardwareMap);

        flywheel = hardware.flywheel;
        intake = hardware.intake;

        // Initialize localization
        drive = LocalizationHelper.initializeForTeleOp(hardwareMap);

        // Display controls
        telemetry.addLine("=== CONTROLS ===");
        telemetry.addLine("LB: Toggle Hold Mode (continuous)");
        telemetry.addLine("RB: One-Shot (2 sec hold)");
        telemetry.addLine("B: Emergency Stop");
        telemetry.addLine("D-Left: Switch Blue/Red Goal");
        telemetry.addLine("D-Up/Down: Manual RPM +/-");
        telemetry.addLine("");
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Update localization
            LocalizationHelper.update(drive);
            Pose2d currentPose = LocalizationHelper.getCurrentPose(drive);

            // Get controls
            boolean lb = gamepad1.left_bumper;
            boolean rb = gamepad1.right_bumper;
            boolean bStop = gamepad1.b;
            boolean dpadLeft = gamepad1.dpad_left;
            boolean dpadUp = gamepad1.dpad_up;
            boolean dpadDown = gamepad1.dpad_down;

            // === B BUTTON: EMERGENCY STOP ===
            if (bStop) {
                holdMode = false;
                oneShotActive = false;
                manualOverride = false;
                stopFlywheel();
            }

            // === D-PAD LEFT: TOGGLE BLUE/RED GOAL ===
            if (dpadLeft && !prevDpadLeft) {
                targetBlueGoal = !targetBlueGoal;
                if (targetBlueGoal) {
                    goalX = BLUE_GOAL_X;
                    goalY = BLUE_GOAL_Y;
                } else {
                    goalX = RED_GOAL_X;
                    goalY = RED_GOAL_Y;
                }
                manualOverride = false;  // Reset override when switching goals
            }
            prevDpadLeft = dpadLeft;

            // === D-PAD UP/DOWN: MANUAL RPM OVERRIDE ===
            if (dpadUp && !prevDpadUp) {
                targetRpm = clamp(targetRpm + MANUAL_RPM_INCREMENT, MIN_RPM, MAX_RPM);
                manualOverride = true;
            }
            prevDpadUp = dpadUp;

            if (dpadDown && !prevDpadDown) {
                targetRpm = clamp(targetRpm - MANUAL_RPM_INCREMENT, MIN_RPM, MAX_RPM);
                manualOverride = true;
            }
            prevDpadDown = dpadDown;

            // Calculate distance and target RPM (if not in manual override)
            double distanceM = calculateDistance(currentPose.position.x, currentPose.position.y, goalX, goalY);

            if (!manualOverride) {
                double rpm = computeTargetRPM(distanceM);
                targetRpm = clamp(rpm, MIN_RPM, MAX_RPM);
            }

            // === LEFT BUMPER: TOGGLE HOLD MODE ===
            if (lb && !prevLB) {
                holdMode = !holdMode;
                if (!holdMode) {
                    stopFlywheel();
                }
                oneShotActive = false;  // Cancel one-shot if LB pressed
                manualOverride = false;  // Reset override
            }
            prevLB = lb;

            // === RIGHT BUMPER: ONE-SHOT MODE ===
            if (rb && !prevRB && !lb) {
                // Start one-shot sequence
                oneShotActive = true;
                oneShotStartMs = 0L;
                oneShotSpinupStartMs = System.currentTimeMillis();
                holdMode = false;
                manualOverride = false;  // Calculate fresh RPM for one-shot

                // Lock in current distance calculation
                double rpm = computeTargetRPM(distanceM);
                targetRpm = clamp(rpm, MIN_RPM, MAX_RPM);
            }
            prevRB = rb;

            // === HOLD MODE ACTIVE ===
            if (holdMode) {
                spinTo(targetRpm);
            }

            // === ONE-SHOT MODE ACTIVE ===
            if (oneShotActive) {
                // Check for timeout
                if (System.currentTimeMillis() - oneShotSpinupStartMs > ONE_SHOT_SPINUP_TIMEOUT_MS) {
                    oneShotActive = false;
                    stopFlywheel();
                } else {
                    // Keep spinning to target
                    spinTo(targetRpm);

                    if (oneShotStartMs == 0L) {
                        // Waiting to reach speed
                        if (atSpeed(targetRpm)) {
                            oneShotStartMs = System.currentTimeMillis();
                        }
                    } else {
                        // Holding at speed
                        if (System.currentTimeMillis() - oneShotStartMs >= ONE_SHOT_HOLD_MS) {
                            oneShotActive = false;
                            stopFlywheel();
                        }
                    }
                }
            }

            // === STOP FLYWHEEL IF NO MODE ACTIVE ===
            if (!holdMode && !oneShotActive) {
                stopFlywheel();
            }

            // === TELEMETRY ===
            displayTelemetry(currentPose, distanceM);

            sleep(20);
        }
    }

    /**
     * Calculate distance from robot to goal using Pythagorean theorem
     * Converts inches to meters
     */
    private double calculateDistance(double robotX, double robotY, double goalX, double goalY) {
        return LocalizationHelper.calculateDistanceMeters(robotX, robotY, goalX, goalY);
    }

    /**
     * Compute required flywheel RPM using ballistic physics
     * Based on projectile motion equations
     */
    private double computeTargetRPM(double dMeters) {
        final double G = 9.80665;  // Gravity (m/s²)
        double theta = Math.toRadians(THETA_DEG);
        double h = H_GOAL_M - H_LAUNCH_M;

        // Projectile motion formula to find required exit velocity
        double denom = 2.0 * Math.pow(Math.cos(theta), 2) * (dMeters * Math.tan(theta) - h);
        if (denom < 0.05) denom = 0.05;  // Safety: avoid division by near-zero

        double vExit = Math.sqrt((G * dMeters * dMeters) / denom);  // m/s

        // Convert exit velocity to wheel surface speed
        // Account for KEFF (flap wheels give pre-velocity, not 100% efficient transfer)
        double wheelRPM = (60.0 * vExit) / (KEFF * Math.PI * WHEEL_DIAMETER_M);

        // Convert wheel RPM to motor RPM
        return wheelRPM / GEAR_RATIO_WHEEL_PER_MOTOR;
    }

    /**
     * Command flywheel to spin at target RPM
     */
    private void spinTo(double rpm) {
        flywheel.setVelocity(rpmToTicksPerSec(rpm));
    }

    /**
     * Stop flywheel
     */
    private void stopFlywheel() {
        flywheel.setPower(0.0);
        targetRpm = 0.0;
    }

    /**
     * Check if flywheel is at target speed (within tolerance)
     */
    private boolean atSpeed(double rpmTarget) {
        double rpmNow = ticksPerSecToRpm(flywheel.getVelocity());
        return Math.abs(rpmNow - rpmTarget) <= SPEED_TOL_RPM;
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

    /**
     * Display comprehensive telemetry
     */
    private void displayTelemetry(Pose2d pose, double distanceM) {
        telemetry.addLine("=== FLYWHEEL CONTROL ===");

        // Mode status
        String mode = "OFF";
        if (holdMode) mode = "HOLD (LB)";
        if (oneShotActive) {
            if (oneShotStartMs == 0L) {
                mode = "ONE-SHOT (Spinning Up)";
            } else {
                long remaining = (ONE_SHOT_HOLD_MS - (System.currentTimeMillis() - oneShotStartMs)) / 1000;
                mode = "ONE-SHOT (Hold " + remaining + "s)";
            }
        }
        telemetry.addData("Mode", mode);

        if (manualOverride) {
            telemetry.addData("Override", "MANUAL");
        }

        // Goal tracking
        telemetry.addData("Goal", targetBlueGoal ? "BLUE" : "RED");

        // Position
        telemetry.addLine("");
        telemetry.addLine("=== POSITION ===");
        telemetry.addData("X", "%.1f in", pose.position.x);
        telemetry.addData("Y", "%.1f in", pose.position.y);
        telemetry.addData("Heading", "%.1f°", Math.toDegrees(pose.heading.toDouble()));

        // Distance to goal
        telemetry.addLine("");
        telemetry.addLine("=== TARGET ===");
        telemetry.addData("Distance", "%.2f m (%.1f in)", distanceM, distanceM / 0.0254);

        // Check if in valid range
        String rangeStatus = "OK";
        if (distanceM < MIN_RANGE_M) rangeStatus = "TOO CLOSE";
        if (distanceM > MAX_RANGE_M) rangeStatus = "TOO FAR";
        telemetry.addData("Range", rangeStatus);

        // Flywheel status
        telemetry.addLine("");
        telemetry.addLine("=== FLYWHEEL ===");
        telemetry.addData("Target RPM", "%.0f", targetRpm);

        double measuredRPM = ticksPerSecToRpm(flywheel.getVelocity());
        telemetry.addData("Actual RPM", "%.0f", measuredRPM);

        boolean isAtSpeed = atSpeed(targetRpm);
        telemetry.addData("At Speed", isAtSpeed ? "✓ YES" : "✗ NO");

        if (targetRpm > 0) {
            double error = Math.abs(measuredRPM - targetRpm);
            telemetry.addData("Error", "%.0f RPM", error);
        }

        telemetry.update();
    }
}
