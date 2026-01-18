package org.firstinspires.ftc.teamcode.qualifiers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "EmergencyTeleOp", group = "qualifiers")
public class EmergencyTeleOp extends LinearOpMode {

    // Fixed shooting velocity (adjust as needed)
    private static final double FIXED_FLYWHEEL_VELOCITY = 1200.0;

    // Servo positions
    private static final double BLOCKER_OPEN = 0.6;
    private static final double BLOCKER_CLOSED = 0.95;

    // Hardware
    qualifiersHardwareMap hardware = new qualifiersHardwareMap();

    DcMotorEx flywheel;
    DcMotorEx intake;
    DcMotorEx uptake;
    DcMotor fl;
    DcMotor fr;
    DcMotor bl;
    DcMotor br;
    Servo blocker;
    IMU imu;

    // Button debouncing
    private boolean prevRightBumper = false;
    private boolean prevLeftBumper = false;
    private boolean prevB = false;

    // Shooting state
    private boolean isShooting = false;

    @Override
    public void runOpMode() throws InterruptedException {

        hardware.init(hardwareMap);

        flywheel = hardware.flywheel;
        intake = hardware.intake;
        uptake = hardware.uptake;
        blocker = hardware.blocker;
        imu = hardware.imu;

        // Get drive motors directly from hardware map
        fl = hardware.fl;
        fr = hardware.fr;
        bl = hardware.bl;
        br = hardware.br;

        imu = hardware.imu;;

        // Close blocker initially
        blocker.setPosition(BLOCKER_CLOSED);

        telemetry.addLine("=== EMERGENCY TELEOP ===");
        telemetry.addLine("Field-centric drive with fixed velocity");
        telemetry.addData("Fixed Flywheel Velocity", FIXED_FLYWHEEL_VELOCITY);
        telemetry.addLine();
        telemetry.addLine("Controls:");
        telemetry.addLine("LB: Toggle Intake/Uptake");
        telemetry.addLine("RB: Toggle Shooting");
        telemetry.addLine("B: Emergency Stop All");
        telemetry.addLine("Both Sticks: Reset Heading");
        telemetry.addData("Status", "Ready to start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // ========================================================================
            // BUTTON CONTROLS
            // ========================================================================

            boolean rightBumper = gamepad1.right_bumper;
            boolean leftBumper = gamepad1.left_bumper;
            boolean bButton = gamepad1.b;

            // LEFT BUMPER: Toggle intake and uptake
            if (leftBumper && !prevLeftBumper) {
                if (intake.getPower() == 0) {
                    intake.setPower(0.8);
                    uptake.setVelocity(1000);
                } else {
                    intake.setPower(0);
                    uptake.setVelocity(0);
                }
            }
            prevLeftBumper = leftBumper;

            // RIGHT BUMPER: Toggle shooting
            if (rightBumper && !prevRightBumper) {
                isShooting = !isShooting;

                if (isShooting) {
                    // Start shooting
                    flywheel.setVelocity(FIXED_FLYWHEEL_VELOCITY);
                    sleep(1000);
                    blocker.setPosition(BLOCKER_OPEN);
                    intake.setPower(0.8);
                    uptake.setVelocity(1000);
                } else {
                    // Stop shooting
                    flywheel.setPower(0);
                    blocker.setPosition(BLOCKER_CLOSED);
                }
            }
            prevRightBumper = rightBumper;

            // B BUTTON: Emergency stop everything
            if (bButton && !prevB) {
                isShooting = false;
                flywheel.setPower(0);
                intake.setPower(0);
                uptake.setPower(0);
                blocker.setPosition(BLOCKER_CLOSED);
            }
            prevB = bButton;

            // ========================================================================
            // FIELD-CENTRIC MECANUM DRIVE
            // ========================================================================

            // Heading reset - press both stick buttons
            if (gamepad1.right_stick_button && gamepad1.left_stick_button) {
                imu.resetYaw();
            }

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x * 0.75;

            // Get robot heading for field-centric drive
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            // Calculate motor powers
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double flPower = (rotY + rotX + rx) / denominator;
            double blPower = (rotY - rotX + rx) / denominator;
            double frPower = (rotY - rotX - rx) / denominator;
            double brPower = (rotY + rotX - rx) / denominator;

            fl.setPower(flPower);
            bl.setPower(blPower);
            fr.setPower(frPower);
            br.setPower(brPower);

            // ========================================================================
            // TELEMETRY
            // ========================================================================

            telemetry.addLine("=== EMERGENCY TELEOP ===");
            telemetry.addData("Shooting", isShooting ? "ON" : "OFF");
            telemetry.addData("Flywheel Velocity", "%.0f", flywheel.getVelocity());
            telemetry.addData("Target Velocity", FIXED_FLYWHEEL_VELOCITY);
            telemetry.addData("Intake", intake.getPower() > 0 ? "ON" : "OFF");
            telemetry.addData("Blocker", blocker.getPosition() == BLOCKER_OPEN ? "OPEN" : "CLOSED");
            telemetry.addLine();
            telemetry.addLine("=== FIELD-CENTRIC DRIVE ===");
            telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(botHeading));
            telemetry.addLine();
            telemetry.addLine("=== CONTROLS ===");
            telemetry.addLine("LB: Toggle Intake/Uptake");
            telemetry.addLine("RB: Toggle Shooting");
            telemetry.addLine("B: Emergency Stop All");
            telemetry.addLine("Both Sticks: Reset Heading");
            telemetry.update();
        }
    }
}
