
package org.firstinspires.ftc.teamcode.qualifiers;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@TeleOp(name = "Testing", group = "qualifiers")
public class Testing extends LinearOpMode {

    qualifiersHardwareMap hardware = new qualifiersHardwareMap();


    private double targetRpm = 0.0;
    private double blockerpos = .95;

    boolean startup = false;
    boolean lb_prev = false;
    boolean rb_prev = false;


    DcMotorEx flywheel;
    DcMotorEx intake;
    DcMotorEx uptake;
    DcMotorEx fl;
    DcMotorEx fr;
    DcMotorEx bl;
    DcMotorEx br;

    Servo blocker;


    @Override
    public void runOpMode() throws InterruptedException {
        //init stuff
        hardware.init(hardwareMap);

        flywheel = hardware.flywheel;
        intake = hardware.intake;
        uptake = hardware.uptake;
        blocker = hardware.blocker;
        fl = hardware.fl;
        fr = hardware.fr;
        bl = hardware.bl;
        br = hardware.br;

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            final boolean lb = gamepad1.left_bumper;
            final boolean rb = gamepad1.right_bumper;


            if (lb && !lb_prev) {
                //targetRpm += 500;
                targetRpm += 10;
            }

            if (rb && !rb_prev) {
                targetRpm = 1000;
            }

            if (gamepad1.y) {
                intake.setPower(.8);
                uptake.setPower(.8);
            } else if (gamepad1.a) {
                intake.setPower(0);
                uptake.setPower(0);
            }


            if (gamepad1.dpad_up) {
                blockerpos = .95;
                //servo closed
            }
            if (gamepad1.dpad_down) {
                blockerpos = .6;
                //servo open
            }

            flywheel.setVelocity(targetRpm);
            blocker.setPosition(blockerpos);
            telemetry.addLine("Controls: LB = increase speed, RB = reset speed");
            telemetry.addLine("Target RPM:" + targetRpm);
            telemetry.addLine("Actual RPM:" + flywheel.getVelocity());
            telemetry.update();

            // Update previous button states
            lb_prev = lb;
            rb_prev = rb;


            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = (gamepad1.right_stick_x)*.75;

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
}