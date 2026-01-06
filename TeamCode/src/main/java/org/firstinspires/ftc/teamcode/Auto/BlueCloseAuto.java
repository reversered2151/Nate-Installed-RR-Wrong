package org.firstinspires.ftc.teamcode.Auto;
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.qualifiers.qualifiersHardwareMap;

@Config
@Autonomous(name = "Close Blue Auto", group = "Autonomous")
public class BlueCloseAuto extends LinearOpMode {
    qualifiersHardwareMap hardware = new qualifiersHardwareMap();
    static double launchX = -16;
    static double launchY = -16;
    static Vector2d launchPose = new Vector2d(launchX,launchY);



    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(-56, -43, Math.toRadians(45));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        mechanisms m = new mechanisms(hardwareMap);

        TrajectoryActionBuilder ShootPreload = drive.actionBuilder(initialPose)
                .splineTo(launchPose,Math.toRadians(45))
                //shoot
                .waitSeconds(3);

        TrajectoryActionBuilder IntakeFirstStack = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(-8,-26),Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(-8,-48), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(-8,-40), Math.toRadians(-90));

        TrajectoryActionBuilder ShootFirstStack = drive.actionBuilder(initialPose)
                .splineTo(launchPose,Math.toRadians(225))
                //shoot
                .waitSeconds(3);

        TrajectoryActionBuilder IntakeSecondStack = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(16,-26),Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(16,-48), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(16,-40), Math.toRadians(-90));

        TrajectoryActionBuilder ShootSecondStack = drive.actionBuilder(initialPose)
                .splineTo(launchPose,Math.toRadians(225))
                .waitSeconds(3);

        // actions that need to happen on init; for instance, a claw tightening.

        //init loop
        while (!isStopRequested() && !opModeIsActive()) {

        }

        telemetry.addData("hi bozo ", 67);
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        Action shootFirst = ShootPreload.build();
        Action intakeFirst = IntakeFirstStack.build();
        Action shootSecond = ShootFirstStack.build();
        Action intakeSecond = IntakeSecondStack.build();
        Action shootThird = ShootSecondStack.build();

        Actions.runBlocking(
                new SequentialAction(
                        m.blockerClose(), 
                        shootFirst,
                        m.shootingSequence,
                        new SleepAction(0.5),

                        intakeFirst,
                        shootSecond,
                        m.shootingSequence,
                        new SleepAction(0.5),

                        intakeSecond,
                        shootThird,
                        m.shootingSequence

                )
        );
    }
}