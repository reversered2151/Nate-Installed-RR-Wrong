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

import org.firstinspires.ftc.teamcode.LocalizationHelper;
import org.firstinspires.ftc.teamcode.RRStuff.MecanumDrive;
import org.firstinspires.ftc.teamcode.qualifiers.qualifiersHardwareMap;

import java.util.Vector;

@Config
@Autonomous(name = "Close Blue Auto", group = "Autonomous")
public class BlueCloseAuto extends LinearOpMode {
    qualifiersHardwareMap hardware = new qualifiersHardwareMap();
    static double launchX = -24;
    static double launchY = -24;
    static Vector2d launchPose = new Vector2d(launchX,launchY);



    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(-60, -37, Math.toRadians(0));
        MecanumDrive drive = LocalizationHelper.initializeForAuto(hardwareMap, initialPose);
        mechanisms m = new mechanisms(hardwareMap);

        TrajectoryActionBuilder ShootPreload = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(launchPose,Math.toRadians(225));
                //shoot

        TrajectoryActionBuilder IntakeFirstStack = drive.actionBuilder(new Pose2d(launchX, launchY, Math.toRadians(225)))
                .strafeToLinearHeading(new Vector2d(-10.5,-20),Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(-10.5,-54), Math.toRadians(-90));

        TrajectoryActionBuilder gate = drive.actionBuilder(new Pose2d( -10.5, -54, Math.toRadians(-90)))
                .strafeToLinearHeading(new Vector2d(0, -54),Math.toRadians(-90));

        TrajectoryActionBuilder ShootFirstStack = drive.actionBuilder(new Pose2d( -7, -40, Math.toRadians(-90)))
                .strafeToLinearHeading(new Vector2d(launchX,launchY+2),Math.toRadians(225));
                //shoot

        TrajectoryActionBuilder IntakeSecondStack = drive.actionBuilder(new Pose2d( launchX, launchY, Math.toRadians(225)))
                .strafeToLinearHeading(new Vector2d(15,-25),Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(15,-58), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(15,-52), Math.toRadians(-90));

        TrajectoryActionBuilder ShootSecondStack = drive.actionBuilder(new Pose2d( 15, -40,Math.toRadians(-90)))
                .strafeToLinearHeading(new Vector2d(launchX, launchY+4),Math.toRadians(225));

        TrajectoryActionBuilder IntakeThirdStack = drive.actionBuilder(new Pose2d( launchX, launchY, Math.toRadians(225)))
                .strafeToLinearHeading(new Vector2d(39,-25),Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(39,-58), Math.toRadians(-90));

        TrajectoryActionBuilder ShootThirdStack = drive.actionBuilder(new Pose2d( 39, -58,Math.toRadians(-90)))
                .strafeToLinearHeading(new Vector2d(launchX, launchY+6),Math.toRadians(225));

        TrajectoryActionBuilder leave = drive.actionBuilder(new Pose2d(launchX, launchY, Math.toRadians(225)))
                .strafeToLinearHeading(new Vector2d(-24,-45),Math.toRadians(-90));
        // actions that need to happen on init; for instance, a claw tightening.

        //init loop
        while (!isStopRequested() && !opModeIsActive()) {

        }

        telemetry.addData("hi bozo ", 67);
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        Action shootPre = ShootPreload.build();
        Action intakeFirst = IntakeFirstStack.build();
        Action shootFirst = ShootFirstStack.build();
        Action intakeSecond = IntakeSecondStack.build();
        Action shootSecond = ShootSecondStack.build();
        Action gateClear = gate.build();
        Action intakeThird = IntakeThirdStack.build();
        Action shootThird = ShootThirdStack.build();
        Action Leave = leave.build();

        Actions.runBlocking(
                new SequentialAction(
                        m.blockerClose(),
                        m.flywheelSpin(),
                        shootPre,
                        m.shootingSequence,
                        m.blockerClose(),
                        m.flywheelStop(),
                        new SleepAction(.5),

                        intakeFirst,
                        gateClear,
                        m.flywheelSpin(),
                        shootFirst,
                        m.shootingSequence2,
                        m.blockerClose(),
                        m.flywheelStop(),
                        new SleepAction(.5),

                        intakeSecond,
                        m.flywheelSpin(),
                        shootSecond,
                        m.shootingSequence3,
                        m.blockerClose(),
                        m.flywheelStop(),
                        new SleepAction(.5),

                        intakeThird,
                        m.flywheelSpin(),
                        shootThird,
                        m.shootingSequence4,
                        m.blockerClose(),
                        m.flywheelStop(),
                        new SleepAction(.5),

                        Leave

                )
        );

        LocalizationHelper.savePoseForTeleOp(drive);
    }
}