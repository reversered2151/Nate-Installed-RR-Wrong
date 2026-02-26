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

@Config
@Autonomous(name = "Close Red Auto", group = "Autonomous")
public class RedCloseAuto extends LinearOpMode {
    qualifiersHardwareMap hardware = new qualifiersHardwareMap();
    static double launchX = -24;
    public static double launchY = 24;
    public static double launchY2 = 20;
    public static double launchY3 = 18;

    static Vector2d launchPose = new Vector2d(launchX,launchY);



    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(-60, 37, Math.toRadians(0));
        MecanumDrive drive = LocalizationHelper.initializeForAuto(hardwareMap, initialPose);
        mechanisms m = new mechanisms(hardwareMap);

        TrajectoryActionBuilder ShootPreload = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(launchPose,Math.toRadians(45));
                //shoot

        TrajectoryActionBuilder IntakeFirstStack = drive.actionBuilder(new Pose2d(launchX, launchY, Math.toRadians(45)))
                .strafeToLinearHeading(new Vector2d(-7,20),Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-7,54), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-7,40), Math.toRadians(90));

        TrajectoryActionBuilder ShootFirstStack = drive.actionBuilder(new Pose2d( -7, 40, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(launchX, launchY2),Math.toRadians(45));
                //shoot

        TrajectoryActionBuilder IntakeSecondStack = drive.actionBuilder(new Pose2d( launchX, launchY, Math.toRadians(45)))
                .strafeToLinearHeading(new Vector2d(15,20),Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(15,58), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(15,40), Math.toRadians(90));

        TrajectoryActionBuilder ShootSecondStack = drive.actionBuilder(new Pose2d( 15, 40,Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(launchX, launchY3),Math.toRadians(45));

        TrajectoryActionBuilder leave = drive.actionBuilder(new Pose2d(launchX, launchY, Math.toRadians(45)))
                .strafeToLinearHeading(new Vector2d(-24,45),Math.toRadians(90));
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
        Action Leave = leave.build();

        Actions.runBlocking(
                new SequentialAction(
                        m.blockerClose(),
                        shootFirst,
                        m.shootingSequence,
                        m.blockerClose(),
                        new SleepAction(.5),

                        intakeFirst,
                        shootSecond,
                        m.shootingSequence2,
                        m.blockerClose(),
                        new SleepAction(.5),

                        intakeSecond,
                        shootThird,
                        m.shootingSequence3,
                        m.blockerClose(),
                        new SleepAction(.5),

                        Leave

                )
        );

        LocalizationHelper.savePoseForTeleOp(drive);
    }
}
