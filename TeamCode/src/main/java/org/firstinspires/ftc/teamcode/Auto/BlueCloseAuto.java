package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.LocalizationHelper;
import org.firstinspires.ftc.teamcode.RRStuff.MecanumDrive;
import org.firstinspires.ftc.teamcode.qualifiers.qualifiersHardwareMap;

@Config
@Autonomous(name = "Close Blue Auto", group = "Autonomous")
public class BlueCloseAuto extends LinearOpMode {
    qualifiersHardwareMap hardware = new qualifiersHardwareMap();
    static double launchX = -24;
    static double launchY = -24;
    static Vector2d launchPose = new Vector2d(launchX,launchY);



    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(-57, -42, Math.toRadians(45));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        mechanisms m = new mechanisms(hardwareMap);

        TrajectoryActionBuilder ShootPreload = drive.actionBuilder(initialPose)
                .splineTo(launchPose,Math.toRadians(45));
                //shoot

        TrajectoryActionBuilder IntakeFirstStack = ShootPreload
                .strafeToLinearHeading(new Vector2d(-12,-22),Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(-12,-48), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(-12,-40), Math.toRadians(-90));

        TrajectoryActionBuilder ShootFirstStack = IntakeFirstStack
                .splineTo(launchPose,Math.toRadians(45));
                //shoot

        TrajectoryActionBuilder IntakeSecondStack = ShootFirstStack
                .strafeToLinearHeading(new Vector2d(13,-22),Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(13,-49), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(13,-40), Math.toRadians(-90));

        TrajectoryActionBuilder ShootSecondStack = IntakeSecondStack
                .splineTo(launchPose,Math.toRadians(45.5));

        TrajectoryActionBuilder leave = ShootSecondStack
                .strafeToLinearHeading(new Vector2d(15,15),Math.toRadians(45));
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

                        intakeFirst,
                        shootSecond,
                        m.shootingSequence2,
                        m.blockerClose(),

                        intakeSecond,
                        shootThird,
                        m.shootingSequence3,
                        m.blockerClose(),

                        Leave

                )
        );

        // Save final pose for TeleOp handoff
        LocalizationHelper.savePoseForTeleOp(drive);
    }
}