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

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name = "Close Red Auto", group = "Autonomous")
public class RedCloseAuto extends LinearOpMode {
    static double launchX = -24;
    static double launchY = 24;
    static Vector2d launchPose = new Vector2d(launchX,launchY);

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(-57, 42, Math.toRadians(45));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        //Claw claw = new Claw(hardwareMap);
        //Lift lift = new Lift(hardwareMap);

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .lineToX(launchX)
                //Scan for which artifacts and shoot (waitSeconds is placeholder for shooting)
                .waitSeconds(4)
                .strafeToLinearHeading(new Vector2d(-12,32),Math.toRadians(-270))
                //Pick up artifacts
                .strafeToLinearHeading(new Vector2d(-12,52), Math.toRadians(-270))
                .strafeToLinearHeading(new Vector2d(-12,40), Math.toRadians(-270))
                .splineTo(launchPose,Math.toRadians(135))
                .waitSeconds(4)
                .strafeToLinearHeading(new Vector2d(12,32),Math.toRadians(-270))
                ///Pick up artifacts
                .strafeToLinearHeading(new Vector2d(12,52), Math.toRadians(-270))
                .strafeToLinearHeading(new Vector2d(12,40), Math.toRadians(-270))
                .splineTo(launchPose,Math.toRadians(135))
                .waitSeconds(4);

        // actions that need to happen on init; for instance, a claw tightening.

        //init loop
        while (!isStopRequested() && !opModeIsActive()) {

        }

        telemetry.addData("hi bozo ", 67);
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        Action trajectoryActionChosen;
        trajectoryActionChosen = tab1.build();
        Actions.runBlocking(
                new SequentialAction(
                        trajectoryActionChosen
                )
        );
    }
}
