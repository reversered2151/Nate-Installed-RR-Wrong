package regionals;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.RRStuff.MecanumDrive;

import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import subsystems.Shooting;
import subsystems.flywheel;
import subsystems.fullIntake;
import subsystems.intake;
import subsystems.servo;
import subsystems.uptake;

@TeleOp(name = "RED teleop")
public class RedRegionalsTeleOp extends NextFTCOpMode {

    // Goal coordinates (converted: Pedro - 72)
    private static final double RED_GOAL_X = 59;
    private static final double RED_GOAL_Y = 65;

    //todo update these to play with for the regression
    private static final double RED_GOAL_X_DISTANCE = 59;
    private static final double RED_GOAL_Y_DISTANCE = 65;

    private boolean flywheelActive = false;
    private double targetHeading = 0;
    private boolean turningToGoal = false;

    // Preset poses (converted: Pedro - 72)
    private static final Pose2d RED_AUTO_END = new Pose2d(42, 18, Math.toRadians(0));
    private static final Pose2d RED_GOAL = new Pose2d(38, 62, Math.toRadians(270));
    private static final Pose2d CENTER = new Pose2d(0, 0, Math.toRadians(0));

    private MecanumDrive drive;

    public RedRegionalsTeleOp() {
        addComponents(
                BulkReadComponent.INSTANCE,
                new SubsystemComponent(flywheel.INSTANCE, intake.INSTANCE, uptake.INSTANCE, servo.INSTANCE),
                BindingsComponent.INSTANCE
        );
    }

    @Override
    public void onInit() {
        servo.INSTANCE.close().schedule();

        Pose2d initialPose;
        if (PoseStorage.currentPose != null) {
            initialPose = PoseStorage.currentPose;
        } else {
            initialPose = CENTER;
        }
        drive = new MecanumDrive(hardwareMap, initialPose);
    }

    @Override
    public void onWaitForStart() {
        drive.updatePoseEstimate();
        Pose2d pose = drive.localizer.getPose();
        telemetry.addLine("RED TELE");
        telemetry.addLine();

        telemetry.addData("Position", "X: %.1f, Y: %.1f, H: %.1f°",
                pose.position.x, pose.position.y, Math.toDegrees(pose.heading.toDouble()));
        telemetry.addLine("DPAD: UP=AutoEnd, DOWN=Center, RIGHT=Goal");
        telemetry.update();

        if (gamepad1.dpad_up) {
            drive.localizer.setPose(RED_AUTO_END);
        } else if (gamepad1.dpad_down) {
            drive.localizer.setPose(CENTER);
        } else if (gamepad1.dpad_right) {
            drive.localizer.setPose(RED_GOAL);
        }
    }

    @Override
    public void onStartButtonPressed() {
        // Both stick buttons: Reset heading (current facing = new forward)
        Gamepads.gamepad1().leftStickButton().and(Gamepads.gamepad1().rightStickButton())
                .whenBecomesTrue(
                        new LambdaCommand("Reset Heading")
                                .setStart(() -> {
                                    Pose2d pose = drive.localizer.getPose();
                                    drive.localizer.setPose(new Pose2d(
                                            pose.position, Math.toRadians(0)));
                                })
                                .setIsDone(() -> true)
                );

        // RIGHT TRIGGER: Toggle flywheel
        Gamepads.gamepad1().rightTrigger().greaterThan(0.5).whenBecomesTrue(
                new LambdaCommand("Toggle Flywheel")
                        .setStart(() -> {
                            flywheelActive = !flywheelActive;
                            if (!flywheelActive) {
                                flywheel.INSTANCE.setTargetVelocity(0);
                            }
                        })
                        .setIsDone(() -> true)
        );

        // RIGHT BUMPER: Feed and shoot
        Gamepads.gamepad1().rightBumper().whenBecomesTrue(
                new LambdaCommand("Feed and Shoot")
                        .setStart(() -> Shooting.feedAndShoot().schedule())
                        .setIsDone(() -> true)
        );

        // LEFT BUMPER: Toggle intake/uptake
        Gamepads.gamepad1().leftBumper().whenBecomesTrue(
                new LambdaCommand("Toggle Feed")
                        .setStart(() -> {
                            if (intake.INSTANCE.isOn()) {
                                fullIntake.off().schedule();
                            } else {
                                fullIntake.on().schedule();
                            }
                        })
                        .setIsDone(() -> true)
        );

        // Y BUTTON: Turn to goal
        Gamepads.gamepad1().y().whenBecomesTrue(
                new LambdaCommand("Turn to Goal")
                        .setStart(() -> {
                            Pose2d pose = drive.localizer.getPose();
                            targetHeading = calculateHeadingToGoal(pose);
                            turningToGoal = true;
                        })
                        .setIsDone(() -> {
                            double current = drive.localizer.getPose().heading.toDouble();
                            double diff = targetHeading - current;
                            while (diff > Math.PI) diff -= 2 * Math.PI;
                            while (diff < -Math.PI) diff += 2 * Math.PI;
                            return Math.abs(diff) < Math.toRadians(5);
                        })
                        .setStop((interrupted) -> {
                            turningToGoal = false;
                        })
        );

        // B BUTTON: Emergency stop
        Gamepads.gamepad1().b().whenBecomesTrue(
                new LambdaCommand("Emergency Stop")
                        .setStart(() -> {
                            flywheelActive = false;
                            Shooting.emergencyStop().schedule();
                        })
                        .setIsDone(() -> true)
        );

        // DPAD UP: Reset position
        Gamepads.gamepad1().dpadUp().whenBecomesTrue(
                new LambdaCommand("Reset Position")
                        .setStart(() -> drive.localizer.setPose(
                                new Pose2d(0, 0, Math.toRadians(0))))
                        .setIsDone(() -> true)
        );
    }

    private double calculateHeadingToGoal(Pose2d pose) {
        double dx = RED_GOAL_X - pose.position.x;
        double dy = RED_GOAL_Y - pose.position.y;
        double heading = Math.atan2(dy, dx);

        // Normalize to [-PI, PI]
        while (heading > Math.PI) heading -= 2 * Math.PI;
        while (heading < -Math.PI) heading += 2 * Math.PI;

        return heading;
    }

    @Override
    public void onUpdate() {
        drive.updatePoseEstimate();
        Pose2d pose = drive.localizer.getPose();

        // Field-centric driving
        double y = -gamepad1.left_stick_y;   // forward
        double x = -gamepad1.left_stick_x;   // strafe
        double rx = -gamepad1.right_stick_x * 0.7;  // rotation

        if (turningToGoal) {
            // P-controller for turn to goal
            double current = pose.heading.toDouble();
            double error = targetHeading - current;
            while (error > Math.PI) error -= 2 * Math.PI;
            while (error < -Math.PI) error += 2 * Math.PI;
            double turnPower = error * 0.8;
            turnPower = Math.max(-0.5, Math.min(0.5, turnPower));
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), turnPower));
        } else {
            // Field-centric rotation
            double heading = pose.heading.toDouble();
            double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
            double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);

            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(rotY, rotX), rx));
        }

        // Flywheel velocity control
        double distance = getDistanceToGoal();
        double calculatedVelocity = Shooting.calculateVelocity(distance);

        if (flywheelActive) {
            double velocity = Shooting.calculateVelocity(distance);
            flywheel.INSTANCE.setTargetVelocity(velocity);
        }

        telemetry.addLine("=== SHOOTING ===");
        telemetry.addData("Flywheel", flywheelActive ? "ACTIVE" : "OFF");
        telemetry.addData("Distance to Goal", "%.2f m", distance);
        telemetry.addData("Calculated Velocity", "%.0f", calculatedVelocity);
        telemetry.addData("Current Velocity", "%.0f", flywheel.INSTANCE.getCurrentVelocity());
        telemetry.addData("At Speed", flywheel.INSTANCE.isAtSpeed() ? "YES" : "NO");
        telemetry.addLine();
        telemetry.addLine("=== POSITION ===");
        telemetry.addData("X", "%.2f", pose.position.x);
        telemetry.addData("Y", "%.2f", pose.position.y);
        telemetry.addData("Heading", "%.1f°", Math.toDegrees(pose.heading.toDouble()));
        telemetry.addLine();
        telemetry.addLine("=== CONTROLS ===");
        telemetry.addLine("RT: Toggle Flywheel | RB: Shoot");
        telemetry.addLine("LB: Toggle Intake | B: Emergency Stop");
        telemetry.addData("Intake", intake.INSTANCE.isOn() ? "ON" : "OFF");
        telemetry.update();
    }

    private double getDistanceToGoal() {
        Pose2d pose = drive.localizer.getPose();
        double dx = RED_GOAL_X_DISTANCE - pose.position.x;
        double dy = RED_GOAL_Y_DISTANCE - pose.position.y;
        return Math.sqrt(dx * dx + dy * dy) * 0.0254;
    }
}
