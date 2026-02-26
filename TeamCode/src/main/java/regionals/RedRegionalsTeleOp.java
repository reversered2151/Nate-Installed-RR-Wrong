package regionals;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.*;

import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.PedroDriverControlled;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.DriverControlledCommand;

@TeleOp(name = "RED teleop")
public class RedRegionalsTeleOp extends NextFTCOpMode {

    private static final double RED_GOAL_X = 131;
    private static final double RED_GOAL_Y = 137;

    //todo update these to play with for the regression
    private static final double RED_GOAL_X_DISTANCE = 131;
    private static final double RED_GOAL_Y_DISTANCE = 137;

    private boolean flywheelActive = false;
    private double targetHeading = 0;

    private static final Pose RED_AUTO_END = new Pose(114, 90, Math.toRadians(0));
    private static final Pose RED_GOAL = new Pose(110, 134, Math.toRadians(270));
    private static final Pose CENTER = new Pose(72, 72, Math.toRadians(0));

    public RedRegionalsTeleOp() {
        addComponents(
                BulkReadComponent.INSTANCE,
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(flywheel.INSTANCE, intake.INSTANCE, uptake.INSTANCE, servo.INSTANCE),
                BindingsComponent.INSTANCE
        );
    }

    @Override
    public void onInit() {
        servo.INSTANCE.close().schedule();

        if (PoseStorage.currentPose != null) {
            PedroComponent.follower().setPose(PoseStorage.currentPose);
        } else {
            PedroComponent.follower().setPose(CENTER);  // default to start position
        }
    }

    @Override
    public void onWaitForStart() {
        Pose pose = PedroComponent.follower().getPose();
        telemetry.addLine("RED TELE");
        telemetry.addLine();

        telemetry.addData("Position", "X: %.1f, Y: %.1f, H: %.1f°",
                pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
        telemetry.addLine("DPAD: UP=AutoEnd, DOWN=Center, RIGHT=Goal");
        telemetry.update();

        if (gamepad1.dpad_up) {
            PedroComponent.follower().setPose(RED_AUTO_END);
        } else if (gamepad1.dpad_down) {
            PedroComponent.follower().setPose(CENTER);
        } else if (gamepad1.dpad_right) {
            PedroComponent.follower().setPose(RED_GOAL);
        }
    }

    @Override
    public void onStartButtonPressed() {
        // Field centric driving
        DriverControlledCommand driverControlled = new PedroDriverControlled(
                Gamepads.gamepad1().leftStickY().negate(),
                Gamepads.gamepad1().leftStickX().negate(),
                Gamepads.gamepad1().rightStickX().map(value -> -value * 0.7),
                false
        );
        driverControlled.schedule();

        // Both stick buttons: Reset heading (current facing = new forward)
        Gamepads.gamepad1().leftStickButton().and(Gamepads.gamepad1().rightStickButton())
                .whenBecomesTrue(
                        new LambdaCommand("Reset Heading")
                                .setStart(() -> {
                                    Pose pose = PedroComponent.follower().getPose();
                                    PedroComponent.follower().setPose(new Pose(pose.getX(), pose.getY(), Math.toRadians(0)));
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

        Gamepads.gamepad1().y().whenBecomesTrue(
                new LambdaCommand("Turn to Goal")
                        .setStart(() -> {
                            Pose pose = PedroComponent.follower().getPose();
                            targetHeading = calculateHeadingToGoal(pose);
                            PedroComponent.follower().turnTo(targetHeading);
                        })
                        .setIsDone(() -> {
                            double current = PedroComponent.follower().getPose().getHeading();
                            double diff = targetHeading - current;
                            while (diff > Math.PI) diff -= 2 * Math.PI;
                            while (diff < -Math.PI) diff += 2 * Math.PI;
                            return Math.abs(diff) < Math.toRadians(5);
                        })
                        .setStop((interrupted) -> {
                            PedroComponent.follower().breakFollowing();
                            PedroComponent.follower().startTeleopDrive();
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
                        .setStart(() -> PedroComponent.follower().setPose(new Pose(72, 72, Math.toRadians(0))))
                        .setIsDone(() -> true)
        );
    }

    private double calculateHeadingToGoal(Pose pose) {
        double dx = RED_GOAL_X - pose.getX();
        double dy = RED_GOAL_Y - pose.getY();
        double heading = Math.atan2(dy, dx);

        // Normalize to [-PI, PI]
        while (heading > Math.PI) heading -= 2 * Math.PI;
        while (heading < -Math.PI) heading += 2 * Math.PI;

        return heading;
    }

    @Override
    public void onUpdate() {
        Pose pose = PedroComponent.follower().getPose();
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
        telemetry.addData("X", "%.2f", pose.getX());
        telemetry.addData("Y", "%.2f", pose.getY());
        telemetry.addData("Heading", "%.1f°", Math.toDegrees(pose.getHeading()));
        telemetry.addLine();
        telemetry.addLine("=== CONTROLS ===");
        telemetry.addLine("RT: Toggle Flywheel | RB: Shoot");
        telemetry.addLine("LB: Toggle Intake | B: Emergency Stop");
        telemetry.addData("Intake", intake.INSTANCE.isOn() ? "ON" : "OFF");
        telemetry.update();
    }

    private double getDistanceToGoal() {
        Pose pose = PedroComponent.follower().getPose();
        double dx = RED_GOAL_X_DISTANCE - pose.getX();
        double dy = RED_GOAL_Y_DISTANCE - pose.getY();
        return Math.sqrt(dx * dx + dy * dy) * 0.0254;
    }
}
