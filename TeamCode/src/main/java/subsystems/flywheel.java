package subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

public class flywheel implements Subsystem {
    public static final flywheel INSTANCE = new flywheel();
    private flywheel() { }

    private MotorEx motor = new MotorEx("flywheel").reversed();

    private double currentGoalVelocity = 0;
    private static final double SPEED_TOLERANCE = 200;

    private static double kV = 0.00041;
    private static double kP = 0.000001;

    /*----------------------------------------------------------------------------------------------------------------------------------*/

    // Run until at speed then finish
    public Command runUntilAtSpeed(double velocity) {
        return new LambdaCommand("RunUntilAtSpeed(" + velocity + ")")
                .setStart(() -> currentGoalVelocity = velocity)
                .setIsDone(() -> isAtSpeed(velocity))
                .requires(this);
    }

    // Run until at speed OR timeout, whichever comes first
    public Command runUntilAtSpeedOrTimeout(double velocity, double timeoutSeconds) {
        final long[] startTime = {0};
        return new LambdaCommand("SpinUp(" + velocity + ")")
                .setStart(() -> {
                    currentGoalVelocity = velocity;
                    startTime[0] = System.nanoTime();
                })
                .setIsDone(() -> isAtSpeed(velocity) ||
                    (System.nanoTime() - startTime[0]) >= (long)(timeoutSeconds * 1e9))
                .requires(this);
    }

    // Run at velocity for a fixed duration then finish
    public Command runForDuration(double velocity, double durationSeconds) {
        final long[] startTime = {0};
        return new LambdaCommand("RunFor(" + velocity + ")")
                .setStart(() -> {
                    currentGoalVelocity = velocity;
                    startTime[0] = System.nanoTime();
                })
                .setIsDone(() -> (System.nanoTime() - startTime[0]) >= (long)(durationSeconds * 1e9))
                .requires(this);
    }

    /*----------------------------------------------------------------------------------------------------------------------------------*/

    // Check against current goal
    public boolean isAtSpeed() {
        double currentVelocity = motor.getState().getVelocity();
        return Math.abs(currentVelocity - currentGoalVelocity) <= SPEED_TOLERANCE;
    }

    // Check against specific velocity
    public boolean isAtSpeed(double targetVelocity) {
        double currentVelocity = motor.getState().getVelocity();
        return Math.abs(currentVelocity - targetVelocity) <= SPEED_TOLERANCE;
    }

    public void setTargetVelocity(double velocity) {
        currentGoalVelocity = velocity;
    }

    public double getGoalVelocity() {
        return currentGoalVelocity;
    }

    // For telemetry
    public double getCurrentVelocity() {
        return motor.getState().getVelocity();
    }

    /*----------------------------------------------------------------------------------------------------------------------------------*/

    // Run forever until stop
    public Command runAtVelocity(double velocity) {
        return new LambdaCommand("RunAtVelocity(" + velocity + ")")
                .setStart(() -> currentGoalVelocity = velocity)
                .setIsDone(() -> false)
                .requires(this);
    }

    public Command stop() {
        return runAtVelocity(0);
    }

    // Set velocity to 0 and finish immediately (for use in sequences)
    public Command stopImmediate() {
        return new LambdaCommand("StopFlywheel")
                .setStart(() -> currentGoalVelocity = 0)
                .setIsDone(() -> true)
                .requires(this);
    }

    /*----------------------------------------------------------------------------------------------------------------------------------*/

    private double lastPower = 0;

    public double getLastPower() {
        return lastPower;
    }

    @Override
    public void periodic() {
        double currentVelocity = motor.getState().getVelocity();
        double error = currentGoalVelocity - currentVelocity;
        lastPower = kV * currentGoalVelocity + kP * error;
        motor.setPower(lastPower);
    }
}