package subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

public class uptake implements Subsystem {
    public static final uptake INSTANCE = new uptake();
    private uptake() { }

    private MotorEx motor = new MotorEx("uptake").brakeMode();

    public Command turnOn(double power) {
        return new LambdaCommand("Uptake On")
                .setStart(() -> motor.setPower(power))
                .setIsDone(() -> true)
                .requires(this);
    }

    public Command turnOn() {
        return turnOn(0.8);
    }

    public Command turnOff() {
        return new LambdaCommand("Uptake Off")
                .setStart(() -> motor.setPower(0))
                .setIsDone(() -> true)
                .requires(this);
    }

    @Override
    public void periodic() { }
}