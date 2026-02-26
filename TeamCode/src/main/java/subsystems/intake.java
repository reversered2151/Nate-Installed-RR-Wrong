package subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

public class intake implements Subsystem {
    public static final intake INSTANCE = new intake();
    private intake() { }

    private MotorEx motor = new MotorEx("intake")
            .reversed();

    private boolean isOn = false;

    public Command turnOn(double power) {
        return new LambdaCommand("Intake On")
                .setStart(() -> {
                    motor.setPower(power);
                    isOn = true;
                })
                .setIsDone(() -> true)
                .requires(this);
    }

    public Command turnOn() {
        return turnOn(.8);
    }

    public Command turnOff() {
        return new LambdaCommand("Intake Off")
                .setStart(() -> {
                    motor.setPower(0);
                    isOn = false;
                })
                .setIsDone(() -> true)
                .requires(this);
    }

    public boolean isOn() {
        return isOn;
    }

    @Override
    public void periodic() { }
}