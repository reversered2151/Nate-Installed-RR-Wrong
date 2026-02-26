package subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.ParallelGroup;

public class fullIntake {

    public static Command on() {
        return new ParallelGroup(
                intake.INSTANCE.turnOn(),
                uptake.INSTANCE.turnOn()
        );
    }

    public static Command off() {
        return new ParallelGroup(
                intake.INSTANCE.turnOff(),
                uptake.INSTANCE.turnOff()
        );
    }

}