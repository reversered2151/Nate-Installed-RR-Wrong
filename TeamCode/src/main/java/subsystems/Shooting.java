package subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;

public class Shooting {


    //todo update these for new bot
    private static final double REGRESSION_SLOPE = 439.42018;
    private static final double REGRESSION_INTERCEPT = 764.10156;
    private static final double MIN_VELOCITY = 500.0;
    private static final double MAX_VELOCITY = 6000.0;

    public static double calculateVelocity(double distanceMeters) {
        double vel = REGRESSION_SLOPE * distanceMeters + REGRESSION_INTERCEPT;
        return Math.max(MIN_VELOCITY, Math.min(MAX_VELOCITY, vel));
    }

    public static Command shoot(double velocity) {
        return new SequentialGroup(

                // 1. Shut off intake and o pen servo
                fullIntake.off(),
                servo.INSTANCE.open(),

                // 2. Spin flywheel up to speed (max 1s timeout)
                flywheel.INSTANCE.runUntilAtSpeedOrTimeout(velocity, 2.0),

                // 3. Turn on intake then run flywheel for 1s while balls feed
                fullIntake.on(),
                flywheel.INSTANCE.runForDuration(velocity, 1.5),

                // 4. Stop everything and close
                flywheel.INSTANCE.stopImmediate(),
                servo.INSTANCE.close()
        );
    }

    public static Command autoShoot(double velocity) {
        return new SequentialGroup(
                new ParallelGroup(
                        fullIntake.off(),
                        servo.INSTANCE.open()
                ),
                new Delay(0.3),
                fullIntake.on(),
                new Delay(0.5),
                servo.INSTANCE.close()
        );
    }


    public static Command feedAndShoot() {
        return new SequentialGroup(
                fullIntake.off(),
                servo.INSTANCE.open(),
                new Delay(.5),
                fullIntake.on(),
                new Delay(1.5),
                servo.INSTANCE.close()
        );
    }

    public static Command emergencyStop() {
        return new ParallelGroup(
                flywheel.INSTANCE.stop(),
                intake.INSTANCE.turnOff(),
                uptake.INSTANCE.turnOff()
        );
    }
}