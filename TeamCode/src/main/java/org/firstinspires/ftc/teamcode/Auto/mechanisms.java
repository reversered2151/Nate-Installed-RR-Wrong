package org.firstinspires.ftc.teamcode.Auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class mechanisms {
    private DcMotorEx flywheel;
    private Servo blocker;

    private DcMotorEx intake;
    private DcMotorEx uptake;

    public mechanisms(HardwareMap hardwareMap) {
        flywheel = hardwareMap.get(DcMotorEx.class,"flywheel");
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        blocker = hardwareMap.get(Servo.class, "blocker");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        uptake = hardwareMap.get(DcMotorEx.class, "uptake");
        uptake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        uptake.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public class intup implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intake.setPower(.8);
            uptake.setPower(.8);
            return false;
        }
    }

    public Action intup() {
        return new intup();
    }

    public class intupoff implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intake.setPower(0);
            uptake.setPower(0);
            return false;
        }
    }

    public Action intupoff() {
        return new intupoff();
    }


    public class flywheelSpin implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            flywheel.setVelocity(1237);
            return false;
        }
    }
    public Action flywheelSpin() {
        return new flywheelSpin();
    }

    public class flywheelSpin2 implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            flywheel.setVelocity(1230);
            return false;
        }
    }
    public Action flywheelSpin2() {
        return new flywheelSpin2();
    }

    public class flywheelStop implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            flywheel.setVelocity(0);
            return false;
        }
    }
    public Action flywheelStop() {
        return new flywheelStop();
    }

    public class blockerClose implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            blocker.setPosition(0.95);
            return false;
        }
    }
    public Action blockerClose() {
        return new blockerClose();
    }

    public class blockerOpen implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            blocker.setPosition(0.6);
            return false;
        }
    }
    public Action blockerOpen() {
        return new blockerOpen();
    }

    public SequentialAction shootingSequence = new SequentialAction(
        intupoff(),
        flywheelSpin(),
        new SleepAction(1),
        blockerOpen(),
        intup(),
        new SleepAction(1),
        flywheelSpin2(),
        new SleepAction(1),
        flywheelStop(),
        blockerClose()
    );

    public SequentialAction shootingSequence2 = new SequentialAction(
            intupoff(),
            flywheelSpin(),
            new SleepAction(1),
            blockerOpen(),
            intup(),
            new SleepAction(1),
            flywheelSpin2(),
            new SleepAction(1),
            flywheelStop(),
            blockerClose()
    );

    public SequentialAction shootingSequence3 = new SequentialAction(
            intupoff(),
            flywheelSpin(),
            new SleepAction(1),
            blockerOpen(),
            intup(),
            new SleepAction(1),
            flywheelSpin2(),
            new SleepAction(1),
            flywheelStop(),
            blockerClose()
    );




}