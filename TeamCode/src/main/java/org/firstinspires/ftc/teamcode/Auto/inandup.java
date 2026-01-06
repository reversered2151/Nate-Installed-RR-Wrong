package org.firstinspires.ftc.teamcode.Auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class inandup {
    private DcMotorEx intake;
    private DcMotorEx uptake;

    public inandup(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        uptake = hardwareMap.get(DcMotorEx.class, "uptake");
        uptake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        uptake.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public class kms implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intake.setPower(.7);
            uptake.setPower(.7);
            return false;
        }
    }

    public Action kms() {
        return new kms();
    }
}
