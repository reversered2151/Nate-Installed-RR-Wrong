package org.firstinspires.ftc.teamcode.qualifiers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

public class qualifiersHardwareMap {
    public DcMotorEx bl, br, fl, fr, flywheel, intake, uptake;

    //    public NormalizedColorSensor colorSensor;
    public IMU imu;

//    HuskyLens huskyLens;

    public void init(HardwareMap hardwareMap){
        // Color Sensor
//        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "color");
        // Camera
//        huskyLens  = hardwareMap.get(HuskyLens.class,"huskyLens");

        br = hardwareMap.get(DcMotorEx.class, "backRight");
        bl = hardwareMap.get(DcMotorEx.class, "backLeft");
        fr = hardwareMap.get(DcMotorEx.class, "frontRight");
        fl = hardwareMap.get(DcMotorEx.class, "frontLeft");

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        uptake = hardwareMap.get(DcMotorEx.class, "uptake");
        uptake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        uptake.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");

        // Zero power behavior
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flywheel = hardwareMap.get(DcMotorEx.class,"flywheel");
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        //TODO FIGURE OUT WHICH NEEDS TO BE REVERSE OR FORWARD
//         fl.setDirection(DcMotorSimple.Direction.REVERSE);
//         bl.setDirection(DcMotorSimple.Direction.REVERSE);
//         fr.setDirection(DcMotorSimple.Direction.FORWARD);
//         br.setDirection(DcMotorSimple.Direction.FORWARD);
    }
}