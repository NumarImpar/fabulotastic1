package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorREV2mDistance;

public class Hardware {
    public DcMotorEx frontLeftWheel = null;
    public DcMotorEx frontRightWheel = null;
    public DcMotorEx backLeftWheel = null;
    public DcMotorEx backRightWheel = null;
    public Rev2mDistanceSensor distSensor = null;
    public TouchSensor touchSensor = null;

    public void init(HardwareMap hwMap) {
        //Sensors
        distSensor = hwMap.get(Rev2mDistanceSensor.class, "distance");
        touchSensor = hwMap.get(TouchSensor.class, "buton0");

        //-----------------------------Wheels------------------------------------------
        frontLeftWheel = hwMap.get(DcMotorEx.class, "FL");
        frontRightWheel = hwMap.get(DcMotorEx.class, "FR");
        backLeftWheel = hwMap.get(DcMotorEx.class, "BL");
        backRightWheel = hwMap.get(DcMotorEx.class, "BR");

        frontLeftWheel.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        frontRightWheel.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backLeftWheel.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backRightWheel.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        frontLeftWheel.setDirection(DcMotorEx.Direction.FORWARD);
        frontRightWheel.setDirection(DcMotorEx.Direction.REVERSE);
        backLeftWheel.setDirection(DcMotorEx.Direction.FORWARD);
        backRightWheel.setDirection(DcMotorEx.Direction.REVERSE);

        frontLeftWheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRightWheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeftWheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRightWheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        frontLeftWheel.setPower(0);
        frontRightWheel.setPower(0);
        backLeftWheel.setPower(0);
        backRightWheel.setPower(0);
    }


}
