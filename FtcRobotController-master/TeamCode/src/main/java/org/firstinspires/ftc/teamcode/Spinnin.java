package org.firstinspires.ftc.teamcode;

import static java.lang.Double.max;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Utilities.ControllerInput;

@Autonomous
public class Spinnin extends LinearOpMode {

    Hardware robot;
    ControllerInput controller;
    private ElapsedTime runtime = new ElapsedTime();
    double delta = 4;
    final double EPS = 4;
    final double MAX_DIST = 25;
    final double MIN_DIST = 15;

    BNO055IMU imu; //inertial measurement unit

    private void drivefwd(double pow) {
        robot.frontLeftWheel.setPower(pow);
        robot.frontRightWheel.setPower(pow);
        robot.backLeftWheel.setPower(pow);
        robot.backRightWheel.setPower(pow);
    }

    private void drive(double vx, double vy) {
        //vx *= Math.sqrt(2);
        robot.frontLeftWheel.setPower((vy + vx) / 2.5);
        robot.frontRightWheel.setPower((vy - vx) / 2.5);
        robot.backLeftWheel.setPower((vy + vx) / 2.5);
        robot.backRightWheel.setPower((vy - vx) / 2.5);
    }

    private void driveRight(double pow) {
        robot.frontRightWheel.setPower(pow);
        robot.backRightWheel.setPower(pow);
    }

    private void driveLeft(double pow) {
        robot.frontLeftWheel.setPower(pow);
        robot.backLeftWheel.setPower(pow);
    }

    private void restorePos() {
        double curr_heading = getHeading();
        while (curr_heading < -EPS || curr_heading > EPS) {
            if (curr_heading < 180) {
                driveRight(0.5);
                driveLeft(0);
            } else {
                driveLeft(0.5);
                driveRight(0);
            }
            curr_heading = getHeading();
        }

    }

    private void strafe(double pow) {
        robot.frontLeftWheel.setPower(pow);
        robot.frontRightWheel.setPower(-pow);
        robot.backLeftWheel.setPower(-pow);
        robot.backRightWheel.setPower(pow);
    }


    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Hardware();
        robot.init(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            double dist = robot.distSensor.getDistance(DistanceUnit.CM);
            if (dist >= 80 || (dist >= MIN_DIST && dist <= MAX_DIST))
                continue;
            double target_dist = (MIN_DIST + MAX_DIST) / 2;
            strafe((dist - target_dist) / 80);
            telemetry.addData("Distance", dist);
            telemetry.update();
        }
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
//        parameters.loggingEnabled = true;
//        parameters.loggingTag = "IMU";
//        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
//        imu = hardwareMap.get(BNO055IMU.class, "imu");
//        imu.initialize(parameters);
//
//        waitForStart();
//
////        while(opModeIsActive() && (runtime.seconds() < 5.0)) {
////            drivefwd(runtime.seconds()/5/2);
//
//        double vy, vx;
//
////        while(opModeIsActive() && runtime.seconds() < 1)
////            drive(0, 1);
//
//        while (opModeIsActive() && getHeading() < 30) {
//            driveLeft(0.6);
//            driveRight(0.2);
//        }
//
//        restorePos();

//        while (opModeIsActive() && getHeading() < 60) {
//            driveLeft(0.4);
//            driveRight(1);
//        }
//
//        restorePos();
//
//        while (opModeIsActive() && getHeading() < 60) {
//            driveLeft(1);
//            driveRight(0.4);
//        }
//
//        restorePos();

        //telemetry.addData("Seconds left of spinnin", 10 - runtime.seconds()
    }

    public double getHeading() {
        return -imu.getAngularOrientation().firstAngle;
    }
}

