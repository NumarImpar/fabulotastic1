package org.firstinspires.ftc.teamcode;

import static java.lang.Double.max;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Utilities.ControllerInput;

@TeleOp
public class Drive extends LinearOpMode {

    Hardware robot;
    ControllerInput controller;
    double coef = 1.0;
    double semn = 1;

    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public void runOpMode() throws InterruptedException {
        controller = new ControllerInput(gamepad1);

        robot = new Hardware();
        robot.init(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            controller.update();

            double leftStickY = controller.left_stick_y;
            double leftStickX = controller.left_stick_x;
            double rightStickX = controller.right_stick_x;
            double rightStickY = controller.right_stick_y;

            double r1 = - rightStickY + rightStickX + leftStickX;
            double r2 = - rightStickY - rightStickX - leftStickX;
            double r3 = - rightStickY - rightStickX + leftStickX;
            double r4 = - rightStickY + rightStickX - leftStickX;

            double maxim = 1;
            double minim = -1;

            if(maxim < r1)maxim = r1;
            if(maxim < r2) maxim = r2;
            if(maxim < r3) maxim = r3;
            if(maxim < r4) maxim = r4;

            if(minim > r1) minim = r1;
            if(minim > r2) minim = r2;
            if(minim > r3) minim = r3;
            if(minim > r4) minim = r4;

            r1 = -1 + 2*(r1-minim)/(maxim-minim);
            r2 = -1 + 2*(r2-minim)/(maxim-minim);
            r3 = -1 + 2*(r3-minim)/(maxim-minim);
            r4 = -1 + 2*(r4-minim)/(maxim-minim);

            if (controller.rightBumperOnce())
                coef = Math.max(0.2, coef - 0.1);
            if (controller.leftBumperOnce())
                coef = Math.min(1, coef + 0.1);

            if(controller.YOnce())
                semn *= -1;



            robot.frontLeftWheel.setPower(r1 * coef * semn);
            robot.frontRightWheel.setPower(r2 * coef *semn);
            robot.backLeftWheel.setPower(r3 * coef * semn);
            robot.backRightWheel.setPower(r4* coef * semn);

            telemetry.addData("Senzor distanta", robot.distSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("Buton", robot.touchSensor.isPressed());
            telemetry.addData("coeficient putere", coef);
            telemetry.update();

        }
    }
}
