package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Robot: LeftBlueOdometry", group="Robot")
public class LeftBlueOdometry extends OpMode {


    RobotHardware robot = new RobotHardware();


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

    }

    @Override
    public void loop() {

        SparkFunOTOS.Pose2D pos = robot.myOtos.getPosition();
        SparkFunOTOS.Pose2D target = new SparkFunOTOS.Pose2D(27, 27, 0);
        move(pos, target);

        //Telemetry Data
        telemetry.addData("X coordinate", pos.x);
        telemetry.addData("Y coordinate", (pos.y));
        telemetry.addData("Heading angle", pos.h);
        telemetry.addData("Say", "Happy Little Pixels");
        telemetry.update();
    }

    private void move (SparkFunOTOS.Pose2D pos, SparkFunOTOS.Pose2D target) {
        double rx;
        double y;
        double x;
        double frontLeftPower;
        double backLeftPower;
        double frontRightPower;
        double backRightPower;
        double powerScaleFactor = .5;
        double denominator;
        double diffX = (target.x - pos.x);
        double diffY = (target.y - pos.y);
        double diffH = -(target.h - pos.h);
        y = diffY;
        x = diffX;
        rx = diffH;
        if (Math.abs(diffX) < .5 && Math.abs(diffY) < .5 && Math.abs(diffH) < .5) {
            robot.frontLeftMotor.setPower(0);
            robot.frontRightMotor.setPower (0);
            robot.backLeftMotor.setPower(0);
            robot.backRightMotor.setPower(0);
        }
        else {
            denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), .1);
            frontLeftPower = (y + x + rx) / denominator;
            backLeftPower = (y - x + rx) / denominator;
            frontRightPower = (y - x - rx) / denominator;
            backRightPower = (y + x - rx) / denominator;

            robot.frontLeftMotor.setPower(frontLeftPower * .1);
            robot.frontRightMotor.setPower(frontRightPower * .1);
            robot.backLeftMotor.setPower(backLeftPower * .1);
            robot.backRightMotor.setPower(backRightPower * .1);
        }

    }
}
//    private boolean move (SparkFunOTOS.Pose2D pos, SparkFunOTOS.Pose2D target ) {
//        boolean done = false;
//        double diffX = (target.x - pos.x);
//        double diffY = (target.y - pos.y);
//        double diffH = (target.h - pos.h);
//        double frontLeftPower;
//        double backLeftPower;
//        double frontRightPower;
//        double backRightPower;
//        double powerScaleFactor = .5;
//        double denominator;
//        double rx;
//        double y;
//        double x;
//
//        if (Math.abs(diffX) < .5 && Math.abs(diffY) < .5 && Math.abs(diffH) < .5) {
//            done = true;
//            return done;
//        }
//        double distance_to_target = (diffX * diffX) + (diffY * diffY);
//        distance_to_target = Math.sqrt(distance_to_target);
//        y = diffY;
//        x = diffX;
//        rx = diffH;
//
//        if (distance_to_target > 8) {
//            y = diffY;
//            x = diffX;
//            rx = diffH;
//            denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), .5);
//            frontLeftPower = (y + x + rx) / denominator;
//            backLeftPower = (y - x + rx) / denominator;
//            frontRightPower = (y - x - rx) / denominator;
//            backRightPower = (y + x - rx) / denominator;
//        }
//        else {
//            powerScaleFactor = (distance_to_target / 8) * .5;
//            denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), .5);
//            frontLeftPower = (y + x + rx) / denominator;
//            backLeftPower = (y - x + rx) / denominator;
//            frontRightPower = (y - x - rx) / denominator;
//            backRightPower = (y + x - rx) / denominator;
//
//        }
//        robot.frontLeftMotor.setPower(frontLeftPower);
//        robot.frontRightMotor.setPower(frontRightPower);
//        robot.backLeftMotor.setPower(backLeftPower);
//        robot.backRightMotor.setPower(backRightPower);
//    }



