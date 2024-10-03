package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;

    @TeleOp(name="Robot: DriveTrainTest", group="Robot")
    public class DriveTrainTest extends OpMode {
        RobotHardware robot = new RobotHardware();
        boolean slow_mode;

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
            if (gamepad1.dpad_right ||
                    gamepad1.dpad_left ||
                    gamepad1.dpad_up ||
                    gamepad1.dpad_down) {
                slow_mode = true;

                //Drives Forward Slowly
                if (gamepad1.dpad_up) {
                    robot.backLeftMotor.setPower(.18);
                    robot.frontLeftMotor.setPower(.18);
                    robot.backRightMotor.setPower(.18);
                    robot.frontRightMotor.setPower(.18);
                }
                //Drives Backward Slowly
                if (gamepad1.dpad_down) {
                    robot.backLeftMotor.setPower(-.18);
                    robot.frontLeftMotor.setPower(-.18);
                    robot.backRightMotor.setPower(-.18);
                    robot.frontRightMotor.setPower(-.18);
                }
                //Strafes Right Slowly
                if (gamepad1.dpad_right && !gamepad1.b) {
                    robot.backLeftMotor.setPower(.24);
                    robot.frontLeftMotor.setPower(-.24);
                    robot.backRightMotor.setPower(-.24);
                    robot.frontRightMotor.setPower(.24);
                }
                //Strafe Left Slowly
                if (gamepad1.dpad_left && !gamepad1.b) {
                    robot.frontLeftMotor.setPower(.24);
                    robot.frontRightMotor.setPower(-.24);
                    robot.backLeftMotor.setPower(-.24);
                    robot.backRightMotor.setPower(.24);
                }
                //Turn Right Slowly
                if (gamepad1.dpad_right && gamepad1.b) {
                    robot.frontLeftMotor.setPower(0.12);
                    robot.frontRightMotor.setPower(-0.12);
                    robot.backLeftMotor.setPower(0.12);
                    robot.backRightMotor.setPower(-0.12);
                }
                //Turns Left Slowly
                if (gamepad1.dpad_left && gamepad1.b) {
                    robot.frontLeftMotor.setPower(-0.12);
                    robot.frontRightMotor.setPower(0.12);
                    robot.backLeftMotor.setPower(-0.12);
                    robot.backRightMotor.setPower(0.12);
                }
            }
            else {
                slow_mode=false;
                double y = -gamepad1.left_stick_y; // Remember, this is reversed!
                double x = -gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
                double rx = gamepad1.right_stick_x;
                // Denominator is the largest motor power (absolute value) or 1
                // This ensures all the powers maintain the same ratio, but only when
                // at least one is out of the range [-1, 1]
                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                double frontLeftPower = (y + x + rx) / denominator;
                double backLeftPower = (y - x + rx) / denominator;
                double frontRightPower = (y - x - rx) / denominator;
                double backRightPower = (y + x - rx) / denominator;
                robot.frontLeftMotor.setPower(frontLeftPower);
                robot.frontRightMotor.setPower(frontRightPower);
                robot.backLeftMotor.setPower(backLeftPower);
                robot.backRightMotor.setPower(backRightPower);
            }



        }

    }

