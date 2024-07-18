package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "RightRedAuto")
    public class RightRedAuto extends LinearOpMode {
        RobotHardware robot = new RobotHardware();
        Drive drive = new Drive(robot, telemetry, this);
        Strafe strafe = new Strafe(robot, telemetry, this);
        GyroTurn gyroTurn = new GyroTurn(robot, telemetry, this);
        ReadSensor readSensor = new ReadSensor(robot, telemetry, this);
        Claws claws = new Claws(robot, telemetry, this);
        FindProp findProp = new FindProp(robot, telemetry, this);
        PropLocation propLocation;
        int back_distance;
        int side_distance;



        @Override
        public void runOpMode() throws InterruptedException {
            robot.init(hardwareMap);
            robot.auto_init();
            robot.rightPixelLockServo.setPosition(robot.RIGHT_PIXEL_LOCK);
            robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            waitForStart();
            robot.wristServo.setPosition(robot.UPWARDS_WRIST);
            drive.forward(69, .25);
            Thread.sleep(500);
            propLocation = findProp.FindPropForward();
            double WristValue = -1.76E-04*(robot.ARM_PIXEL_SCORE) + 1.61;

            switch(propLocation){
                case LEFT: //same as leftblueright
                    strafe.right(15, .25);
                    drive.backward(5, .25);
                    gyroTurn.goodEnough(90);
                    Thread.sleep(500);
                    robot.rightPixelLockServo.setPosition(robot.RIGHT_PIXEL_UNLOCK);
                    Thread.sleep(500);
                    drive.forward(35, .25);
                    robot.intakeMotor.setPower(-.25);
                    drive.backward(15, .25);
                    robot.wristServo.setPosition(WristValue);
                    robot.armMotor.setTargetPosition(robot.ARM_PIXEL_SCORE);
                    robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.armMotor.setPower(-1);
                    drive.backward(72, .25);
                    robot.intakeMotor.setPower(0);
                    back_distance = (int) readSensor.distance(robot.rearRightDistanceSensor);
                    drive.move_to_backboard_one_pixel(back_distance);
                    strafe.right(9, .25);
                    claws.RightClawOpen();
                    Thread.sleep(500);
                    drive.forward(5, .25);
                    robot.wristServo.setPosition(robot.GRAB_WRIST);
                    robot.armMotor.setTargetPosition(0);
                    strafe.left(75, .3);
                    break;

                case CENTER:
                    robot.intakeMotor.setPower(-.25);
                    drive.forward(13, .25);
                    robot.rightPixelLockServo.setPosition(robot.RIGHT_PIXEL_UNLOCK);
                    Thread.sleep(300);
                    drive.backward(26, .25);
                    robot.intakeMotor.setPower(0);
                    gyroTurn.goodEnough(90);
                    robot.wristServo.setPosition(WristValue);
                    robot.armMotor.setTargetPosition(robot.ARM_PIXEL_SCORE);
                    robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.armMotor.setPower(-1);
                    drive.backward(72, .25);
                    back_distance = (int) readSensor.distance(robot.rearRightDistanceSensor);
                    drive.move_to_backboard_one_pixel(back_distance);
                    claws.RightClawOpen();
                    Thread.sleep(500);
                    drive.forward(10, .25);
                    robot.wristServo.setPosition(robot.GRAB_WRIST);
                    robot.armMotor.setTargetPosition(0);
                    strafe.left(65, .25);
                    break;

                case RIGHT: //same a leftblueleft
                    robot.intakeMotor.setPower(-.25);
                    strafe.right(24, .25);
                    robot.rightPixelLockServo.setPosition(robot.RIGHT_PIXEL_UNLOCK);
                    Thread.sleep(300);
                    drive.backward(20, .25);
                    Thread.sleep(500);
                    drive.backward(18, .25);
                    robot.intakeMotor.setPower(0);
                    gyroTurn.goodEnough(90);
                    robot.wristServo.setPosition(WristValue);
                    robot.armMotor.setTargetPosition(robot.ARM_PIXEL_SCORE);
                    robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.armMotor.setPower(-1);
                    drive.backward(39, .25);
                    strafe.right(12, .25);
                    Thread.sleep(250);
                    back_distance = (int) readSensor.distance(robot.rearRightDistanceSensor);
                    drive.move_to_backboard_one_pixel(back_distance);
                    claws.RightClawOpen();
                    Thread.sleep(500);
                    drive.forward(14, .25);
                    robot.wristServo.setPosition(robot.GRAB_WRIST);
                    robot.armMotor.setTargetPosition(0);
                    strafe.left(40, .3);
                    break;
                    }

                robot.armMotor.setTargetPosition(robot.ARM_RESET);
                drive.backward(10, .2);
                robot.armServo.setPosition(robot.SHORT_ARM);
                Thread.sleep(30000);
            }
        }


