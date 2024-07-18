package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "RightBlueAutoTwoPixel")

public class RightBlueAutoTwoPixel extends LinearOpMode {
    RobotHardware robot = new RobotHardware();
    Drive drive = new Drive(robot, telemetry, this);
    Strafe strafe = new Strafe(robot, telemetry, this);
    GyroTurn gyroTurn = new GyroTurn(robot, telemetry, this);
    ReadSensor readSensor = new ReadSensor(robot, telemetry, this);
    Claws claws = new Claws(robot, telemetry, this);
    FindProp findProp = new FindProp(robot, telemetry, this);
    PropLocation propLocation;
    int back_distance;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.auto_init();
        robot.rightPixelLockServo.setPosition(robot.RIGHT_PIXEL_LOCK);
        robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        waitForStart();
        robot.wristServo.setPosition(robot.UPWARDS_WRIST);
        drive.backward(88, .25);
        propLocation = findProp.FindPropBackward();
        double WristValue = -1.76E-04*(robot.ARM_PIXEL_SCORE_HIGH) + 1.61;


        switch (propLocation) {
            case LEFT: //same as right leftredautotwopixel
                //1st Pixel
                strafe.left(15, .32);
                gyroTurn.goodEnough(-89);
                strafe.left(25, .32);
                drive.forward(31,.32);
                robot.intakeMotor.setPower(-.25);
                robot.rightPixelLockServo.setPosition(robot.RIGHT_PIXEL_UNLOCK);
                Thread.sleep(250);
                drive.backward(22,.32);
                robot.intakeMotor.setPower(0);
                strafe.right(65,.32);
                gyroTurn.goodEnough(88);
                //2nd Pixel Scoring
                drive.backward(190, .7);
                robot.wristServo.setPosition(WristValue);
                robot.armMotor.setTargetPosition(robot.ARM_PIXEL_SCORE_HIGH);
                robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.armMotor.setPower(1);
                gyroTurn.goodEnough(90);
                strafe.left_backboard_align(68,.25,PropLocation.LEFT);
                gyroTurn.goodEnough(90);
                back_distance = (int) readSensor.distance(robot.rearLeftDistanceSensor);
                drive.move_to_backboard_two_pixel(back_distance);
                claws.RightClawOpen();
                claws.LeftClawOpen();
                Thread.sleep(350);
                drive.forward(4, .25);
                break;

            case CENTER: //center is the same turns are different
                drive.backward(8, .25);
                robot.intakeMotor.setPower(-.25);
                robot.rightPixelLockServo.setPosition(robot.RIGHT_PIXEL_UNLOCK);
                Thread.sleep(250);
                drive.backward(29,.25);
                robot.intakeMotor.setPower(0);
                gyroTurn.goodEnough(89);
                Thread.sleep(300);
                //2nd pixel scoring
                drive.backward(190, .5);
                robot.wristServo.setPosition(WristValue);
                robot.armMotor.setTargetPosition(robot.ARM_PIXEL_SCORE_HIGH);
                robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.armMotor.setPower(1);
                gyroTurn.goodEnough(88);
                Thread.sleep(500);
                strafe.left_backboard_align(56, .25,PropLocation.CENTER);
                Thread.sleep(500);
                back_distance = (int) readSensor.distance(robot.rearRightDistanceSensor);
                drive.move_to_backboard_two_pixel(back_distance);
                claws.RightClawOpen();
                claws.LeftClawOpen();
                Thread.sleep(500);
                drive.forward(4, .25);
                break;

            case RIGHT: //same as left leftredautotwopixel
                strafe.left(33, .25);
                robot.intakeMotor.setPower(-.25);
                robot.rightPixelLockServo.setPosition(robot.RIGHT_PIXEL_UNLOCK);
                drive.backward(36, .25);
                robot.intakeMotor.setPower(0);
                gyroTurn.goodEnough(88);
                Thread.sleep(300);
                //2nd Pixel Scoring
                drive.backward(218, .5);
                robot.wristServo.setPosition(WristValue);
                robot.armMotor.setTargetPosition(robot.ARM_PIXEL_SCORE_HIGH);
                robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.armMotor.setPower(1);
                strafe.left_backboard_align(37, .25, PropLocation.RIGHT);
                gyroTurn.goodEnough(90);
                back_distance = (int) readSensor.distance(robot.rearRightDistanceSensor);
                drive.move_to_backboard_two_pixel(back_distance);
                claws.RightClawOpen();
                claws.LeftClawOpen();
                Thread.sleep(500);
                drive.forward(4, .25);
                break;
        }
        robot.wristServo.setPosition(robot.GRAB_WRIST);
        robot.armMotor.setTargetPosition(robot.ARM_RESET);
        robot.armMotor.setPower(-1);
        Thread.sleep(30000);
    }
}
