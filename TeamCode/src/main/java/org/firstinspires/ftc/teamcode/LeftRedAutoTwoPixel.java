package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "LeftRedAutoTwoPixel")

public class LeftRedAutoTwoPixel extends LinearOpMode {
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
        robot.leftPixelLockServo.setPosition(robot.LEFT_PIXEL_LOCK);
        robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        waitForStart();
        robot.wristServo.setPosition(robot.UPWARDS_WRIST);
        drive.backward(86, .25);
        int leftDistance = (int) readSensor.distance(robot.leftDistanceSensor);
        int rightDistance = (int) readSensor.distance(robot.rightDistanceSensor);
        propLocation = findProp.FindPropBackward();
        double WristValue = -1.76E-04*(robot.ARM_PIXEL_SCORE_HIGH) + 1.61;


        switch (propLocation){
            case LEFT:
                strafe.right(33, .25);
                robot.intakeMotor.setPower(-.25);
                robot.leftPixelLockServo.setPosition(robot.LEFT_PIXEL_UNLOCK);
                drive.backward(36, .25);
                robot.intakeMotor.setPower(0);
                gyroTurn.goodEnough(-90);
                Thread.sleep(300);
                //2nd Pixel Scoring
                drive.backward(218, .5);
                robot.wristServo.setPosition(WristValue);
                robot.armMotor.setTargetPosition(robot.ARM_PIXEL_SCORE_HIGH);
                robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.armMotor.setPower(-1);
                strafe.right_backboard_align(36, .25,PropLocation.LEFT);
                back_distance = (int) readSensor.distance(robot.rearRightDistanceSensor);
                drive.move_to_backboard_two_pixel(back_distance);
                claws.RightClawOpen();
                claws.LeftClawOpen();
                Thread.sleep(500);
                drive.forward(4, .25);
                break;

            case CENTER:
                drive.backward(12, .25);
                robot.intakeMotor.setPower(-.25);
                robot.leftPixelLockServo.setPosition(robot.LEFT_PIXEL_UNLOCK);
                Thread.sleep(250);
                drive.backward(25,.25);
                robot.intakeMotor.setPower(0);
                gyroTurn.goodEnough(-90);
                Thread.sleep(300);
                //2nd pixel scoring
                drive.backward(190, .5);
                robot.wristServo.setPosition(WristValue);
                robot.armMotor.setTargetPosition(robot.ARM_PIXEL_SCORE_HIGH);
                robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.armMotor.setPower(-1);
                strafe.right_backboard_align(56, .25,PropLocation.CENTER);
                back_distance = (int) readSensor.distance(robot.rearRightDistanceSensor);
                drive.move_to_backboard_two_pixel(back_distance);
                claws.RightClawOpen();
                claws.LeftClawOpen();
                Thread.sleep(500);
                drive.forward(4, .25);
                break;

            case RIGHT:

                //first part from leftRedAuto
                strafe.right(15, .25);
                gyroTurn.goodEnough(89);
                strafe.right(20, .25);
                drive.forward(35,.25);
                robot.intakeMotor.setPower(-.25);
                robot.leftPixelLockServo.setPosition(robot.LEFT_PIXEL_UNLOCK);
                Thread.sleep(250);
                drive.backward(20,.25);
                robot.intakeMotor.setPower(0);
                strafe.left(65,.25);
                gyroTurn.goodEnough(-90);
                Thread.sleep(300);
                //2nd Pixel Scoring
                drive.backward(190, .75);
                robot.wristServo.setPosition(WristValue);
                robot.armMotor.setTargetPosition(robot.ARM_PIXEL_SCORE_HIGH);
                robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.armMotor.setPower(-1);
                strafe.right_backboard_align(72, .35, PropLocation.RIGHT);
                back_distance = (int) readSensor.distance(robot.rearRightDistanceSensor);
                drive.move_to_backboard_two_pixel(back_distance);
                claws.RightClawOpen();
                claws.LeftClawOpen();

                drive.forward(4, .25);
                break;



        }
        robot.wristServo.setPosition(robot.GRAB_WRIST);
        robot.armMotor.setTargetPosition(robot.ARM_RESET);
        robot.armMotor.setPower(-1);
        Thread.sleep(30000);
    }

}
