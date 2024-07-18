package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RobotHardware.ARM_PIXEL_DROP;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "LeftRedAuto")

public class LeftRedAuto extends LinearOpMode {
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
        robot.rightPixelLockServo.setPosition(robot.RIGHT_PIXEL_LOCK);
        robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        waitForStart();
        robot.wristServo.setPosition(robot.UPWARDS_WRIST);
        drive.backward(86, .25);
        int leftDistance = (int) readSensor.distance(robot.leftDistanceSensor);
        int rightDistance = (int) readSensor.distance(robot.rightDistanceSensor);
        propLocation = findProp.FindPropBackward();

        //left not completed
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
                drive.backward(228, .5);
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
                drive.backward(200, .5);
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
                drive.backward(200, .75);
                break;



        }

        robot.wristServo.setPosition(robot.GRAB_WRIST);
        Thread.sleep(30000);


    }
}