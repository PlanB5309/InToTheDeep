package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RobotHardware.ARM_PIXEL_DROP;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

    @Autonomous(name = "RightBlueAuto")

    public class RightBlueAuto extends LinearOpMode {
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
            robot.leftPixelLockServo.setPosition(robot.LEFT_PIXEL_LOCK);
            robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
            waitForStart();
            robot.wristServo.setPosition(robot.UPWARDS_WRIST);
            drive.backward(90, .25);
            propLocation = findProp.FindPropBackward();


            switch (propLocation) {
                case LEFT: //same as right leftredautotwopixel
                    strafe.left(15, .25);
                    gyroTurn.goodEnough(-89);
                    strafe.left(25, .25);
                    drive.forward(35,.25);
                    robot.intakeMotor.setPower(-.25);
                    robot.rightPixelLockServo.setPosition(robot.RIGHT_PIXEL_UNLOCK);
                    Thread.sleep(250);
                    drive.backward(20,.25);
                    robot.intakeMotor.setPower(0);
                    strafe.right(65,.25);
                    gyroTurn.goodEnough(88);
                    Thread.sleep(300);
                    //2nd Pixel Scoring
                    drive.backward(200, .5);
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
                    drive.backward(200, .5);
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
                    drive.backward(228, .5);
                    break;
            }

            robot.wristServo.setPosition(robot.GRAB_WRIST);
            Thread.sleep(30000);
    }
}

