package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "LeftBlueAuto")
public class LeftBlueAuto extends LinearOpMode {
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
        robot.leftPixelLockServo.setPosition(robot.LEFT_PIXEL_LOCK);
        robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        waitForStart();
        robot.wristServo.setPosition(robot.UPWARDS_WRIST);
        drive.forward(69, .25);
        Thread.sleep(500);
        propLocation = findProp.FindPropForward();
        double WristValue = -1.76E-04*(robot.ARM_PIXEL_SCORE) + 1.61;

        switch (propLocation) {
            case LEFT:
                robot.intakeMotor.setPower(-.25);
                strafe.left(22, .25);
                robot.leftPixelLockServo.setPosition(robot.LEFT_PIXEL_UNLOCK);
                Thread.sleep(300);
                drive.backward(15, .25);
                Thread.sleep(500);
                drive.backward(8, .25);
                robot.intakeMotor.setPower(0);
                gyroTurn.goodEnough(-90);
                robot.wristServo.setPosition(WristValue);
                robot.armMotor.setTargetPosition(robot.ARM_PIXEL_SCORE);
                robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.armMotor.setPower(-1);
                drive.backward(58, .25);
                strafe.left(7,.25);
                back_distance = (int) readSensor.distance(robot.rearRightDistanceSensor);
                drive.move_to_backboard_one_pixel(back_distance);
                claws.RightClawOpen();
                Thread.sleep(500);
                drive.forward(14, .25);
                robot.wristServo.setPosition(robot.GRAB_WRIST);
                robot.armMotor.setTargetPosition(0);
                strafe.right(55, .25);
                break;

            case CENTER:
                robot.intakeMotor.setPower(-.25);
                drive.forward(10, .25);
                robot.leftPixelLockServo.setPosition(robot.LEFT_PIXEL_UNLOCK);
                Thread.sleep(300);
                drive.backward(14, .25);
                Thread.sleep(300);
                drive.backward(8, .25);
                robot.intakeMotor.setPower(0);
                gyroTurn.goodEnough(-90);
                robot.wristServo.setPosition(WristValue);
                robot.armMotor.setTargetPosition(robot.ARM_PIXEL_SCORE);
                robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.armMotor.setPower(-1);
                drive.backward(73, .25);
                strafe.left(10,.25);
                back_distance = (int) readSensor.distance(robot.rearRightDistanceSensor);
                drive.move_to_backboard_one_pixel(back_distance);
                claws.RightClawOpen();
                Thread.sleep(500);
                drive.forward(10, .25);
                robot.wristServo.setPosition(robot.GRAB_WRIST);
                robot.armMotor.setTargetPosition(0);
                strafe.right(60, .3);
                break;

            case RIGHT:
                strafe.left(15, .25);
                drive.backward(5, .25);
                gyroTurn.goodEnough(-90);
                Thread.sleep(1000);
                robot.leftPixelLockServo.setPosition(robot.LEFT_PIXEL_UNLOCK);
                drive.forward(35, .25);
                robot.intakeMotor.setPower(-.25);
                drive.backward(15, .25);
                robot.wristServo.setPosition(WristValue);
                robot.armMotor.setTargetPosition(robot.ARM_PIXEL_SCORE);
                robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.armMotor.setPower(-1);
                drive.backward(72, .25);
                robot.intakeMotor.setPower(0);
                strafe.left(21, .25);
                back_distance = (int) readSensor.distance(robot.rearRightDistanceSensor);
                drive.move_to_backboard_one_pixel(back_distance);
                claws.RightClawOpen();
                Thread.sleep(500);
                drive.forward(5, .25);
                robot.wristServo.setPosition(robot.GRAB_WRIST);
                robot.armMotor.setTargetPosition(0);
                strafe.right(85, .25);
                break;
        }

        robot.armMotor.setTargetPosition(robot.ARM_RESET);
        drive.backward(10, .2);
        robot.armServo.setPosition(robot.SHORT_ARM);
        Thread.sleep(30000);

    }
}








