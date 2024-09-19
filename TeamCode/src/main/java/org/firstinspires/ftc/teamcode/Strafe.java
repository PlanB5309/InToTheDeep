package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Strafe {
    RobotHardware robot;
    Telemetry telemetry;
    LinearOpMode linearOpMode;
    DriveTrain driveTrain;
    ReadSensor readSensor;


    public Strafe(RobotHardware robot, Telemetry telemetry, LinearOpMode linearOpMode){
        this.robot = robot;
        this.telemetry = telemetry;
        this.linearOpMode = linearOpMode;
        this.driveTrain = new DriveTrain(robot, telemetry,linearOpMode);
        this.readSensor = new ReadSensor(robot, telemetry, linearOpMode);
    }

    public void left (int distance, double speed){
        if (!linearOpMode.opModeIsActive())
            return;
        driveTrain.stop_and_reset_enconders();
        driveTrain.run_using_encoder();
        robot.frontRightMotor.setTargetPosition(robot.STRAFE_CLICKS_PER_CENTIMETER * distance);
        robot.backRightMotor.setTargetPosition(robot.STRAFE_CLICKS_PER_CENTIMETER * -distance);
        robot.frontLeftMotor.setTargetPosition(robot.STRAFE_CLICKS_PER_CENTIMETER * -distance);
        robot.backLeftMotor.setTargetPosition(robot.STRAFE_CLICKS_PER_CENTIMETER * distance);
        driveTrain.run_to_position();
        robot.frontRightMotor.setPower(speed);
        robot.backRightMotor.setPower(-speed);
        robot.frontLeftMotor.setPower(-speed);
        robot.backLeftMotor.setPower(speed);

        while (driveTrain.isBusy() && linearOpMode.opModeIsActive())
            Thread.yield();
        driveTrain.stop();
    }

    public void right (int distance, double speed){
        if (!linearOpMode.opModeIsActive())
            return;
        driveTrain.stop_and_reset_enconders();
        driveTrain.run_using_encoder();
        robot.frontRightMotor.setTargetPosition(robot.STRAFE_CLICKS_PER_CENTIMETER * -distance);
        robot.backRightMotor.setTargetPosition(robot.STRAFE_CLICKS_PER_CENTIMETER * distance);
        robot.frontLeftMotor.setTargetPosition(robot.STRAFE_CLICKS_PER_CENTIMETER * distance);
        robot.backLeftMotor.setTargetPosition(robot.STRAFE_CLICKS_PER_CENTIMETER * -distance);
        driveTrain.run_to_position();
        robot.frontRightMotor.setPower(-speed);
        robot.backRightMotor.setPower(speed);
        robot.frontLeftMotor.setPower(speed);
        robot.backLeftMotor.setPower(-speed);
        while (driveTrain.isBusy() && linearOpMode.opModeIsActive())
            Thread.yield();
        driveTrain.stop();
    }

    //strafe right



    //
    //purpose:Find where to place second pixel
    //How does it do it: takes two back distance sensors and when one sees the backboard and the other sees the back wall it knows it found the edge.
    // It then updates its position and moves to the target position

}
