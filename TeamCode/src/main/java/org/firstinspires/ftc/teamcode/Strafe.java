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
    public void left_backboard_align (int distance, double speed, PropLocation propLocation) {
        double left_back;
        double right_back;
        boolean found_edge = false;
        int newTarget = 0;
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
        while (driveTrain.isBusy() && linearOpMode.opModeIsActive() && found_edge == false) {
            left_back = readSensor.distance(robot.rearLeftDistanceSensor);
            right_back = readSensor.distance(robot.rearRightDistanceSensor);

            if (Math.abs(left_back - right_back) > 10) {
                found_edge = true;
            }
        }
        if (found_edge == true) {

            if (propLocation == PropLocation.LEFT)
                newTarget = robot.backRightMotor.getCurrentPosition() + (robot.STRAFE_CLICKS_PER_CENTIMETER * 41);

            if (propLocation == PropLocation.CENTER)
                newTarget = robot.backRightMotor.getCurrentPosition() + (robot.STRAFE_CLICKS_PER_CENTIMETER * 30);

            if (propLocation == PropLocation.RIGHT)
                newTarget = robot.backRightMotor.getCurrentPosition() + (robot.STRAFE_CLICKS_PER_CENTIMETER * 11);

            robot.frontRightMotor.setTargetPosition(-newTarget);
            robot.backRightMotor.setTargetPosition(newTarget);
            robot.frontLeftMotor.setTargetPosition(newTarget);
            robot.backLeftMotor.setTargetPosition(-newTarget);

            while (driveTrain.isBusy() && linearOpMode.opModeIsActive()){
                Thread.yield();
            }
        }
        driveTrain.stop();
    }

    //
    //purpose:Find where to place second pixel
    //How does it do it: takes two back distance sensors and when one sees the backboard and the other sees the back wall it knows it found the edge.
    // It then updates its position and moves to the target position
    public void right_backboard_align (int distance, double speed, PropLocation propLocation) {
        double left_back;
        double right_back;
        boolean found_edge = false;
        int newTarget = 0;
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
        while (driveTrain.isBusy() && linearOpMode.opModeIsActive() && found_edge == false) {
            left_back = readSensor.distance(robot.rearLeftDistanceSensor);
            right_back = readSensor.distance(robot.rearRightDistanceSensor);

            if (Math.abs(right_back - left_back) > 10) {
                found_edge = true;
            }
        }
        if (found_edge == true) {

            if (propLocation == PropLocation.LEFT)
                newTarget = robot.backLeftMotor.getCurrentPosition() + (robot.STRAFE_CLICKS_PER_CENTIMETER * 13);

            if (propLocation == PropLocation.CENTER)
                newTarget = robot.backLeftMotor.getCurrentPosition() + (robot.STRAFE_CLICKS_PER_CENTIMETER * 32);

            if (propLocation == PropLocation.RIGHT)
                newTarget = robot.backLeftMotor.getCurrentPosition() + (robot.STRAFE_CLICKS_PER_CENTIMETER * 44);

            robot.frontRightMotor.setTargetPosition(newTarget);
            robot.backRightMotor.setTargetPosition(-newTarget);
            robot.frontLeftMotor.setTargetPosition(-newTarget);
            robot.backLeftMotor.setTargetPosition(newTarget);

            while (driveTrain.isBusy() && linearOpMode.opModeIsActive()){
                Thread.yield();
            }
        }
        driveTrain.stop();
    }

}
