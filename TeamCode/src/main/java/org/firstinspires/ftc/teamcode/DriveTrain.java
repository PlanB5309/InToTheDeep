package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DriveTrain {
    RobotHardware robot;
    Telemetry telemetry;
    LinearOpMode linearOpMode;

    public DriveTrain(RobotHardware robot, Telemetry telemetry, LinearOpMode linearOpMode){
        this.robot = robot;
        this.telemetry = telemetry;
        this.linearOpMode = linearOpMode;
    }

    public void stop()
    {
        if (!linearOpMode.opModeIsActive())
            return;
        robot.backLeftMotor.setPower(0);
        robot.backRightMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.frontLeftMotor.setPower(0);
    }

    public void run_to_position()
    {
        if (!linearOpMode.opModeIsActive())
            return;
        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void run_using_encoder()
    {
        if (!linearOpMode.opModeIsActive())
            return;
        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void setSpeed(double speed) {
        if (!linearOpMode.opModeIsActive())
            return;
        robot.backLeftMotor.setPower(speed);
        robot.backRightMotor.setPower(speed);
        robot.frontRightMotor.setPower(speed);
        robot.frontLeftMotor.setPower(speed);
    }

    public boolean isBusy () {
        int numNotBusy = 0;
        if (!linearOpMode.opModeIsActive())
            return false;
        if (!robot.backRightMotor.isBusy())
            numNotBusy++;
        if (!robot.backLeftMotor.isBusy())
            numNotBusy++;
        if (!robot.frontLeftMotor.isBusy())
            numNotBusy++;
        if (!robot.frontRightMotor.isBusy())
            numNotBusy++;
        if (numNotBusy >= 3)
            return false;
        return true;
    }

    public void stop_and_reset_encoders() {
        if (!linearOpMode.opModeIsActive())
            return;
        robot.backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setTargetPosition(int target) {
        if (!linearOpMode.opModeIsActive())
            return;
        robot.backLeftMotor.setTargetPosition(target);
        robot.backRightMotor.setTargetPosition(target);
        robot.frontRightMotor.setTargetPosition(target);
        robot.frontLeftMotor.setTargetPosition(target);
    }

}

