package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DriveTrain {
    RobotHardware robot;



    public DriveTrain(RobotHardware robot){
        this.robot = robot;


    }

    public void stop()
    {
        robot.backLeftMotor.setPower(0);
        robot.backRightMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.frontLeftMotor.setPower(0);
    }



}

