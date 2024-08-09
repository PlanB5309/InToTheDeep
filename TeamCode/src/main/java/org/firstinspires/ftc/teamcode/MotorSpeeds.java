package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MotorSpeeds {
    RobotHardware robot;
    double frontLeftSpeed;
    double frontRightSpeed;
    double backLeftSpeed;
    double backRightSpeed;



    public MotorSpeeds (RobotHardware robot) {
        this.robot = robot;
        frontLeftSpeed = 0;
        frontRightSpeed = 0;
        backLeftSpeed = 0;
        backRightSpeed = 0;
    }

    public MotorSpeeds (double frontLeftSpeed,
                        double frontRightSpeed,
                        double backLeftSpeed,
                        double backRightSpeed,
                        RobotHardware robot ) {
        this.frontLeftSpeed = frontLeftSpeed;
        this.frontRightSpeed = frontRightSpeed;
        this.backLeftSpeed = backLeftSpeed;
        this.backRightSpeed = backRightSpeed;
    }


    public void setMotorSpeed (MotorSpeeds motorSpeeds, double maxSpeed){
        robot.frontLeftMotor.setPower(motorSpeeds.frontLeftSpeed);
        robot.frontRightMotor.setPower(motorSpeeds.frontRightSpeed);
        robot.backLeftMotor.setPower(motorSpeeds.backLeftSpeed);
        robot.backRightMotor.setPower(motorSpeeds.backRightSpeed);
    }
}
