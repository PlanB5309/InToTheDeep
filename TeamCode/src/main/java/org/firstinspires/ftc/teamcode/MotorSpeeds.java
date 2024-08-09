package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MotorSpeeds {
    RobotHardware robot;
    DistanceToTarget distanceToTarget;
    double frontLeftSpeed;
    double frontRightSpeed;
    double backLeftSpeed;
    double backRightSpeed;
    double speedFactor;



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

    public MotorSpeeds findMotorSpeeds (DistanceToTarget dtt, double maxSpeed){
        double denominator;
        denominator = Math.max(Math.abs(dtt.diffy) + Math.abs(dtt.diffx) + Math.abs(dtt.diffh), .1);
        frontLeftSpeed = (dtt.diffy + dtt.diffx + dtt.diffh) / denominator;
        frontRightSpeed = (dtt.diffy - dtt.diffx - dtt.diffh) / denominator;
        backLeftSpeed = (dtt.diffy - dtt.diffx + dtt.diffh) / denominator;
        backRightSpeed = (dtt.diffy + dtt.diffx - dtt.diffh) / denominator;

        if (dtt.vector > 8){
            speedFactor = maxSpeed;
        }

        else {
            speedFactor = dtt.vector / 8 * maxSpeed;
        }
        frontLeftSpeed = frontLeftSpeed * speedFactor;
        frontRightSpeed = frontRightSpeed * speedFactor;
        backLeftSpeed = backLeftSpeed * speedFactor;
        backRightSpeed = backRightSpeed * speedFactor;
        return this;
    }

    public void setMotorSpeeds (MotorSpeeds motorSpeeds, double maxSpeed){
        robot.frontLeftMotor.setPower(motorSpeeds.frontLeftSpeed);
        robot.frontRightMotor.setPower(motorSpeeds.frontRightSpeed);
        robot.backLeftMotor.setPower(motorSpeeds.backLeftSpeed);
        robot.backRightMotor.setPower(motorSpeeds.backRightSpeed);
    }
}
