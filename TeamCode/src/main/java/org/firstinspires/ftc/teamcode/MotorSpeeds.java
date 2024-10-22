package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MotorSpeeds {
    RobotHardware robot;
    DistanceToTarget distanceToTarget;
    Target target;
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

    public MotorSpeeds findMotorSpeeds (DistanceToTarget dtt, double maxSpeed, double currentAngle, Target.Destination type){
        double sin;
        double cos;
        double denominator;
        sin = Math.sin(Math.toRadians(-currentAngle));
        cos = Math.cos(Math.toRadians(-currentAngle));
        //x2=cosβx1−sinβy1
        //y2=sinβx1+cosβy1
        double x2 = (cos * dtt.diffx) - (sin * dtt.diffy);
        double y2 = (sin * dtt.diffx) + (cos * dtt.diffy);
        denominator = Math.max(Math.abs(dtt.diffy) + Math.abs(dtt.diffx), 1);
        frontLeftSpeed = (x2 + y2 ) / denominator;
        frontRightSpeed = (x2 - y2 ) / denominator;
        backLeftSpeed = (x2 - y2 ) / denominator;
        backRightSpeed = (x2 + y2 ) / denominator;

        double turnSpeed = TurnSpeed(dtt.diffh);
        if (dtt.diffh < 0)
            turnSpeed = -turnSpeed;
        frontLeftSpeed = frontLeftSpeed + turnSpeed;
        frontRightSpeed = frontRightSpeed - turnSpeed;
        backLeftSpeed = backLeftSpeed + turnSpeed;
        backRightSpeed = backRightSpeed - turnSpeed;

        if ((dtt.vector > 8 || dtt.vector < -8) || Target.Destination.WAYPOINT == type){
            speedFactor = maxSpeed;
        }

        else {
            speedFactor = dtt.vector / 8 * maxSpeed;
        }

        frontLeftSpeed = frontLeftSpeed * speedFactor;
        frontRightSpeed = frontRightSpeed * speedFactor;
        backLeftSpeed = backLeftSpeed * speedFactor;
        backRightSpeed = backRightSpeed * speedFactor;

        if (frontLeftSpeed > 0 && frontLeftSpeed < .1)
            frontLeftSpeed = .1;
        else if (frontLeftSpeed < 0 && frontLeftSpeed > -.1)
            frontLeftSpeed = -.1;

        if (frontRightSpeed > 0 && frontRightSpeed < .1)
            frontRightSpeed = .1;
        else if (frontRightSpeed < 0 && frontRightSpeed > -.1)
            frontRightSpeed = -.1;

        if (backLeftSpeed > 0 && backLeftSpeed < .1)
            backLeftSpeed = .1;
        else if (backLeftSpeed < 0 && backLeftSpeed > -.1)
            backLeftSpeed = -.1;

        if (backRightSpeed > 0 && backRightSpeed < .1)
            backRightSpeed = .1;
        else if (backRightSpeed < 0 && backRightSpeed > -.1)
            backRightSpeed = -.1;
        return this;
    }

    public void setMotorSpeeds (MotorSpeeds motorSpeeds, double maxSpeed){
        robot.frontLeftMotor.setPower(motorSpeeds.frontLeftSpeed);
        robot.frontRightMotor.setPower(motorSpeeds.frontRightSpeed);
        robot.backLeftMotor.setPower(motorSpeeds.backLeftSpeed);
        robot.backRightMotor.setPower(motorSpeeds.backRightSpeed);
    }

    private double TurnSpeed (double diff) {
        diff = Math.abs(diff);
        if (diff > 80) {
            return robot.HIGH_TURN_POWER;
        }
        else if (diff < 5) {
            return robot.LOW_TURN_POWER;
        }
        else
            return .005667 * diff + .046667;
    }
}
