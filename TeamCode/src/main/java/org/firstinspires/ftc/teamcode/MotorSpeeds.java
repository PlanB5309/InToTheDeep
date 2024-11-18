package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class MotorSpeeds {
    RobotHardware robot;
    DistanceToTarget distanceToTarget;

    Target target;
    Telemetry telemetry;
    double frontLeftSpeed;
    double frontRightSpeed;
    double backLeftSpeed;
    double backRightSpeed;
    double speedFactor;



    public MotorSpeeds (RobotHardware robot, Telemetry telemetry) {
        this.robot = robot;
        frontLeftSpeed = 0;
        frontRightSpeed = 0;
        backLeftSpeed = 0;
        backRightSpeed = 0;
        this.telemetry = telemetry;
    }

    public MotorSpeeds (double frontLeftSpeed,
                        double frontRightSpeed,
                        double backLeftSpeed,
                        double backRightSpeed,
                        RobotHardware robot
                        ){
        this.frontLeftSpeed = frontLeftSpeed;
        this.frontRightSpeed = frontRightSpeed;
        this.backLeftSpeed = backLeftSpeed;
        this.backRightSpeed = backRightSpeed;

    }


    public MotorSpeeds findMotorSpeeds (DistanceToTarget dtt, Pose2D pos, Target target){
        double sin;
        double cos;
        double denominator;
        sin = Math.sin(-pos.getHeading(AngleUnit.RADIANS));
        cos = Math.cos(-pos.getHeading(AngleUnit.RADIANS));
        //x2=cosβx1−sinβy1
        //y2=sinβx1+cosβy1
        double x2 = (cos * dtt.diffx) - (sin * dtt.diffy);
        double y2 = (sin * dtt.diffx) + (cos * dtt.diffy);
        denominator = Math.max(Math.abs(dtt.diffy) + Math.abs(dtt.diffx) + Math.abs(dtt.diffh), 1);
        frontLeftSpeed = (x2 + y2 + dtt.diffh) / denominator;
        frontRightSpeed = (x2 - y2 - dtt.diffh) / denominator;
        backLeftSpeed = (x2 - y2 + dtt.diffh) / denominator;
        backRightSpeed = (x2 + y2 - dtt.diffh) / denominator;


        robot.turnSpeed = TurnSpeed(dtt.diffh, target.tp.maxAngle);
        if (dtt.diffh < 0)
            robot.turnSpeed = -robot.turnSpeed;
        frontLeftSpeed = frontLeftSpeed + robot.turnSpeed;
        frontRightSpeed = frontRightSpeed - robot.turnSpeed;
        backLeftSpeed = backLeftSpeed + robot.turnSpeed;
        backRightSpeed = backRightSpeed - robot.turnSpeed;



        if ((dtt.vector > 8 || dtt.vector < -8)){
            speedFactor = target.tp.maxSpeed;
        }

        else {
            speedFactor = dtt.vector / 8 * target.tp.maxSpeed;
        }

        frontLeftSpeed = frontLeftSpeed * speedFactor;
        frontRightSpeed = frontRightSpeed * speedFactor;
        backLeftSpeed = backLeftSpeed * speedFactor;
        backRightSpeed = backRightSpeed * speedFactor;

        if (frontLeftSpeed > 0 && frontLeftSpeed < target.tp.minSpeed)
            frontLeftSpeed = target.tp.minSpeed;
        else if (frontLeftSpeed < 0 && frontLeftSpeed > -target.tp.minSpeed)
            frontLeftSpeed = -target.tp.minSpeed;

        if (frontRightSpeed > 0 && frontRightSpeed < target.tp.minSpeed)
            frontRightSpeed = target.tp.minSpeed;
        else if (frontRightSpeed < 0 && frontRightSpeed > -target.tp.minSpeed)
            frontRightSpeed = -target.tp.minSpeed;

        if (backLeftSpeed > 0 && backLeftSpeed < target.tp.minSpeed)
            backLeftSpeed = target.tp.minSpeed;
        else if (backLeftSpeed < 0 && backLeftSpeed > -target.tp.minSpeed)
            backLeftSpeed = -target.tp.minSpeed;

        if (backRightSpeed > 0 && backRightSpeed < target.tp.minSpeed)
            backRightSpeed = target.tp.minSpeed;
        else if (backRightSpeed < 0 && backRightSpeed > -target.tp.minSpeed)
            backRightSpeed = -target.tp.minSpeed;
        return this;
    }

    public void setMotorSpeeds (MotorSpeeds motorSpeeds){
        robot.frontLeftMotor.setPower(motorSpeeds.frontLeftSpeed);
        robot.frontRightMotor.setPower(motorSpeeds.frontRightSpeed);
        robot.backLeftMotor.setPower(motorSpeeds.backLeftSpeed);
        robot.backRightMotor.setPower(motorSpeeds.backRightSpeed);
    }


    private double TurnSpeed (double diff, double maxAngleDiff) {
        diff = Math.abs(diff);
        if (diff > 60) {
            return robot.HIGH_TURN_POWER;
        }
        if (diff < 10 && diff > maxAngleDiff)
            return robot.LOW_TURN_POWER;
        
        else{
            if (diff < maxAngleDiff)
                return 0;
        }
            return robot.MEDIUM_TURN_POWER;
    }
}
