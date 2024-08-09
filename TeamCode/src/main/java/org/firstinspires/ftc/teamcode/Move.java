package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Move {
    RobotHardware robot;
    Telemetry telemetry;
    MotorSpeeds motorSpeeds;
    DistanceToTarget distanceToTarget;

    public Move (RobotHardware robot, Telemetry telemetry, MotorSpeeds motorSpeeds){
        this.robot = robot;
        this.telemetry = telemetry;
        this.motorSpeeds = motorSpeeds;
        distanceToTarget = new DistanceToTarget();
    }

    public boolean moveIt (SparkFunOTOS.Pose2D pos, Target target){
        boolean done = false;
        distanceToTarget = distanceToTarget.find(pos, target);
        motorSpeeds.findMotorSpeeds(distanceToTarget, target.maxSpeed);
        done = distanceToTarget.closeEnough(distanceToTarget);
        if (done == true){
            motorSpeeds.setMotorSpeeds(motorSpeeds, 0);
        }
        motorSpeeds.setMotorSpeeds(motorSpeeds, target.maxSpeed);
        return false;
    }
}
