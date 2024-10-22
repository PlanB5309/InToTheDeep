package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

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

    public boolean moveIt (Pose2D pos, Target target){
        boolean done = false;
        distanceToTarget = distanceToTarget.find(pos, target);
        motorSpeeds.findMotorSpeeds(distanceToTarget, target.maxSpeed, pos.getHeading(AngleUnit.DEGREES), target.type);
        done = distanceToTarget.closeEnough(distanceToTarget, target.type);
        if (done == true)
            motorSpeeds.setMotorSpeeds(motorSpeeds, 0);

        else
            motorSpeeds.setMotorSpeeds(motorSpeeds, target.maxSpeed);
        return done;
    }


}
