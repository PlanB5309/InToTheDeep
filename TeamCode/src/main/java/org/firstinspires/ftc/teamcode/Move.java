package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Move {
    RobotHardware robot;
    Telemetry telemetry;
    MotorSpeeds motorSpeeds;
    DistanceToTarget distanceToTarget;
    DriveTrain driveTrain;

    public Move (RobotHardware robot, Telemetry telemetry, MotorSpeeds motorSpeeds){
        this.robot = robot;
        this.telemetry = telemetry;
        this.motorSpeeds = motorSpeeds;
        distanceToTarget = new DistanceToTarget();
        driveTrain = new DriveTrain(robot);
    }

    public boolean moveIt (Pose2D pos, Target target){
        boolean done = false;
        distanceToTarget = distanceToTarget.find(pos, target);

        motorSpeeds.findMotorSpeeds(distanceToTarget, pos, target);
        done = distanceToTarget.closeEnough(distanceToTarget, target);
        if (done == true)
            driveTrain.stop();

        else
            motorSpeeds.setMotorSpeeds(motorSpeeds);

        return done;
    }
}
