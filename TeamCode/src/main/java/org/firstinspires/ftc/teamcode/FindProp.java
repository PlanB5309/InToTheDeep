package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class FindProp {
    RobotHardware robot;
    Telemetry telemetry;
    LinearOpMode linearOpMode;
    ReadSensor readSensor;
    PropLocation propLocation;

    public FindProp (RobotHardware robot, Telemetry telemetry, LinearOpMode linearOpMode) {
        this.robot = robot;
        this.telemetry = telemetry;
        this.linearOpMode = linearOpMode;
        this.readSensor = new ReadSensor(robot, telemetry, linearOpMode);
    }

    public FindProp (RobotHardware robot, Telemetry telemetry) {
        this.robot = robot;
        this.telemetry = telemetry;
        this.readSensor = new ReadSensor(robot, telemetry);
    }

    public PropLocation FindPropForward(){
        double leftDistance = readSensor.distance(robot.leftDistanceSensor);
        double rightDistance = readSensor.distance(robot.rightDistanceSensor);
        propLocation = propLocation.CENTER;
        if (leftDistance < robot.PROP_THRESHOLD) {
            propLocation = propLocation.LEFT;
        }
        if (rightDistance < robot.PROP_THRESHOLD) {
            propLocation = propLocation.RIGHT;
        }
        return (propLocation);
    }

    public PropLocation FindPropBackward(){
        double leftDistance = readSensor.distance(robot.leftDistanceSensor);
        double rightDistance = readSensor.distance(robot.rightDistanceSensor);
        propLocation = propLocation.CENTER;
        if (leftDistance < robot.PROP_THRESHOLD) {
            propLocation = propLocation.RIGHT;
        }
        if (rightDistance < robot.PROP_THRESHOLD) {
            propLocation = propLocation.LEFT;
        }
        return (propLocation);
    }
}