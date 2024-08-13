package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Arrays;

public class ReadSensor {
    RobotHardware robot;
    Telemetry telemetry;
    LinearOpMode linearOpMode;

    public ReadSensor(RobotHardware robot, Telemetry telemetry, LinearOpMode linearOpMode){
        this.robot = robot;
        this.telemetry = telemetry;
        this.linearOpMode = linearOpMode;
    }

    public ReadSensor(RobotHardware robot, Telemetry telemetry){
        this.robot = robot;
        this.telemetry = telemetry;
    }

    public double distance(Rev2mDistanceSensor Sensor) {
        double distance[] = new double[5];
        double result;
        for (int i = 0; i < 5; i++) {
            distance[i] = Sensor.getDistance(DistanceUnit.CM);
        }
        Arrays.sort(distance);
        result = (distance[1] + distance[2] + distance[3]) / 3;
        return result;
    }



}
