package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Claws {
    RobotHardware robot;
    Telemetry telemetry;
    LinearOpMode linearOpMode;

    public Claws(RobotHardware robot, Telemetry telemetry, LinearOpMode linearOpMode){
        this.robot = robot;
        this.telemetry = telemetry;
        this.linearOpMode = linearOpMode;
    }

    public void LeftClawOpen (){
        if (!linearOpMode.opModeIsActive())
            return;
        robot.leftClawServo.setPosition(robot.LEFT_CLAW_OPEN);
    }

    public void RightClawOpen (){
        if (!linearOpMode.opModeIsActive())
            return;
        robot.rightClawServo.setPosition(robot.RIGHT_CLAW_OPEN);
    }

    public void LeftClawClose (){
        if (!linearOpMode.opModeIsActive())
            return;
        robot.leftClawServo.setPosition(robot.LEFT_CLAW_CLOSE);


    }
    public void RightClawClose (){
        if (!linearOpMode.opModeIsActive())
            return;
        robot.rightClawServo.setPosition(robot.RIGHT_CLAW_CLOSE);
    }

}
