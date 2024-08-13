package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Robot: LeftBlueOdometry", group="Robot")
public class LeftBlueOdometry extends OpMode {


    RobotHardware robot = new RobotHardware();
    Move move;
    MotorSpeeds motorSpeeds;
    States state;
    PropLocation propLocation;
    FindProp findProp = new FindProp(robot, telemetry);
    Target findProp_T = new Target(0, 22.25, 0, .3);
    Target centerforward_T = new Target(0, 29.5, 0, .25);
    Target centerDropPurple_T = new Target(0, 21.5, 0, .25);
    Target centerTurnToBoard_T = new Target(0, 21.5, -90, .4);
    Target centerBackupCloseToBoard_T = new Target();
    Target strafeToCenter = new Target();
    Target strafeToParkCenter = new Target();
    



    @Override
    public void init() {
        robot.init(hardwareMap);
        robot.rightClawServo.setPosition(robot.RIGHT_CLAW_CLOSE);
        robot.armServo.setPosition(robot.SHORT_ARM);
        robot.hookServo.setPosition(robot.HOOK_IN);
        robot.leftPixelLockServo.setPosition(robot.LEFT_PIXEL_LOCK);
        robot.wristServo.setPosition(robot.UPWARDS_WRIST);
        motorSpeeds = new MotorSpeeds(robot);
        move = new Move(robot, telemetry, motorSpeeds);
        state = States.START;
    }

    @Override
    public void loop() {
        SparkFunOTOS.Pose2D pos = robot.myOtos.getPosition();
        switch (state){
            case START:
                if (move.moveIt(pos, findProp_T)){
                    state = States.FIND_PROP_S;
                }
                break;

            case FIND_PROP_S:
                propLocation = findProp.FindPropForward();
                switch (propLocation){
                    case LEFT:
                        break;

                    case CENTER:
                        state = States.CENTER_FORWARD_S;
                        break;

                    case RIGHT:
                        break;

                }
                break;

            case CENTER_FORWARD_S:
                robot.intakeMotor.setPower(-.25);
                robot.leftPixelLockServo.setPosition(robot.LEFT_PIXEL_UNLOCK);
                if (move.moveIt(pos, centerforward_T))
                    state = States.CENTER_DROP_P_PIXEL_S;
                break;

            case CENTER_DROP_P_PIXEL_S:
                if (move.moveIt(pos, centerDropPurple_T))
                    state = States.CENTER_TURN_TO_BACKBOARD_S;
                break;

            case CENTER_TURN_TO_BACKBOARD_S:
                robot.intakeMotor.setPower(0);
                if (Math.abs(pos.h - centerTurnToBoard_T.h) < .5){
                    motorSpeeds.setMotorSpeeds(motorSpeeds, 0);
                    state = States.PARK;
                }
                else {
                    
                    if (move.moveIt(pos, centerTurnToBoard_T))
                        state = States.CENTER_DRIVE_CLOSE_TO_BACKBOARD_S;
                }
                break;

            case CENTER_DRIVE_CLOSE_TO_BACKBOARD_S:

                break;

            case CENTER_DRIVE_EXACTLY_TO_BACKBOARD_S:

                break;

            case CENTER_DROP_Y_PIXEL_S:

                break;

            case PARK:

                break;

        }

        //Telemetry Data
        telemetry.addData("X coordinate", pos.x);
        telemetry.addData("Y coordinate", (pos.y));
        telemetry.addData("Heading angle", pos.h);
        telemetry.addData("Current State", state);
        telemetry.update();


    }
}