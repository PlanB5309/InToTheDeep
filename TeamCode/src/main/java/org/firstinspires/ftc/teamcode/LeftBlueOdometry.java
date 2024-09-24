package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name="Robot: LeftBlueOdometry", group="Robot")
public class LeftBlueOdometry extends OpMode {


    RobotHardware robot = new RobotHardware();
    Move move;
    MotorSpeeds motorSpeeds;
    States state;
    ReadSensor readSensor = new ReadSensor(robot, telemetry);
    Target driveToSubmersible = new Target(28, 0, 0, .3);
    Target backAwayFromSubmersible = new Target(27,0,0,.1);
    Target park = new Target(0,60,0,.1);
    Target target = new Target();

    @Override
    public void init() {
        robot.init(hardwareMap);
        robot.backClawServo.setPosition(robot.BACK_CLAW_CLOSE);
//        robot.armServo.setPosition(robot.SHORT_ARM);
//        robot.hookServo.setPosition(robot.HOOK_IN);
//        robot.leftPixelLockServo.setPosition(robot.LEFT_PIXEL_LOCK);
        motorSpeeds = new MotorSpeeds(robot);
        move = new Move(robot, telemetry, motorSpeeds);
        state = States.START;
//        target = findProp_T;
    }
    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

    }

    @Override
    public void loop() {
        SparkFunOTOS.Pose2D pos = robot.myOtos.getPosition();
        switch (state){
            case START:
                break;

//            case CENTER_FORWARD_S:
//
//                if (move.moveIt(pos, target)) {
//                    target = centerDropPurple_T;
//                    state = States.CENTER_DROP_P_PIXEL_S;
//                }
//                break;
//
//            case CENTER_DROP_P_PIXEL_S:
//                if (move.moveIt(pos, target)) {
//                    target = centerTurnToBoard_T;
//                    state = States.CENTER_TURN_TO_BACKBOARD_S;
//                }
//                break;
//
//            case CENTER_TURN_TO_BACKBOARD_S:
//                if (move.moveIt(pos, target)) {
//                    state = States.CENTER_DRIVE_CLOSE_TO_BACKBOARD_S;
//                    target = centerBackupCloseToBoard_T;
//                }
//                break;
//
//            case CENTER_DRIVE_CLOSE_TO_BACKBOARD_S:
//                if (move.moveIt(pos, target)) {
//                    state = States.READ_DISTANCE_SENSOR_S;
//
//                    robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    robot.armMotor.setPower(-1);
//                    //target = backupExactlyToBoard_T;
//                }
//                break;
//
//            case READ_DISTANCE_SENSOR_S:
//                //robot.myOtos.resetTracking();
//                //backupExactlyToBoard_T.set(x, pos.y, -90, .2);
//                target = backupExactlyToBoard_T;
//                state = States.CENTER_DRIVE_EXACTLY_TO_BACKBOARD_S;
//                //state = States.DONE_FOR_NOW;
//                break;
//
//            case CENTER_DRIVE_EXACTLY_TO_BACKBOARD_S:
//                if (move.moveIt(pos, target))
//                    //state = States.CENTER_DROP_Y_PIXEL_S;
//                    state = States.DONE_FOR_NOW;
//                break;
//
//            case CENTER_DROP_Y_PIXEL_S:
//                state = States.PARK;
//                target = park_T;
//                break;

            case PARK:
                if (move.moveIt(pos, target)) {
                    robot.frontLeftMotor.setPower(0);
                    robot.frontRightMotor.setPower(0);
                    robot.backLeftMotor.setPower(0);
                    robot.backRightMotor.setPower(0);
                }
                break;
            case DONE_FOR_NOW:
                robot.frontLeftMotor.setPower(0);
                robot.frontRightMotor.setPower(0);
                robot.backLeftMotor.setPower(0);
                robot.backRightMotor.setPower(0);
                break;
        }

        //Telemetry Data
        telemetry.addData("X coordinate", pos.x);
        telemetry.addData("Y coordinate", (pos.y));
        telemetry.addData("Heading angle", pos.h);
        telemetry.addData("Current State", state);
        telemetry.addData("TargetX", target.x);
        telemetry.addData("TargetY", target.y);
        telemetry.addData("TargetH", target.h);
        telemetry.addData("Target Maxspeed", target.maxSpeed);
        telemetry.update();


    }
}