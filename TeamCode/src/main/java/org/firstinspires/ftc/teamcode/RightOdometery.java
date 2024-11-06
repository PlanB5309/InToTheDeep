package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;


@TeleOp(name="Robot: RightOdometry", group="Robot")
public class RightOdometery extends OpMode {


    RobotHardware robot = new RobotHardware();
    Move move;
    MotorSpeeds motorSpeeds;
    States state;
    DriveTrain driveTrain;
    double timeToStop;
    ReadSensor readSensor = new ReadSensor(robot, telemetry);
    double oldTime = 0;


    //Target Profiles
    TargetProfile wayPoint = new TargetProfile(.6,.2,4,10,3);
    TargetProfile close = new TargetProfile(.45, .1, 2, 5, 5);
    TargetProfile prettyClose = new TargetProfile(.3, .1, 1,2, 8);

    //Targets
    Target driveToSubmersible_T = new Target(30, 14, 90, prettyClose);
    Target turnForSubmersible_T = new Target (25, 14, 90, close);
    Target turnCorrectly_T = new Target (28, 14, 0, wayPoint);
    Target backAwayFromSubmersible_T = new Target(23,14,-90, close);
    Target towardsSamples_T = new Target( 26, -21, -90, wayPoint);
    Target lineUpSamples_T = new Target(38,-21,-90, wayPoint);
    Target eatKrill_T = new Target(38,-25,-90, wayPoint);
    //possibly need another to drive forward and eat krill
    Target driveToWall_T = new Target (4,-39,-90, close);
    //spit out block afterwards
    Target pickUpSpecimen_T = new Target (1, -32, -90, close);
    Target scoreSecondSpecimen_T = new Target (32, 11, 90, close);
    Target turnCorrectly2_T = new Target (28, 14, 0, wayPoint);
    Target towardsSamples2_T = new Target( 26, -30, -90, wayPoint);
    Target lineUpSamples2_T = new Target (26, -30, -90, close);
    Target eatKrill2_T = new Target (26, -35, -90, close);
    Target drivetoWall2_T = new Target (4, -32, -90, close);
    Target pickUpThirdSpecimen_T = new Target (1, -32, -90, close);
    Target scoreThirdSpecimen_T = new Target (32, 16, 90, close);
    Target park_T = new Target(-30,-2,0, wayPoint);
    Target target = new Target();


    @Override
    public void init() {
        robot.init(hardwareMap);
        robot.backClawServo.setPosition(robot.BACK_CLAW_CLOSE);
        robot.frontClawServo.setPosition(robot.FRONT_CLAW_CLOSE);
        //specimenMotor
        robot.specimenMotor.setTargetPosition(0);
        robot.specimenMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.specimenMotor.setPower(1);
        //sampleMotor
        robot.sampleMotor.setTargetPosition(0);
        robot.sampleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.sampleMotor.setPower(1);
        motorSpeeds = new MotorSpeeds(robot, telemetry);
        move = new Move(robot, telemetry, motorSpeeds);
        state = States.START;
//        target = findProp_T;
    }
    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        timeToStop = System.currentTimeMillis()+30000;
        //sampleMotor
        robot.sampleMotor.setTargetPosition(0);
        robot.sampleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.sampleMotor.setPower(1);
        //armMotor
        robot.armMotor.setTargetPosition(100);
        robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armMotor.setPower(1);
        //specimenMotor
        robot.specimenMotor.setTargetPosition(robot.ABOVE_SECOND_BAR);
        robot.specimenMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.specimenMotor.setPower(1);
        target = turnForSubmersible_T;
        resetRuntime();
    }

    @Override
    public void loop() {
        robot.odo.bulkUpdate();
        double newTime = getRuntime();
        double loopTime = newTime-oldTime;
        double frequency = 1/loopTime;
        double liftTime = 0;

        oldTime = newTime;
        Pose2D pos = robot.odo.getPosition();
        String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}",
                pos.getX(DistanceUnit.MM),
                pos.getY(DistanceUnit.MM),
                pos.getHeading(AngleUnit.DEGREES));
        Pose2D vel = robot.odo.getVelocity();
        String velocity = String.format(Locale.US,"{XVel: %.3f, YVel: %.3f, HVel: %.3f}", vel.getX(DistanceUnit.MM), vel.getY(DistanceUnit.MM), vel.getHeading(AngleUnit.DEGREES));


        switch (state){
            case START:
                target = turnForSubmersible_T;
                state = States.TURN_NEAR_SUBMERSIBLE_S;
                break;

            case TURN_NEAR_SUBMERSIBLE_S:

                if (move.moveIt(pos, target)) {
                    target = driveToSubmersible_T;
                    state = States.DRIVE_TO_BAR_S;
                }
                break;

            case DRIVE_TO_BAR_S:
                move.moveIt(pos, target);
                state = States.DONE_FOR_NOW;
//                if (move.moveIt(pos, target)) {
//                    state = States.SCORING;
//                }
                break;

            case SCORING:
                robot.specimenMotor.setTargetPosition(robot.BELOW_SECOND_BAR);
                if (!robot.specimenMotor.isBusy()) {
                    liftTime = System.currentTimeMillis() + 250;
                    state = States.CLAWS_UP;
                }
                break;

            case CLAWS_UP:
                robot.frontClawServo.setPosition(robot.FRONT_CLAW_OPEN_UP);
                robot.backClawServo.setPosition(robot.BACK_CLAW_OPEN_UP);
                if (System.currentTimeMillis() >= liftTime)
                    state = States.LIFT_DOWN;
                break;

            case LIFT_DOWN:
                robot.specimenMotor.setTargetPosition(robot.GRAB_SPECIMEN);
                if (!robot.specimenMotor.isBusy())
                    state = States.CLAWS_DOWN;
                break;

            case CLAWS_DOWN:
                robot.frontClawServo.setPosition(robot.FRONT_CLAW_OPEN_DOWN);
                robot.backClawServo.setPosition(robot.BACK_CLAW_OPEN_DOWN);
                state = States.BACK_UP_FROM_SUBMERISBLE_S;
                break;


            case BACK_UP_FROM_SUBMERISBLE_S:
                if (move.moveIt(pos, target)) {
                    target = towardsSamples_T;
                    state = States.HEAD_TOWARDS_KRILL_R_S;
                }
                break;
//
//            case HEAD_TOWARDS_KRILL_R_S:
//                if (move.moveIt(pos, target)) {
//                    target = lineUpSamples_T;
//                    state = States.EAT_KRILL_R_S;
//                }
//                break;
//
//            case EAT_KRILL_R_S:
//                robot.intakeServo.setPosition(0);
//                if (move.moveIt(pos, target)) {
//                    target = eatKrill_T;
//                    state = States.HEAD_TOWARDS_OBSERVATION_S;
//                }
//                break;
//
//            case HEAD_TOWARDS_OBSERVATION_S:
//                robot.intakeServo.setPosition(.5);
//                if (move.moveIt(pos, target)) {
//                    target = eatKrill_T;
//                    state = States.PARK;                }
//                break;


            case PARK:
                robot.specimenMotor.setTargetPosition(robot.GRAB_SPECIMEN);
                if (move.moveIt(pos, target)) {
                    target = park_T;
                    robot.backLeftMotor.setPower(0);
                    robot.backRightMotor.setPower(0);
                    robot.frontRightMotor.setPower(0);
                    robot.frontLeftMotor.setPower(0);
                }
                break;

            case DONE_FOR_NOW:
                robot.backLeftMotor.setPower(0);
                robot.backRightMotor.setPower(0);
                robot.frontRightMotor.setPower(0);
                robot.frontLeftMotor.setPower(0);
                break;
        }

        //Telemetry Data
        telemetry.addData("Current State", state);
        telemetry.addData("TargetX", target.x);
        telemetry.addData("TargetY", target.y);
        telemetry.addData("TargetH", target.h);
        telemetry.addData("Position", data);
        telemetry.addData("Velocity", velocity);
        //gets the raw data from the X encoder
        telemetry.addData("X Encoder:", robot.odo.getEncoderX());
        //gets the raw data from the Y encoder
        telemetry.addData("Y Encoder:",robot.odo.getEncoderY());
        //prints/gets the current refresh rate of the Pinpoint
        telemetry.addData("Pinpoint Frequency", robot.odo.getFrequency());
        telemetry.addData("Status", robot.odo.getDeviceStatus());
        //prints the control system refresh rate
        telemetry.addData("REV Hub Frequency: ", frequency);
        telemetry.addData("turnSpeed", robot.turnSpeed);
        telemetry.update();

        if (getRuntime() >= 30){
            terminateOpModeNow();
        }
    }
}