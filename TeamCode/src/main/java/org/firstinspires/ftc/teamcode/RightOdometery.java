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
    DriveTrain driveTrain = new DriveTrain(robot);
    double timeToStop;
    ReadSensor readSensor = new ReadSensor(robot, telemetry);
    double oldTime = 0;
    double spitTime = 0;


    //Target Profiles
    TargetProfile wayPoint = new TargetProfile(.6,.2,4,10,3);
    TargetProfile close = new TargetProfile(.45, .1, 2, 5, 5);
    TargetProfile prettyClose = new TargetProfile(.3, .1, 1,2, 8);
    TargetProfile samplePickup = new TargetProfile(.2, .1, 1,5, 2);
    TargetProfile specimenPickup = new TargetProfile(.7, .1, 1,3, 2);


    //Targets
    Target turnNearSubmersible_T = new Target (28, 14, 75, close);
    Target driveToBar_T = new Target(30.75, 14, 90, prettyClose);
    Target turnCorrectly_T = new Target (28, 14, 0, wayPoint);
    Target backUpFromSubmersible_T = new Target(23,14,-90, wayPoint);
    Target driveTowardsSamples_T = new Target( 26, -16.5, -90, wayPoint);
    Target lineUpSamples_T = new Target(36.5,-16.5,-90, wayPoint);
    Target pickUpSample_T = new Target(36.5,-30,-90, samplePickup);
    Target driveToSpecimen_T = new Target (2,-34,-90, specimenPickup);
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
        target = turnNearSubmersible_T;
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
                target = turnNearSubmersible_T;
                state = States.TURN_NEAR_SUBMERSIBLE_S;
                break;

            case TURN_NEAR_SUBMERSIBLE_S:
                if (move.moveIt(pos, target)) {
                    target = driveToBar_T;
                    state = States.DRIVE_TO_BAR_S;
                }
                break;

            case DRIVE_TO_BAR_S:
                if (move.moveIt(pos, target)) {
                    state = States.SCORING;
                    driveTrain.stop();
                }
                break;

            case SCORING:
                robot.specimenMotor.setTargetPosition(robot.BELOW_SECOND_BAR);
                if (!robot.specimenMotor.isBusy()) {
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
                target = backUpFromSubmersible_T;
                state = States.BACK_UP_FROM_SUBMERISBLE_S;
                break;


            case BACK_UP_FROM_SUBMERISBLE_S:
                if (move.moveIt(pos, target)) {
                    target = driveTowardsSamples_T;
                    state = States.DRIVE_TOWARDS_SAMPLE_S;
                }
                break;

            case DRIVE_TOWARDS_SAMPLE_S:
                if (move.moveIt(pos, target)) {
                    target = lineUpSamples_T;
                    state = States.LINE_UP_ON_SAMPLE_S;
                }
                break;

            case LINE_UP_ON_SAMPLE_S:
                if (move.moveIt(pos,target)){
                    target = pickUpSample_T;
                    state = States.PICK_UP_SAMPLE_S;
                }

            case PICK_UP_SAMPLE_S:
                robot.intakeServo.setPosition(0);
                if (move.moveIt(pos, target)) {
                    robot.intakeServo.setPosition(.5);
                    target = driveToSpecimen_T;
                    state = States.DRIVE_TO_SPECIMEN_S;
                }
                break;

            case DRIVE_TO_SPECIMEN_S:
                robot.intakeServo.setPosition(.5);
                if (move.moveIt(pos, target)) {
                    target = pickUpSpecimen_T;
                    state = States.DROP_SAMPLE_S;
                    spitTime = System.currentTimeMillis() + 250;
                    driveTrain.stop();
                }
                break;

            case DROP_SAMPLE_S:
                robot.intakeServo.setPosition(1);
                if (System.currentTimeMillis() >= spitTime) {
                    target = pickUpSpecimen_T;
                    state = States.DONE_FOR_NOW;
//                    state = States.LOADING;
                }
                break;

            case LOADING:
                robot.frontClawServo.setPosition(robot.FRONT_CLAW_CLOSE);
                robot.backClawServo.setPosition(robot.BACK_CLAW_CLOSE);
                robot.specimenMotor.setTargetPosition(robot.ABOVE_SECOND_BAR);
                robot.specimenMotor.setPower(1);
                if (!robot.specimenMotor.isBusy()){
                    //target = turnNearSubmersible_T;
                    state = States.DONE_FOR_NOW;
                }

            case PARK:
                robot.specimenMotor.setTargetPosition(robot.GRAB_SPECIMEN);
                if (move.moveIt(pos, target)) {
                    target = park_T;
                   driveTrain.stop();
                }
                break;

            case DONE_FOR_NOW:
                driveTrain.stop();
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
        telemetry.update();

        if (getRuntime() >= 30){
            terminateOpModeNow();
        }
    }
}