package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;

@TeleOp(name="Robot: RightOdometryFourSpecimen", group="Robot")
public class RightOdometeryFourSpecimen extends OpMode {

    RobotHardware robot = new RobotHardware();
    Move move;
    MotorSpeeds motorSpeeds;
    States state;
    DriveTrain driveTrain = new DriveTrain(robot);
    GyroTurn gyroTurn = new GyroTurn(robot,telemetry);
    double timeToStop;
    double oldTime = 0;
    double spitTime = 0;
    double close_to_wall;
    double newTime = 0;
    double loopTime = 0;
    double frequency;
    double timer = 0;
    double time_to_reach_wall = 0;
    double distanceToWall;


    //Target Profiles
    TargetProfile batOutOfHell = new TargetProfile(1,.85,10, 15, 6);
    TargetProfile wayPoint = new TargetProfile(.85, .2, 5, 10, 4);
    TargetProfile close = new TargetProfile(.5, .125, 2, 5, 5);
    TargetProfile closer = new TargetProfile(.3, .1, 1.5, 3, 8);
    TargetProfile samplePickup = new TargetProfile(.275, .25, 1, 4, .1);
    TargetProfile specimenPickup = new TargetProfile(.85, .1, 2, 3, 5);
    TargetProfile scoreSpecimen = new TargetProfile (.5, .1, 2, 10, 5);
    TargetProfile superWayPoint = new TargetProfile(.875, .2, 5, 15, 3);
    TargetProfile slowerWayPoint = new TargetProfile(.825, .2, 4, 8, 5);
    TargetProfile normalSpeed = new TargetProfile(.725, .3, 3, 6, 4);

    //Targets
    Target turnNearSubmersible_T = new Target(25, 13, 57, wayPoint);
    //Overshoots the turn the back right wheel hits the submersible before it is correct
    Target turnNearSubmersibleAgain_T = new Target(25, 6, 55, superWayPoint);
    Target turnNearSubmersibleAgainAgain_T = new Target(25, 4, 55, superWayPoint);
    Target turnNearSubmersibleAgainAgainAgain_T = new Target(25, 2, 55, superWayPoint);
    Target driveToBar_T = new Target(31.75, 17, 90, close);
    Target driveToBarAgain_T = new Target(31.75, 13, 90, scoreSpecimen);
    Target driveToBarAgainAgain_T = new Target(31.75, 10, 90, scoreSpecimen);
    Target driveToBarAgainAgainAgain_T = new Target(31.75, 8, 90, scoreSpecimen);
    Target backUpFromSubmersible_T = new Target(23, 14, 90, wayPoint);
    Target spinAtSubmersible_T = new Target(23, 14, -90, close);
    Target waypointTowardsSamples_T = new Target(21, -14, -90, slowerWayPoint);
    //was 23, 14
    Target driveTowardsSamples_T = new Target(26, -18, -90, wayPoint);
    Target lineUpSamples_T = new Target(36, -19, -90, normalSpeed);
    Target lineUpSamplesAgain_T = new Target(34, -29, -90, normalSpeed);
    Target pickUpSample_T = new Target(36, -31, -90, samplePickup);
    Target pickUpSampleAgain_T = new Target(35, -38, -90, samplePickup);
    Target driveToSpecimen_T = new Target(2, -34, -90, specimenPickup);
    Target driveToObservationZone_T = new Target(2, -34, -90, slowerWayPoint);
    Target lineUpOnSpecimen_T = new Target(5, -33, -90, specimenPickup);
    //spit out block afterwards
    Target pickUpSpecimen_T = new Target(-1, -30.5, -90, closer);
    Target pickUpSpecimenAgain_T = new Target(-1, -33.5, -90, specimenPickup);
    Target backUpFromWall_T = new Target(10, -30.5, -90, wayPoint);
    Target park_T = new Target(0, -33, 90, batOutOfHell);
    Target target = new Target();


    @Override
    public void init() {
        robot.init(hardwareMap);
        robot.backClawServo.setPosition(robot.BACK_CLAW_CLOSE);
        robot.frontClawServo.setPosition(robot.FRONT_CLAW_CLOSE);
        robot.armBlockServo.setPosition(robot.BLOCK_ARM);
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
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */

    @Override
    public void start() {
        timeToStop = System.currentTimeMillis() + 30000;
        //sampleMotor
        robot.sampleMotor.setTargetPosition(0);
        robot.sampleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.sampleMotor.setPower(1);
        //armMotor
        robot.armMotor.setTargetPosition(300);
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
        newTime = getRuntime();
        loopTime = newTime - oldTime;
        frequency = 1 / loopTime;
        oldTime = newTime;
        robot.odo.bulkUpdate();
        Pose2D pos = robot.odo.getPosition();
        String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}",
                pos.getX(DistanceUnit.MM),
                pos.getY(DistanceUnit.MM),
                pos.getHeading(AngleUnit.DEGREES));
        Pose2D vel = robot.odo.getVelocity();
        String velocity = String.format(Locale.US, "{XVel: %.3f, YVel: %.3f, HVel: %.3f}", vel.getX(DistanceUnit.MM), vel.getY(DistanceUnit.MM), vel.getHeading(AngleUnit.DEGREES));


        switch (state) {
            //have it go further along the bar, so there is more room for the third specimen to be scored
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
                score();
                target = backUpFromSubmersible_T;
                state = States.BACK_UP_FROM_SUBMERISBLE_S;
                break;

            case BACK_UP_FROM_SUBMERISBLE_S:
                if (move.moveIt(pos, target)) {
                    target = spinAtSubmersible_T;
                    state = States.WAYPOINT_TOWARDS_SAMPLE_S;
                }
                break;

            case SPIN_AT_SUBMERSIBLE_S:
                if (spin(pos, target)) {
                    target = waypointTowardsSamples_T;
                    state = States.WAYPOINT_TOWARDS_SAMPLE_S;
                }
                break;

                //working on placing samples in observation zone

            case WAYPOINT_TOWARDS_SAMPLE_S:
                if (move.moveIt(pos, target)) {
                    target = driveTowardsSamples_T;
                    state = States.DRIVE_TOWARDS_SAMPLE_S;
                }
                break;

            case DRIVE_TOWARDS_SAMPLE_S:
                robot.frontClawServo.setPosition(robot.FRONT_CLAW_OPEN_DOWN);
                robot.backClawServo.setPosition(robot.BACK_CLAW_OPEN_DOWN);
                if (move.moveIt(pos, target)) {
                    target = lineUpSamples_T;
                    state = States.LINE_UP_ON_SAMPLE_S;
                }
                break;

            case LINE_UP_ON_SAMPLE_S:
                if (move.moveIt(pos, target)) {
                    target = pickUpSample_T;
                    state = States.PICK_UP_SAMPLE_S;
                    gyroTurn.goodEnough(target.h);
                    gyroTurn.goodEnough(target.h);
                }

            case PICK_UP_SAMPLE_S:
                robot.armMotor.setTargetPosition(0);
                robot.intakeServo.setPosition(0);
                if (move.moveIt(pos, target)) {
                    robot.intakeServo.setPosition(.5);
                    robot.armMotor.setTargetPosition(500);
                    robot.sampleMotor.setTargetPosition(300);
                    target = driveToSpecimen_T;
                    state = States.DRIVE_TO_OBSERVATION_ZONE_S;
                }
                break;

            case DRIVE_TO_OBSERVATION_ZONE_S:
                robot.intakeServo.setPosition(.5);
                if (move.moveIt(pos, target)) {
                    target = pickUpSpecimen_T;
                    state = States.DROP_SAMPLE_S;
                    spitTime = System.currentTimeMillis() + 500;
                    driveTrain.stop();
                }
                break;

            case DROP_SAMPLE_S:
                robot.intakeServo.setPosition(1);
                if (System.currentTimeMillis() >= spitTime) {
                    robot.sampleMotor.setTargetPosition(0);
                    robot.armMotor.setTargetPosition(200);
                    target = lineUpSamplesAgain_T;
                    state = States.LINE_UP_ON_SAMPLE_JR_S;
                }
                break;

                //picking up a second sample to put into the observation zone
                //this is where we start changing things ^stays the same

            case LINE_UP_ON_SAMPLE_JR_S:
                if (move.moveIt(pos, target)) {
                    target = pickUpSampleAgain_T;
                    state = States.PICK_UP_SAMPLE_JR_S;
                    gyroTurn.goodEnough(target.h);
                    gyroTurn.goodEnough(target.h);
                }
                break;

            //need a new target for picking up those samples
            case PICK_UP_SAMPLE_JR_S:
                robot.armMotor.setTargetPosition(0);
                robot.intakeServo.setPosition(0);
                if (move.moveIt(pos, target)) {
                    robot.intakeServo.setPosition(.5);
                    robot.armMotor.setTargetPosition(500);
                    robot.sampleMotor.setTargetPosition(300);
                    target = driveToSpecimen_T;
                    state = States.DRIVE_TO_SPECIMEN_S;
                }
                break;

            case DRIVE_TO_SPECIMEN_S:
                robot.intakeServo.setPosition(.5);
                if (move.moveIt(pos, target)) {
                    target = pickUpSpecimen_T;
                    state = States.DROP_SAMPLE_JR_S;
                    spitTime = System.currentTimeMillis() + 500;
                    driveTrain.stop();
                }
                break;

            case DROP_SAMPLE_JR_S:
                robot.intakeServo.setPosition(1);
                if (System.currentTimeMillis() >= spitTime) {
                    robot.sampleMotor.setTargetPosition(0);
                    robot.armMotor.setTargetPosition(200);
                    time_to_reach_wall = (System.currentTimeMillis() + 600);
                    state = States.REACH_WALL_EXACTLY_S;
                }
                break;

                //get ready to pick up the first specimen from the wall

            case REACH_WALL_EXACTLY_S:
                robot.backLeftMotor.setPower(.4);
                robot.frontLeftMotor.setPower(-.4);
                robot.backRightMotor.setPower(-.4);
                robot.frontRightMotor.setPower(.4);
                robot.specimenMotor.setTargetPosition(robot.GRAB_SPECIMEN_WALL);
                if (System.currentTimeMillis() > time_to_reach_wall) {
                    driveTrain.stop();
                    state = States.WAIT_FOR_CLAWS_S;
                }
                break;

            case WAIT_FOR_CLAWS_S:
                robot.frontClawServo.setPosition(robot.FRONT_CLAW_CLOSE);
                robot.backClawServo.setPosition(robot.BACK_CLAW_CLOSE);
                timer = System.currentTimeMillis() + 200;
                while (System.currentTimeMillis() < timer)
                    Thread.yield();
                state  = States.LOADING;
                break;

            case LOADING:
                robot.intakeServo.setPosition(.5);
                robot.specimenMotor.setTargetPosition(robot.ABOVE_SECOND_BAR);
                if (!robot.specimenMotor.isBusy() || robot.specimenMotor.getCurrentPosition() > robot.ABOVE_THE_WALL) {
                    target = backUpFromWall_T;
                    state = States.BACK_UP_FROM_WALL_S;
                }
                break;

                //start moving to score the first wall specimen onto the submersible

            case BACK_UP_FROM_WALL_S:
                if (move.moveIt(pos, target)) {
                    target = turnNearSubmersibleAgain_T;
                    state = States.TURN_NEAR_SUBMERSIBLE_JR_S;
                }
                break;

            case TURN_NEAR_SUBMERSIBLE_JR_S:
                if (move.moveIt(pos, target)) {
                    target = driveToBarAgain_T;
                    state = States.DRIVE_TO_BAR_JR_S;
                }
                break;

            case DRIVE_TO_BAR_JR_S:
                if (move.moveIt(pos, target)) {
                    state = States.SCORING_JR;
                }
                break;

            case SCORING_JR:
                score();
                target = lineUpOnSpecimen_T;
                state = States.DRIVE_TO_SPECIMEN_JR_S;
                break;

                //get ready to pick up the second wall specimen

            case DRIVE_TO_SPECIMEN_JR_S:
                if (!robot.specimenMotor.isBusy()) {
                    robot.frontClawServo.setPosition(robot.FRONT_CLAW_OPEN_DOWN);
                    robot.backClawServo.setPosition(robot.BACK_CLAW_OPEN_DOWN);
                }
                if (move.moveIt(pos, target)) {
                    target = pickUpSpecimenAgain_T;
                    time_to_reach_wall = (System.currentTimeMillis() + 500);
                    state = States.REACH_WALL_EXACTLY_JR_S;
                }
                break;

            //Loading happens before hand and then it runs into the specimen on the wall

            case REACH_WALL_EXACTLY_JR_S:
                robot.backLeftMotor.setPower(.4);
                robot.frontLeftMotor.setPower(-.4);
                robot.backRightMotor.setPower(-.4);
                robot.frontRightMotor.setPower(.4);
                robot.specimenMotor.setTargetPosition(robot.GRAB_SPECIMEN_WALL);
                if (System.currentTimeMillis() > time_to_reach_wall) {
                    driveTrain.stop();
                    state = States.WAIT_FOR_CLAWS_JR_S;
                }
                break;

            case WAIT_FOR_CLAWS_JR_S:
                robot.frontClawServo.setPosition(robot.FRONT_CLAW_CLOSE);
                robot.backClawServo.setPosition(robot.BACK_CLAW_CLOSE);
                timer = System.currentTimeMillis() + 200;
                while (System.currentTimeMillis() < timer)
                    Thread.yield();
                state  = States.LOADING_JR;
                break;

            case LOADING_JR:
                robot.intakeServo.setPosition(.5);
                while (System.currentTimeMillis() < timer)
                    Thread.yield();
                robot.specimenMotor.setTargetPosition(robot.ABOVE_SECOND_BAR);
                robot.specimenMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.specimenMotor.setPower(1);
                if (!robot.specimenMotor.isBusy() || robot.specimenMotor.getCurrentPosition() > robot.ABOVE_THE_WALL) {
                    target = backUpFromWall_T;
                    state = States.BACK_UP_FROM_WALL_JR_S;
                }
                break;

            case BACK_UP_FROM_WALL_JR_S:
                if (move.moveIt(pos, target)) {
                    target = turnNearSubmersibleAgainAgain_T;
                    state = States.TURN_NEAR_SUBMERSIBLE_THIRD_S;
                }
                break;

            case TURN_NEAR_SUBMERSIBLE_THIRD_S:
                if (move.moveIt(pos, target)) {
                    target = driveToBarAgainAgain_T;
                    state = States.DRIVE_TO_BAR_THIRD_S;
                }
                break;

            case DRIVE_TO_BAR_THIRD_S:
                if (move.moveIt(pos, target)) {
                    state = States.SCORING_THIRD;
                }
                break;

            case SCORING_THIRD:
                score();
                target = driveToSpecimen_T;
                state = States.DRIVE_TO_SPECIMEN_THIRD_S;
                break;

//            case SCORING_THIRD:
//                score();
//                target = park_T;
//                state = States.PARK;
//                break;

                //get ready to pick up the third wall specimen

            //ALL NEW STUFF THE TARGETS NEED TO BE ADJUSTED

            case DRIVE_TO_SPECIMEN_THIRD_S:
                time_to_reach_wall = System.currentTimeMillis() + 600;
                if (move.moveIt(pos, target)) {
                    state = States.REACH_WALL_EXACTLY_THIRD_S;
                }
                break;

            case REACH_WALL_EXACTLY_THIRD_S:
                robot.backLeftMotor.setPower(.4);
                robot.frontLeftMotor.setPower(-.4);
                robot.backRightMotor.setPower(-.4);
                robot.frontRightMotor.setPower(.4);
                robot.specimenMotor.setTargetPosition(robot.GRAB_SPECIMEN_WALL);
                if (System.currentTimeMillis() > time_to_reach_wall) {
                    driveTrain.stop();
                    state = States.WAIT_FOR_CLAWS_THIRD_S;
                }
                break;

            case WAIT_FOR_CLAWS_THIRD_S:
                robot.frontClawServo.setPosition(robot.FRONT_CLAW_CLOSE);
                robot.backClawServo.setPosition(robot.BACK_CLAW_CLOSE);
                timer = System.currentTimeMillis() + 200;
                while (System.currentTimeMillis() < timer)
                    Thread.yield();
                state  = States.LOADING_THIRD;
                break;

            case LOADING_THIRD:
                robot.intakeServo.setPosition(.5);
                while (System.currentTimeMillis() < timer)
                    Thread.yield();
                robot.specimenMotor.setTargetPosition(robot.ABOVE_SECOND_BAR);
                robot.specimenMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.specimenMotor.setPower(1);
                if (!robot.specimenMotor.isBusy() || robot.specimenMotor.getCurrentPosition() > robot.ABOVE_THE_WALL) {
                    target = backUpFromWall_T;
                    state = States.BACK_UP_FROM_WALL_THIRD_S;
                }
                break;

            case BACK_UP_FROM_WALL_THIRD_S:
                if (move.moveIt(pos, target)) {
                    target = turnNearSubmersibleAgainAgainAgain_T;
                    state = States.TURN_NEAR_SUBMERSIBLE_FOURTH_S;
                }
                break;

            case TURN_NEAR_SUBMERSIBLE_FOURTH_S:
                if (move.moveIt(pos, target)) {
                    target = driveToBarAgainAgainAgain_T;
                    state = States.DRIVE_TO_BAR_FOURTH_S;
                }
                break;

            case DRIVE_TO_BAR_FOURTH_S:
                if (move.moveIt(pos, target)) {
                    state = States.SCORING_FOURTH;
                }
                break;

            case SCORING_FOURTH:
                score();
                target = park_T;
                state = States.PARK;
                break;


            case PARK:
                if (move.moveIt(pos, target)) {
                    robot.specimenMotor.setTargetPosition(robot.GRAB_SPECIMEN);
                    robot.frontClawServo.setPosition(robot.FRONT_CLAW_CLOSE);
                    robot.backClawServo.setPosition(robot.BACK_CLAW_CLOSE);
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
        telemetry.addData("Y Encoder:", robot.odo.getEncoderY());
        //prints/gets the current refresh rate of the Pinpoint
        telemetry.addData("Pinpoint Frequency", robot.odo.getFrequency());
        telemetry.addData("Status", robot.odo.getDeviceStatus());
        //prints the control system refresh rate
        telemetry.addData("REV Hub Frequency: ", frequency);
        telemetry.update();

        if (getRuntime() >= 30) {
            terminateOpModeNow();
        }
    }

    private void score() {
        boolean done = false;
        States scoreState = States.SCORING;

        while (!done) {
            switch (scoreState) {
                case SCORING:
                    robot.specimenMotor.setTargetPosition(robot.BELOW_SECOND_BAR);
                    //make a custom is busy where you can allow a room of wiggle room like a good enough
                    if (!robot.specimenMotor.isBusy()) {
                        scoreState = States.CLAWS_UP;
                    }
                    break;

                case CLAWS_UP:
                    robot.frontClawServo.setPosition(robot.FRONT_CLAW_OPEN_UP);
                    robot.backClawServo.setPosition(robot.BACK_CLAW_OPEN_UP);
                    robot.specimenMotor.setTargetPosition(robot.GRAB_SPECIMEN);
                    done = true;
                    break;
            }
        }
    }

    private boolean spin(Pose2D pos, Target target) {
        boolean done;
        done = false;
        if (Math.abs(pos.getHeading(AngleUnit.DEGREES) - Math.abs(target.h)) < target.tp.maxAngle)
            done = true;
        if (Math.abs(pos.getHeading(AngleUnit.DEGREES) - Math.abs(target.h)) > 90){
            robot.backRightMotor.setPower(-1);
            robot.frontRightMotor.setPower(-1);
            robot.backLeftMotor.setPower(1);
            robot.frontLeftMotor.setPower(1);
        }
        if (Math.abs(pos.getHeading(AngleUnit.DEGREES) - Math.abs(target.h)) > 60){
            robot.backRightMotor.setPower(-.7);
            robot.frontRightMotor.setPower(-.7);
            robot.backLeftMotor.setPower(.7);
            robot.frontLeftMotor.setPower(.7);
        }
        if (Math.abs(pos.getHeading(AngleUnit.DEGREES) - Math.abs(target.h)) > 15){
            robot.backRightMotor.setPower(-.4);
            robot.frontRightMotor.setPower(-.4);
            robot.backLeftMotor.setPower(.4);
            robot.frontLeftMotor.setPower(.4);
        }
        if (Math.abs(pos.getHeading(AngleUnit.DEGREES) - Math.abs(target.h)) > 5){
            robot.backRightMotor.setPower(-.1);
            robot.frontRightMotor.setPower(-.1);
            robot.backLeftMotor.setPower(.1);
            robot.frontLeftMotor.setPower(.1);
        }
        return done;
    }
}
