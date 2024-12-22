package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;


@TeleOp(name="Robot: LeftOdometery", group="Robot")
public class LeftOdometery extends OpMode {


    RobotHardware robot = new RobotHardware();
    Move move;
    MotorSpeeds motorSpeeds;
    States state;
    double timeToStop;
    double spitTime;
    DriveTrain driveTrain = new DriveTrain(robot);
    ReadSensor readSensor = new ReadSensor(robot, telemetry);
    double oldTime = 0;
    //Target Profiles
    TargetProfile batOutOfHell = new TargetProfile(1,.85,10, 15, 5);
    TargetProfile wayPoint = new TargetProfile(.6, .2, 4, 10, 3);
    TargetProfile close = new TargetProfile(.5, .1, 2, 5, 5);
    TargetProfile closer = new TargetProfile(.3, .1, 1, 3, 8);
    TargetProfile samplePickup = new TargetProfile(.2, .2, 1, 4, .1);
    TargetProfile specimenPickup = new TargetProfile(.85, .1, 2, 3, 5);
//    TargetProfile close = new TargetProfile(.4, .1, 2, 2, 4);
//    TargetProfile samplePickup = new TargetProfile(.2, .1, 1,5, 2);

    //Targets
    Target offWall_T = new Target(0,-7, 0, wayPoint);
    Target toBasket_T = new Target (8, -20, 25, wayPoint);
    Target lineUpBasket_T = new Target (12, -14, 25, wayPoint);
    Target atBasket_T = new Target (23.5,-9.25,45, closer);
    Target awayFromBasket_T = new Target (7,-17,0, wayPoint);
    Target lineupSample_T = new Target(3,-36,0, close);
    Target lineUpSampleAgain_T = new Target (9, -36, 0, close);
    Target loadSample_T = new Target(11,-36,0, samplePickup);
    Target loadSampleAgain_T = new Target (14,-36,0, samplePickup);
    Target loadSampleAgainAgain_T = new Target (10, -36, 0, samplePickup);



    Target target = new Target();


    @Override
    public void init() {
        robot.init(hardwareMap);
        robot.backClawServo.setPosition(robot.BACK_CLAW_CLOSE);
        robot.frontClawServo.setPosition(robot.FRONT_CLAW_CLOSE);
        motorSpeeds = new MotorSpeeds(robot, telemetry);
        move = new Move(robot, telemetry, motorSpeeds);
        state = States.START;
    }
    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        timeToStop = System.currentTimeMillis()+30000;
        robot.sampleMotor.setTargetPosition(0);
        robot.sampleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armMotor.setTargetPosition(300);
        robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);;
        robot.armMotor.setPower(1);
        resetRuntime();
    }

    @Override
    public void loop() {
        robot.odo.bulkUpdate();
        double newTime = getRuntime();
        double loopTime = newTime-oldTime;
        double frequency = 1/loopTime;


        oldTime = newTime;
        Pose2D pos = robot.odo.getPosition();
        String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}",
                pos.getX(DistanceUnit.MM),
                pos.getY(DistanceUnit.MM),
                pos.getHeading(AngleUnit.DEGREES));
        Pose2D vel = robot.odo.getVelocity();
        String velocity = String.format(Locale.US,"{XVel: %.3f, YVel: %.3f, HVel: %.3f}", vel.getX(DistanceUnit.MM), vel.getY(DistanceUnit.MM), vel.getHeading(AngleUnit.DEGREES));

        switch (state){
            //SCORE FIRST SAMPLE (PRELOADED)
            case START:
                robot.armMotor.setTargetPosition(robot.RAISE_ARM_TO_BASKET);
                target = offWall_T;
                state = States.OFF_WALL_S;
                break;

            case OFF_WALL_S:
                if (move.moveIt(pos, target)) {
                    target = lineUpBasket_T;
                    state = States.LINE_UP_BASKET_S;
                }
                break;

            case LINE_UP_BASKET_S:
                if (move.moveIt(pos, target)) {
                    target = atBasket_T;
                    state = States.WAIT_FOR_ARM_S;
                }
                break;

            case WAIT_FOR_ARM_S:
                if (!robot.armMotor.isBusy()){
                    robot.sampleMotor.setTargetPosition(robot.EXTEND_ARM_TO_BASKET);
                    state = States.AT_BASKET_S;
                }
                break;

            case AT_BASKET_S:
                if (move.moveIt(pos, target)) {
                    scoreSample();
                    target = lineupSample_T;
                    state = States.LINEUP_SAMPLE_S;

                }
                break;

                //PICK UP SECOND SAMPLE
            case LINEUP_SAMPLE_S:
                if (move.moveIt(pos, target)) {
                    target = loadSample_T;
                    state = States.LOAD_SAMPLE_S;
                }
                break;

            case LOAD_SAMPLE_S:
                robot.intakeServo.setPosition(0);
                robot.armMotor.setTargetPosition(150);
                if (move.moveIt(pos,target)){
                    robot.intakeServo.setPosition(.5);
                    robot.armMotor.setTargetPosition(300);
                    target = toBasket_T;
                    state = States.TO_BASKET_AGAIN_S;
                }
                break;
                //SCORE SECOND SAMPLE

            case TO_BASKET_AGAIN_S:
               if (move.moveIt(pos, target)){
                   robot.armMotor.setTargetPosition(robot.RAISE_ARM_TO_BASKET);
                   state = States.WAIT_FOR_ARM_AGAIN_S;
               }
               break;

            case WAIT_FOR_ARM_AGAIN_S:
                if (!robot.armMotor.isBusy()){
                    robot.sampleMotor.setTargetPosition(robot.EXTEND_ARM_TO_BASKET);
                    target = lineUpBasket_T;
                    state = States.LINE_UP_BASKET_AGAIN_S;
                }
                break;

            case LINE_UP_BASKET_AGAIN_S:
                if (move.moveIt(pos, target)){
                    target = atBasket_T;
                    state = States.AT_BASKET_AGAIN_S;
                }
                break;

            case AT_BASKET_AGAIN_S:
                if (move.moveIt(pos,target)) {
                    scoreSample();
                    target = lineUpSampleAgain_T;
                    state = States.LINEUP_SAMPLE_AGAIN_S;
                }
                break;


                //PICK UP THIRD SAMPLE
            case LINEUP_SAMPLE_AGAIN_S:
                if (move.moveIt(pos,target)) {
                    state = States.LOAD_SAMPLE_AGAIN_S;
                    target = loadSampleAgain_T;
                }
                break;

            case LOAD_SAMPLE_AGAIN_S:
                robot.intakeServo.setPosition(0);
                robot.armMotor.setTargetPosition(150);
                if (move.moveIt(pos,target)){
                    state = States.TO_BASKET_3_S;
                    target = awayFromBasket_T;
                }
                break;
                //SCORE THIRD SAMPLE
            //the stuff after this is USELESS

            case TO_BASKET_3_S:
                if(move.moveIt(pos,target)){
                    scoreSample();
                    target = loadSampleAgainAgain_T;
                    state = States.LINEUP_SAMPLE_3_S;
                }
                break;

            case LINEUP_SAMPLE_3_S:
                if(move.moveIt(pos,target)){
                    state = States.LOAD_SAMPLE_3_S;
                    target = loadSampleAgainAgain_T;
                }
                break;

            case LOAD_SAMPLE_3_S:
                robot.intakeServo.setPosition(0);
                robot.armMotor.setTargetPosition(150);
                if(move.moveIt(pos,target)){
                    state = States.TO_BASKET_4_S;
                    target = awayFromBasket_T;
                }
                break;

            case TO_BASKET_4_S:
                if (move.moveIt(pos,target)){
                    scoreSample();
                    state = States.DONE_FOR_NOW;
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

        telemetry.update();

        if (getRuntime() >= 30){
            terminateOpModeNow();
        }
    }
    private void scoreSample(){
        boolean done = false;
        States score_s = States.RAISE_EXTEND_ARM_S;
        while (!done){
            robot.odo.bulkUpdate();
            Pose2D pos = robot.odo.getPosition();
            switch (score_s){
                case RAISE_EXTEND_ARM_S:

                    robot.armMotor.setTargetPosition(robot.RAISE_ARM_TO_BASKET);
                    robot.sampleMotor.setTargetPosition(robot.EXTEND_ARM_TO_BASKET);
                    robot.armMotor.setPower(1);
                    while (robot.armMotor.isBusy())
                        Thread.yield();
                    robot.sampleMotor.setPower(1);
                    while (robot.sampleMotor.isBusy())
                        Thread.yield();
                    score_s =States.AT_BASKET_S;
                    target = atBasket_T;
                    break;

                case AT_BASKET_S:
                    if (move.moveIt(pos, target)) {
                        score_s = States.SCORE_SAMPLE_S;
                        spitTime = System.currentTimeMillis() + 1000;
                    }
                    break;

                case SCORE_SAMPLE_S:
                    robot.intakeServo.setPosition(1);
                    if (System.currentTimeMillis() >= spitTime) {
                        robot.intakeServo.setPosition(.5);
                        score_s = States.RETRACT_SAMPLE_MOTOR_S;
                    }
                    break;

                case RETRACT_SAMPLE_MOTOR_S:
                    robot.sampleMotor.setTargetPosition(100);
                    target = awayFromBasket_T;
                    score_s = States.AWAY_FROM_BASKET_S;
                    break;

                case AWAY_FROM_BASKET_S:
                    if (move.moveIt(pos, target)) {
                        robot.armMotor.setTargetPosition(robot.GRAB_SAMPLE);
                        while (robot.armMotor.isBusy())
                            Thread.yield();
                        score_s = States.LINEUP_SAMPLE_S;
                        target = lineupSample_T;
                        done = true;
                    }
                    break;


            } //end switch
            telemetry.addData("current state",score_s);
            telemetry.update();
        } //end while
    } //end private void
}