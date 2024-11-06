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
    double hawkTuahSpitOutThatThing;
    ReadSensor readSensor = new ReadSensor(robot, telemetry);
    double oldTime = 0;
    TargetProfile close = new TargetProfile(.4, .1, 2, 2, 4);
    Target awayFromWall_T = new Target(0,-6,0, close);
    Target closeToBasket_T = new Target(9, -22, 45, close);
    Target atTheBasket_T = new Target (23, -7, 45, close);
    Target cripWalkAwayFromBasket_T = new Target (3,-22,0, close);
    Target stareKrillDown_T = new Target (3,-36,0, close);
    Target eatKrill_T = new Target (13.5,-36,0, close);
    Target stakeOutLair_T = new Target (3,-51,0, close);
    Target hideInLair_T = new Target (-7,-51,0, close);


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
        robot.sampleMotor.setTargetPosition(robot.EXTEND_ARM_TO_BASKET);
        robot.armMotor.setTargetPosition(robot.RAISE_ARM_TO_BASKET);
        robot.sampleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
            case START:
                target = awayFromWall_T;
                state = States.RIDE_AT_DAWN_AWAY_FROM_WALL_S;
                break;

            case RIDE_AT_DAWN_AWAY_FROM_WALL_S:
                if (move.moveIt(pos, target)) {
                    target = closeToBasket_T;
                    state = States.DRIVE_CLOSE_TO_BASKET_S;
                }
                break;

            case DRIVE_CLOSE_TO_BASKET_S:
                if (move.moveIt(pos, target)) {
                    target = atTheBasket_T;
                    state = States.MOVE_ARM_1_S;
                    robot.frontLeftMotor.setPower(0);
                    robot.frontRightMotor.setPower(0);
                    robot.backLeftMotor.setPower(0);
                    robot.backRightMotor.setPower(0);
                }
                break;

            case MOVE_ARM_1_S:
                robot.sampleMotor.setTargetPosition(robot.EXTEND_ARM_TO_BASKET);
                robot.armMotor.setTargetPosition(robot.RAISE_ARM_TO_BASKET);
                while (robot.sampleMotor.isBusy() || robot.armMotor.isBusy()){
                    robot.sampleMotor.setPower(1);
                    robot.armMotor.setPower(1);
                }
                    state = States.AT_THE_BASKET_S;
                break;

            case AT_THE_BASKET_S:
                if (move.moveIt(pos, target)) {
                    target = cripWalkAwayFromBasket_T;
                    state = States.SPIT_OUT_KRILL_1_S;
                    hawkTuahSpitOutThatThing = System.currentTimeMillis() + 1000;
                }
                break;


            case SPIT_OUT_KRILL_1_S:
                robot.intakeServo.setPosition(1);
                if (System.currentTimeMillis() >= hawkTuahSpitOutThatThing)
                {
                    target = cripWalkAwayFromBasket_T;
                    robot.intakeServo.setPosition(.5);
                    state = States.BACK_AWAY_FROM_BASKET_S;
                }

                break;

            case BACK_AWAY_FROM_BASKET_S:
                robot.sampleMotor.setTargetPosition(0);
                if (move.moveIt(pos, target)) {
                    target = stareKrillDown_T;
                    robot.armMotor.setTargetPosition(0);
                    state = States.LOWER_ARM_S;
                }
                break;

            case LOWER_ARM_S:
                state = States.DONE_FOR_NOW;
                break;

            case HEAD_TOWARDS_KRILL_L_S:
                state = States.DONE_FOR_NOW;
                break;

            case DONE_FOR_NOW:
                robot.frontLeftMotor.setPower(0);
                robot.frontRightMotor.setPower(0);
                robot.backLeftMotor.setPower(0);
                robot.backRightMotor.setPower(0);
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
}