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
    TargetProfile close = new TargetProfile(.4, .1, 2, 2, 4);
    TargetProfile samplePickup = new TargetProfile(.2, .1, 1,5, 2);
    Target toBasket_T = new Target (0,-17,0,close);
    Target atBasket_T = new Target (11.5,-17,0,close);
    Target lineupSample1_T = new Target(-24.44,-22.33,-45,close);
    Target loadSample1_T = new Target(-11,-29,-45,samplePickup);


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
            case START:
                target = toBasket_T;
                state = States.TO_BASKET_1_S;
                break;

            case TO_BASKET_1_S:
                if (move.moveIt(pos, target)) {
                    state = States.RAISE_EXTEND_ARM_S;
                    driveTrain.stop();
                }
                break;

            case RAISE_EXTEND_ARM_S:

                robot.armMotor.setTargetPosition(robot.RAISE_ARM_TO_BASKET);
                robot.sampleMotor.setTargetPosition(robot.EXTEND_ARM_TO_BASKET);
                robot.armMotor.setPower(1);
                while (robot.armMotor.isBusy())
                    Thread.yield();
                robot.sampleMotor.setPower(1);
                while (robot.sampleMotor.isBusy())
                    Thread.yield();
                state =States.AT_BASKET_1_S;
                target = atBasket_T;
                break;

            case AT_BASKET_1_S:
                if (move.moveIt(pos, target)) {
                    state = States.SCORE_SAMPLE_1_S;
                   spitTime = System.currentTimeMillis() + 1500;
                }
                break;

            case SCORE_SAMPLE_1_S:
                robot.intakeServo.setPosition(1);
                if (System.currentTimeMillis() >= spitTime) {
                    robot.intakeServo.setPosition(.5);
                    state = States.RETRACT_ARM_1_S;
                }
                break;

            case RETRACT_ARM_1_S:
                robot.sampleMotor.setTargetPosition(100);
                while (robot.sampleMotor.isBusy())
                    Thread.yield();
                target = toBasket_T;
                state = States.TO_BASKET_2_S;
                break;

            case TO_BASKET_2_S:
                if (move.moveIt(pos, target)) {
                    robot.armMotor.setTargetPosition(200);
                    while (robot.armMotor.isBusy())
                        Thread.yield();
                    state = States.LINEUP_SAMPLE_1_S;
                    target = lineupSample1_T;
                }
                break;

            case LINEUP_SAMPLE_1_S:
                if (move.moveIt(pos, target)) {
                    state = States.LOAD_SAMPLE_1_S;
                    target = loadSample1_T;
                }
                break;

            case LOAD_SAMPLE_1_S:
                robot.intakeServo.setPosition(0);
                if (move.moveIt(pos,target)){
                    state = States.TO_BASKET_3_S;
                    target = toBasket_T;
                }
                break;

            case TO_BASKET_3_S:
               if (move.moveIt(pos, target)){
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
}