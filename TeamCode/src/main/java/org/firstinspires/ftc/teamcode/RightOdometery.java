package org.firstinspires.ftc.teamcode;

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
    Target driveToSubmersible_T = new Target(14, -30, 0, .5, Target.Destination.DESTINATION);
    Target backAwayFromSubmersible_T = new Target(14,-27,0,.4, Target.Destination.DESTINATION);
    Target towardsSamples_T = new Target( -21, -27, 0, .4, Target.Destination.DESTINATION);
    Target stalkKrill_T = new Target(-21,-48,0,.5, Target.Destination.WAYPOINT);
    Target eatKrill_T = new Target(-30,-51,0,.5, Target.Destination.WAYPOINT);
    Target park_T = new Target(-30,-2,0,.3, Target.Destination.WAYPOINT);
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
        timeToStop = System.currentTimeMillis()+30000;
        robot.specimenMotor.setTargetPosition(robot.ABOVE_SECOND_BAR);
        robot.specimenMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.specimenMotor.setPower(1);
        target = driveToSubmersible_T;
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
                state = States.DRIVE_TO_BAR_S;
                break;

            case DRIVE_TO_BAR_S:
                if (move.moveIt(pos, target)) {
                    state = States.LOWER_SPECIMEN_LIFT_S;
                }
                break;


            case LOWER_SPECIMEN_LIFT_S:
                robot.specimenMotor.setTargetPosition(robot.BELOW_SECOND_BAR);
                while (robot.specimenMotor.isBusy()){
                    robot.specimenMotor.setPower(-1);
                }
                state = States.OPEN_CLAWS_S;
                break;

            case OPEN_CLAWS_S:
                robot.frontClawServo.setPosition(robot.FRONT_CLAW_OPEN);
                robot.backClawServo.setPosition(robot.BACK_CLAW_OPEN);
                target = backAwayFromSubmersible_T;
                    state = States.BACK_UP_FROM_SUBMERISBLE_S;
                break;

            case BACK_UP_FROM_SUBMERISBLE_S:
                if (move.moveIt(pos, target)) {
                    target = towardsSamples_T;
                    state = States.HEAD_TOWARDS_KRILL_R_S;
                }
                break;

            case HEAD_TOWARDS_KRILL_R_S:
                if (move.moveIt(pos, target)) {
                    target = stalkKrill_T;
                    state = States.EAT_KRILL_R_S;
                }
                break;

            case EAT_KRILL_R_S:
                if (move.moveIt(pos, target)) {
                    target = eatKrill_T;
                    state = States.PARK;
                }
                break;


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
                break;
        }

        //Telemetry Data
        telemetry.addData("Current State", state);
        telemetry.addData("TargetX", target.x);
        telemetry.addData("TargetY", target.y);
        telemetry.addData("TargetH", target.h);
        telemetry.addData("Target Maxspeed", target.maxSpeed);
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