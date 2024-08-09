package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Robot: LeftBlueOdometry", group="Robot")
public class LeftBlueOdometry extends OpMode {


    RobotHardware robot = new RobotHardware();
    Move move;
    MotorSpeeds motorSpeeds;
    Target findProp = new Target(0, 27.25, 0, .2);
    Target placePurpleCenter = new Target(0, 34.5, 0, .1);
    Target backupCenter = new Target(0, 21.5, 0, .1);
    Target turnToBoard = new Target(3, 21.5, 90, .2);
    Target backupToBoard = new Target();
    Target strafeToCenter = new Target();
    Target strafeToParkCenter = new Target();
    


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        motorSpeeds = new MotorSpeeds(robot);
        move = new Move(robot, telemetry, motorSpeeds);

    }

    @Override
    public void loop() {

        SparkFunOTOS.Pose2D pos = robot.myOtos.getPosition();
        SparkFunOTOS.Pose2D target = new SparkFunOTOS.Pose2D(27, 27, 0);
        move.moveIt(pos, findProp);

        //Telemetry Data
        telemetry.addData("X coordinate", pos.x);
        telemetry.addData("Y coordinate", (pos.y));
        telemetry.addData("Heading angle", pos.h);
        telemetry.addData("Say", "Happy Little Pixels");
        telemetry.update();


    }
}