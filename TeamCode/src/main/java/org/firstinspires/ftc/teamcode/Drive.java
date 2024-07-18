package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RobotHardware.ONE_PIXEL_BOARD_DISTANCE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Drive {
    //fix that

    RobotHardware robot;
    Telemetry telemetry;
    LinearOpMode linearOpMode;
    DriveTrain driveTrain;
    ReadSensor readSensor;


    public Drive(RobotHardware robot, Telemetry telemetry, LinearOpMode linearOpMode){
        this.robot = robot;
        this.telemetry = telemetry;
        this.linearOpMode = linearOpMode;
        this.driveTrain = new DriveTrain(robot, telemetry,linearOpMode);
    }

    public void forward (int distance, double speed){
        if (!linearOpMode.opModeIsActive())
            return;
        driveTrain.stop_and_reset_enconders();
        driveTrain.run_using_encoder();
        driveTrain.setTargetPosition(robot.CLICKS_PER_CENTIMETER * distance);
        driveTrain.run_to_position();
        driveTrain.setSpeed(speed);
        while (driveTrain.isBusy() && linearOpMode.opModeIsActive())
            Thread.yield();
        driveTrain.stop();
    }

    public void backward (int distance, double speed){
        if (!linearOpMode.opModeIsActive())
            return;
        driveTrain.stop_and_reset_enconders();
        driveTrain.run_using_encoder();
        driveTrain.setTargetPosition(robot.CLICKS_PER_CENTIMETER * -distance);
        driveTrain.run_to_position();
        driveTrain.setSpeed(-speed);
        while (driveTrain.isBusy() && linearOpMode.opModeIsActive())
            Thread.yield();
        driveTrain.stop();
    }
    public void backward_auto (int distance, double speed, int howHigh) throws InterruptedException {
        if (!linearOpMode.opModeIsActive())
            return;
        driveTrain.stop_and_reset_enconders();
        driveTrain.run_using_encoder();
        driveTrain.setTargetPosition(robot.CLICKS_PER_CENTIMETER * -distance);
        driveTrain.run_to_position();
        driveTrain.setSpeed(-speed);

        while ((driveTrain.isBusy() && linearOpMode.opModeIsActive())){
            Thread.yield();
        }


        driveTrain.stop();
    }

    public void forward_auto (int distance, double speed, int howHigh) throws InterruptedException {
        if (!linearOpMode.opModeIsActive())
            return;
        driveTrain.stop_and_reset_enconders();
        driveTrain.run_using_encoder();
        driveTrain.setTargetPosition(robot.CLICKS_PER_CENTIMETER * distance);
        driveTrain.run_to_position();
        driveTrain.setSpeed(speed);

        while ((driveTrain.isBusy() && linearOpMode.opModeIsActive())){
            Thread.yield();
        }


        driveTrain.stop();
    }

    public void move_to_backboard_one_pixel (int back_distance){
        if (!linearOpMode.opModeIsActive())
            return;
        if (back_distance > robot.ONE_PIXEL_BOARD_DISTANCE){
            backward (Math.abs((int)back_distance- robot.ONE_PIXEL_BOARD_DISTANCE), .2);
            telemetry.addData("amountMovedBackward", (Math.abs((int)back_distance- robot.ONE_PIXEL_BOARD_DISTANCE)));
            telemetry.update();
        }
        if (back_distance < robot.ONE_PIXEL_BOARD_DISTANCE){
            forward (Math.abs((int)back_distance- robot.ONE_PIXEL_BOARD_DISTANCE), .2);
            telemetry.addData("amountMovedForward", (Math.abs((int)back_distance- robot.ONE_PIXEL_BOARD_DISTANCE)));
        }
    }

    public void move_to_backboard_two_pixel (int back_distance){
        if (!linearOpMode.opModeIsActive())
            return;
        if (back_distance > robot.TWO_PIXEL_BOARD_DISTANCE){
            backward (Math.abs((int)back_distance- robot.TWO_PIXEL_BOARD_DISTANCE), .2);
            telemetry.addData("amountMovedBackward", (Math.abs((int)back_distance- robot.TWO_PIXEL_BOARD_DISTANCE)));
            telemetry.update();
        }
        if (back_distance < robot.TWO_PIXEL_BOARD_DISTANCE){
            forward (Math.abs((int)back_distance- robot.TWO_PIXEL_BOARD_DISTANCE), .2);
            telemetry.addData("amountMovedForward", (Math.abs((int)back_distance- robot.TWO_PIXEL_BOARD_DISTANCE)));
        }
    }
}


