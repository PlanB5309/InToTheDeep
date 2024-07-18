package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class GyroTurn {
    RobotHardware robot;
    Telemetry telemetry;
    LinearOpMode linearOpMode;
    double currHeading;

    // State used for updating telemetry
    private Orientation angles;
    private Acceleration gravity;
    private DriveTrain driveTrain;

    public GyroTurn(RobotHardware robot, Telemetry telemetry, LinearOpMode linearOpMode) {
        this.robot = robot;
        this.telemetry = telemetry;
        this.linearOpMode = linearOpMode;
        driveTrain = new DriveTrain(robot, telemetry, linearOpMode);
    }

    public void goodEnough(double target) {

        double diff;
        double speed;

        if (!linearOpMode.opModeIsActive())
            return;
        driveTrain.stop_and_reset_enconders();
        driveTrain.run_using_encoder();
        updateHeading();

        while (Math.abs(currHeading - target) > 1 && linearOpMode.opModeIsActive()) {

            diff = Math.abs(currHeading - target);
            speed = TurnSpeed(diff);

            if (target > currHeading)
                TurnRight(speed);
            else if (target < currHeading)
                TurnLeft(speed);

            updateHeading();

        }
        driveTrain.stop ();
    }



    private double TurnSpeed (double diff) {

        if (diff > 80) {
        return robot.HIGH_TURN_POWER;
        }
        else if (diff < 5) {
            return robot.LOW_TURN_POWER;
        }
        else
            return .005667 * diff + .046667;
    }

    private void TurnLeft (double speed) {

        robot.frontLeftMotor.setPower(speed);
        robot.backLeftMotor.setPower(speed);
        robot.frontRightMotor.setPower(-speed);
        robot.backRightMotor.setPower(-speed);

    }

    private void TurnRight (double speed) {

        robot.frontLeftMotor.setPower(-speed);
        robot.backLeftMotor.setPower(-speed);
        robot.frontRightMotor.setPower(speed);
        robot.backRightMotor.setPower(speed);

    }

    public void updateHeading() {
        angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gravity = robot.imu.getGravity();
        currHeading = angles.firstAngle;
        telemetry.addData("Heading: ", currHeading);
        telemetry.update();
        }
}
