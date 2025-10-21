package org.firstinspires.ftc.teamcode;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class GyroTurn {
    RobotHardware robot;
    Telemetry telemetry;
    double currHeading;

    // State used for updating telemetry

    private DriveTrain driveTrain;

    public GyroTurn(RobotHardware robot, Telemetry telemetry) {
        this.robot = robot;
        this.telemetry = telemetry;

        driveTrain = new DriveTrain(robot);
    }

    public void goodEnough(double target) {

        double diff;
        double speed;

        updateHeading();

        while (Math.abs(currHeading - target) > 1 ) {

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
        robot.odo.bulkUpdate();
        Pose2D pos = robot.odo.getPosition();
        currHeading = pos.getHeading(AngleUnit.DEGREES);
        telemetry.addData("Heading: ", currHeading);
        telemetry.update();
        }
}
