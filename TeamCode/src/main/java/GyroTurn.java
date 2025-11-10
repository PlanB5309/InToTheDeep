import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class GyroTurn {
    RobotHardware robot;
    Telemetry telemetry;
    double currHeading;

    // State used for updating telemetry

    public GyroTurn(RobotHardware robot, Telemetry telemetry) {
        this.robot = robot;
        this.telemetry = telemetry;
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
        robot.setDrivePower(0,0); // stop turning
    }


    // Determine how fast to turn based on how far we are from the target
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

    // Turn the robot left at the desired speed
    private void TurnLeft (double speed) {
        robot.setDrivePower(speed, -speed);

    }

    // Turn the robot right at the desired speed
    private void TurnRight (double speed) {
        robot.setDrivePower(-speed, speed);
    }

    public void updateHeading() {
        robot.odo.bulkUpdate();
        Pose2D pos = robot.odo.getPosition();
        currHeading = pos.getHeading(AngleUnit.DEGREES);
        telemetry.addData("Heading: ", currHeading);
        telemetry.update();
        }
}
