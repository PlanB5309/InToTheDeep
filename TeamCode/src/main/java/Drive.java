import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Drive {
    RobotHardware robot;
    Telemetry telemetry;
    LinearOpMode opMode;

    public Drive (RobotHardware robot, Telemetry telemetry, LinearOpMode opMode) {
        this.robot = robot;
        this.telemetry = telemetry;
        this.opMode = opMode;
    }
    public void forward(double distance, double speed) {
        robot.resetDriveEncoders();
        robot.setTargets((int)distance);
        robot.driveWhileBusy(speed, 3.0);
    }

    public void backward(double distance, double speed) {
        robot.resetDriveEncoders();
        robot.setTargets((int)-distance);
        robot.driveWhileBusy(-speed, 3.0);
    }
}
