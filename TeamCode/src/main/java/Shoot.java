import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Shoot {
    RobotHardware robot;
    Telemetry telemetry;
    LinearOpMode  opMode;

    public Shoot (RobotHardware robot, Telemetry telemetry, LinearOpMode opMode) {
        this.robot = robot;
        this.telemetry = telemetry;
        this.opMode = opMode;
    }

    public void threeBalls (double first, double second, double third) throws InterruptedException {
        if (opMode.opModeIsActive()) {
            robot.setRevolverPosition(first);
            Thread.sleep(1500);
            kickball();
        }
        if (opMode.opModeIsActive()) {
            robot.setRevolverPosition(second);
            Thread.sleep(1500);
            kickball();
        }
        if (opMode.opModeIsActive()) {
            robot.setRevolverPosition(third);
            Thread.sleep(1500);
            kickball();
        }
    }

    private void kickball() throws InterruptedException {
        robot.setKickerPosition(RobotHardware.KICK_POSITION);
        Thread.sleep(500);
        robot.setKickerPosition(RobotHardware.KICK_RESET);
        Thread.sleep(500);
    }
}
