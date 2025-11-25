import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Load {
    RobotHardware robot;
    Telemetry telemetry;
    LinearOpMode  opMode;
    Drive drive = new Drive(robot, telemetry, opMode);

    public Load (RobotHardware robot, Telemetry telemetry, LinearOpMode opMode) {
        this.robot = robot;
        this.telemetry = telemetry;
        this.opMode = opMode;
    }

    public void threeBalls (double first, double second, double third) throws InterruptedException {
        robot.setIntakeSpeed(1);
        if (opMode.opModeIsActive()) {
            robot.setRevolverPosition(first);
            Thread.sleep(1500);
            drive.backward(10,.5);

        }
        if (opMode.opModeIsActive()) {
            robot.setRevolverPosition(second);
            Thread.sleep(1500);
            drive.backward(20,.5);
        }
        if (opMode.opModeIsActive()) {
            robot.setRevolverPosition(third);
            Thread.sleep(1500);
            drive.backward(30,.5);
        }
    }

}