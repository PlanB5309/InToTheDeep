import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.List;


@Autonomous(name="Robot: SlowLoad", group="Robot")
public class slowLoad extends LinearOpMode {
    // Create a RobotHardware object to be used to access robot hardware.
    // Prefix any hardware functions with "robot." to access this class.
    RobotHardware robot       = new RobotHardware(this);
    GyroTurn gyroTurn = new GyroTurn(robot,telemetry);
    patterns pattern = patterns.PGP;
    @Override
    public void runOpMode() {

        // initialize all the hardware, using the hardware class. See how clean and simple this is?
        robot.init();

        robot.resetDriveEncoders();
        //robot.limelight.start();

        // Wait for the game to start (driver presses START)

        waitForStart();
        robot.setIntakeSpeed(1);
        robot.setRevolverPosition(RobotHardware.LOAD_1);
        sleep(1000);
        backward(10 * robot.CLICKS_PER_CENTIMETER, .05);
        robot.setRevolverPosition(RobotHardware.LOAD_2);
        sleep(1000);
        backward(20 * robot.CLICKS_PER_CENTIMETER, .05);
        robot.setRevolverPosition(RobotHardware.LOAD_3);
        sleep(1000);
        backward(30 * robot.CLICKS_PER_CENTIMETER, .05);
    }

    private void backward(double distance, double speed) {
        robot.resetDriveEncoders();
        robot.setTargets((int)-distance);
        robot.driveWhileBusy(-speed, 3.0);
    }
}
