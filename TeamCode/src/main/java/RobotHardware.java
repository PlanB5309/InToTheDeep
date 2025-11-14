/* Copyright (c) 2022 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;

/*
 * This file works in conjunction with the External Hardware Class sample called: ConceptExternalHardwareClass.java
 * Please read the explanations in that Sample about how to use this class definition.
 *
 * This file defines a Java Class that performs all the setup and configuration for a sample robot's hardware (motors and sensors).
 * It assumes three motors (left_drive, right_drive and arm) and two servos (left_hand and right_hand)
 *
 * This one file/class can be used by ALL of your OpModes without having to cut & paste the code each time.
 *
 * Where possible, the actual hardware objects are "abstracted" (or hidden) so the OpMode code just makes calls into the class,
 * rather than accessing the internal hardware directly. This is why the objects are declared "private".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with *exactly the same name*.
 *
 * Or... In OnBot Java, add a new file named RobotHardware.java, select this sample, and select Not an OpMode.
 * Also add a new OpMode, select the sample ConceptExternalHardwareClass.java, and select TeleOp.
 *
 */

public class RobotHardware {

    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    private DcMotor leftDrive   = null;
    private DcMotor rightDrive  = null;
    private DcMotor rightLaunch = null;
    private DcMotor leftLaunch = null;
    private DcMotor activeIntake = null;
    public Limelight3A limelight;

    // Define servos
    private Servo kicker = null;
    private Servo revolver = null;
    private Servo angle = null;

    // object for pin point computer
    public GoBildaPinpointDriver odo = null;

    // Define Revolver constants.
    // Make them public so they CAN be used by the calling OpMode
    public static final double LOAD_1 =  .11;
    public static final double LAUNCH_1 = .47;
    public static final double LOAD_2 = .34;
    public static final double LAUNCH_2   = .725;
    public static final double LOAD_3   =  .63;
    public static final double LAUNCH_3 =  .98;
    public static final double KICK_POSITION = 0;
    public static final double KICK_RESET = .4;
    public static final double CLICKS_PER_CENTIMETER = 23;
    static final double        DRIVE_SPEED             = 0.6;
    static final double        TURN_SPEED              = 0.5;

    // Set Turn Speed Constants
    public final double HIGH_TURN_POWER = 0.52;
    public final double LOW_TURN_POWER = 0.1;

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public RobotHardware (LinearOpMode opmode) {
        myOpMode = opmode;
    }

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * <p>
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    public void init()    {
        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        leftDrive  = myOpMode.hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = myOpMode.hardwareMap.get(DcMotor.class, "rightDrive");
        rightLaunch   = myOpMode.hardwareMap.get(DcMotor.class, "rightLaunch");
        leftLaunch = myOpMode.hardwareMap.get(DcMotor.class, "leftLaunch");
        activeIntake = myOpMode.hardwareMap.get(DcMotor.class, "activeIntake");


        // Define the servos in the hardware map
        kicker = myOpMode.hardwareMap.get(Servo.class, "kicker");
        revolver = myOpMode.hardwareMap.get(Servo.class, "revolver");
        angle = myOpMode.hardwareMap.get(Servo.class, "angle");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        activeIntake.setDirection(DcMotor.Direction.REVERSE);

        // Reverse one of the launch motors
        rightLaunch.setDirection(DcMotor.Direction.FORWARD);
        leftLaunch.setDirection(DcMotor.Direction.REVERSE);

        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        leftLaunch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLaunch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        odo = myOpMode.hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.resetPosAndIMU();

        // Map the limelight object from the hardware map
        limelight = myOpMode.hardwareMap.get(Limelight3A .class, "limelight");
        myOpMode.telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);

        //set servo positions
        revolver.setPosition(LOAD_1);
        kicker.setPosition(KICK_RESET);

        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }

    /**
     * Calculates the left/right motor powers required to achieve the requested
     * robot motions: Drive (Axial motion) and Turn (Yaw motion).
     * Then sends these power levels to the motors.
     *
     * @param Drive     Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param Turn      Right/Left turning power (-1.0 to 1.0) +ve is CW
     */
    public void driveRobot(double Drive, double Turn) {
        // Combine drive and turn for blended motion.
        double left  = Drive + Turn;
        double right = Drive - Turn;

        // Scale the values so neither exceed +/- 1.0
        double max = Math.max(Math.abs(left), Math.abs(right));
        if (max > 1.0)
        {
            left /= max;
            right /= max;
        }

        // Use existing function to drive both wheels.
        setDrivePower(left, right);
    }

    /**
     * Pass the requested wheel motor powers to the appropriate hardware drive motors.
     *
     * @param leftWheel     Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param rightWheel    Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     */
    public void setDrivePower(double leftWheel, double rightWheel) {
        // Output the values to the motor drives.
        leftDrive.setPower(leftWheel);
        rightDrive.setPower(rightWheel);
    }

    public void bumpAnglePosition() {
        angle.setPosition(angle.getPosition() +.0001);
    }

    public void lowerAnglePosition() {
        angle.setPosition(angle.getPosition() -.0001);
    }

    public double getAnglePosition() {
        return angle.getPosition();
    }

    public void setAngle(double setAngle){
        angle.setPosition(setAngle);
    }

    public void setRevolverPosition(double barrelSetting) {
        revolver.setPosition(barrelSetting);
    }

    public void setLaunchSpeed(double speed) {
        leftLaunch.setPower(speed);
        rightLaunch.setPower(speed);
    }

    public void setIntakeSpeed(double speed) {
        activeIntake.setPower(speed);
    }

    public void incAngle(double degrees) {
        angle.setPosition(angle.getPosition() + degrees);
    }

    public double getAngle() {
        return angle.getPosition();
    }

    public void setKickerPosition(double degrees) {
        kicker.setPosition(degrees);
    }

    public double getFireSpeed() {
        return leftLaunch.getPower();
    }

    public double getLeftDriveEncoderValue() {
        return leftDrive.getCurrentPosition();
    }

    public double getRightDriveEncoderValue() {
        return rightDrive.getCurrentPosition();
    }

    //Below are functions that will hopefully simplify common actions during autonomous
    public void turn(double degrees) {
        //rotates the robot by degrees
    }

    public void move(double distance) {
        //moves the robot forward by distance (backward if negative)
    }

    public int getPattern() {
        //reads the pattern QR code and returns the position of the green ball in the sequence
        return 0;
    }

    // public void findTrajectory(double distance, double height, double gravity, double windResistance) {
    // hopefully will find the right angle and speed to shoot the ball to some point at a specific distance and height from the robot
    //}
    public void resetDriveEncoders () {
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Set targets for the encoders to run to position
    public void setTargets(int target) {
        leftDrive.setTargetPosition(target);
        rightDrive.setTargetPosition(target);
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void driveWhileBusy(double speed, double timeout) {
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        leftDrive.setPower(speed);
        rightDrive.setPower(speed);
        while ((leftDrive.isBusy() || rightDrive.isBusy()) &&
                runtime.seconds() < timeout) {
            // Display it for the driver.
            myOpMode.telemetry.addData("Currently at",  " at %7d :%7d",
                    leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition());
            myOpMode.telemetry.update();
        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        // Turn off RUN_TO_POSITION
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
