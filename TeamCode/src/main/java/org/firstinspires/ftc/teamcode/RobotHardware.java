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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorDigitalTouch;

/**
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
 * Or.. In OnBot Java, add a new file named RobotHardware.java, drawing from this Sample; select Not an OpMode.
 * Also add a new OpMode, drawing from the Sample ConceptExternalHardwareClass.java; select TeleOp.
 *
 */

public class RobotHardware {

    // Create Motors
    public DcMotor backLeftMotor   = null;
    public DcMotor backRightMotor  = null;
    public DcMotor frontLeftMotor   = null;
    public DcMotor frontRightMotor = null;
    public DcMotor armMotor = null;
    public DcMotor specimenMotor = null;
    public DcMotor sampleMotor = null;

    // Motor Constants

    //Create Servos
    public Servo intakeServo = null;
    public Servo frontClawServo = null;
    public Servo backClawServo = null;
    public Servo hookServo = null;
    public Servo kickServo = null;

    //Create Lights
    public RevBlinkinLedDriver lights;

    //Create Sensors
    public Rev2mDistanceSensor SpecimenDistanceSensor;
    BNO055IMU imu;
    public RevTouchSensor SpecimenTouchSensor;
    // Declare OpMode member for the Odometry Computer
    public GoBildaPinpointDriver odo = null;

    //Sensor Constants
    public static final double AT_THE_WALL = 3;

    //Motor Constants

    public static final int ABOVE_SECOND_BAR = 1955;
    public static final int ABOVE_THE_WALL = 900;
    //THIS IS THE MOVEMENT TO SCORE THE SPECIMEN
    public static final int BELOW_SECOND_BAR = 1250;
    public static final int GRAB_SPECIMEN = 0;
    public static final int EXTEND_ARM_TO_BASKET = 3083;
    public static final int RAISE_ARM_TO_BASKET = 3643;
    public static final int DRIVE_HEIGHT = 1500;
    public static final int ARM_LOCK = 0;


    //Servo Constants
    //Claws
    public static final double BACK_CLAW_CLOSE = 0.48;
    public static final double BACK_CLAW_OPEN_DOWN = .85;
    public static final double BACK_CLAW_OPEN_UP = .13;
    public static final double FRONT_CLAW_CLOSE = 0.5;
    public static final double FRONT_CLAW_OPEN_DOWN = .13;
    public static final double FRONT_CLAW_OPEN_UP = .85;
    public static final double HOOK_IN = .6;
    public static final double HOOK_OUT = 0;
    //Arm
    public static final double SHORT_ARM = 1;
    public static final double GRAB_ARM = .62;
    public static final double ARM_LIMIT = 4055;

    //Kick Servo
    public static final double KICK_SERVO_OUT = .35;
    public static final double KICK_SERVO_MIDDLE = .7;
    public static final double KICK_SERVO_IN = .95;

    //Rates
    public static final int CLICKS_PER_CENTIMETER = 18;
    public static final int STRAFE_CLICKS_PER_CENTIMETER = 20;

    //Turning Speeds
    public final double HIGH_TURN_POWER = 0.52;
    public final double MEDIUM_TURN_POWER = .3;
    public final double LOW_TURN_POWER = 0.1;








    /* local OpMode members. */
    public HardwareMap hwMap           =  null;



    // Define a constructor that allows the OpMode to pass a reference to itself.
    public RobotHardware() {}

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     *
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    public void init(HardwareMap ahwMap)    {
        // Save reference to Hardware map
        hwMap = ahwMap;
        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        backLeftMotor  = hwMap.get(DcMotor.class, "backLeftMotor");
        backRightMotor = hwMap.get(DcMotor.class, "backRightMotor");
        frontLeftMotor = hwMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hwMap.get(DcMotor.class, "frontRightMotor");
        armMotor = hwMap.get(DcMotor.class, "armMotor");
        specimenMotor = hwMap.get(DcMotor.class, "specimenMotor");
        sampleMotor = hwMap.get(DcMotor.class, "sampleMotor");

        intakeServo = hwMap.get(Servo.class, "intakeServo");
        frontClawServo = hwMap.get(Servo.class, "frontClawServo");
        backClawServo = hwMap.get(Servo.class, "backClawServo");
        hookServo = hwMap.get(Servo.class, "hookServo");
        kickServo = hwMap.get(Servo.class, "kickServo");

        lights = hwMap.get(RevBlinkinLedDriver.class, "lights");

        odo = hwMap.get(GoBildaPinpointDriver.class, "odo");
        SpecimenTouchSensor = hwMap.get(RevTouchSensor.class, "SpecimenTouchSensor");
        SpecimenDistanceSensor = hwMap.get(Rev2mDistanceSensor.class, "SpecimenDistanceSensor");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        specimenMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        sampleMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);

        //Schuy's one and only contribution
        hookServo.setPosition(HOOK_IN);



        //Using Encoders
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        specimenMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sampleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        specimenMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Not Using Encoders
        sampleMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Odometery
        odo.setOffsets(-84.0, -168.0);
        odo.resetPosAndIMU();
    }
}
