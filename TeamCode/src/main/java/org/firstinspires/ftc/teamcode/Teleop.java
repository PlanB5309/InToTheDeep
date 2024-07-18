/* Copyright (c) 2017 FIRST. All rights reserved.
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

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * This particular OpMode executes a Tank Drive control TeleOp a direct drive robot
 * The code is structured as an Iterative OpMode
 *
 * In this mode, the left and right joysticks control the left and right motors respectively.
 * Pushing a joystick forward will make the attached motor drive forward.
 * It raises and lowers the claw using the Gamepad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Robot: Teleop", group="Robot")
public class Teleop extends OpMode {
    RobotHardware robot = new RobotHardware();
    double WristPosition = robot.RESTING_WRIST;
    double ArmLength = robot.SHORT_ARM;
    long time_arm_move;
    long time_close_claws;
    long time_arm_move_out;
    long time_claws_grab_confident;
    boolean wrist_controlled = false;
    boolean slow_mode;
    Claws claws;
    PreloadStates preloadState = PreloadStates.NOT_RUNNING;
    LoadPixelStates loadpixelState = LoadPixelStates.NOT_RUNNING;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        robot.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.hookServo.setPosition(robot.HOOK_IN);
        robot.leftClawServo.setPosition(robot.LEFT_CLAW_OPEN);
        robot.rightClawServo.setPosition(robot.RIGHT_CLAW_OPEN);
        robot.leftPixelLockServo.setPosition(robot.LEFT_PIXEL_UNLOCK);
        robot.rightPixelLockServo.setPosition(robot.RIGHT_PIXEL_UNLOCK);
        robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_FOREST_PALETTE);

    }

    @Override
    public void loop() {

        //Slow Mode
        //strafe and turn right slowly with dpad
        if (gamepad1.dpad_right ||
                gamepad1.dpad_left ||
                gamepad1.dpad_up ||
                gamepad1.dpad_down) {
            slow_mode = true;

            if (gamepad1.dpad_right) {
                //turn right slowly with dpad
                if (gamepad1.b) {
                    robot.frontLeftMotor.setPower(0.12);
                    robot.frontRightMotor.setPower(-0.12);
                    robot.backLeftMotor.setPower(0.12);
                    robot.backRightMotor.setPower(-0.12);
                }
                //strafe right slowly with dpad
                else {
                    robot.frontLeftMotor.setPower(-.24);
                    robot.frontRightMotor.setPower(.24);
                    robot.backLeftMotor.setPower(.24);
                    robot.backRightMotor.setPower(-.24);
                }

                //strafe and turn left slowly with dpad
            } else if (gamepad1.dpad_left) {
                //turn left slowly with dpad
                if (gamepad1.b) {
                    robot.frontLeftMotor.setPower(-0.12);
                    robot.frontRightMotor.setPower(0.12);
                    robot.backLeftMotor.setPower(-0.12);
                    robot.backRightMotor.setPower(0.12);
                }
                //strafe left slowly with dpad
                else {
                    robot.frontLeftMotor.setPower(.24);
                    robot.frontRightMotor.setPower(-.24);
                    robot.backLeftMotor.setPower(-.24);
                    robot.backRightMotor.setPower(.24);
                }
                //drive backward slowly with dpad
            } else if (gamepad1.dpad_up) {
                robot.frontLeftMotor.setPower(-.18);
                robot.frontRightMotor.setPower(-.18);
                robot.backLeftMotor.setPower(-.18);
                robot.backRightMotor.setPower(-.18);

                //drive forward slowly with dpad
            } else if (gamepad1.dpad_down) {
                robot.frontLeftMotor.setPower(.18);
                robot.frontRightMotor.setPower(.18);
                robot.backLeftMotor.setPower(.18);
                robot.backRightMotor.setPower(.18);
            }
        } else slow_mode = false;


        //set the power to the wheels
        if (slow_mode == false) {
            double y = gamepad1.left_stick_y; // Remember, this is reversed!
            double x = -gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;
            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;
            robot.frontLeftMotor.setPower(frontLeftPower);
            robot.frontRightMotor.setPower(frontRightPower);
            robot.backLeftMotor.setPower(backLeftPower);
            robot.backRightMotor.setPower(backRightPower);
        }

        //Drone Launch Controls
        if (gamepad1.right_trigger > .5)
            robot.droneMotor.setPower(1);
        else robot.droneMotor.setPower(0);

        //Hook Servo
        if (gamepad1.x) {
            robot.hookServo.setPosition(robot.HOOK_IN);
        }
        if (gamepad1.y) {
            robot.hookServo.setPosition(robot.HOOK_OUT);
        }

        //Attachments



        //Right Claw Controls
        if (gamepad2.right_trigger > .5) {
            robot.rightClawServo.setPosition(robot.RIGHT_CLAW_CLOSE);
        }
        if (gamepad2.right_bumper) {
            robot.rightClawServo.setPosition(robot.RIGHT_CLAW_OPEN);
        }

        //Left Claw Controls
        if (gamepad2.left_trigger > .5) {
            robot.leftClawServo.setPosition(robot.LEFT_CLAW_CLOSE);
        }
        if (gamepad2.left_bumper) {
            robot.leftClawServo.setPosition(robot.LEFT_CLAW_OPEN);
        }

        //Wrist Controls
        if (gamepad2.dpad_up && WristPosition < 1) {
            WristPosition = WristPosition + robot.WRIST_SERVO_CHANGE_RATE;
            robot.wristServo.setPosition(WristPosition + robot.WRIST_SERVO_CHANGE_RATE);
            wrist_controlled = true;
        }
        if (gamepad2.dpad_down && WristPosition > 0) {
            WristPosition = WristPosition - robot.WRIST_SERVO_CHANGE_RATE;
            robot.wristServo.setPosition(WristPosition - robot.WRIST_SERVO_CHANGE_RATE);
            wrist_controlled = true;
        }

        if (robot.armMotor.getCurrentPosition() > 3014 &&
                wrist_controlled == false &&
                preloadState == PreloadStates.NOT_RUNNING){
            WristPosition = -1.76E-04*(robot.armMotor.getCurrentPosition()) + 1.61;
            robot.wristServo.setPosition(WristPosition);
        }

        //Pixel Lock Controls
        //Upwards on gamepad
        if(gamepad2.left_stick_y < -.5){
            robot.rightPixelLockServo.setPosition(robot.RIGHT_PIXEL_LOCK);
            robot.leftPixelLockServo.setPosition(robot.LEFT_PIXEL_LOCK);
            robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_GREEN);
        }
        //Downwards on gamepad
        if (gamepad2.left_stick_y > .5){
            robot.rightPixelLockServo.setPosition(robot.RIGHT_PIXEL_UNLOCK);
            robot.leftPixelLockServo.setPosition(robot.LEFT_PIXEL_UNLOCK);
            robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
        }

        //Panic Button
        if(gamepad2.left_stick_button){
            loadpixelState = LoadPixelStates.NOT_RUNNING;
            preloadState = PreloadStates.NOT_RUNNING;
            robot.armMotor.setPower(0);
            robot.armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }



        //AutoLoad Controls

        //Preload
        Preload();
        if (preloadState == PreloadStates.NOT_RUNNING &&
            loadpixelState == LoadPixelStates.NOT_RUNNING &&
            gamepad2.x) {
                wrist_controlled = false;
                preloadState = PreloadStates.MOVE_SERVOS;
        }

        //LoadPixel
        LoadPixel();
        if (loadpixelState == LoadPixelStates.NOT_RUNNING &&
            preloadState == PreloadStates.NOT_RUNNING &&
            gamepad2.y) {
                loadpixelState = LoadPixelStates.ARM_OUT;
        }

        if (loadpixelState == LoadPixelStates.NOT_RUNNING &&
            preloadState == PreloadStates.NOT_RUNNING){
                robot.armMotor.setPower(gamepad2.right_stick_y);
        }

            //Intake Controls
            if (gamepad2.b) {
                robot.intakeMotor.setPower(-1);
            } else if (gamepad2.a) {
                robot.intakeMotor.setPower(1);
            } else {
                robot.intakeMotor.setPower(0);
            }

            //Extend armservo
            if (gamepad2.dpad_right && ArmLength < 1) {
                ArmLength = ArmLength + robot.ARM_SERVO_CHANGE_RATE;
                robot.armServo.setPosition(ArmLength + robot.ARM_SERVO_CHANGE_RATE);
            }
            if (gamepad2.dpad_left && ArmLength > 0) {
                ArmLength = ArmLength - robot.ARM_SERVO_CHANGE_RATE;
                robot.armServo.setPosition(ArmLength - robot.ARM_SERVO_CHANGE_RATE);
            }

            //Telemetry Data
//            telemetry.addData("LeftOdometryWheel", robot.leftOdometry.getCurrentPosition());
//            telemetry.addData("MiddleOdometryWheel", robot.middleOdometry.getCurrentPosition());
//            telemetry.addData("RightOdometryWheel", robot.rightOdometry.getCurrentPosition());
            telemetry.addData("Wrist Value: ", robot.wristServo.getPosition());
            telemetry.addData("Arm (Up/Down) Value: ", robot.armMotor.getCurrentPosition());
            telemetry.addData("Arm (In/Out) Value", robot.armServo.getPosition());
            telemetry.addData("State of preload", preloadState);
            telemetry.addData("State of LoadPixel", loadpixelState);
            telemetry.addData("State of armTouchSensor", robot.armTouchSensor.getValue());
            telemetry.addData("Say", "Happy Little Pixels");
//            telemetry.addData("Left Distance Sensor",robot.leftDistanceSensor.getDistance(DistanceUnit.CM));
//            telemetry.addData("Right Distance Sensor",robot.rightDistanceSensor.getDistance(DistanceUnit.CM));
//            telemetry.addData("Rear Distance Sensor",robot.rearDistanceSensor.getDistance(DistanceUnit.CM));
            telemetry.update();




            /*
             * Code to run ONCE after the driver hits STOP
             */
        }
        private void Preload() {
            switch (preloadState) {

                case MOVE_SERVOS:
                    time_arm_move = System.currentTimeMillis() + 500;
                    robot.wristServo.setPosition(robot.WRIST_PRE_GRAB);
                    robot.leftClawServo.setPosition(robot.LEFT_CLAW_OPEN);
                    robot.rightClawServo.setPosition(robot.RIGHT_CLAW_OPEN);
                    robot.armServo.setPosition(robot.SHORT_ARM);
                    preloadState = PreloadStates.MOVE_ARM;
                    robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
                    break;

                case MOVE_ARM:
                    if (System.currentTimeMillis() > time_arm_move) {
                        /*
                        robot.armMotor.setTargetPosition(robot.ARM_READY);
                        robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        */
                        robot.armMotor.setPower(-1);
                        preloadState = PreloadStates.WAIT_FOR_ARM;
                    }
                    break;

                case WAIT_FOR_ARM:

                    if (robot.armTouchSensor.isPressed()){

                    //if (robot.armMotor.isBusy() == false) {
                        robot.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        robot.armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        preloadState = PreloadStates.NOT_RUNNING;
                    }
                    break;

                case NOT_RUNNING:
                    break;
            }

        }

    private void LoadPixel() {
        switch (loadpixelState) {
            case ARM_OUT:
                time_arm_move_out = System.currentTimeMillis() + 500;
                robot.armServo.setPosition(robot.GRAB_ARM);
                robot.wristServo.setPosition(robot.GRAB_WRIST);
                loadpixelState = LoadPixelStates.CLOSE_CLAWS;
                robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
                break;

            case CLOSE_CLAWS:
                if (System.currentTimeMillis() > time_arm_move_out) {
                    time_close_claws = System.currentTimeMillis() + 250;
                    robot.leftClawServo.setPosition(robot.LEFT_CLAW_CLOSE);
                    robot.rightClawServo.setPosition(robot.RIGHT_CLAW_CLOSE);
                    loadpixelState = LoadPixelStates.UNLOCK_PIXEL_SERVOS;
                }
                break;

            case UNLOCK_PIXEL_SERVOS:
                if (System.currentTimeMillis() > time_close_claws){
                    time_claws_grab_confident = System.currentTimeMillis() + 500;
                    robot.leftPixelLockServo.setPosition(robot.LEFT_PIXEL_UNLOCK);
                    robot.rightPixelLockServo.setPosition(robot.RIGHT_PIXEL_UNLOCK);
                    loadpixelState = LoadPixelStates.MOVE_SERVOS;
                }
                break;

            case MOVE_SERVOS:
                if (System.currentTimeMillis() > time_claws_grab_confident) {
                    robot.armServo.setPosition(robot.SHORT_ARM);
                    robot.wristServo.setPosition(robot.UPWARDS_WRIST);
                    loadpixelState = LoadPixelStates.MOVE_ARM;
                    time_arm_move = System.currentTimeMillis() + 1000;
                }
                break;

            case MOVE_ARM:
                if (System.currentTimeMillis() > time_arm_move) {
                    robot.armMotor.setTargetPosition(robot.ARM_UP);
                    robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.armMotor.setPower(.7);
                    loadpixelState = LoadPixelStates.WAIT_FOR_ARM;
                }
                break;

            case WAIT_FOR_ARM:
                if (robot.armMotor.isBusy() == false) {
                    robot.armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    WristPosition = robot.wristServo.getPosition();
                    loadpixelState = loadpixelState.NOT_RUNNING;
                }
                break;

            case NOT_RUNNING:
                break;

        }

    }
}


