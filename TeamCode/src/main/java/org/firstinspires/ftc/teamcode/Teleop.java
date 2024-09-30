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
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

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
    double ArmLength = robot.SHORT_ARM;
    long time_arm_move;
    long time_close_claws;
    long time_arm_move_out;
    long time_claws_grab_confident;
    boolean wrist_controlled = false;
    boolean slow_mode;

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
        robot.frontClawServo.setPosition(robot.FRONT_CLAW_OPEN);
        robot.backClawServo.setPosition(robot.BACK_CLAW_OPEN);
        robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_FOREST_PALETTE);
    }

    @Override
    public void loop() {

//        SparkFunOTOS.Pose2D pos = robot.myOtos.getPosition();

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
                    robot.frontLeftMotor.setPower(-0.12);
                    robot.frontRightMotor.setPower(0.12);
                    robot.backLeftMotor.setPower(-0.12);
                    robot.backRightMotor.setPower(0.12);
                }
                //strafe right slowly with dpad
                else {
                    robot.frontLeftMotor.setPower(.24);
                    robot.frontRightMotor.setPower(-.24);
                    robot.backLeftMotor.setPower(-.24);
                    robot.backRightMotor.setPower(.24);
                }

                //strafe and turn left slowly with dpad
            } else if (gamepad1.dpad_left) {
                //turn left slowly with dpad
                if (gamepad1.b) {
                    robot.frontLeftMotor.setPower(0.12);
                    robot.frontRightMotor.setPower(-0.12);
                    robot.backLeftMotor.setPower(0.12);
                    robot.backRightMotor.setPower(-0.12);
                }
                //strafe left slowly with dpad
                else {
                    robot.frontLeftMotor.setPower(-.24);
                    robot.frontRightMotor.setPower(.24);
                    robot.backLeftMotor.setPower(.24);
                    robot.backRightMotor.setPower(-.24);
                }
                //drive backward slowly with dpad
            } else if (gamepad1.dpad_up) {
                robot.frontLeftMotor.setPower(.18);
                robot.frontRightMotor.setPower(.18);
                robot.backLeftMotor.setPower(.18);
                robot.backRightMotor.setPower(.18);

                //drive forward slowly with dpad
            } else if (gamepad1.dpad_down) {
                robot.frontLeftMotor.setPower(-.18);
                robot.frontRightMotor.setPower(-.18);
                robot.backLeftMotor.setPower(-.18);
                robot.backRightMotor.setPower(-.18);
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

        if (gamepad2.left_bumper) {
            robot.frontClawServo.setPosition(robot.FRONT_CLAW_OPEN);
            robot.backClawServo.setPosition(robot.BACK_CLAW_OPEN);
        }

        if (gamepad2.left_trigger>=.5){
            robot.frontClawServo.setPosition(robot.FRONT_CLAW_CLOSE);
            robot.backClawServo.setPosition(robot.BACK_CLAW_CLOSE);
        }

        robot.armMotor.setPower(gamepad2.right_stick_y);

        if (gamepad2.dpad_up)
            robot.specimenMotor.setPower(-1);

        if (gamepad2.dpad_down)
            robot.specimenMotor.setPower(1);

        if (gamepad2.dpad_up == false && gamepad2.dpad_down == false)
            robot.specimenMotor.setPower(0);

        if (gamepad2.right_bumper)
            robot.intakeServo.setPosition(1);

        if (gamepad2.right_trigger>.5)
            robot.intakeServo.setPosition(0);

        if (gamepad2.right_bumper == false && gamepad2.right_trigger < .5)
            robot.intakeServo.setPosition(.5);

        if (gamepad2.dpad_left)
            robot.sampleMotor.setPower(1);

        if (gamepad2.dpad_right)
            robot.sampleMotor.setPower(-1);

        if (gamepad2.dpad_left == false && gamepad2.dpad_right == false)
            robot.sampleMotor.setPower(0);

        telemetry.addData("front Left encoder",robot.frontLeftMotor.getCurrentPosition());
        telemetry.addData("front Right encoder",robot.frontRightMotor.getCurrentPosition());
        telemetry.addData("Back Left encoder",robot.backLeftMotor.getCurrentPosition());
        telemetry.addData("Back Right encoder",robot.backRightMotor.getCurrentPosition());
        telemetry.update();

//        telemetry.addData("X coordinate", pos.x);
//        telemetry.addData("Y coordinate", (pos.y * 1.25)) ;
//        telemetry.addData("Heading angle", pos.h);
//        telemetry.update();
//



            /*
             * Code to run ONCE after the driver hits STOP
             */
        }

    }



