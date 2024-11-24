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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;

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
    boolean retractArm = false;
    boolean slow_mode;
    States state = States.NOT_RUNNING;
    double liftTime = 0;

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
        robot.frontClawServo.setPosition(robot.FRONT_CLAW_CLOSE);
        robot.backClawServo.setPosition(robot.BACK_CLAW_CLOSE);
        robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_FOREST_PALETTE);

    }

    @Override
    public void loop() {

        robot.odo.bulkUpdate();
        //Driver
        //slow_mode
        if (gamepad1.dpad_right ||
                gamepad1.dpad_left ||
                gamepad1.dpad_up ||
                gamepad1.dpad_down) {
            slow_mode = true;

            //Drives Forward Slowly
            if (gamepad1.dpad_up) {
                robot.backLeftMotor.setPower(.18);
                robot.frontLeftMotor.setPower(.18);
                robot.backRightMotor.setPower(.18);
                robot.frontRightMotor.setPower(.18);
            }
            //Drives Backward Slowly
            if (gamepad1.dpad_down) {
                robot.backLeftMotor.setPower(-.18);
                robot.frontLeftMotor.setPower(-.18);
                robot.backRightMotor.setPower(-.18);
                robot.frontRightMotor.setPower(-.18);
            }
            //Strafes Right Slowly
            if (gamepad1.dpad_right && !gamepad1.b) {
                robot.backLeftMotor.setPower(.24);
                robot.frontLeftMotor.setPower(-.24);
                robot.backRightMotor.setPower(-.24);
                robot.frontRightMotor.setPower(.24);
            }
            //Strafe Left Slowly
            if (gamepad1.dpad_left && !gamepad1.b) {
                robot.frontLeftMotor.setPower(.24);
                robot.frontRightMotor.setPower(-.24);
                robot.backLeftMotor.setPower(-.24);
                robot.backRightMotor.setPower(.24);
            }
            //Turn Right Slowly
            if (gamepad1.dpad_right && gamepad1.b) {
                robot.frontLeftMotor.setPower(0.12);
                robot.frontRightMotor.setPower(-0.12);
                robot.backLeftMotor.setPower(0.12);
                robot.backRightMotor.setPower(-0.12);
            }
            //Turns Left Slowly
            if (gamepad1.dpad_left && gamepad1.b) {
                robot.frontLeftMotor.setPower(-0.12);
                robot.frontRightMotor.setPower(0.12);
                robot.backLeftMotor.setPower(-0.12);
                robot.backRightMotor.setPower(0.12);
            }
        }
        //Drive with Joystick
        else {
            slow_mode=false;
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
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

        //Attachments

        //Ready to grab a specimen
        if (gamepad2.left_bumper) {
            robot.frontClawServo.setPosition(robot.FRONT_CLAW_OPEN_DOWN);
            robot.backClawServo.setPosition(robot.BACK_CLAW_OPEN_DOWN);
            robot.specimenMotor.setTargetPosition(robot.GRAB_SPECIMEN);
            robot.specimenMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.specimenMotor.setPower(1);
        }

        if (gamepad2.x && state == States.LOADING)
            state = States.SCORING;
        score();


        //Grabbing a specimen
        if (gamepad2.left_trigger>=.5 && state == state.NOT_RUNNING)
            state = States.LOADING;
        load();

        //Sample arm up and down
        robot.armMotor.setPower(gamepad2.right_stick_y);

        //SpecimenMotor
        if (gamepad2.dpad_up){
            robot.specimenMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.specimenMotor.setPower(1);
            state = States.NOT_RUNNING;
        }

        if (gamepad2.dpad_down){
            robot.specimenMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.specimenMotor.setPower(-1);
            state = States.NOT_RUNNING;
        }

        if (gamepad2.dpad_up == false &&
            gamepad2.dpad_down == false &&
            gamepad2.left_trigger<.5 &&
            gamepad2.left_bumper == false &&
            state == States.NOT_RUNNING)
                robot.specimenMotor.setPower(0);

        //IntakeServo
        //Spitting Out Samples
        if (gamepad2.right_bumper)
            robot.intakeServo.setPosition(1);

        //Eating Samples
        if (gamepad2.right_trigger>.5)
            robot.intakeServo.setPosition(0);

        if (gamepad2.right_bumper == false && gamepad2.right_trigger < .5)
            robot.intakeServo.setPosition(.5);

        //SampleMotor
        //
        //Hold the Arm Extension Motor at Low Power
        if (gamepad2.left_stick_button) {
            robot.sampleMotor.setPower(-.1);
            retractArm = true;
        }
        if (gamepad2.dpad_left) {
            robot.sampleMotor.setPower(-1);
            retractArm = false;
        }
        if (gamepad2.dpad_right) {
            robot.sampleMotor.setPower(1);
            retractArm = false;
        }

        if (gamepad2.dpad_left == false &&
            gamepad2.dpad_right == false  &&
            retractArm == false)
            robot.sampleMotor.setPower(0);

        //HookServo
        if (gamepad1.right_bumper)
            robot.hookServo.setPosition(robot.HOOK_OUT);

        if (gamepad1.left_bumper)
            robot.hookServo.setPosition(robot.HOOK_IN);

        Pose2D pos = robot.odo.getPosition();
        String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}",
                pos.getX(DistanceUnit.INCH),
                pos.getY(DistanceUnit.INCH),
                pos.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Position", data);
        telemetry.addData("Bar Height", robot.specimenMotor.getCurrentPosition());
        telemetry.addData("Basket Height", robot.sampleMotor.getCurrentPosition());
        telemetry.addData("HOW FAR ARM MOTOR GOES UP", robot.armMotor.getCurrentPosition());
        telemetry.addData("State", state.name());
        telemetry.update();

        //touch sensor for specimen lift
        if (robot.SpecimenTouchSensor.isPressed() && state != States.LOADING) {
            robot.specimenMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.specimenMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    } //end of loop

        private void load(){
            switch (state){
                case NOT_RUNNING:
                    break;
                case LOADING:
                    robot.frontClawServo.setPosition(robot.FRONT_CLAW_CLOSE);
                    robot.backClawServo.setPosition(robot.BACK_CLAW_CLOSE);
                    robot.specimenMotor.setTargetPosition(robot.ABOVE_SECOND_BAR);
                    robot.specimenMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.specimenMotor.setPower(1);
                    break;
            }
        }
    private void score() {
        switch (state) {
            case NOT_RUNNING:
                break;

            case SCORING:
                robot.specimenMotor.setTargetPosition(robot.BELOW_SECOND_BAR);
                if (!robot.specimenMotor.isBusy()) {
                    liftTime = System.currentTimeMillis() + 250;
                    state = States.CLAWS_UP;
                }
                break;

            case CLAWS_UP:
                robot.frontClawServo.setPosition(robot.FRONT_CLAW_OPEN_UP);
                robot.backClawServo.setPosition(robot.BACK_CLAW_OPEN_UP);
                if (System.currentTimeMillis() >= liftTime)
                    state = States.LIFT_DOWN;
                break;

            case LIFT_DOWN:
                    robot.specimenMotor.setTargetPosition(robot.GRAB_SPECIMEN);
                if (!robot.specimenMotor.isBusy())
                    state = States.CLAWS_DOWN;
                break;

            case CLAWS_DOWN:
                robot.frontClawServo.setPosition(robot.FRONT_CLAW_OPEN_DOWN);
                robot.backClawServo.setPosition(robot.BACK_CLAW_OPEN_DOWN);
                state = States.NOT_RUNNING;
                break;

        }
    }
}