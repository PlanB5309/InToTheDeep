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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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

@TeleOp(name="Diagnostic", group="Robot")
public class Diagnostic extends OpMode {
    RobotHardware robot = new RobotHardware();
    double WristPosition = robot.RESTING_WRIST;
    double ArmLength = robot.SHORT_ARM;
    long time_arm_move;
    long time_close_claws;
    long time_arm_move_out;
    long time_claws_grab_confident;
    boolean grabbing = false;
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
        telemetry.speak("I am Bob Ross");
        telemetry.update();




    }

    @Override
    public void loop() {
        //Telemetry Data
        telemetry.addData("Say", "Happy Little Pixels");
        telemetry.addData("State of armTouchSensor", robot.armTouchSensor.getValue());
        telemetry.addData("Left Distance Sensor",robot.leftDistanceSensor.getDistance(DistanceUnit.CM));
        telemetry.addData("Right Distance Sensor",robot.rightDistanceSensor.getDistance(DistanceUnit.CM));
        telemetry.addData("Rear Right Distance Sensor",robot.rearRightDistanceSensor.getDistance(DistanceUnit.CM));
        telemetry.addData("Rear Left Distance Sensor",robot.rearLeftDistanceSensor.getDistance(DistanceUnit.CM));
        telemetry.update();




        /*
         * Code to run ONCE after the driver hits STOP
         */
    }
}



