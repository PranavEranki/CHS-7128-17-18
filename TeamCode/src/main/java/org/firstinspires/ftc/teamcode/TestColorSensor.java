/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Color Test", group="Autonomous")

public class TestColorSensor extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    ColorSensor colorSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // variable = condition ? (if condition true, return this) : else, (return this);
        
        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */

        colorSensor = hardwareMap.colorSensor.get("colorSensor");


        // eg: Set the drive motor directions:
        // "Reverse" the motor that runs backwards when connected directly to the battery
        // leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        // rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        colorSensor.enableLed(false);           //does nothing for some reason

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Color", "Red: " + colorSensor.red());
            telemetry.addData("Color", "Green: " + colorSensor.green());
            telemetry.addData("Color", "Blue: " + colorSensor.blue());

            //enough = more than 0 and less than 100
            boolean enoughRed = colorSensor.red() > 0 && colorSensor.red() < 100;
            boolean enoughBlue = colorSensor.blue() > 0 && colorSensor.blue() < 100;
            boolean enoughGreen = colorSensor.green() > 0 && colorSensor.green() < 100;

            //lot =
            boolean lotOfRed = colorSensor.red() > 100 && colorSensor.red() < 200;
            boolean lotOfBlue = colorSensor.blue() > 100 && colorSensor.blue() < 200;
            boolean lotOfGreen = colorSensor.green() > 100 && colorSensor.green() < 200;

            //check for brown or gray
            if(enoughRed && enoughBlue &&  enoughGreen){
                telemetry.addData("Color", "Color: Brown");
            }else if(lotOfRed && lotOfBlue && lotOfGreen){
                telemetry.addData("Color", "Color: Gray");
            }else{
                //check for blue or red
                if(lotOfBlue && !lotOfRed){
                    telemetry.addData("Color", "Color: Blue");
                }else if(lotOfRed && lotOfBlue){
                    telemetry.addData("Color", "Color: Red");
                }else{
                    telemetry.addData("Color", "Color: N.A.");
                }
            }

            telemetry.update();

            /*leftMotor.setPower(0.5);
            rightMotor.setPower(0.5);

            while (colorSensor.red() > 50) {
                leftMotor.setPower(-0.5);
                rightMotor.setPower(0.5);
            }

            while (colorSensor.blue() > 50){
                leftMotor.setPower(0.5);
                rightMotor.setPower(-0.5);
            }

            leftMotor.setPower(0);
            rightMotor.setPower(0);*/

            // eg: Run wheels in tank mode (note: The joystick goes negative when pushed forwards)
            // leftMotor.setPower(-gamepad1.left_stick_y);
            // rightMotor.setPower(-gamepad1.right_stick_y);
        }
    }
}
