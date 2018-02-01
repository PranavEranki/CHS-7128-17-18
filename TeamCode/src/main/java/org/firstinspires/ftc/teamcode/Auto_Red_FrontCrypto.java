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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * <p>
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name = "Autonomous Red Front Cryptobox", group = "Autonomous")

public class Auto_Red_FrontCrypto extends LinearOpMode {

    private static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    private static final double DRIVE_GEAR_REDUCTION = 16.0 / 27.0;     // This is < 1.0 if geared UP
    private static final double WHEEL_DIAMETER_INCHES = 3.54331;     // For figuring circumference
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    private static final double HEADING_THRESHOLD = 1;
    private static final double P_TURN_COEFF = 0.1;     // Larger is more responsive, but also less stable
    private static final double P_DRIVE_COEFF = 0.15;     // Larger is more responsive, but also less stable

    private static final double DRIVE_SPEED = 0.2;
    private static final double SLOW_DRIVE_SPEED = 0.1;
    private static final double TURN_SPEED = 0.1;
    private static final double LIFT_SPEED = 0.5;

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private VuforiaLocalizer vuforia;

    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private GyroSensor gyro;
    private DcMotor liftMotor;
    private Servo leftClamp;
    private Servo rightClamp;

    private int numLeft = 0;
    private int numCenter = 0;
    private int numRight = 0;

    private VuforiaTrackables relicTrackables;
    private VuforiaTrackable relicTemplate;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */

        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        gyro = hardwareMap.gyroSensor.get("gyro");
        liftMotor  = hardwareMap.dcMotor.get("liftMotor");
        leftClamp  = hardwareMap.servo.get("leftClamp");
        rightClamp = hardwareMap.servo.get("rightClamp");

        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        initVuforia();

        telemetry.addData("Status", "Wait for gyro to initialize");
        telemetry.update();

        gyro.calibrate();

        telemetry.addData("Status", "Press play to begin");
        telemetry.update();

        waitForStart();
        runtime.reset();

        relicTrackables.activate();

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();

        vuforia();

        int moveBackDist = 1;

        gyroDrive(SLOW_DRIVE_SPEED, -moveBackDist);

        closeServo();

        waitSec(1.5);

        liftBlock();

        gyroDrive(SLOW_DRIVE_SPEED, 24 + moveBackDist);

        gyroTurn(TURN_SPEED, 270);

        gyroDrive(DRIVE_SPEED, getCryptoboxDistance(numLeft, numCenter, numRight));

        gyroTurn(TURN_SPEED, 0);

        gyroDrive(SLOW_DRIVE_SPEED, 4);

        lowerBlock();

        openServo();

        waitSec(1.5);

        int moveBackDist2 = 4;

        gyroDrive(SLOW_DRIVE_SPEED, -moveBackDist2);

        gyroDrive(SLOW_DRIVE_SPEED, 6 + moveBackDist2);

        waitSec(0.5);

        gyroDrive(DRIVE_SPEED, -4);
    }

    private void liftBlock() {
        liftMotor.setPower(LIFT_SPEED);
        waitSec(1);
        liftMotor.setPower(0);
    }

    private void lowerBlock() {
        liftMotor.setPower(-LIFT_SPEED);
        waitSec(1);
        liftMotor.setPower(0);
    }


    private void initVuforia() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AZHAQRH/////AAAAGQ6K1+aY90TlgdxPiNC5zmmAVabLoenLsZ227ubCjDrC6b3cQtOUwnrB3ZNQWwDB6eHbzv8w67EJBp6R2AyPHf5X0gErSGY5HH4PR+IX4Ls2HTgft7F3A8SAUII/q7A70faLEz2U/seCvob/m53IwouLLa86D6WaMOdNi0lziagI1gS/cANAHhhb3fesRCKYxJvMONUyfMf6FXva4Mt7FIr6yy7qfY7bbkkcYMRSN9yPgHsdN6SxXkmKEZUs86sjl7yEYgefFyogwtbPOIvEvhn+8qh1HuliFc0b2mGhakbaXqYgzTpSmqTppmetn1LyK8U1zDnwaO5xftJ/ea6iKGhZQ6AIA3SM2kb57O/Za5za";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");
    }

    private void vuforia() {
        for (int i = 0; i < 20 && opModeIsActive(); i++) {
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                telemetry.addData("VuMark", "%s visible", vuMark);
                if (vuMark == RelicRecoveryVuMark.LEFT) numLeft++;
                else if (vuMark == RelicRecoveryVuMark.CENTER) numCenter++;
                else numRight++;
            } else {
                telemetry.addData("VuMark", "not visible");
                i = -1;
            }

            telemetry.update();

        }
        telemetry.addData("numLeft", numLeft);
        telemetry.addData("numCenter", numCenter);
        telemetry.addData("numRight", numRight);
        telemetry.update();
    }

    private void closeServo() {
        leftClamp.setPosition(0.675);
        rightClamp.setPosition(0.325);
    }

    private void openServo() {
        leftClamp.setPosition(0);
        rightClamp.setPosition(1);
    }

    private double getCryptoboxDistance(int left, int center, int right) {

        int max = Math.max(Math.max(left, center), right);

        RelicRecoveryVuMark mark;
        if (max == left) mark = RelicRecoveryVuMark.LEFT;
        else if (max == center) mark = RelicRecoveryVuMark.CENTER;
        else mark = RelicRecoveryVuMark.RIGHT;

        double dist = 0.555;
        double width = 7.63;
        if (mark == RelicRecoveryVuMark.RIGHT) return dist + (width * 0.5);
        if (mark == RelicRecoveryVuMark.CENTER) return dist + (width * 1.5);
        return dist + (width * 2.5);
    }

    // ------------------------------------------------------------
    // ------------------ GYRO METHODS ----------------------------
    // ------------------------------------------------------------

    private void gyroDrive(double speed, double distance) {
        int target = gyro.getHeading();

        int moveCounts = (int) (distance * COUNTS_PER_INCH);
        int newLeftTarget = leftMotor.getCurrentPosition() + moveCounts;
        int newRightTarget = rightMotor.getCurrentPosition() + moveCounts;

        leftMotor.setTargetPosition(newLeftTarget);
        rightMotor.setTargetPosition(newRightTarget);

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        speed = Range.clip(Math.abs(speed), 0.0, 1.0);
        leftMotor.setPower(speed);
        rightMotor.setPower(speed);

        while (opModeIsActive() && leftMotor.isBusy() && rightMotor.isBusy()) {
            int currHeading = gyro.getHeading();
            int diff = difference(currHeading, target);

            telemetry.addData("Gyro Heading", "Heading: " + currHeading);
            telemetry.addData("Difference", "Difference: " + diff);
            telemetry.update();

            if (diff < 0) { // too far left
                leftMotor.setPower(speed);
                rightMotor.setPower(speed * (1.0 - (diff / 180.0)));
            } else { // too far right
                leftMotor.setPower(speed * (1.0 - (diff / 180.0)));
                rightMotor.setPower(speed);
            }
        }

        leftMotor.setPower(0);
        rightMotor.setPower(0);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void gyroTurn(double turnSpeed, int target) {
        int currHeading = gyro.getHeading();

        int difference = difference(currHeading, target);

        while (opModeIsActive() && Math.abs(difference) > 1) {
            telemetry.addData("Gyro Heading", "Heading: " + currHeading);
            telemetry.addData("Difference", "Difference: " + difference);
            telemetry.update();

            if (difference > 0) {
                leftMotor.setPower(-turnSpeed);
                rightMotor.setPower(turnSpeed);
            } else {
                leftMotor.setPower(turnSpeed);
                rightMotor.setPower(-turnSpeed);
            }

            currHeading = gyro.getHeading();
            difference = difference(currHeading, target);
        }

        rightMotor.setPower(0);
        leftMotor.setPower(0);
    }

    private int difference(int currHeading, int target) {
        int ret = currHeading - target;
        if (ret > 180) ret -= 360;
        if (ret < -180) ret += 360;
        return ret;
    }

    private void waitSec(double seconds) {
        double time = this.time;
        while (opModeIsActive() && this.time - time < seconds) {};
    }
}
