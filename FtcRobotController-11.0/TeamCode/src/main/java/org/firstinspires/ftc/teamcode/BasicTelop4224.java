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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Basic Telop 4224", group="Iterative OpMode")
public class BasicTelop4224 extends OpMode
{
    // Declare OpMode members.

    //keeps track of time
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotor intakeRightMotor = null;
    private DcMotor intakeLeftMotor = null;
    private DcMotor flywheelRightMotor = null;
    private DcMotor flywheelLeftMotor = null;
    private boolean isFastFlywheelOn = false;
    private boolean isSlowFlywheelOn = false;
    private boolean isSlowModeOn = false;
    private double driveSpeed = Constants.DRIVE_SPEED;



    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");



        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        frontLeftDrive  = hardwareMap.get(DcMotor.class, Constants.FRONT_LEFT_MOTOR);
        backLeftDrive = hardwareMap.get(DcMotor.class, Constants.BACK_LEFT_MOTOR);
        frontRightDrive = hardwareMap.get(DcMotor.class, Constants.FRONT_RIGHT_MOTOR);
        backRightDrive = hardwareMap.get(DcMotor.class, Constants.BACK_RIGHT_MOTOR);
        intakeLeftMotor = hardwareMap.get(DcMotor.class, Constants.INTAKE_LEFT_MOTOR);
        intakeRightMotor = hardwareMap.get(DcMotor.class, Constants.INTAKE_RIGHT_MOTOR);
        flywheelRightMotor = hardwareMap.get(DcMotor.class, Constants.FLYWHEEL_RIGHT_MOTOR);
        flywheelLeftMotor = hardwareMap.get(DcMotor.class, Constants.FLYWHEEL_LEFT_MOTOR);

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);
        intakeLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE


        );
        intakeRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheelRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        intakeRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }



    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        runtime.reset();
        
    }


    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        double y = -gamepad1.left_stick_y; // Remember, Y stick is reversed!
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;


        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        frontLeftDrive.setPower((y + x + rx) / denominator * driveSpeed);
        backLeftDrive.setPower((y - x + rx) / denominator * driveSpeed);
        frontRightDrive.setPower((y - x - rx) / denominator * driveSpeed);
        backRightDrive.setPower((y + x - rx) / denominator * driveSpeed);

        telemetry.addData("front Left Power: ",y + x + rx);
        telemetry.addData("back Left Power: ",y - x + rx);
        telemetry.addData("front Right Power: ",y - x - rx);
        telemetry.addData("back Right Power: ",y + x - rx);

        telemetry.update();








        if (gamepad1.right_bumper) {


            intakeLeftMotor.setPower(Constants.INTAKE_LEFT_SPEED);
            intakeRightMotor.setPower(Constants.INTAKE_RIGHT_SPEED);

        } else if (gamepad1.right_trigger > 0.25) {



            intakeLeftMotor.setPower(Constants.INTAKE_LEFT_SPEED);
            intakeRightMotor.setPower(Constants.INTAKE_RIGHT_SPEED);

        } else {
            intakeLeftMotor.setPower(0);
            intakeRightMotor.setPower(0);
        }


        



       if (gamepad1.yWasPressed()){
           isFastFlywheelOn = !isFastFlywheelOn;
       }
       if (gamepad1.xWasPressed()){
            isSlowFlywheelOn = !isSlowFlywheelOn;
        }

       if (isFastFlywheelOn == true){
            isSlowFlywheelOn = false;
           flywheelRightMotor.setPower(Constants.FLYWHEEL_SPEED_ONE);
           flywheelLeftMotor.setPower(Constants.FLYWHEEL_SPEED_ONE);
       }
       else if (isSlowFlywheelOn == true) {
           isFastFlywheelOn = false;
           flywheelRightMotor.setPower(Constants.FLYWHEEL_SPEED_TWO);
           flywheelLeftMotor.setPower(Constants.FLYWHEEL_SPEED_TWO);
       }
       else {
           flywheelLeftMotor.setPower(0);
           flywheelRightMotor.setPower(0);
       }




       if (gamepad1.leftBumperWasPressed()){
            isSlowModeOn = !isSlowModeOn;
       }
       if (isSlowModeOn == true){

           driveSpeed = Constants.SLOW_DRIVE_SPEED;
       }
       else {
           driveSpeed = Constants.DRIVE_SPEED;
       }

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
