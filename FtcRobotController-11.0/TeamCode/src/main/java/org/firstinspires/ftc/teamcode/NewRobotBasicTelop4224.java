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

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Position;

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

@TeleOp(name="New Robot Basic Telop 4224", group="Iterative OpMode")
public class NewRobotBasicTelop4224 extends OpMode
{
    // Declare OpMode members.

    //keeps track of time
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotor intakeMotor = null;
    private DcMotor elevatorMotor = null;
    private DcMotor turretMotor = null;
    private DcMotor flywheelMotor = null;
    private boolean isSlowModeOn = false;
    private double driveSpeed = Constants.DRIVE_SPEED;
    private boolean isLimelightAlignPressed = false;
    private  boolean isLimelightAlignOn = false;
    private Limelight3A limelight = null;



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
        intakeMotor = hardwareMap.get(DcMotor.class, Constants.INTAKE_MOTOR);
        elevatorMotor = hardwareMap.get(DcMotor.class, Constants.ELEVATOR_MOTOR);
        turretMotor = hardwareMap.get(DcMotor.class, Constants.TURRET_MOTOR);
        flywheelMotor = hardwareMap.get(DcMotor.class, Constants.FLYWHEEL_MOTOR);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
       intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD

       );
        elevatorMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }



    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {

            limelight.setPollRateHz(100);//how many times we ask the limelight for info
            limelight.pipelineSwitch(1);
            limelight.start();
    }


    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop()
    {LLResult llresult = limelight.getLatestResult();

        if (llresult != null) {
            telemetry.addData("Target Found", llresult.isValid());
            telemetry.addData("tx", llresult.getTx());
            telemetry.addData("ty", llresult.getTy());
        }

        if (isLimelightAlignOn && llresult != null && llresult.isValid()) {
            double tx = llresult.getTx();
            double kP = 0.03;
            double turn = -tx * kP;

            frontLeftDrive.setPower(-turn);
            backLeftDrive.setPower(-turn);
            frontRightDrive.setPower(turn);
            backRightDrive.setPower(turn);

            telemetry.addData("Aligning", true);
        } else {
            telemetry.addData("Aligning", false);
        }
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


            intakeMotor.setPower(Constants.INTAKE_SPEED);


        } else if (gamepad1.left_bumper) {


            intakeMotor.setPower(-Constants.INTAKE_SPEED);


        } else {
            intakeMotor.setPower(0);
        }

       if (gamepad1.left_trigger > 0.25){
           elevatorMotor.setPower(Constants.ELEVATOR_SPEED);
       }

       else {
            elevatorMotor.setPower(0);
        }

       if (gamepad1.left_trigger > 0.25) {
           turretMotor.setPower(Constants.TURRET_SPEED);
       }
       else {
           turretMotor.setPower(0);

       }
       if (gamepad1.x) {
           flywheelMotor.setPower(Constants.FLYWHEEL_SPEED_ONE);
       }
       else if (gamepad1.y) {
           flywheelMotor.setPower(Constants.FLYWHEEL_SPEED_TWO);
       }
       else {
           flywheelMotor.setPower(0);
       }
        if (gamepad1.left_trigger > 0.25){
            if (isLimelightAlignPressed == false) {
                isLimelightAlignOn = !isLimelightAlignOn;
                isLimelightAlignPressed = true;
            }
        }
        else {
            isLimelightAlignPressed = false;
        }
        LLResult llResult = limelight.getLatestResult();

        if (CanShoot(llResult.getBotpose().getPosition())) {
            gamepad1.rumble(1.0, 1.0, 500);
        }
        else {
            gamepad1.rumble(0,0,0);
        }



    }
    private boolean CanShoot(Position position) {
        return false;
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
