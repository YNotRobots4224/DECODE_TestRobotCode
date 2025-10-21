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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

@Autonomous(name="Basic Auto 4224", group="Iterative OpMode")
public class BasicAuto4224 extends OpMode
{
    // Declare OpMode members.

    //keeps track of time
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotor intakeMotor = null;
    private DcMotor flywheelRightMotor = null;
    private DcMotor flywheelLeftMotor = null;
    private double driveSpeed = 1;
    private double actionEndTime = 0;
    private double[] actionTimes;
    private int currentActionIndex = 0;



  



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
        flywheelRightMotor = hardwareMap.get(DcMotor.class, Constants.FLYWHEEL_RIGHT_MOTOR);
        flywheelLeftMotor = hardwareMap.get(DcMotor.class, Constants.FLYWHEEL_LEFT_MOTOR);

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

        actionTimes = new double[5];
        actionTimes[0] = .2;
        actionTimes[1] = .23;
        actionTimes[2] = 2;
        actionTimes[3] = .25;
        actionTimes[4] = .25;
        actionEndTime = actionTimes[0];
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

        if (currentActionIndex == 0){
            driveRobot(-1,0,0);
        }
        else if (currentActionIndex == 1){
            driveRobot(0,-1,0);
        }
        else if (currentActionIndex == 2) {
            turnIntakeOn(true);
            driveRobot(0, 0, 0);
        }
        else if (currentActionIndex == 3){
            driveRobot(0,1,0);
        }
        else if (currentActionIndex == 4){
            driveRobot(0,0,1);
        }
        else {
            turnIntakeOn(false);
            driveRobot(0,0,0);

            return;
        }


        if (runtime.seconds() > actionEndTime){
            currentActionIndex++;
            if(currentActionIndex < actionTimes.length){
                actionEndTime += actionTimes[currentActionIndex];
            }
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    public void driveRobot(double x, double y, double rx){
        frontLeftDrive.setPower((y + x + rx) * Constants.DRIVE_SPEED);
        backLeftDrive.setPower((y - x + rx) * Constants.DRIVE_SPEED);
        frontRightDrive.setPower((y - x - rx) * Constants.DRIVE_SPEED);
        backRightDrive.setPower((y + x - rx) * Constants.DRIVE_SPEED);
    }
    public void turnIntakeOn(boolean on){
        if (on == true){
            intakeMotor.setPower(Constants.INTAKE_SPEED);
        }
        else {
            intakeMotor.setPower(0);
        }
    }
}
