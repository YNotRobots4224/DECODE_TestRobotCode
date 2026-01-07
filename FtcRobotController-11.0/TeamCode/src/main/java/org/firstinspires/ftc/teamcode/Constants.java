package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class Constants {
    public final static double INTAKE_RIGHT_SPEED = 1;
    public final static double INTAKE_LEFT_SPEED = 1;
    public final static double DRIVE_SPEED = 1;
    public final static double SLOW_DRIVE_SPEED = 1;
    public final static double FLYWHEEL_SPEED_TWO = .75;
    public final static double FLYWHEEL_SPEED_ONE = .9;
    public final static String IMU = "imu";// I2C 0
    public final static String FRONT_LEFT_MOTOR = "front_left_drive";// port 3
    public final static String FRONT_RIGHT_MOTOR = "front_right_drive"; // port 2
    public final static String BACK_LEFT_MOTOR = "back_left_drive"; // port 0
    public final static String BACK_RIGHT_MOTOR = "back_right_drive"; //port 1
    public final static String INTAKE_LEFT_MOTOR = "intake_left_motor"; // expansion port 0
    public final static String INTAKE_RIGHT_MOTOR = "intake_right_motor"; // expansion port 3
    public final static String FLYWHEEL_LEFT_MOTOR = "flywheel_left_motor"; // expansion port 1
    public final static String FLYWHEEL_RIGHT_MOTOR = "flywheel_right_motor";// expansion port 2
    public final static String SERVO_BLOCKER = "servo_blocker";// port 0.1
    private final static RevHubOrientationOnRobot.LogoFacingDirection IMU_LOGO_DIRECTION = RevHubOrientationOnRobot.LogoFacingDirection.UP;
    private final static RevHubOrientationOnRobot.UsbFacingDirection IMU_USB_DIRECTION = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;
    public final static  RevHubOrientationOnRobot IMU_ORIENTATION = new RevHubOrientationOnRobot(IMU_LOGO_DIRECTION, IMU_USB_DIRECTION);
}



