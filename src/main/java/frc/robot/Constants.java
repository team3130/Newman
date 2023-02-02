// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    /**
     * CAN
     */
    public final static int CAN_SpinnyBar = 55;
    public final static int CAN_PNMMODULE = 1;
    public final static int CAN_LeftFrontSteer = 37;
    public final static int CAN_LeftFrontDrive = 17;
    public final static int CAN_RightFrontSteer = 14;
    public final static int CAN_RightFrontDrive = 5;
    public final static int CAN_LeftBackSteer = 7;
    public final static int CAN_LeftBackDrive = 12;
    public final static int CAN_RightBackSteer = 13;
    public final static int CAN_RightBackDrive = 29;
    public final static int CANCoderTopRight = 36;
    public final static int CANCoderBottomRight = 11;
    public final static int CANCoderTopLeft = 9;
    public final static int CANCoderBottomLeft = 3;
    // Order should match side
    public static final int[] turningId = new int[] {CAN_LeftFrontSteer, CAN_LeftBackSteer, CAN_RightFrontSteer, CAN_RightBackSteer};
    public static final int[] spinningId = new int[] {CAN_LeftFrontDrive, CAN_LeftBackDrive, CAN_RightFrontDrive, CAN_RightBackDrive};
    public final static int[] CANCoders = new int[] {CANCoderTopLeft, CANCoderBottomLeft, CANCoderTopRight, CANCoderBottomRight};
    /**
     * PNM ID's
     */
    public static final int PNM_LargeSolenoid = 0;
    public static final int PNM_SmallSolenoid = 1;
    /**
     * Timer Settings
     */
    public static double timetoIntake = 0.25;
    /**
     * Encoder offsets
     */
    public static final double kTopLeftOffset = Math.toRadians(264.90);
    public static final double kBottomLeftOffset = Math.toRadians(275.27);
    public static final double kTopRightOffset = Math.toRadians(129.3);
    public static final double kBottomRightOffset = Math.toRadians(357.71484);
    public static final double[] kCanCoderOffsets = new double[] {kTopLeftOffset, kBottomLeftOffset, kTopRightOffset, kBottomRightOffset};
    /**
     * Gear ratio and ticks per rev
     */
    public final static double kDriveGearRatio = 6.75; // checked 1/19
    public final static double kSteerGearRatio = 150d/7d; // checked 1/19
    public static final double kEncoderResolution = 2048;

    public static final double kWheelDiameter = Units.inchesToMeters(3.86);
    public static final double SteerTicksToRads = 1/(kEncoderResolution * kSteerGearRatio) * Math.PI * 2; // multiply by position
    public static final double SteerTicksToRadsPerSecond = SteerTicksToRads * 10; // multiply by velocity
    public final static double DriveTicksToMeters = kWheelDiameter * Math.PI * 1/(kEncoderResolution * kDriveGearRatio); // multiply by
    public static final double DriveTicksToMetersPerSecond = DriveTicksToMeters * 10; // multiply by velocity
    public final static double kMaxSteerVoltage = 5d;
    public final static double kMaxDriveVoltage = 9d;

    /**
     * Length and width as measured as distances between center of wheels
     */
    // the left-to-right distance between the drivetrain wheels, should be measured from center to center
	public static final double trackWidth_m = 0.61;
	// the front-to-back distance between the drivetrain wheels, should be measured from center to center
	public static final double wheelBase_m = 0.61;

    /**
     * For swerve drive
     * translations for the distance to each wheel from the center of the bot.
     * Remember that forward (0 radians) is positive X
     * Check:
     *  right half the bot up half the bot      (0.5, 0.5)
     *  right half the bot down half the bot    (-0.5, 0.5)
     *  left half the bot up half the bot       (0.5, -0.5)
     *  left half the bot down half the bot     (-0.5, -0.5)
     * These look like coordinates to each wheel with the order being:
     *  top right,
     *  bottom right,
     *  top left,
     *  bottom left,
     */
	public static final Translation2d[] moduleTranslations = {
		new Translation2d(wheelBase_m / 2.0, trackWidth_m / 2.0),
		new Translation2d(-wheelBase_m / 2.0, trackWidth_m / 2.0),
		new Translation2d(wheelBase_m / 2.0, -trackWidth_m / 2.0),
		new Translation2d(-wheelBase_m / 2.0, -trackWidth_m / 2.0)
	};

    public static final boolean kNavxReversed = true;


    public static double SwerveKp = 0.5;
    public static double SwerveKi = 0;
    public static double SwerveKd = 0.01;
    public static double SwerveKf = 0;


    public static double openLoopRampRate = 0.7;

    public static double kPhysicalMaxSpeedMetersPerSecond = 3.6;

    public static double kDeadband = 0.075;

    public static double kMaxAccelerationDrive = 7;
    public static double kMaxAccelerationAngularDrive = 3;

    public static double kResetTime = 1.5;

    public static class Side {
         public static final int LEFT_FRONT = 0;
         public static final int LEFT_BACK = 1;
         public static final int RIGHT_FRONT = 2;
         public static final int RIGHT_BACK = 3;
    }

    public static class Buttons {
        /**
     * Gamepad Button List
     */
    public static final int LST_BTN_A = 1;
    public static final int LST_BTN_B = 2;
    public static final int LST_BTN_X = 3;
    public static final int LST_BTN_Y = 4;
    public static final int LST_BTN_LBUMPER = 5;
    public static final int LST_BTN_RBUMPER = 6;
    public static final int LST_BTN_WINDOW = 7;
    public static final int LST_BTN_MENU = 8;
    public static final int LST_BTN_LJOYSTICKPRESS = 9;
    public static final int LST_BTN_RJOYSTICKPRESS = 10;

    /**
     * Gamepad POV List
     */
    public static final int LST_POV_UNPRESSED = -1;
    public static final int LST_POV_N = 0;
    public static final int LST_POV_NE = 45;
    public static final int LST_POV_E = 90;
    public static final int LST_POV_SE = 135;
    public static final int LST_POV_S = 180;
    public static final int LST_POV_SW = 225;
    public static final int LST_POV_W = 270;
    public static final int LST_POV_NW = 315;

    /**
     * Gamepad Axis List
     */
    public static final int LST_AXS_LJOYSTICKX = 0;
    public static final int LST_AXS_LJOYSTICKY = 1;
    public static final int LST_AXS_LTRIGGER = 2;
    public static final int LST_AXS_RTRIGGER = 3;
    public static final int LST_AXS_RJOYSTICKX = 4;
    public static final int LST_AXS_RJOYSTICKY = 5;
    }
}
