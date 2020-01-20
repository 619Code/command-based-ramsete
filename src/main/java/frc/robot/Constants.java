/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    public static final int kLeftMotor1Port = 10;
    public static final int kLeftMotor2Port = 11;
    public static final int kLeftMotor3Port = 12;
    public static final int kRightMotor1Port = 13;
    public static final int kRightMotor2Port = 14;
    public static final int kRightMotor3Port = 15;

    public static final int[] kLeftEncoderPorts = new int[] { 10, 11, 12 };
    public static final int[] kRightEncoderPorts = new int[] { 13, 14, 15 };
    public static final boolean kLeftEncoderReversed = false;
    public static final boolean kRightEncoderReversed = true;

    public static final double kTrackwidthMeters = 0.69;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
        kTrackwidthMeters);

    public static final int kEncoderCPR = 42;
    public static final double kWheelDiameterMeters = 0.1016;
    public static final double ENCODER_TICK_PER_REV = 6; //42 / 7 which is the wheel to rotation ratio.
    public static final double kEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelDiameterMeters * Math.PI) / (double) ENCODER_TICK_PER_REV;

    

     // PATHWEAVER
    // Feedforward/Feedback gains (FROM CHARACTERIZATION)
    public static final double ksVolts = 0.216;
    public static final double kvVoltSecondsPerInch = 0.0692;
    public static final double kvVoltSecondsPerMeter = 39.3701 * kvVoltSecondsPerInch;
    
    public static final double kaVoltSecondsSquaredPerInch = 0.0127;
    public static final double kaVoltSecondsSquaredPerMeter = 0.2;

    public static final double kPDriveVel = 0.601;

    // DifferentialDrive Kinematics
    public static final double kTrackwidthInches = 21;

    // Max Trajectory Velocity/Acceleration
    public static final double kMaxSpeedFeetPerSecond = 10;
    public static final double kMaxAccelerationFeetPerSecondSquared = 1.5;

    // Ramsete parameters
    public static final double kRamseteBMeters = 2;
    public static final double kRamseteZetaMeters = 0.7;

    // What even the hell
    public static final boolean kGyroReversed = true;

    // Drive Stuff regular
    public static final int NEO_LIMIT = 60;
    public static final double DRIVE_RATIO = 6.87;
    public static final double WHEEL_DIAMETER = 4;
    public static final double WHEEL_DIAMETER_M = 0.1016;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 1;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;

    // Reasonable baseline values for a RAMSETE follower in units of meters and
    // seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
  }
}
