/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SPI;

import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {

  // The motors on the left side of the drive.
  private SparkMaxDriveMotors m_leftMotors;

  // The motors on the right side of the drive.
  private SparkMaxDriveMotors m_rightMotors;

  // The robot's drive
  private DifferentialDrive m_drive;

  private DifferentialDrive m_odemetry;

  AHRS navx;

  
  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  /**
   * Creates a new DriveSubsystem.
   */
  public DriveSubsystem() {

    m_leftMotors = new SparkMaxDriveMotors(
      DriveConstants.kLeftMotor1Port, 
      DriveConstants.kLeftMotor2Port, 
      DriveConstants.kLeftMotor3Port);

    m_rightMotors = new SparkMaxDriveMotors(
        DriveConstants.kRightMotor1Port, 
        DriveConstants.kRightMotor2Port, 
        DriveConstants.kRightMotor3Port);  


    m_drive = new DifferentialDrive(m_leftMotors.getMasterMotor(), 
                                    m_rightMotors.getMasterMotor());

    initSensors();    
    resetEncoders();
    
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
  } 

  public void initSensors() {
    navx = new AHRS(SPI.Port.kMXP);
    resetGyro();
    resetEncoders();
  }
   
  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(Rotation2d.fromDegrees(getHeading()), this.m_leftMotors.getEncoderDistanceInMeters(),
                      this.m_rightMotors.getEncoderDistanceInMeters());
  }

  public void resetGyro() {
    navx.reset();
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(this.m_leftMotors.getWheelSpeedInMetersPerSecond()
                    ,this.m_rightMotors.getWheelSpeedInMetersPerSecond());
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotors.getMasterMotor().setVoltage(leftVolts);
    m_rightMotors.getMasterMotor().setVoltage(-rightVolts);
  }

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  public void resetEncoders() {
    this.m_leftMotors.ResetEncoder();
    this.m_rightMotors.ResetEncoder();
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings, the returned reading is in meters.
   */
  public double getAverageEncoderDistance() {
    return (this.m_leftMotors.getEncoderDistanceInMeters() + this.m_rightMotors.getEncoderDistanceInMeters()) / 2.0;
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public CANEncoder getLeftEncoder() {
    return this.m_leftMotors.encoder;
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public CANEncoder getRightEncoder() {
    return this.m_rightMotors.encoder;
  }

  /**
   * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /**
   * Zeroes the heading of the robot.
   */
  public void zeroHeading() {
    navx.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from 180 to 180
   */
  public double getHeading() {
    return Math.IEEEremainder(navx.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return navx.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }
}
