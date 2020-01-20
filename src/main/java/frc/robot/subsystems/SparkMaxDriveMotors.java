package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
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

import frc.robot.Constants.DriveConstants;

public class SparkMaxDriveMotors 
{
    public CANSparkMax[] motors;  
    public CANEncoder encoder;
    public CANSparkMax getMasterMotor() {
        return motors[0];
    } 

    public SparkMaxDriveMotors(final int canId1, final int canId2, final int canId3) {
        motors = new CANSparkMax[3];
        motors[0] = CreateNeoSparkMax(canId1);
        motors[1] = CreateNeoSparkMax(canId1);
        motors[2] = CreateNeoSparkMax(canId1);

        motors[1].follow(motors[0]);
        motors[2].follow(motors[0]);

        this.encoder = this.motors[0].getEncoder();        
    }

    private CANSparkMax CreateNeoSparkMax(final int canId) {
        CANSparkMax sparkMax = new CANSparkMax(canId, MotorType.kBrushless);
        sparkMax.setIdleMode(IdleMode.kBrake);
        sparkMax.setSmartCurrentLimit(DriveConstants.NEO_LIMIT);
        return sparkMax;
    }

    public double getWheelSpeedInMetersPerSecond() {
        return this.encoder.getVelocity()/DriveConstants.DRIVE_RATIO * DriveConstants.WHEEL_DIAMETER_M/60;
    }

    public double getEncoderDistanceInMeters() {
        return (DriveConstants.WHEEL_DIAMETER * Math.PI * (this.encoder.getPosition() / DriveConstants.ENCODER_TICK_PER_REV));
    }

    public void ResetEncoder() {
        this.encoder.setPosition(-1);
    }
} 