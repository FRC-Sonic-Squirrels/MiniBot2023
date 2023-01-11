/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;


import static frc.robot.Constants.DriveConstants.kDDriveVel;
import static frc.robot.Constants.DriveConstants.kPDriveVel;
import static frc.robot.Constants.DriveConstants.kaVoltSecondsSquaredPerMeter;
import static frc.robot.Constants.DriveConstants.ksVolts;
import static frc.robot.Constants.DriveConstants.kvVoltSecondsPerMeter;
import static frc.robot.Constants.DriveConstants.kEncoderCPR;
import static frc.robot.Constants.DriveConstants.kDistancePerWheelRevolutionMeters;
import static frc.robot.Constants.DriveConstants.kGearReduction;
import static frc.robot.Constants.DriveConstants;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.EncoderType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {
  /**
   * Class variables for storing data and objects that this subsystem needs to
   * operate.
   *
   * These variables are declared private to this class, because no other object
   * should need direct access to these objects. Any changes or commands to the 
   * drivetrain should come through public methods like the tankDrive() function.
   */

  private CANSparkMax leftNEO;
  private CANSparkMax rightNEO;
  private MotorController leftSide;
  private MotorController rightSide;
  private DifferentialDrive drive;
  private CANEncoder m_leftEncoder;
  private CANEncoder m_rightEncoder;
  private SimpleMotorFeedforward  feedforward = 
  new SimpleMotorFeedforward(ksVolts, kvVoltSecondsPerMeter, kaVoltSecondsSquaredPerMeter);
  private PIDController left_PIDController = new PIDController(kPDriveVel, 0.0, kDDriveVel);
  private PIDController right_PIDController =  new PIDController(kPDriveVel, 0.0, kDDriveVel);

  // The gyro sensor
  // private final Gyro gyro = new ADXRS450_Gyro();
  private final AHRS gyro = new AHRS(SPI.Port.kMXP);

  // Odometry class for tracking robot pose
  private DifferentialDriveOdometry odometry;

  public DriveSubsystem() {
    leftNEO = new CANSparkMax(Constants.DriveConstants.LEFT_NEO_CANID, MotorType.kBrushless);
    rightNEO = new CANSparkMax(Constants.DriveConstants.RIGHT_NEO_CANID, MotorType.kBrushless);
   
    // set all NEOs to factory defaults
    leftNEO.restoreFactoryDefaults();
    rightNEO.restoreFactoryDefaults();

    rightNEO.setInverted(true);

    rightNEO.setIdleMode(CANSparkMax.IdleMode.kBrake);
    leftNEO.setIdleMode(CANSparkMax.IdleMode.kBrake);

    // assign each motor to a MotorControllerGroup
    leftSide = new MotorControllerGroup(leftNEO);
    rightSide = new MotorControllerGroup(rightNEO);

    // create our DifferentialDrive class
    drive = new DifferentialDrive(leftSide, rightSide);

    leftEncoder = leftNEO.getEncoder(EncoderType.kHallSensor, kEncoderCPR);
    rightEncoder = rightNEO.getEncoder(EncoderType.kHallSensor, kEncoderCPR);
  
    // set scaling factor for CANEncoder.getPosition() so that it matches the output of
    // Encoder.getDistance() method.
    leftEncoder.setPositionConversionFactor(
        kDistancePerWheelRevolutionMeters * kGearReduction );
    rightEncoder.setPositionConversionFactor(
        kDistancePerWheelRevolutionMeters * kGearReduction );

    // Native scale is RPM. Scale velocity so that it is in meters/sec
    leftEncoder.setVelocityConversionFactor(
        kDistancePerWheelRevolutionMeters * kGearReduction / (60.0));
    rightEncoder.setVelocityConversionFactor(
        kDistancePerWheelRevolutionMeters * kGearReduction / (60.0));

    resetEncoders();
    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

  }
  
  @Override
  public void periodic() {
        // Note: periodic() is run by the scheduler, always. No matter what.
    // Update the odometry in the periodic block
    double leftDist = leftEncoder.getPosition();
    double rightDist = rightEncoder.getPosition();
    odometry.update(Rotation2d.fromDegrees(getHeading()), leftDist, rightDist);

    // log drive train and data to Smartdashboard
    SmartDashboard.putNumber("IMU_Yaw", gyro.getYaw());
    /* Display 9-axis Heading (requires magnetometer calibration to be useful) */
    SmartDashboard.putNumber("IMU_FusedHeading", gyro.getFusedHeading());

    // report the wheel speed, position, and pose
    SmartDashboard.putNumber("left_wheel_Velocity", leftEncoder.getVelocity());
    SmartDashboard.putNumber("right_wheel_Velocity", rightEncoder.getVelocity());

    SmartDashboard.putNumber("left_wheel_Distance", leftDist); // leftEncoder.getPosition());
    SmartDashboard.putNumber("right_wheel_Distance", rightDist); // rightEncoder.getPosition());

    SmartDashboard.putNumber("DistFactorL", leftEncoder.getPositionConversionFactor() );
    SmartDashboard.putNumber("DistFactorR", rightEncoder.getPositionConversionFactor() );

    Pose2d currentPose = odometry.getPoseMeters();
    SmartDashboard.putNumber("pose_x",currentPose.getTranslation().getX());
    SmartDashboard.putNumber("pose_y",currentPose.getTranslation().getY());
    SmartDashboard.putNumber("pose_x_inches",currentPose.getTranslation().getX() * 39.3701);
    SmartDashboard.putNumber("pose_y_inches",currentPose.getTranslation().getY() * 39.3701);
    SmartDashboard.putNumber("pose_theta", currentPose.getRotation().getDegrees());
  }


  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /**
   * Returns the Feedforward settings for the drivetrain.
   * 
   * @return Feedforward
   */
  public SimpleMotorFeedforward getFeedforward() {
    return feedforward;
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
        leftEncoder.getVelocity(),
        rightEncoder.getVelocity());
  }

  /**
   * Returns the left PIDController object
   *
   * @return PIDController
   */
  public PIDController getLeftPidController() {
    return left_PIDController;
  }

  /**
   * Returns the right PIDController object
   *
   * @return PIDController
   */
  public PIDController getRightPidController() {
    return right_PIDController;
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }

  public void arcadeDrive(double xSpeed, double zRotation) {
    drive.arcadeDrive(xSpeed, zRotation);
  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    drive.tankDrive(leftSpeed, rightSpeed);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftSide.setVoltage(leftVolts);
    rightSide.setVoltage(rightVolts);
    drive.feed();
  }
  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  public void resetEncoders() {
    leftEncoder.setPosition(0.0); // was Encoder.reset();
    rightEncoder.setPosition(0.0); // was Encoder.reset();
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    // Was Encoder.getDistance()
    return (leftEncoder.getPosition() + rightEncoder.getPosition()) / 2.0;
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public CANEncoder getLeftEncoder() {
    return leftEncoder;
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public CANEncoder getRightEncoder() {
    return rightEncoder;
  }

  /**
   * Zeroes the heading of the robot.
   */
  public void zeroHeading() {
    gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from 180 to 180
   */
  public double getHeading() {
    // According to https://pdocs.kauailabs.com/navx-mxp/guidance/yaw-drift/
    // it is easier to use getAngle() instead of gyro.getFusedHeading() because it
    // does not require calibrating the magnetometer. Once motors are energized compass
    // readings are unreliable.
    // return gyro.getFusedHeading() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);

    return Math.IEEEremainder(gyro.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }
}
