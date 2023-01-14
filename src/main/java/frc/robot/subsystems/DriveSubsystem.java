/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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

  private WPI_Pigeon2 m_pigeon =
    new WPI_Pigeon2(17);

  public DriveSubsystem() {
    leftNEO = new CANSparkMax(Constants.DriveConstants.LEFT_NEO_CANID, MotorType.kBrushless);
    rightNEO = new CANSparkMax(Constants.DriveConstants.RIGHT_NEO_CANID, MotorType.kBrushless);
   
    // set all NEOs to factory defaults, dont want a previous deploy interfering.
    leftNEO.restoreFactoryDefaults();
    rightNEO.restoreFactoryDefaults();

    // assign each motor to a MotorControllerGroup
    leftSide = new MotorControllerGroup(leftNEO);
    rightSide = new MotorControllerGroup(rightNEO);

    rightSide.setInverted(true);

    // create our DifferentialDrive class
    drive = new DifferentialDrive(leftSide, rightSide);
    m_pigeon.setAccumZAngle(0);
    m_pigeon.reset();
  }
  
  @Override
  public void periodic() {
    {
      var ba = new short[3];
      var vector = new double[3];

      if (m_pigeon.getBiasedAccelerometer(ba) == ErrorCode.OK) {
        // vector towards the ground
        SmartDashboard.putNumber("accel X", (ba[0] / 16384.0) * 10);
        SmartDashboard.putNumber("accel Y", (ba[1] / 16384.0) * 10);
        SmartDashboard.putNumber("accel Z", (ba[2] / 16384.0) * 10);
      } else {
        System.out.println("FAILED accel");
      }

      if (m_pigeon.getGravityVector(vector) == ErrorCode.OK) {
        // vector towards the ground
        SmartDashboard.putNumber("GV Gravity Vector X", vector[0]);
        SmartDashboard.putNumber("GV Gravity Vector Y", vector[1]);
        SmartDashboard.putNumber("GV Gravity Vector Z", vector[2]);
      }
      if (m_pigeon.getYawPitchRoll(vector) == ErrorCode.OK) {
        // Array to fill with yaw[0], pitch[1], and roll[2] data.
        // Yaw is within [-368,640, +368,640] degrees.
        // Pitch is within [-90,+90] degrees.
        // Roll is within [-90,90] degrees.
        SmartDashboard.putNumber("GV Robot Yaw", vector[0]);
        SmartDashboard.putNumber("GV Robot Pitch", vector[1]);
        SmartDashboard.putNumber("GV Robot Roll", vector[2]);
      }
      if (m_pigeon.getRawGyro(vector) == ErrorCode.OK) {
        // measured in degrees per second
        SmartDashboard.putNumber("GV Robot rotation X deg per sec", vector[0]);
        SmartDashboard.putNumber("GV Robot rotation Y deg per sec", vector[1]);
        SmartDashboard.putNumber("GV Robot rotation Z deg per sec", vector[2]);
        
      }

      if (m_pigeon.getAccumGyro(vector) == ErrorCode.OK) {
        // measured in degrees per second
        SmartDashboard.putNumber("GV Robot Accumulative rotation X deg per sec", vector[0]);
        SmartDashboard.putNumber("GV Robot Accumulative rotation Y deg per sec", vector[1]);
        SmartDashboard.putNumber("GV Robot Accumulative rotation Z deg per sec", vector[2]);
      }
    }
  }

  public void arcadeDrive(double xSpeed, double zRotation) {
    drive.arcadeDrive(xSpeed, zRotation);
  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    drive.tankDrive(leftSpeed, rightSpeed);
  }
}
