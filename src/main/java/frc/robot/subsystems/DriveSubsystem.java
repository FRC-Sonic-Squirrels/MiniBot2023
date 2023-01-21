/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
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

  private final RelativeEncoder m_lefEncoder;
  private final RelativeEncoder m_righEncoder;

  private final double GEAR_RATIO = 1.0/18.0;
  private double REVELUTIONS_TO_INCHES = GEAR_RATIO * (6.06250 * Math.PI);



  public DriveSubsystem() {
    leftNEO = new CANSparkMax(Constants.DriveConstants.LEFT_NEO_CANID, MotorType.kBrushless);
    rightNEO = new CANSparkMax(Constants.DriveConstants.RIGHT_NEO_CANID, MotorType.kBrushless);
   
    // set all NEOs to factory defaults
    leftNEO.restoreFactoryDefaults();
    rightNEO.restoreFactoryDefaults();

    rightNEO.setInverted(true);

    // assign each motor to a MotorControllerGroup
    leftSide = new MotorControllerGroup(leftNEO);
    rightSide = new MotorControllerGroup(rightNEO);


    // create our DifferentialDrive class
    drive = new DifferentialDrive(leftSide, rightSide);
    m_lefEncoder = leftNEO.getEncoder();
    m_righEncoder = rightNEO.getEncoder();




  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("distanceTotal",getDistanceInInches());
  }

  public void arcadeDrive(double xSpeed, double zRotation) {
    drive.arcadeDrive(xSpeed, zRotation);
  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    drive.tankDrive(leftSpeed, rightSpeed);
  }
  public double getDistanceInInches() {
    double leftValue = m_lefEncoder.getPosition() * REVELUTIONS_TO_INCHES;
    double rightValue = m_righEncoder.getPosition() * REVELUTIONS_TO_INCHES;

    SmartDashboard.putNumber("distanceLeft",leftValue );
    SmartDashboard.putNumber("distanceRight",rightValue );
    SmartDashboard.putNumber("inchesPerSecond", (((m_lefEncoder.getVelocity()+m_righEncoder.getVelocity())/2)/REVELUTIONS_TO_INCHES)/60);
    SmartDashboard.putNumber("RPM", (m_lefEncoder.getVelocity()+m_righEncoder.getVelocity())/2);
    //RPM for 1"/s = 57.295779513 ; multiply this by however many inches ; might create a function to do this for me... maybe 
    return (leftValue + rightValue) /2;
  }
}
