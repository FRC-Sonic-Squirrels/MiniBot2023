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
  private RelativeEncoder m_leftEncoder;
  private RelativeEncoder m_righEncoder;
  private CANSparkMax leftNEO;
  private CANSparkMax rightNEO;
  private MotorController leftSide;
  private MotorController rightSide;
  private DifferentialDrive drive;
  private final double GEAR_RATIO = 1.0/18.0;
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

    m_leftEncoder = leftNEO.getEncoder();
    m_righEncoder = rightNEO.getEncoder();
  }
  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("totalDistance", getDistanceInches());
    // This method will be called once per scheduler run
  }

  public void arcadeDrive(double xSpeed, double zRotation) {
    drive.arcadeDrive(xSpeed, zRotation);
  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    drive.tankDrive(leftSpeed, rightSpeed);
  }




public double getDistanceInches(){
double REVOLUTIONS_TO_INCHES = GEAR_RATIO * (6.0 * Math.PI);
double leftvalue = m_leftEncoder.getPosition() * REVOLUTIONS_TO_INCHES;
double RightValue = m_righEncoder.getPosition() * REVOLUTIONS_TO_INCHES;
return (leftvalue + RightValue) / 2;

}
}