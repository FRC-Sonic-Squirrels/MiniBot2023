// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SetDriveDistanceSubsystem;

public class DriveSetDistance extends CommandBase {
  /** Creates a new DriveSetDistance. */
  private SetDriveDistanceSubsystem setDriveDistanceSubsystem;
  //WE need:
  //drivetrain 
  //target 
  //speed to travel at

  public DriveSetDistance(SetDriveDistanceSubsystem setDriveDistanceSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.setDriveDistanceSubsystem = setDriveDistanceSubsystem;
   addRequirements(setDriveDistanceSubsystem);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
     
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
       setDriveDistanceSubsystem.tankDrive(10, 10);
    //Set the speed to something low 
    //10% maybe 
    //note: we need to put this in execute insted of initialize because 
    //we need to constantly tell the motor what to do because 
    //if we dont give it an instruction for long enough it triggers motor safety 
    //which is a protection system to make sure motors are always update in a periodic manner 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //set our drive train to 0% output 
      setDriveDistanceSubsystem.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //should end when drive train encoder value is > than our goal 
    return SetDriveDistanceSubsystem.getDistanceInches() >= 100;
  }
}