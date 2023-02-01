// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class TurnTimearcadedrive extends CommandBase {
  private DriveSubsystem drive;
  private double turn;
  private double speed;
  private double offset;
  /** Creates a new TurnTimearcadedrive. */
  public TurnTimearcadedrive(DriveSubsystem drive, double turn, double speed) {
  addRequirements(drive);
  this.drive = drive;
  this.speed = speed;
  this.turn = turn;
  
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    offset = drive.getDistanceInches();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.arcadeDrive(speed, turn);
    SmartDashboard.putNumber("TurnTimearcadedrive", drive.getDistanceInches() - offset);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.arcadeDrive(0, 0);


  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (drive.getDistanceInches()>= turn)
    return true;
    else {
    return false;
    }
  }
}
