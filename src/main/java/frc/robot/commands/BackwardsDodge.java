// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class BackwardsDodge extends CommandBase {
  private DriveSubsystem driveSubsystem;
  private CommandXboxController driveController;
  /** Creates a new BackwardsDodge. */
  public BackwardsDodge(DriveSubsystem driveSubsystem, CommandXboxController driveController) {
    this.driveSubsystem = driveSubsystem;
    this.driveController = driveController;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  double BackwardsDodgeForInSeconds = Timer.getFPGATimestamp();
  double CommandLengthInSeconds = 0.2;
  double BackSpeed = Constants.DriveConstants.BACKWARDDODGE_SPD;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    BackwardsDodgeForInSeconds = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveSubsystem.arcadeDrive(BackSpeed, driveController.getRightX());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (BackwardsDodgeForInSeconds >= BackwardsDodgeForInSeconds + CommandLengthInSeconds) {
      return true;
    }
    else {
      return false;
    }
  }
}
