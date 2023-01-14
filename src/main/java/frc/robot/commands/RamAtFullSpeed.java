// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants;

public class RamAtFullSpeed extends CommandBase {
  private DriveSubsystem driveSubsystem;
  private CommandXboxController driveController;
  /** Creates a new RamAtFullSpeed. */
  public RamAtFullSpeed(DriveSubsystem driveSubsystem, CommandXboxController driveController) {
    this.driveSubsystem = driveSubsystem;
    this.driveController = driveController;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  double RamAtFullSpeedForInSeconds = Timer.getFPGATimestamp();
  double CommandLengthInSeconds = 0.5;
  double RamSpeed = Constants.DriveConstants.FORWARDDODGE_SPD;
  double TimesRan = 0;

  @Override
  public void initialize() {
    RamAtFullSpeedForInSeconds = Timer.getFPGATimestamp();
    TimesRan = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (TimesRan == 0) {
      driveSubsystem.tankDrive(RamSpeed*.5, RamSpeed*.5);
      TimesRan += 1;
    }
    else {
      driveSubsystem.tankDrive(RamSpeed, RamSpeed);
      TimesRan += 1;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Timer.getFPGATimestamp() - RamAtFullSpeedForInSeconds >= CommandLengthInSeconds) {
      return true;
    }
    else {
      return false;
    }
  }
}
