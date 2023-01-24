// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.DriveSubsystem;

public class DriveForward extends CommandBase {
  private DriveSubsystem driveSubsystem;
  private double Speed;
  private double xSpeed;
  /** Creates a new DriveForward. */
  public DriveForward(DriveSubsystem driveSubsystem, Double Speed, double xSpeed) {
    this.driveSubsystem = driveSubsystem;
    this.Speed = Speed;
    this.xSpeed = xSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      driveSubsystem.arcadeDrive(Speed, xSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
