// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveTime extends CommandBase {
  private DriveSubsystem drive;
  private double seconds;
  private double speed;
  private double startTime;
  /** Creates a new DriveTime. */
  public DriveTime(DriveSubsystem drive, double sec, double speed) {
    addRequirements(drive);
    this.drive = drive;
    seconds = sec;
    this.speed = speed;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
      }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  drive.arcadeDrive(speed, 0);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(startTime - Timer.getFPGATimestamp() >= seconds ){
      return true;
    }
    else {
    return false;
    }
  }
}
//done and (should be) good