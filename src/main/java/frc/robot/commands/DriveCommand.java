/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;

/**
 * An example command that uses an example subsystem.
 */
public class DriveCommand extends CommandBase {
  private DriveSubsystem driveSubsystem;
  private CommandXboxController driveController;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveCommand(DriveSubsystem driveSubsystem, CommandXboxController driveController) {
    this.driveSubsystem = driveSubsystem;
    this.driveController = driveController;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem); // Drive is required because its the only way we can interface with the motors from this command in a way thats easily controllable through a fucntion.
  }

  double DRIVEMULTIPLIER_SAFETY = Constants.DriveConstants.DRIVEMULTIPLIER_SAFETY;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DRIVEMULTIPLIER_SAFETY = Constants.DriveConstants.DRIVEMULTIPLIER_SAFETY;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveSubsystem.arcadeDrive(driveController.getLeftY()*DRIVEMULTIPLIER_SAFETY, driveController.getRightX()*DRIVEMULTIPLIER_SAFETY);
    SmartDashboard.putNumber("DriveControllerLeftY", driveController.getLeftY()*DRIVEMULTIPLIER_SAFETY);
    SmartDashboard.putNumber("DriveControllerRightX", driveController.getRightX()*DRIVEMULTIPLIER_SAFETY);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
