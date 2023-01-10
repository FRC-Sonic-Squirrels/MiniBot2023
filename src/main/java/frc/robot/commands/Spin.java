// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Spinner;

public class Spin extends CommandBase {
  /** Creates a new Spin. */
  private Spinner m_spinner;

  public Spin(Spinner spinner) {
    m_spinner = spinner;
    addRequirements(spinner);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_spinner.setPercentOutput(0.1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_spinner.setPercentOutput(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
