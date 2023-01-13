/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.BackwardsDodge;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.RamAtFullSpeed;
import frc.robot.commands.Spin;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Spinner;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final Spinner spinner = new Spinner();

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  // private XboxController controller = new XboxController(DriveConstants.DRIVECONTROLLER_ID);
  private CommandXboxController controller = new CommandXboxController(DriveConstants.DRIVECONTROLLER_ID);

  // private final JoystickButton spinnerButton =
  //     new JoystickButton(controller, XboxController.Button.kRightBumper.value);

  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    
    driveSubsystem.setDefaultCommand(new DriveCommand(driveSubsystem, controller));

    // default spinner command is to stop spinning
    spinner.setDefaultCommand(new InstantCommand(() -> spinner.setPercentOutput(0.0)));
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // spine while holding spinner button down
    // spinnerButton.onTrue(new Spin(spinner));
    // spinnerButton.onFalse(new InstantCommand(() -> spinner.setPercentOutput(0.0)));

    controller.rightBumper()
      .onTrue(new Spin(spinner))
      .onFalse(Commands.runOnce(() -> spinner.setPercentOutput(0.0), spinner));

    controller.a()
      .onTrue(new BackwardsDodge(driveSubsystem, controller));
      /*
    controller.b()
      .onTrue(null);
    controller.x()
      .onTrue(null);
      */
    controller.y()
      .onTrue(new RamAtFullSpeed(driveSubsystem, controller));
  }



  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  /*public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    //return autoCommand;
  }*/
}
