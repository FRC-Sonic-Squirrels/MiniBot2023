/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.DriveForward;
import frc.robot.commands.Spin;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Spinner;
import frc.robot.subsystems.GyroHandler;

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
  private final GyroHandler gyroHandler = new GyroHandler();

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
    spinner.setDefaultCommand(new InstantCommand(() -> spinner.setPercentOutput(0.0), spinner));
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


    /*
    controller.leftBumper() // LB
      .onTrue(null)
      .onFalse(null);
    */

    controller.rightBumper() // RB
      .onTrue(new Spin(spinner))
      .onFalse(Commands.runOnce(() -> spinner.setPercentOutput(0.0), spinner));

    var UpKey = new Trigger(() -> SmartDashboard.getBoolean("2", false));
    /*x.onTrue(new Spin(spinner))
    .onFalse(Commands.runOnce(() -> spinner.setPercentOutput(0.0), spinner));*/
    UpKey.toggleOnTrue(
      new DriveForward(driveSubsystem, true)
      );
    
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
