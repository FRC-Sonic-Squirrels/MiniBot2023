package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;


public class OffsetElevatorCommand extends CommandBase {
    private final ElevatorSubsystem elevatorSubsystem;
    private final double offsetAmount;
    private final int elevator;

    public OffsetElevatorCommand(ElevatorSubsystem elevatorSubsystem, int elevator, double offsetAmount) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.offsetAmount = offsetAmount;
        this.elevator = elevator;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.elevatorSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        elevatorSubsystem.offsetElevatorMotor(elevator, offsetAmount);

    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
