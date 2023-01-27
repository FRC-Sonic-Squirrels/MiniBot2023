package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {

    // With eager singleton initialization, any static variables/fields used in the 
    // constructor must appear before the "INSTANCE" variable so that they are initialized 
    // before the constructor is called when the "INSTANCE" variable initializes.

    private WPI_TalonFX elevatorMotor1;

    public ElevatorSubsystem() {
        elevatorMotor1 = new WPI_TalonFX(Constants.FALCONELEVATORTEST_ID);
        elevatorMotor1.set(0.0);
        elevatorMotor1.setSelectedSensorPosition(0);
    }

    public void setElevatorMotor1(double setSpeed){
        elevatorMotor1.set(setSpeed);
    }

    public void stopElevatorMotor(float motor){
        if (motor == 1){
            elevatorMotor1.stopMotor();
        }
    }

    public void offsetElevatorMotor(int motor, double pos){
        if (motor == 1){
            double current_motor_position = elevatorMotor1.getSelectedSensorPosition();
            elevatorMotor1.set(ControlMode.Position, current_motor_position+pos);
        }
    }

}

