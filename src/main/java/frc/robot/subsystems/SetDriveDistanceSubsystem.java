package frc.robot.subsystems;



import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SetDriveDistanceSubsystem extends SubsystemBase{
    
    private MotorController forewordrightside; 
    private MotorController forewordleftside;
    private DifferentialDrive forewordrive; 
 

    public SetDriveDistanceSubsystem() {
        
        forewordleftside = new MotorControllerGroup(forewordleftside);
        forewordrightside = new MotorControllerGroup(forewordrightside);
        
        forewordrive = new DifferentialDrive(forewordleftside, forewordrightside);

    }
    @Override
    public void periodic() {

    }

    public void tankDrive (double leftSpeed, double rightSpeed) {
            forewordrive.tankDrive(leftSpeed, rightSpeed);
    }
    public static int getDistanceInches() {
        return 0;
    }


}

