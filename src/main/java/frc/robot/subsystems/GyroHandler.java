// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class GyroHandler extends SubsystemBase {

  private WPI_Pigeon2 pigeon2 = 
    new WPI_Pigeon2(Constants.DeviceConstants.PIGEON2_CANID);

  private short[] BiasedAccelerometerValues = {0,0,0};
  private double[] GravityVectorValues = {0.0, 0.0, 0.0};
  private double[] YawPitchRollValues = {0.0, 0.0, 0.0};
  private double[] RawGyroValues = {0.0, 0.0, 0.0};
  private double[] AccumGyroValues = {0.0, 0.0, 0.0};

  private GyroHandler() {}

  @Override
  public void periodic() {
    pigeon2.getBiasedAccelerometer(BiasedAccelerometerValues);
    pigeon2.getGravityVector(GravityVectorValues);
    pigeon2.getYawPitchRoll(YawPitchRollValues);
    pigeon2.getRawGyro(RawGyroValues);
    pigeon2.getAccumGyro(AccumGyroValues);
  }

  public short[] getBiasedAccelerometerValues(){
    return BiasedAccelerometerValues;
  }

  public short[] get_BiasedAccelerometerValues() {
      return BiasedAccelerometerValues;
  }
  
  public double[] get_GravityVectorValues() {
      return GravityVectorValues;
  }
  
  public double[] get_YawPitchRollValues() {
      return YawPitchRollValues;
  }
  
  public double[] get_RawGyroValues() {
      return RawGyroValues;
  }
  
  public double[] get_AccumGyroValues() {
      return AccumGyroValues;
  }
}
