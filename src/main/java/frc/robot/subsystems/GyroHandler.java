// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class GyroHandler extends SubsystemBase {
  /** Creates a new GyroHandler. */

  private WPI_Pigeon2 pigeon2 =
    new WPI_Pigeon2(Constants.DeviceConstants.PIGEON2_CANID);
    
  public GyroHandler() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public short[] get_BiasedAccelerometerValues() {
    var BiasedAccelerometerValues = new short[3];
    pigeon2.getBiasedAccelerometer(BiasedAccelerometerValues);
    return BiasedAccelerometerValues;
  }

  public double[] get_GravityVectorValues() {
    var GravityVectorValues = new double[3];
    pigeon2.getGravityVector(GravityVectorValues);
    return GravityVectorValues;
  }

  public double[] get_YawPitchRollValues() {
    var YawPitchRollValues = new double[3];
    pigeon2.getYawPitchRoll(YawPitchRollValues);
    return YawPitchRollValues;
  }

  public double[] get_RawGyroValues() {
    var RawGyroValues = new double[3];
    pigeon2.getRawGyro(RawGyroValues);
    return RawGyroValues;
  }

  public double[] get_AccumGyroValues() {
    var AccumGyroValues = new double[3];
    pigeon2.getAccumGyro(AccumGyroValues);
    return AccumGyroValues;
  }
}
