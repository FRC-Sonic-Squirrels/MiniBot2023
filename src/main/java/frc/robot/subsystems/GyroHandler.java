// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.numbers.N3;
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
  private double[] Quaternion6d = {0.0, 0.0, 0.0, 0.0};

  private GyroHandler() {}

  @Override
  public void periodic() {
    pigeon2.getBiasedAccelerometer(BiasedAccelerometerValues);
    pigeon2.getGravityVector(GravityVectorValues);
    pigeon2.getYawPitchRoll(YawPitchRollValues);
    pigeon2.getRawGyro(RawGyroValues);
    pigeon2.getAccumGyro(AccumGyroValues);
    pigeon2.get6dQuaternion(Quaternion6d);
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

  public double[] get_Quaternion6d() {
    return Quaternion6d;
  }

  // ANYTHING BELOW IS ALL TESTING
  // ANYTHING BELOW IS ALL TESTING
  // ANYTHING BELOW IS ALL TESTING
  // ANYTHING BELOW IS ALL TESTING

  public Vector<N3> Quaternion6dToRotationVector() {
    Quaternion PigeonQuaternion6d = new Quaternion(Quaternion6d[0], Quaternion6d[1], Quaternion6d[2], Quaternion6d[3]);
    return PigeonQuaternion6d.toRotationVector();
  }

  public double[] getEulerFromQuaternion6d() {
    // https://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToEuler/indexLocal.htm
    // https://www.euclideanspace.com/maths/standards/index.htm

    double w = Quaternion6d[0];
    double x = Quaternion6d[1];
    double y = Quaternion6d[2];
    double z = Quaternion6d[3];

    double sqw = w*w;
    double sqx = x*x;
    double sqy = y*y;
    double sqz = z*z;

    var heading = Math.atan2(2.0 * (x*y + z*w),(sqx - sqy - sqz + sqw));
    var bank = Math.atan2(2.0 * (y*z + x*w),(-sqx - sqy + sqz + sqw));
    var attitude = Math.asin(-2.0 * (x*z - y*w));

    double[] eulerAnglesFromQuaternion6d = {heading, attitude, bank};
  return eulerAnglesFromQuaternion6d;
}
}
