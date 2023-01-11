/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */


public final class Constants {
    public static final class DriveConstants {
        public static final int LEFT_NEO_CANID = 2;
        public static final int RIGHT_NEO_CANID = 1;
        public static final int DRIVECONTROLLER_ID = 0;

        public static final boolean kLeftEncoderReversed = false;
        public static final boolean kRightEncoderReversed = false;

        public static final boolean kGyroReversed = true;

        // Actual Minibot width (wheel-to-wheel) 15.625 inches or 0.396875 meters
        public static final double kTrackwidthMeters = 0.396875;
        public static final DifferentialDriveKinematics kDriveKinematics =
                new DifferentialDriveKinematics(kTrackwidthMeters);

        // Determined using frc-characterization tool
        // old: 0.16,  4.66,  0.486
        public static final double ksVolts = 0.138;
        public static final double kvVoltSecondsPerMeter = 4.5;
        public static final double kaVoltSecondsSquaredPerMeter = 0.444;

        // Determined using frc-characterization
        public static final double kPDriveVel = 12.0;
        public static final double kDDriveVel = 0.0;

        // SparkMax at 1ms update rate
        // kPDriveVel = 5.65
        // kDDriveVel = 2540.0 

        public static final int kEncoderCPR = 4096;

        // Approximately 6 inch (0.1524 meters) traction wheels, measured 0.15836 m 
        // Measured circumference = 0.4975 m
        public static final double kWheelDiameterMeters = 0.15836;
        public static final double kDistancePerWheelRevolutionMeters =
                kWheelDiameterMeters * Math.PI;

        // gear reduction from NEO to wheels 18:1
        public static final double kGearReduction = 1.0 / 18.0;

        // Assumes the encoders are directly mounted on the motor shafts
        public static final double kEncoderDistancePerPulseMeters =
                (kDistancePerWheelRevolutionMeters * kGearReduction) / (double) kEncoderCPR;

        // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;

    }

    public static final class OIConstants {
        public static final int kDriverController = 0;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond =  1.5;
        public static final double kMaxAccelerationMetersPerSecondSquared = 0.50;

        // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
    }
}