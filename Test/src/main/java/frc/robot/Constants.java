// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
     
  public final static class DriverConstants {
    public static int 
    leftFront = 1,
    leftBack = 2,
    rightFront = 3,
    rightBack = 4,

    /*left front = 1
      left back = 2
      right front = 3
      right back = 4
    */
     
    //Armando controls
    armando = 0,  leftAxis = 1, rightAxis = 5, 
    cascade = 1, claw = 2, clawArm = 3, clawButton = 1, clawOpen = 6, clawClose = 5, clawArmUp = 8, clawArmDown = 7,

    //driver 2 controls
    driver2 = 1;

    public static double driveSpeed = .6, intakeSpeed = 0.6, cascadeSpeed = 1, clawSpeed = 1, clawArmSpeed = 1;

  }

  public static final class AutoConstants{
    public static final double
    KP = 0.2, KI = 0.2, KD = 0.15;
    public static final double kPXController = 3;
    public static final double kDXController = 0;
  }

  public static final class KineConstants {
    public static final double kCountsPerRev = 2048,
        kGearRatio = 6.9048, // may change
        kWheelRadiusInches = 3,

        ksVolts = 0.21274,
        kvVoltSecondsPerMeter = 1.5901,
        kaVoltSecondSquaredPerMeter = 0.2235,
        kpDriveVelocity = 0.43958,

        kTrackWidthMeters = Units.inchesToMeters(25),

        kMaxSpeedMetersPerSecond = 2,
        kMaxAccelerationMetersPerSecSquared = 2,

        kRamseteB = 2,
        kRamseteZeta = .7;

    public static final DifferentialDriveKinematics kDrive = new DifferentialDriveKinematics(kTrackWidthMeters);

  }


}
