// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
    leftFront = 14,
    leftBack = 15,
    rightFront = 12,
    rightBack = 13,
    intake = 1,
    flyWheel = 1,

    /*left front = 1
      left back = 2
      right front = 3
      right back = 4
    */
     
    
    armando = 0,  leftAxis = 1, rightAxis = 1, intakeStart = 6, intakeRev = 5, flywheelStart = 4;

    public static double driveSpeed = 0.5;
    public static double intakeSpeed = 0.6;

  }

}
