// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.drive;
import frc.robot.Constants.AutoConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class autoDrivePIDCommand extends CommandBase {

  public drive drive = new drive();

  private double leftsetPoint = 5;
  private final double driveTick2Feet = 1.0 / 128 * 6 * Math.PI / 12;
  private double leftcurrentLocation = drive.leftSideEncoder * driveTick2Feet;
  private double currentTime = Timer.getFPGATimestamp();

  private double leftcurrentError = leftsetPoint - leftcurrentLocation;
  

  double leftDerivitive = AutoConstants.KP * leftcurrentError;
  double leftIntegral = leftcurrentError * currentTime;
    



  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
