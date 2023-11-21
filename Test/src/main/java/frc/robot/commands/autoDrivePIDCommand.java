// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.drive;
import frc.robot.Constants.AutoConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class autoDrivePIDCommand extends CommandBase {

  private final drive driveSub;
  private final PIDController drivePID;
  private final double goal;
  private boolean done;

  public autoDrivePIDCommand(double setPoint, drive drive) {
    goal = setPoint;
    driveSub = drive;
    drivePID = new PIDController(AutoConstants.KP, AutoConstants.KI, AutoConstants.KD);
    
    drivePID.setSetpoint(setPoint);

    addRequirements(driveSub);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
