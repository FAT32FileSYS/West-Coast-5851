// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class ClawCommand extends CommandBase {
  /** Creates a new ClawCommand. */
  Claw clawSub;
  double speed;

  public ClawCommand(double clawSpeed, Claw claw) {
    // Use addRequirements() here to declare subsystem dependencies.
    clawSub = claw;
    speed = clawSpeed;
    addRequirements(clawSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    clawSub.Move(speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    clawSub.Move(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    clawSub.Move(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
