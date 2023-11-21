// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawArm;

public class ClawArmCommand extends CommandBase {
  /** Creates a new ClawArmCommand. */
  ClawArm clawArmSub;
  double speed;

  public ClawArmCommand(double clawArmSpeed, ClawArm clawArm) {
    // Use addRequirements() here to declare subsystem dependencies.

    clawArmSub = clawArm;
    speed = clawArmSpeed;
    addRequirements(clawArmSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    clawArmSub.Move(speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    clawArmSub.Move(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    clawArmSub.Move(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
