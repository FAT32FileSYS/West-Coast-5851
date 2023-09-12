// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;



public class IntakeCommand extends CommandBase {
  /** Creates a new IntakeCommand. */
  Intake intakeSub;
  double speed;

  public IntakeCommand(double intakeSpeed, Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.

    intakeSub = intake;
    speed = intakeSpeed;
    addRequirements(intakeSub);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeSub.Move(speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeSub.Move(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSub.Move(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
