// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;

import frc.robot.Constants.DriverConstants;;


public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

  VictorSP intake = new VictorSP(DriverConstants.intake);


  public void Move(double IntakeSpeed)
  {
    intake.set(IntakeSpeed);
  }

  public Intake() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
