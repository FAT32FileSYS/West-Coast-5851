// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.Constants.DriverConstants;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Cascade extends SubsystemBase {
  /** Creates a new Cascade. */

  WPI_VictorSPX cascade = new WPI_VictorSPX(DriverConstants.cascade);

  public void Move(double cascadeSpeed)
  {
    cascade.set(cascadeSpeed);
  }

  public Cascade() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
