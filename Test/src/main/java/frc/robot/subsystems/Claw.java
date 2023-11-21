// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriverConstants;

public class Claw extends SubsystemBase {
  /** Creates a new Claw. */
  
  WPI_TalonFX claw = new WPI_TalonFX(DriverConstants.claw);

  public void Move(double clawSpeed)
  {
    claw.set(clawSpeed); 
  }

  public Claw() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
