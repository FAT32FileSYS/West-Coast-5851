// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
  
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriverConstants;

public class drive extends SubsystemBase {  
  /** Creates a new drive. */
  WPI_TalonFX leftFront = new WPI_TalonFX(DriverConstants.leftFront);
  WPI_TalonFX leftBack = new WPI_TalonFX(DriverConstants.leftBack);

  WPI_TalonFX rightFront = new WPI_TalonFX(DriverConstants.rightFront);
  WPI_TalonFX rightBack = new WPI_TalonFX(DriverConstants.rightBack);

  TalonFX intake = new TalonFX(DriverConstants.intake);

  MotorControllerGroup leftSide = new MotorControllerGroup(leftBack, leftFront);
  MotorControllerGroup rightSide = new MotorControllerGroup(rightBack, rightFront);

  DifferentialDrive drive = new DifferentialDrive(leftSide, rightSide);

  public drive() 
  {
    System.out.println("Sub works");
  }


  public void move(double leftSpeed, double rightSpeed)
  {
    leftSide.setInverted(true);
    drive.tankDrive(leftSpeed, rightSpeed);
  }  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
