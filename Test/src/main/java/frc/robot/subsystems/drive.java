// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriverConstants;

public class drive extends SubsystemBase {  
  /** Creates a new drive. */
  public WPI_TalonFX leftFront = new WPI_TalonFX(DriverConstants.leftFront);
  public WPI_TalonFX leftBack = new WPI_TalonFX(DriverConstants.leftBack);

  public WPI_TalonFX rightFront = new WPI_TalonFX(DriverConstants.rightFront);
  public WPI_TalonFX rightBack = new WPI_TalonFX(DriverConstants.rightBack);

  MotorControllerGroup leftSide = new MotorControllerGroup(leftBack, leftFront);
  MotorControllerGroup rightSide = new MotorControllerGroup(rightBack, rightFront);

  public DifferentialDrive drive = new DifferentialDrive(leftSide, rightSide);
  
  public double leftSideEncoder;
  public double rightSideEncoder;

  public drive() 
  {
    System.out.println("Sub works");
  }


  public void move(double leftSpeed, double rightSpeed)
  {
    rightSide.setInverted(true);    
    drive.tankDrive(leftSpeed, rightSpeed);
  }  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
