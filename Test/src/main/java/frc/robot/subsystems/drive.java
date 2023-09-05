// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
  
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriverConstants;

public class drive extends SubsystemBase {
  /** Creates a new drive. */
  VictorSP leftFront = new VictorSP(DriverConstants.leftFront);
  VictorSP leftBack = new VictorSP(DriverConstants.leftBack);

  VictorSP rightFront = new VictorSP(DriverConstants.rightFront);
  VictorSP rightBack = new VictorSP(DriverConstants.rightBack);

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
