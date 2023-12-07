// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive;
import frc.robot.Constants.AutoConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class autoDrivePIDCommand extends CommandBase {

  public double pastTime;
  public double leftMotorOutput;
  public double rightMotorOutput;
  public drive drivePID;
  public double leftPastError;
  public double rightPastError;
  public double leftCurrentLocation;
  public double rightCurrentLocation;
  public double leftTotalIntegral;
  public double rightTotalIntegral;
  public double leftSideEncoder;
  public double rightSideEncoder;
  drive drive = new drive();

/* 
  public autoDrivePIDCommand(drive drive, double leftMotorOutput, double rightMotorOutput)
  {
    drive = drivePID;
    leftMotorOutput = this.leftMotorOutput;
    rightMotorOutput = this.rightMotorOutput;
    addRequirements(drive);
  }*/

  @Override
  public void initialize() {
    pastTime = Timer.getFPGATimestamp();
    drive.leftSideEncoder = 0;
    drive.rightSideEncoder = 0;
    leftPastError = 0;
    rightPastError = 0;
    leftMotorOutput = 0;
    rightMotorOutput = 0;
    leftPastError = 0;
    rightPastError = 0;
    rightCurrentLocation = 0;
    leftCurrentLocation = 0;
    leftTotalIntegral = 0;
    rightTotalIntegral = 0;
    System.out.println("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA");
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    leftSideEncoder = drive.leftFront.getSelectedSensorPosition();
    rightSideEncoder = drive.rightFront.getSelectedSensorPosition();

    autoDrivePID();
    
    System.out.println("left" + leftMotorOutput);
    System.out.println("right" + rightMotorOutput);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }


  public void autoDrivePID()
  {
  final double driveTick2Inches = (6 * Math.PI) / 16384;
  double currentTime = Timer.getFPGATimestamp();
  double dt = currentTime - pastTime;
  System.out.println("dt" + dt + "current time" + currentTime + "past time" + pastTime);
  

  //distance units in inches
  double leftSetPoint = 12; 
  double rightSetPoint = 12;
  
  leftCurrentLocation = leftSideEncoder * driveTick2Inches;
  rightCurrentLocation = rightSideEncoder * driveTick2Inches;

  System.out.println("left location" + leftCurrentLocation + "right location" + rightCurrentLocation);

  double leftCurrentError = leftSetPoint - leftCurrentLocation;
  double rightCurrentError = rightSetPoint - rightCurrentLocation;

  System.out.println("left error" + leftCurrentError + "right error" + rightCurrentError + "leftpast error" + leftPastError + "rightpast error" + rightPastError);
  
  double leftConstant = AutoConstants.KP * leftCurrentError;
  double leftDerivitive = AutoConstants.KD * (leftCurrentError - leftPastError) / dt;
  double leftcurrentIntegral = AutoConstants.KI * ((leftCurrentError * currentTime) - (leftCurrentError * pastTime));

  double rightConstant = AutoConstants.KP * rightCurrentError;
  double rightDerivitive = AutoConstants.KD * (rightCurrentError - rightPastError) / dt;
  double rightcurrentIntegral = AutoConstants.KI * ((rightCurrentError * currentTime) - (rightCurrentError * pastTime));

  leftTotalIntegral += leftcurrentIntegral;
  rightTotalIntegral += rightcurrentIntegral;

  leftMotorOutput = leftConstant + leftDerivitive + leftTotalIntegral;
  rightMotorOutput = rightConstant + rightDerivitive + rightTotalIntegral;

  System.out.println("left mototr output" + leftMotorOutput + "right mtotor output" + rightMotorOutput);
  pastTime = currentTime;
  leftPastError = leftCurrentError;
  rightPastError = rightCurrentError;
  }
  
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
