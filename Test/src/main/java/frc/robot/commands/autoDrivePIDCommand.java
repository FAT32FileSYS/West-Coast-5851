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
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
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
  
  drive drive = new drive();


  final double driveTick2Feet = 1.0 / 128 * 6 * Math.PI / 12;
  double currentTime = Timer.getFPGATimestamp();
  double dt = currentTime - pastTime;
  

  //distance units in feet
  double leftSetPoint = 5; 
  double rightSetPoint = 5;
  
  leftCurrentLocation = drive.leftSideEncoder * driveTick2Feet;
  rightCurrentLocation = drive.rightSideEncoder * driveTick2Feet;

  double leftCurrentError = leftSetPoint - leftCurrentLocation;
  double rightCurrentError = rightSetPoint - rightCurrentLocation;
  
  double leftConstant = AutoConstants.KP * leftCurrentError;
  double leftDerivitive = AutoConstants.KD * (leftCurrentError - leftPastError) / dt;
  double leftcurrentIntegral = AutoConstants.KI * ((leftCurrentError * currentTime) - (leftCurrentError * pastTime));

  double rightConstant = AutoConstants.KP * rightCurrentError;
  double rightDerivitive = AutoConstants.KD * (rightCurrentError - rightPastError) / dt;
  double rightcurrentIntegral = AutoConstants.KI * ((rightCurrentError * currentTime) - (rightCurrentError * pastTime));

  leftTotalIntegral =+ leftcurrentIntegral;
  rightTotalIntegral =+ rightcurrentIntegral;

  leftMotorOutput = leftConstant + leftDerivitive + leftTotalIntegral;
  rightMotorOutput = rightConstant + rightDerivitive + rightTotalIntegral;

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
