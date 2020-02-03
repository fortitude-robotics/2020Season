/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.RobotMap;

import java.util.Vector;
import edu.wpi.first.wpilibj.command.Command;

public class Drive extends Command {
  public Drive() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.drivetrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() 
  {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() 
  {
    Vector<Double> axis = Robot.m_oi.GetControllerRawAxis();
    double RFactor;
    double LFactor;
    double axisX = (double) axis.elementAt(0);
    double axisY = (double) axis.elementAt(1);

    double leftPWR = 0;
    double rightPWR = 0;

    if(axisY <= 0.0)
    {
      if(axisX > 0.0)
      {
        leftPWR = Math.copySign(Math.sqrt(Math.pow(axisY, 2) + Math.pow(axisX, 2)),axisX);
        rightPWR = Math.copySign(Math.sqrt(Math.abs(Math.pow(axisY, 2) - Math.pow(axisX, 2))),(Math.abs(axisY)-axisX));
      }
      else
      {
        leftPWR = Math.copySign(Math.sqrt(Math.pow(axisY, 2) + Math.pow(axisX, 2)),Math.abs(axisY)-Math.abs(axisX));
        rightPWR = Math.sqrt(Math.pow(axisY, 2) + Math.pow(Math.abs(axisX),2));
      }
    }
    else
    {
      if(axisX >= 0.0)
      {
        leftPWR = Math.copySign(Math.sqrt(Math.pow(Math.abs(axisY),2) + Math.pow(axisX, 2)),axisX-axisY);
        rightPWR = Math.copySign(Math.sqrt(Math.pow(Math.abs(axisY),2) + Math.pow(axisX, 2)),-1);
      }
      else
      {
        leftPWR = Math.copySign(Math.sqrt(Math.pow(Math.abs(axisY),2) + Math.pow(axisX, 2)),axisX);
        rightPWR = Math.copySign(Math.sqrt(Math.pow(Math.abs(axisY),2) - Math.pow(axisX, 2)),(Math.abs(axisY)-axisX));
      }
    }
  
    LFactor = leftPWR * RobotMap.SCALEFACTOR; 
    RFactor = rightPWR  * RobotMap.SCALEFACTOR;

    Robot.drivetrain.directpwrfeed(LFactor,RFactor);
  }

  // Make this return true when this Command no longer needs to run execute()
  //Cian was here
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
