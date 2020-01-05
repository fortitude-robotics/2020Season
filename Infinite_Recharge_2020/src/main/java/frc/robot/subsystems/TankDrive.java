/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.
//Cian was here*/
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.Drive;
import edu.wpi.first.wpilibj.drive.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Vector;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


/**
 * Add your docs here.
 */
public class TankDrive extends Subsystem 
{
  private CANSparkMax leftFrontMotor = new CANSparkMax(RobotMap.FRONT_LEFT_MOTOR, MotorType.kBrushless);
  private CANSparkMax rightFrontMotor = new CANSparkMax(RobotMap.FRONT_RIGHT_MOTOR, MotorType.kBrushless);
  private CANSparkMax leftRearMotor = new CANSparkMax(RobotMap.REAR_LEFT_MOTOR, MotorType.kBrushless);
  private CANSparkMax rightRearMotor = new CANSparkMax(RobotMap.REAR_RIGHT_MOTOR, MotorType.kBrushless);
  private DifferentialDrive Drivetrain = new DifferentialDrive(leftFrontMotor,rightFrontMotor);

  @Override
  public void initDefaultCommand() 
  {
    // Set the default command for a subsystem here.
    setDefaultCommand(new Drive());
  }
  public void setdrivevector(Vector<Double> axis)
  {
    double axisX = (double) axis.elementAt(0);
    double axisY = (double) axis.elementAt(1);
    double leftPWR = 0;
    double rightPWR = 0;
    //upper left
    if(axisX <= 0 && axisY >= 0)
    {
      leftPWR = axisY + axisX;
      rightPWR = axisY + Math.abs(axisX);
    }
    //lower left
    if(axisX < 0 && axisY < 0)
    {
      leftPWR = axisY + Math.abs(axisX);
      rightPWR = axisY + axisX;
    }
    //lower right
    if(axisX >= 0 && axisY <= 0)
    {
      leftPWR = axisY + axisX;
      rightPWR = axisY - axisX;
    }
    //upper right
    if(axisX > 0 && axisY > 0)
    {
      leftPWR = axisY + axisX;
      rightPWR = axisY - axisX;
    }

    double LFactor = leftPWR * RobotMap.SCALEFACTOR;
    double RFactor = rightPWR * RobotMap.SCALEFACTOR;

    Drivetrain.tankDrive(LFactor, RFactor);
  }
  public void setfollow()
  {
    leftRearMotor.follow(leftFrontMotor);
    rightRearMotor.follow(rightFrontMotor);
  }
}
