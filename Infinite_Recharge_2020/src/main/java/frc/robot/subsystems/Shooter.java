/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.*;
import com.playingwithfusion.TimeOfFlight;

/**
 * FN-1. normal mode : sensor P0 turn on all till sensor P1 , then turn off upper conveyer. then at
 * sensor P2 all conveyer off and reverse infeed for 3 seconds.
 * FN-2. Firing mode : spins up shooter to rpm for distance , then will run all conveyers feeding in balls once solenoid is out of the way
 * FN-3. Halt mode: instantly stops all conveyers and runs infeed in revers for 3 seconds
 */
public class Shooter extends Subsystem 
{ 
  //launcher
  private final TalonFX shooterUpperA = new TalonFX(RobotMap.SHOOTER_UPPER_A);
  private final TalonFX shooterUpperB = new TalonFX(RobotMap.SHOOTER_UPPER_B);
  private final TalonFX shooterLowerA = new TalonFX(RobotMap.SHOOTER_LOWER_A);
  private final TalonFX shooterLowerB = new TalonFX(RobotMap.SHOOTER_LOWER_B);
  //TalonFX config
  TalonFXConfiguration upper_cfg = new TalonFXConfiguration();
  TalonFXConfiguration lower_cfg = new TalonFXConfiguration();
  //conveyer
  private final TalonSRX lowerConv = new TalonSRX(RobotMap.LOWERCONV);
  private final TalonSRX uppperConv = new TalonSRX(RobotMap.UPPERCONV);
  private final TalonSRX mainConv = new TalonSRX(RobotMap.MAINCONV);
  private final TalonSRX infeed = new TalonSRX(RobotMap.INFEED); 
  //infeed pneumatics
  private final Solenoid intake_lower = new Solenoid(RobotMap.INTAKE_LOWER);
  private final TimeOfFlight tofS = new TimeOfFlight(0);
  //testing
  int n = 0;


  public void setfollow()
  {
    //TOFS
    //follow controll
    shooterUpperB.follow(shooterUpperA);
    shooterLowerB.follow(shooterLowerA);
    //ramp control
    shooterUpperA.configOpenloopRamp(1);
    shooterUpperB.configOpenloopRamp(1);
    shooterLowerA.configOpenloopRamp(1);
    shooterLowerB.configOpenloopRamp(1);

    //voltage comp
    /*
    shooterUpperA.enableVoltageCompensation(true);
    shooterLowerA.enableVoltageCompensation(true);
    shooterUpperB.enableVoltageCompensation(true);
    shooterLowerB.enableVoltageCompensation(true);
    
    shooterUpperA.configVoltageCompSaturation(11);
    shooterLowerA.configVoltageCompSaturation(11);
    shooterUpperB.configVoltageCompSaturation(11);
    shooterLowerB.configVoltageCompSaturation(11);
    */

    //pid pre-settings
    //shooterUpperA.configClosedLoopPeriod(0, 1, RobotMap.timeout);
    //shooterLowerA.configClosedLoopPeriod(0, 1, RobotMap.timeout);
    upper_cfg.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
    lower_cfg.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
    //upper
    upper_cfg.slot0.kF = RobotMap.kF;
    upper_cfg.slot0.kP = RobotMap.kP;
    upper_cfg.slot0.kI = RobotMap.kI;
    upper_cfg.slot0.kD = RobotMap.kD;
    upper_cfg.slot0.integralZone = RobotMap.kIzone;
    upper_cfg.slot0.closedLoopPeakOutput = RobotMap.kPeakOutput;
    upper_cfg.neutralDeadband = RobotMap.Deadband;
    //lower
    lower_cfg.slot0.kF = RobotMap.kF;
    lower_cfg.slot0.kP = RobotMap.kP;
    lower_cfg.slot0.kI = RobotMap.kI;
    lower_cfg.slot0.kD = RobotMap.kD;
    lower_cfg.slot0.integralZone = RobotMap.kIzone;
    lower_cfg.slot0.closedLoopPeakOutput = RobotMap.kPeakOutput;
    lower_cfg.neutralDeadband = RobotMap.Deadband;
    //apply
    //shooterUpperA.configAllSettings(upper_cfg);
    //shooterLowerA.configAllSettings(lower_cfg);
  }
  public double getSpeed()
  {
    return shooterUpperA.getMotorOutputPercent();
  }
  public double getconv()
  {
    return lowerConv.getMotorOutputPercent();
  }
  //set functions
  public void SetShooterPower(double pwr)
  {
    shooterUpperA.set(TalonFXControlMode.PercentOutput, pwr);
    shooterLowerA.set(TalonFXControlMode.PercentOutput, pwr/2);
    
  }
  //solinoid control
  public void intake_drop(boolean val)
  {
    intake_lower.set(val);
  }
  public boolean getsol()
  {
     return intake_lower.get();
  }
  //set for conv
  public void SetIntakePower(double pwr)
  {
    infeed.set(TalonSRXControlMode.PercentOutput,pwr);
  }
  public void SetMainPower(double pwr)
  {
    mainConv.set(TalonSRXControlMode.PercentOutput,pwr);
  }
  public void SetUpperPower(double pwr)
  {
    uppperConv.set(TalonSRXControlMode.PercentOutput,pwr);
  }
  public void SetLowerPower(double pwr)
  {
    lowerConv.set(TalonSRXControlMode.PercentOutput,pwr);
  }
  //time of flight sesor range output - base is ~42 in its current positon
  //anything higher then 50 means a ball is there. use this to autostop the conveyer
  //currently it set to push to hold and wont autostop
  public double GetTOFdistance()
  {
   SmartDashboard.putNumber("TOFS", tofS.getRange());
    return tofS.getRange();
  }
  @Override
  public void initDefaultCommand() 
  {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  //for debugging ======================================================================
  public void print()
  {
    boolean infeed_B = false;
    boolean convLower_B = false;
    boolean convUpper_B = false;
    boolean convMain_B = false;
    if(infeed.getMotorOutputPercent() != 0)
    {
      infeed_B = true;
    }
    if(uppperConv.getMotorOutputPercent() != 0)
    {
      convUpper_B = true;
    }
    if(lowerConv.getMotorOutputPercent() != 0)
    {
      convLower_B = true;
    }
    if(mainConv.getMotorOutputPercent() != 0)
    {
      convMain_B = true;
    }
    int rpm_upper =  (-shooterUpperA.getSelectedSensorVelocity() * 10)/RobotMap.FALCON_CPR;
    int rpm_lower =  (-shooterLowerA.getSelectedSensorVelocity() * 10)/RobotMap.FALCON_CPR;
    rpm_upper = rpm_upper * 60;
    rpm_lower = rpm_lower * 60;
    SmartDashboard.putNumber("RPM Upper", rpm_upper);
    SmartDashboard.putNumber("RPM lower", rpm_lower);
    SmartDashboard.putNumber("VoltageU", -shooterUpperA.getMotorOutputVoltage());
    SmartDashboard.putNumber("VoltageL", -shooterLowerA.getMotorOutputVoltage());
    SmartDashboard.putNumber("PercentU", -shooterUpperA.getMotorOutputPercent());
    SmartDashboard.putNumber("PercentL", -shooterLowerA.getMotorOutputPercent());
    SmartDashboard.putBoolean("Intake", infeed_B);
    SmartDashboard.putBoolean("ConvUpper", convUpper_B);
    SmartDashboard.putBoolean("ConvLower", convLower_B);
    SmartDashboard.putBoolean("ConvMain", convMain_B);
  }
}
