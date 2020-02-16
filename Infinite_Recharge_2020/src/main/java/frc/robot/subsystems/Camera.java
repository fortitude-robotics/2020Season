/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;
//import java.util.Vector;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.commands.Targeting;


/**
 * Add your docs here.
 */
public class Camera extends Subsystem 
{
  Double Px,Py,Parea,dist;
  int stacking = 0;
  double distavg;
  double Yavg;
  double yfinal;
  double distfinal;
  @Override
  public void initDefaultCommand() 
  {
    // Set the default command for a subsystem here.
    setDefaultCommand(new Targeting());
  }
  public void getPositon(Double x, Double y, Double area)
  {
    Px = x;
    Py = y;
    Parea = area;
    double h1 = 12;
    double h2 = 91;
    double a1 = Math.toRadians(25.0716073848);
    double a2 = Math.toRadians(Py);
    double hight = (h2-h1);
    double angle = Math.tan(a1+a2);
    dist = hight/angle;
    /*
    distavg = distavg + dist;
    //Yavg = Yavg + Py;
    if(stacking >= 1000)
    {
      distfinal = distavg/1000;
      yfinal = Yavg/1000;
      distavg = 0;
      Yavg = y;
      stacking = 0;
      System.out.println(" dist " + String.format("%.3f",distfinal));
      System.out.println(" Y: " + String.format("%.7f",Py));
    }
    stacking++;
    */

  }
  public Double getX()
  {
    return Px;
  }
  public double getY()
  {
    return Py;
  }
  public void print()
  {

      //System.out.print("X: " + String.format("%.3f",Px));
      //System.out.println(" Y: " + String.format("%.7f",Py));
      //System.out.println(" area: " + String.format("%.3f",Parea));
      //System.out.println(" dist " + String.format("%.3f",distfinal));

  }
}
