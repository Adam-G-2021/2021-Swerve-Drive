/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoTestCom extends SequentialCommandGroup {
  /**
   * Creates a new AutoTestCom.
   */
  
  public AutoTestCom() {
    super(new DriveAuto(0.0, 0.0, 0.5, 5), new DriveAuto(0.0, 0.0, 0.0, 0.5), new DriveAuto(0.0, 0.0, -0.5, 5), new DriveAuto(0.0, 0.0, 0.0, 1));
   //first # left joystick X-axis
   //2nd # left joystick y axis
   //3rd # right joystick x axis
   //4th # time duration of comand
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());super();
  
  
  
  }
}
