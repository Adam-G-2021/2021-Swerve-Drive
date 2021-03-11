/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class DriveAuto extends CommandBase {
  /**
   * Creates a new DriveAuto.
   */

  double joyX;
  double joyY;
  double joyR;
  double timeWait;

  Timer timer = new Timer();

  public DriveAuto(double x, double y, double r, double t) {
    // Use addRequirements() here to declare subsystem dependencies.
    joyX = x;
    joyY = y;
    joyR = r;
    timeWait = t;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
    timer.reset();
    RobotContainer.driveSub.calibMode = false;
    RobotContainer.driveSub.autoMode = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.driveSub.driveFromStickValues(joyX, joyY, joyR);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double tim = timer.get();
    if (tim >= timeWait) {
      timer.stop();
      return true;
    } else {
      return false;
    }
  }
}
