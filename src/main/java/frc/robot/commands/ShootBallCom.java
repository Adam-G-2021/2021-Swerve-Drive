/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterSub;

public class ShootBallCom extends CommandBase {
  /**
   * Creates a new ShootBallCom.
   */
  double speed;
  public ShootBallCom(double sped) {
    // Use addRequirements() here to declare subsystem dependencies.
    speed = sped;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.stick2.setRumble(GenericHID.RumbleType.kLeftRumble, 0.5);
    RobotContainer.stick2.setRumble(GenericHID.RumbleType.kRightRumble, 0.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ShooterSub.shootPercent(speed - (RobotContainer.stick2.getRawAxis(2)/100) + (RobotContainer.stick2.getRawAxis(3)/100));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ShooterSub.shootPercent(0.0);
    RobotContainer.stick.setRumble(GenericHID.RumbleType.kLeftRumble, 0.0);
    RobotContainer.stick.setRumble(GenericHID.RumbleType.kRightRumble, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
