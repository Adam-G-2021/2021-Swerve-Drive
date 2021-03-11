/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.AutoTestCom;
import frc.robot.commands.DriveCom;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.MoveToZeroAngleCom;
import frc.robot.commands.ResetGyroGyro;
import frc.robot.commands.ResetSwerveAngleCom;
import frc.robot.commands.StreightModeCom;
import frc.robot.commands.ToggleDebugModeCom;
import frc.robot.commands.ToggleFieldCentric;
import frc.robot.commands.autoMoveAround;
import frc.robot.commands.brakeMode;
import frc.robot.subsystems.DriveSub;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.ShooterSub;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  public static final DriveSub driveSub = new DriveSub();
  public static final ShooterSub shooterSub = new ShooterSub();

  private final autoMoveAround m_autoMove = new autoMoveAround();
  private final AutoTestCom m_autoCommand = new AutoTestCom();
  public static final DriveCom driveCom = new DriveCom();

  public static SendableChooser autoMode = new SendableChooser<Command>();

  public static Joystick stick = new Joystick(0);
  public static JoystickButton buttonA = new JoystickButton(stick, 1);
  public static JoystickButton buttonB = new JoystickButton(stick, 2);
  public static JoystickButton buttonL = new JoystickButton(stick, 5);
  public static JoystickButton buttonY = new JoystickButton(stick, 4);
  public static JoystickButton buttonX = new JoystickButton(stick, 3);
  public static JoystickButton buttonR = new JoystickButton(stick, 6);
  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    buttonA.whenPressed(new ResetSwerveAngleCom());
    buttonB.whileHeld(new StreightModeCom());
    buttonL.whenPressed(new ToggleDebugModeCom());
    buttonY.whenPressed(new ToggleFieldCentric());
    buttonX.whenPressed(new ResetGyroGyro());
    buttonR.whileHeld(new brakeMode());
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    autoMode.addOption("spin", m_autoCommand);  // Option spin is connected to the spin command
    autoMode.addOption("Move Around", m_autoMove);  // Option Move Around is connected to the move command
    SmartDashboard.putData(autoMode);
    return (Command)autoMode.getSelected(); // (Command) will make the output of "autoMode.getScelected" into a Command data type. "autoMode.getScelected" will return the command to run
  }
}
