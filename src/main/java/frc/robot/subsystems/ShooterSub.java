/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSub extends SubsystemBase {
  /**
   * Creates a new ShooterSub.
   */

  public static CANSparkMax motor1 = new CANSparkMax(1, MotorType.kBrushless);
  public static CANSparkMax motor2 = new CANSparkMax(2, MotorType.kBrushless);
  
  public static TalonSRX elevatorOuter = new TalonSRX(3);
  public static TalonSRX elevatorInner = new TalonSRX(4);

  public static DigitalInput optical = new DigitalInput(0);

  //public static CANPIDController motor1PID = motor1.getPIDController();
  //public static CANEncoder motor1Encoder = motor1.getEncoder();
  //public static CANPIDController motor2PID = motor2.getPIDController();
  //public static CANEncoder motor2Encoder = motor2.getEncoder();

  public ShooterSub() {

  }

  public static boolean seeBall() {
    return optical.get();
  }

  public static void shootPercent(double speed) {
    motor1.set(-speed);
    motor2.set(speed);
  }

  public static void moveElevator(double speed) {
    elevatorOuter.set(ControlMode.PercentOutput, speed);
    elevatorInner.set(ControlMode.PercentOutput, speed * 2);
  }

  public static void setSparkTest() {
    motor1.set(-0.5);
    motor2.set(0.5);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
