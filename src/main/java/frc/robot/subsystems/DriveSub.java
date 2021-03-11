/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import com.ctre.phoenix.motorcontrol.can.TalonSRX;
//import com.ctre.phoenix.motorcontrol.can.TalonSRXPIDSetConfiguration;
import frc.robot.RobotContainer;
import java.lang.Math;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class DriveSub extends SubsystemBase {
  /**
   * Creates a new DriveSub.
   */

  WPI_TalonSRX talonDriveBL = new WPI_TalonSRX(36);
  WPI_TalonSRX talonTurnBL = new WPI_TalonSRX(37);
  WPI_TalonSRX talonDriveBR = new WPI_TalonSRX(38);
  WPI_TalonSRX talonTurnBR = new WPI_TalonSRX(39);
  WPI_TalonSRX talonDriveFL = new WPI_TalonSRX(32);
  WPI_TalonSRX talonTurnFL = new WPI_TalonSRX(33);
  WPI_TalonSRX talonDriveFR = new WPI_TalonSRX(34);
  WPI_TalonSRX talonTurnFR = new WPI_TalonSRX(35);

  public static Gyro gyro = new ADXRS450_Gyro();

  public static boolean calibMode = true;
  public static boolean ToggleFieldCentric = true;
  public static boolean brakeMode = false;
  public static boolean autoMode = false;
  public static boolean streightMode = false;
  public static double streightAng = 0.0;

  public DriveSub() {
    talonTurnBR.config_kF(0, 0.1);
    talonTurnBR.config_kP(0, 0.07);
    talonTurnBR.config_kI(0, 0);
    talonTurnBR.config_kD(0, 0);
    talonTurnBR.setNeutralMode(NeutralMode.Coast);
  }

  public void resetSwerveAngle() {
    if (calibMode == true) {
      talonTurnBR.setSelectedSensorPosition(0);
      talonTurnFR.setSelectedSensorPosition(0);
      talonTurnBL.setSelectedSensorPosition(0);
      talonTurnFL.setSelectedSensorPosition(0);
    } else {
      /*
      talonTurnBR.set(ControlMode.Position, 0); test
      talonTurnFR.set(ControlMode.Position, 0);
      talonTurnBL.set(ControlMode.Position, 0);
      talonTurnFL.set(ControlMode.Position, 0);
      */
    }
  }
  public void zeroSwerveAngle() {
    talonDriveBL.set(ControlMode.Position, 0);
    talonDriveBR.set(ControlMode.Position, 0);
    talonDriveFL.set(ControlMode.Position, 0);
    talonDriveFR.set(ControlMode.Position, 0);
  }

  public double mod(double a, double b) {
    double c = a % b;
    if (c < 0)
      c += b;
    return c;
  }
  public int mod(int a, int b) {
    int c = a % b;
    if (c < 0)
      c += b;
    return c;
  }

  double[] getDestinationAngle(double dA, double cA, double mag) {
    double ang1 = getDestinationAngle(dA, cA);
    double ang2 = getDestinationAngle((dA + 180) % 360, cA);
    double ret[] = {ang1, mag};
    if (Math.abs(ang2 - cA) < Math.abs(ang1 - cA)) {
      ret[0] = ang2;
      ret[1] = -mag;
    } else {
      ret[0] = ang1;
    }
    return ret;
  }
  double getDestinationAngle(double dA, double cA) {
    int cA_numLoops = (int)Math.floor(cA / 360);  // Get number of full rotations
    dA = dA + (360 * cA_numLoops);  // Adjust desination angle to be closer to current angle
    double mA = mod((cA + 180), 360);
    if (mod((dA - mA), 360) < 180) {
      // Rotate Counterclockwise
      if (dA > cA) {
        dA -= 360;
      }
    } else {
      // Rotate Clockwise
      if (dA < cA) {
        dA += 360;
      }
    }
    return dA;
  }

  public double deadzone(double in, double dead) {
    if ((in < dead) && (in > -dead)) {
      in = 0;
    }
    return in;
  }

  public void resetGyro() {
    gyro.reset();
  }

  public double deadZone(double value, double tolerance) {
    // If value is within the tolerance range, set it to 0
    if ((value < tolerance) && (value > -tolerance)) {
      value = 0;
    }
    return value;
  }

  public void swerveDrive() {
    // Get Joystick Values
    double joyX = RobotContainer.stick.getRawAxis(0);
    joyX = deadZone(joyX, 0.06);
    double joyY = RobotContainer.stick.getRawAxis(1);
    joyY = deadZone(joyY, 0.06);
    double joyR = RobotContainer.stick.getRawAxis(4);
    joyR = deadZone(joyR, 0.06);
    SmartDashboard.putNumber("joyX", joyX);
    SmartDashboard.putNumber("joyY", joyY);
    SmartDashboard.putNumber("joyR", joyR);
    driveFromStickValues(joyX, joyY, joyR);
  }

  public void driveFromStickValues(double joyX, double joyY, double joyR) {
    //#region
    talonTurnBR.config_kF(0, SmartDashboard.getNumber("pidF", 0.00));
    talonTurnBR.config_kP(0, SmartDashboard.getNumber("pidP", 7.00));
    talonTurnBR.config_kI(0, SmartDashboard.getNumber("pidI", 0.0005));
    talonTurnBR.config_kD(0, SmartDashboard.getNumber("pidD", 0.01));
    talonTurnBR.configSelectedFeedbackSensor(FeedbackDevice.Analog);
    if (brakeMode == true) {
      talonTurnBR.setNeutralMode(NeutralMode.Brake);
    } else {
      talonTurnBR.setNeutralMode(NeutralMode.Coast);
    }

    talonTurnFR.config_kF(0, SmartDashboard.getNumber("pidF", 0.00));
    talonTurnFR.config_kP(0, SmartDashboard.getNumber("pidP", 7.00));
    talonTurnFR.config_kI(0, SmartDashboard.getNumber("pidI", 0.0005));
    talonTurnFR.config_kD(0, SmartDashboard.getNumber("pidD", 0.01));
    talonTurnFR.configSelectedFeedbackSensor(FeedbackDevice.Analog);
    if (brakeMode == true) {
      talonTurnFR.setNeutralMode(NeutralMode.Brake);
    } else {
      talonTurnFR.setNeutralMode(NeutralMode.Coast);
    }

    talonTurnBL.config_kF(0, SmartDashboard.getNumber("pidF", 0.00));
    talonTurnBL.config_kP(0, SmartDashboard.getNumber("pidP", 7.00));
    talonTurnBL.config_kI(0, SmartDashboard.getNumber("pidI", 0.0005));
    talonTurnBL.config_kD(0, SmartDashboard.getNumber("pidD", 0.01));
    talonTurnBL.configSelectedFeedbackSensor(FeedbackDevice.Analog);
    if (brakeMode == true) {
      talonTurnBL.setNeutralMode(NeutralMode.Brake);
    } else {
      talonTurnBL.setNeutralMode(NeutralMode.Coast);
    }

    talonTurnFL.config_kF(0, SmartDashboard.getNumber("pidF", 0.00));
    talonTurnFL.config_kP(0, SmartDashboard.getNumber("pidP", 7.00));
    talonTurnFL.config_kI(0, SmartDashboard.getNumber("pidI", 0.0005));
    talonTurnFL.config_kD(0, SmartDashboard.getNumber("pidD", 0.01));
    talonTurnFL.configSelectedFeedbackSensor(FeedbackDevice.Analog);
    if (brakeMode == true) {
      talonTurnFL.setNeutralMode(NeutralMode.Brake);
    } else {
      talonTurnFL.setNeutralMode(NeutralMode.Coast);
    }
    //SmartDashboard.putNumber("pidF", 1);
    //SmartDashboard.putNumber("pidP", 1);
    //SmartDashboard.putNumber("pidI", 1);
    //SmartDashboard.putNumber("pidD", 1);
    //#endregion

    
    double gyroAng = gyro.getAngle();
    SmartDashboard.putNumber("Gyro Angle", gyroAng);

    // Overide rotation amount if trying to keep streight
    if (streightMode == true) {
      joyR = (streightAng - gyroAng) / 45;
      if (joyR > 1)
        joyR = 1;
      if (joyR < -1)
        joyR = -1;
      SmartDashboard.putNumber("Gyro Correct", joyR);
    }

    if (ToggleFieldCentric == false) {
      gyroAng = 0;
    }

    // Get movement vectors (Joystick rotated arround gyro)
    double joyA = (Math.atan2(joyY, joyX) / Math.PI * 180) - gyroAng;
    double joyM = Math.sqrt((joyX * joyX) + (joyY * joyY));
    double moveX = (Math.cos(joyA * Math.PI / 180) * joyM);
    double moveY = (Math.sin(joyA * Math.PI / 180) * joyM);

    // Get rotation vectors
    double turnBLX = Math.cos(Math.toRadians(-45)) * -joyR;
    double turnBLY = -Math.sin(Math.toRadians(-45)) * -joyR;
    double turnBRX = Math.cos(Math.toRadians(-45)) * -joyR;
    double turnBRY = Math.sin(Math.toRadians(-45)) * -joyR;
    double turnFRX = -Math.cos(Math.toRadians(-45)) * -joyR;
    double turnFRY = Math.sin(Math.toRadians(-45)) * -joyR;
    double turnFLX = -Math.cos(Math.toRadians(-45)) * -joyR;
    double turnFLY = -Math.sin(Math.toRadians(-45)) * -joyR;

    // Combine movement and rotation vectors
    double vecBLX = moveX + turnBLX;
    double vecBLY = moveY + turnBLY;
    double vecBRX = moveX + turnBRX;
    double vecBRY = moveY + turnBRY;
    double vecFRX = moveX + turnFRX;
    double vecFRY = moveY + turnFRY;
    double vecFLX = moveX + turnFLX;
    double vecFLY = moveY + turnFLY;

    // Convert vectors to magnitude and direction
    double vecBLA = mod(((Math.atan2(vecBLY, vecBLX) / Math.PI) * -180 + 270), 360);
    double vecBLM = Math.sqrt((vecBLX * vecBLX) + (vecBLY * vecBLY));
    double vecBRA = mod(((Math.atan2(vecBRY, vecBRX) / Math.PI) * -180 + 270), 360);
    double vecBRM = Math.sqrt((vecBRX * vecBRX) + (vecBRY * vecBRY));
    double vecFRA = mod(((Math.atan2(vecFRY, vecFRX) / Math.PI) * -180 + 270), 360);
    double vecFRM = Math.sqrt((vecFRX * vecFRX) + (vecFRY * vecFRY));
    double vecFLA = mod(((Math.atan2(vecFLY, vecFLX) / Math.PI) * -180 + 270), 360);
    double vecFLM = Math.sqrt((vecFLX * vecFLX) + (vecFLY * vecFLY));

    //vecFLA = 200;

    if (calibMode == false) {
      // Get current module rotations
      double vecFLA_current = talonTurnFL.getSelectedSensorPosition() / 2.8444;
      double vecFRA_current = talonTurnFR.getSelectedSensorPosition() / 2.8444;
      double vecBRA_current = talonTurnBR.getSelectedSensorPosition() / 2.8444;
      double vecBLA_current = talonTurnBL.getSelectedSensorPosition() / 2.8444;
      // Get destination rotation; find optimal angle to travel the least distance
      double[] vecFL = getDestinationAngle(vecFLA, vecFLA_current, vecFLM); vecFLA = vecFL[0] * 2.8444; vecFLM = vecFL[1];
      double[] vecFR = getDestinationAngle(vecFRA, vecFRA_current, vecFRM); vecFRA = vecFR[0] * 2.8444; vecFRM = vecFR[1];
      double[] vecBL = getDestinationAngle(vecBLA, vecBLA_current, vecBLM); vecBLA = vecBL[0] * 2.8444; vecBLM = vecBL[1];
      double[] vecBR = getDestinationAngle(vecBRA, vecBRA_current, vecBRM); vecBRA = vecBR[0] * 2.8444; vecBRM = vecBR[1];
      // Output angle to smartdashboard
      SmartDashboard.putNumber("Swerve Output Angle FL", vecFLA);
      SmartDashboard.putNumber("Swerve Output Angle FR", vecFRA);
      SmartDashboard.putNumber("Swerve Output Angle BL", vecBLA);
      SmartDashboard.putNumber("Swerve Output Angle BR", vecBRA);

      //vecFRM = 0.2;
      //vecFRA = 0;
      //vecFLA = 0;
      //vecBRA = 0;
      //vecBLA = 0;

      // Output angle & magnitude to swerve drive
      if (Math.abs(vecBLM) >= 0.1) talonTurnBL.set(ControlMode.Position, vecBLA);
      talonDriveBL.set(ControlMode.PercentOutput, vecBLM / 2);
      if (Math.abs(vecBRM) >= 0.1) talonTurnBR.set(ControlMode.Position, vecBRA);
      talonDriveBR.set(ControlMode.PercentOutput, (vecBRM / 2) + 0.05);
      if (Math.abs(vecFRM) >= 0.1) talonTurnFR.set(ControlMode.Position, vecFRA);
      talonDriveFR.set(ControlMode.PercentOutput, vecFRM / 2);
      if (Math.abs(vecFLM) >= 0.1) talonTurnFL.set(ControlMode.Position, vecFLA);
      talonDriveFL.set(ControlMode.PercentOutput, (vecFLM / 2) + 0.05);
    } else {

      talonTurnBR.set(ControlMode.PercentOutput, joyR / 2);
      talonTurnFR.set(ControlMode.PercentOutput, joyR / 2);
      talonTurnBL.set(ControlMode.PercentOutput, joyR / 2);
      talonTurnFL.set(ControlMode.PercentOutput, joyR / 2);//*/
      //talonDriveBR.set(ControlMode.PercentOutput, -0.15);
      //talonDriveBL.set(ControlMode.PercentOutput, -0.15);
      //talonDriveFR.set(ControlMode.PercentOutput, -0.15);
      //talonDriveFL.set(ControlMode.PercentOutput, -0.15);
    }

    SmartDashboard.putBoolean("Field Centric?", ToggleFieldCentric);
    SmartDashboard.putBoolean("Calibration Mode", calibMode);


    //talonTurnBR.set(ControlMode.Position, joyR * 600);
    //talonTurnFR.set(ControlMode.Position, joyR * 600);
    //talonTurnBL.set(ControlMode.Position, joyR * 600);
    //talonTurnFL.set(ControlMode.Position, joyR * 600);

    //talonTurnBR.set(ControlMode.PercentOutput, joyR / 4);
    //talonTurnBL.set(ControlMode.PercentOutput, joyR / 4);
    //talonTurnFR.set(ControlMode.PercentOutput, joyR / 4);
    //talonTurnFL.set(ControlMode.PercentOutput, joyR / 4);
    //talonTurnBR.setSelectedSensorPosition(0);
    //talonTurnFR.setSelectedSensorPosition(0);
    //talonTurnBL.setSelectedSensorPosition(0);
    //talonTurnFL.setSelectedSensorPosition(0);
    //talonDriveBR.set(joyY / 8);
    //talonDriveBL.set(joyY / 8);
    //talonDriveFR.set(joyY / 8);
    //talonDriveFL.set(joyY / 8);
    

    //talonTurnFL.set(ControlMode.Position, vecFLA);
    SmartDashboard.putNumber("BR Swerve Angle", talonTurnBR.getSelectedSensorPosition());
    SmartDashboard.putNumber("FR Swerve Angle", talonTurnFR.getSelectedSensorPosition());
    SmartDashboard.putNumber("BL Swerve Angle", talonTurnBL.getSelectedSensorPosition());
    SmartDashboard.putNumber("FL Swerve Angle", talonTurnFL.getSelectedSensorPosition());
    //talonDriveFL.set(ControlMode.PercentOutput, deadzone(vecFLM, 0.3));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
