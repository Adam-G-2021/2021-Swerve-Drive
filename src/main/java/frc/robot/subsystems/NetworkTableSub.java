/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NetworkTableSub extends SubsystemBase {
  /**
   * Creates a new NetworkTableSub.
   */

  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable rioTable = inst.getTable("rioTable");

  public NetworkTableSub() {

  }

  public void initNetworkTable() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
