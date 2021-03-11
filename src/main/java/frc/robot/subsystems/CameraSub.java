/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSink;
//import edu.wpi.cscore.MjpegServer;
//import org.opencv.core.Mat;
//import org.opencv.imgproc.Imgproc;
//import edu.wpi.cscore.CvSink;
//import edu.wpi.cscore.CvSource;
//import frc.robot.commands.CameraCom;
//import org.opencv.videoio.VideoCapture;
//import edu.wpi.cscore.Video;

/**
 * Add your docs here.
 */
public class CameraSub extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  static UsbCamera lesserCamera =  CameraServer.getInstance().startAutomaticCapture();
  static VideoSink serverOne = CameraServer.getInstance().getServer("serve_USB Camera 0");
  

  //static CameraServer mainCam = CameraServer.getInstance().startAutomaticCapture();
  //static VideoSink serverOne = CameraServer.getInstance().getServer("serve_USB Camera 0");


  public static void initCamera() {
		serverOne.setSource(lesserCamera);
  }

}
