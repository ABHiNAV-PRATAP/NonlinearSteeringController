/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C.Port;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  public static WPI_VictorSPX
    frontLeftVictor = new WPI_VictorSPX(Constants.kFrontLeftVictor),
    backLeftVictor = new WPI_VictorSPX(Constants.kBackLeftVictor),
    frontRightVictor = new WPI_VictorSPX(Constants.kFrontRightVictor),
    backRightVictor = new WPI_VictorSPX(Constants.kFrontRightVictor);
  
  public static Encoder
    leftDriveEncoder = new Encoder(Constants.kLeftDriveEncoderChannelA, Constants.kLeftDriveEncoderChannelB),
    rightDriveEncoder = new Encoder(Constants.kRightDriveEncoderChannelB, Constants.kRightDriveEncoderChannelB);
  
  public static AHRS
    gyro = new AHRS(Port.kMXP);
}
