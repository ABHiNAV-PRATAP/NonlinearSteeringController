/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Drivetrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public SpeedControllerGroup
    leftMotors = new SpeedControllerGroup(RobotMap.frontLeftVictor, RobotMap.backLeftVictor),
    rightMotors = new SpeedControllerGroup(RobotMap.frontRightVictor, RobotMap.backRightVictor);

  public void drive(double l, double r) {
    leftMotors.set(l);
    rightMotors.set(r);
  }
  public void stop() {
    leftMotors.stopMotor();
    rightMotors.stopMotor();
  }
  
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
