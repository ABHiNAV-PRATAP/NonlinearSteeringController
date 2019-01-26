/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.RobotMap;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import frc.robot.util.control.*;

public class DrivePath extends Command {
  /**
   * General command to drive a trajectory.
   */
  private Trajectory leftTrajectory, rightTrajectory;

  private RamseteFollower ramseteFollower;

  public DrivePath(Trajectory l, Trajectory r) {
    System.out.println("AutoDrive started");
    
    leftTrajectory = l;
    rightTrajectory = r;

    leftFollower = new EncoderFollower(leftTrajectory);
    rightFollower = new EncoderFollower(rightTrajectory);

    leftFollower.configureEncoder(RobotMap.leftDriveEncoder.get(), RobotConstants.kDriveEncoderTicksPerRev, RobotConstants.kWheelDiameter);
    rightFollower.configureEncoder(RobotMap.rightDriveEncoder.get(), RobotConstants.kDriveEncoderTicksPerRev, RobotConstants.kWheelDiameter);
    leftFollower.configurePIDVA(RobotConstants.kP, RobotConstants.kI, RobotConstants.kD, 1 / RobotConstants.kVMax, RobotConstants.kA);
    rightFollower.configurePIDVA(RobotConstants.kP, RobotConstants.kI, RobotConstants.kD, 1 / RobotConstants.kVMax, RobotConstants.kA);

    requires(Robot.drivetrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double leftSpeed = leftFollower.calculate(-RobotMap.leftDriveEncoder.get());
    double rightSpeed = rightFollower.calculate(RobotMap.rightDriveEncoder.get());
    
    double gyroHeading = -RobotMap.gyro.getAngle();
    double desiredHeading = Pathfinder.r2d(leftFollower.getHeading());

    double angleDifference = Pathfinder.boundHalfDegrees(desiredHeading - gyroHeading);
    double turn = 0.8 * (-1.0 / 80.0) * angleDifference;

    // System.out.println("Turn value: " + turn);
    // System.out.println("Gyro value: " + RobotMap.gyro.getAngle());
    // System.out.println("Left: " + leftSpeed);
    // System.out.println("Right: " + rightSpeed);
    // System.out.println("Left Encoder: " + (-RobotMap.leftDriveEncoder.get()));
    // System.out.println("Right Encoder: " + RobotMap.rightDriveEncoder.get());
    Robot.drivetrain.drive(leftSpeed+turn, rightSpeed-turn);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return leftFollower.isFinished() && rightFollower.isFinished();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.drivetrain.stop();
    System.out.println("AutoDrive finished");
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
