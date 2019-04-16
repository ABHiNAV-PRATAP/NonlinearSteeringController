/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.Constants;
import jaci.pathfinder.Trajectory;
import frc.robot.util.control.*;

public class DrivePath extends Command {
  /**
   * General command to drive a trajectory.
   */
  private Trajectory trajectory;

  private NonlinearSteeringFollower nonlinearSteeringFollower;

  public DrivePath(Trajectory trajectory) {
    this.trajectory = trajectory;
    requires(Robot.drive);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    nonlinearSteeringFollower = new NonlinearSteeringFollower(trajectory);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // Update odometry
    nonlinearSteeringFollower.o.updateOdometry();
    // Calculate linear and angular velocities
    double v = nonlinearSteeringFollower.calculateLinearVelocity();
    double w = nonlinearSteeringFollower.calculateAngularVelocity();
    // Set linear and angular velocities to drivetrain
    double l = (-1 * (w * Constants.kWheelBase) + (2 * v)) / 2;
    double r = ((w * Constants.kWheelBase) + (2 * v)) / 2;
    Robot.drive.drive(l, r);
    nonlinearSteeringFollower.updateGoal();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return nonlinearSteeringFollower.isFinished();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.drive.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
