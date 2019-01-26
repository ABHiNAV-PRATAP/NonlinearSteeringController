/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util.control;

/**
 * Class for robot pose. Tracks x, y, and heading.
 */
public class Odometry {
    private double
        x,
        y,
        theta;

    public Odometry(double x, double y, double theta) {
        this.x = x;
        this.y = y;
        this.theta = theta;
    }
    
    public void updateOdometry() {
        x += RobotMap.leftDriveEncoder.getRate() * RobotConstants.kDT * Math.cos(theta);
        y += RobotMap.leftDriveEncoder.getRate() * RobotConstants.kDT * Math.sin(theta);
        theta += RobotMap.gyro.getAngle();
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getHeading() {
        return theta;
    }

    public String toString() {
        return "X: " + x + ", Y: " + y + ", Heading: " + theta;
    }
}
