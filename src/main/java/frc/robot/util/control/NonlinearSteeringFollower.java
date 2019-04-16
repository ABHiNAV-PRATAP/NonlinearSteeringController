/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util.control;

import frc.robot.Constants;
import jaci.pathfinder.Trajectory;

/**
 * Add your docs here.
 */
public class NonlinearSteeringFollower {    
    private double
        x_d = 0,
        y_d = 0,
        theta_d = 0,
        v_d = 0,
        w_d = 0,
        k;

    private int currentSegment;
    private Trajectory traj;
    public Odometry o = new Odometry(0, 0, 0);

    public NonlinearSteeringFollower(Trajectory traj) {
        this.traj = traj;
        currentSegment = 0;
    }
    
    public double getDesiredX() {
        return traj.get(currentSegment).x;
    }
    
    public double getDesiredY() {
        return traj.get(currentSegment).y;
    }
    
    public double getDesiredHeading() {
        return traj.get(currentSegment).heading;
    }
    
    public double getDesiredLinear() {
        return traj.get(currentSegment).velocity;
    }

    public double getDesiredAngular() {
        if (currentSegment < traj.length()-1) {
            double currentTheta = traj.get(currentSegment).heading;
            double nextTheta = traj.get(currentSegment + 1).heading;
            return (nextTheta - currentTheta) / traj.get(currentSegment).dt;
        } else {
            return 0;
        }
    }
    
    public double calculateConstant() {
        return 2 * Constants.kZ * Math.sqrt(Math.pow(w_d, 2) + Constants.kB * Math.pow(v_d, 2));
    }
    
    public void updateGoal() {
        x_d = getDesiredX();
        y_d = getDesiredY();
        theta_d = getDesiredHeading();
        v_d = getDesiredLinear();
        w_d = getDesiredAngular();
        k = calculateConstant();
        currentSegment++;
    }
    
    public double sinc(double theta) {
        if(Math.abs(theta) < 0.001) return 1; // Limit to infinity of sin(x)/x
        else return (Math.sin(theta) / theta);
    }
    
    public double calculateLinearVelocity() {
        double v = v_d * Math.cos(theta_d - o.getHeading()) + k * ((x_d - o.getX()) * Math.cos(o.getHeading()) + (y_d - o.getY()) * Math.sin(o.getHeading()));
        return v;
    }
    
    public double calculateAngularVelocity() {
        double w = w_d + Constants.kB * v_d * sinc(theta_d - o.getHeading()) * ((y_d - o.getY()) * Math.cos(o.getHeading()) - (x_d - o.getX()) * Math.sin(o.getHeading())) + k * (theta_d - o.getHeading());
        return w;
    }

    public boolean isFinished() {
        return currentSegment > traj.length();
    }
}
