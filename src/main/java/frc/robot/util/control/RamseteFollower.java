/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util.control;

import jaci.pathfinder.Trajectory;

/**
 * Add your docs here.
 */
public class RamseteFollower {
    private static final double
        b = 0,
        z = 0,
        wheelBase = 0;
    
    private double
        x_d = 0,
        y_d = 0,
        theta_d = 0,
        v_d = 0,
        w_d = 0;

    private int currentSegment;
    private Trajectory traj;
    private Odometry o;

    public RamseteFollower(Trajectory traj, Odometry o) {
        this.traj = traj;
        this.o = o;
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
        } else return 0;
    }
    
    public double calculateConstant() {
        return 2*z*Math.sqrt(Math.pow(getDesiredAngular(), 2)+b*Math.pow(getDesiredLinear(), 2));
    }
    
    public void updateGoal() {
        x_d = getDesiredX();
        y_d = getDesiredY();
        theta_d = getDesiredHeading();
        v_d = getDesiredLinear();
        w_d = getDesiredAngular();
        k = calculateConstant();
    }
    
    public double calculateLinearVelocity() {
        double v = v_d * Math.cos(theta_d - o.getHeading()) + k*(Math.cos(o.getHeading()) * (x_d-o.getX()) + Math.sin(o.getHeading()) * (y_d-o.getY()));
        return v;
    }
    
    public double calculateAngularVelocity() {
        double w = w_d + b * v_d * sinc(theta_d - o.getHeading()) * ((y_d-o.getY())*cos(o.getHeading())-(x_d-o.getX())*sin(o.getHeading())) + k * (theta_d-o.getHeading());
        return w;
    }        
}
