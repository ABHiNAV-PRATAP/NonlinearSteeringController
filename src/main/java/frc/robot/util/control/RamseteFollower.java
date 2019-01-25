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

    private int currentSegment;
    private Trajectory traj;
    private Odometry odometry;

    public RamseteFollower(Trajectory traj) {
        this.traj = traj;
        currentSegment = 0;
    }

    public double getDesiredAngular() {
        if (currentSegment < traj.length()-1) {
            double lastTheta = traj.get(currentSegment).heading;
            double nextTheta = traj.get(currentSegment + 1).heading;
            return (nextTheta - lastTheta) / traj.get(currentSegment).dt;
        } else return  0;
    }
}
