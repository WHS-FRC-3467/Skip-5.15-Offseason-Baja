// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.clawrollerlasercan;

import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import au.grapplerobotics.interfaces.LaserCanInterface.RegionOfInterest;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;
import frc.lib.io.distancesensor.DistanceSensorIO;
import frc.lib.io.distancesensor.DistanceSensorIOLaserCAN;
import frc.lib.io.distancesensor.DistanceSensorIOSim;
import frc.robot.Ports;

/** Add your docs here. */
public class ClawRollerLaserCANConstants {

    public final static String NAME = "ClawRoller LaserCAN";
    private final static RangingMode RANGING_MODE = RangingMode.SHORT;
    private final static RegionOfInterest ROI = new RegionOfInterest(8, 8, 16, 4);
    private final static TimingBudget TIMING_BUDGET = TimingBudget.TIMING_BUDGET_20MS;

    public static DistanceSensorIOLaserCAN getReal()
    {
        return new DistanceSensorIOLaserCAN(Ports.CLAW_LASERCAN, NAME, RANGING_MODE, ROI,
            TIMING_BUDGET);
    }

    public static DistanceSensorIOSim getSim()
    {
        return new DistanceSensorIOSim(NAME);
    }

    public static DistanceSensorIO getReplay()
    {
        return new DistanceSensorIO() {};
    }
}
