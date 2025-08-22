// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.clawrollerlasercan;

import static edu.wpi.first.units.Units.Meters;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.devices.DistanceSensor;
import frc.lib.io.distancesensor.DistanceSensorIO;

public class ClawRollerLaserCAN extends SubsystemBase {

    private final DistanceSensor distanceSensor;

    public final Trigger triggered;

    public ClawRollerLaserCAN(DistanceSensorIO io)
    {
        distanceSensor = new DistanceSensor(io);
        triggered = new Trigger(() -> distanceSensor.getDistance().isPresent()
            ? distanceSensor.getDistance().get().in(Meters) <= 0.05
            : false);
    }

    @Override
    public void periodic()
    {
        distanceSensor.periodic();
    }


}
