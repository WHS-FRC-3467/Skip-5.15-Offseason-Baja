// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.io.motor.MotorIO.PIDSlot;
import frc.lib.mechanisms.rotary.RotaryMechanism;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

public class Arm extends SubsystemBase {

    private final RotaryMechanism io;

    @RequiredArgsConstructor
    @Getter
    public enum Setpoint {
        STOW(Degrees.of(120.18)),
        CORAL_INTAKE(Degrees.of(135.7)),
        LEVEL_1(Degrees.of(123)),
        LEVEL_2(Degrees.of(97.48)),
        LEVEL_3(Degrees.of(104.48)),
        LEVEL_4(Degrees.of(105.4)),
        CLIMB(Degrees.of(70.0)),
        ALGAE_LOW(Degrees.of(103.3)),
        ALGAE_LOW_P(Rotations.of(.2377)),
        ALGAE_HIGH(Rotations.of(103.3)),
        ALGAE_HIGH_P(Rotations.of(.2446)),
        ALGAE_GROUND(Degrees.of(70.0)),
        PROCESSOR_SCORE(Rotations.of(0.195)),
        BARGE(Degrees.of(130.0));

        private final Angle setpoint;
    }


    public Arm(RotaryMechanism io)
    {
        this.io = io;

    }

    @Override
    public void periodic()
    {
        io.periodic();
    }

    public Command setSetpoint(Setpoint setpoint)
    {
        return this.runOnce(
            () -> io.runPosition(setpoint.getSetpoint(), ArmConstants.CRUISE_VELOCITY,
                ArmConstants.ACCELERATION, ArmConstants.JERK,
                PIDSlot.SLOT_1));
    };

    public boolean nearPosition(Angle targetPosition)
    {
        return MathUtil.isNear(
            io.getPosition().in(BaseUnits.AngleUnit),
            targetPosition.in(BaseUnits.AngleUnit),
            ArmConstants.TOLERANCE.in(BaseUnits.AngleUnit));
    }

    public Command waitForPositionCommand(Angle position)
    {
        return Commands.waitUntil(() -> {
            return nearPosition(position);
        });
    }

    public Command setpointCommandWithWait(Setpoint setpoint)
    {
        return waitForPositionCommand(setpoint.getSetpoint())
            .deadlineFor(setSetpoint(setpoint));
    }


}
