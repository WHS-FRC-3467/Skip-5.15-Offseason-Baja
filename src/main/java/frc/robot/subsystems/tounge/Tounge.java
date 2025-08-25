// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.tounge;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.mechanisms.rotary.RotaryMechanism;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

public class Tounge extends SubsystemBase {

    private final RotaryMechanism io;
    public final Trigger homedTrigger;
    public final Trigger coralContactTrigger;
    public final Trigger hasLoweredTrigger;
    private final Debouncer homedDebouncer = new Debouncer(0.1, DebounceType.kRising);

    @RequiredArgsConstructor
    @Getter
    public enum Setpoint {
        HOMING(Volts.of(-1)),
        STOW(Volts.of(0)),
        RAISED(Volts.of(1)),
        L1(Volts.of(12)),
        DOWN(Volts.of(-12));

        private final Voltage setpoint;
    }

    public Tounge(RotaryMechanism io)
    {
        this.io = io;
        homedTrigger = new Trigger(
            () -> homedDebouncer.calculate(
                io.getSupplyCurrent().gt(Amps.of(2))));

        coralContactTrigger = new Trigger(
            () -> nearPosition(Rotation.of(.29)));

        hasLoweredTrigger = new Trigger(
            () -> nearPosition(Rotation.of(0)));
    }

    @Override
    public void periodic()
    {
        io.periodic();
    }

    public Command setSetpoint(Setpoint setpoint)
    {
        return this.runOnce(
            () -> io.runVoltage(setpoint.getSetpoint()));
    };

    public boolean nearPosition(Angle targetPosition)
    {
        return MathUtil.isNear(
            io.getPosition().in(BaseUnits.AngleUnit),
            targetPosition.in(BaseUnits.AngleUnit),
            ToungeConstants.TOLERANCE.in(BaseUnits.AngleUnit));
    }

    public Command lowerToungeCommand()
    {
        return Commands.sequence(
            setSetpoint(Setpoint.DOWN),
            Commands.race(
                Commands.waitUntil(this.hasLoweredTrigger),
                Commands.waitSeconds(0.5)),
            setSetpoint(Setpoint.STOW));
    }

    public Command homeCommand()
    {
        return Commands.sequence(
            setSetpoint(Setpoint.HOMING),
            Commands.waitUntil(homedTrigger),
            Commands.runOnce(() -> io.setEncoderPosition(Rotation.of(0))),
            setSetpoint(Setpoint.STOW));
    }
}
