// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.io.motor.MotorIO.PIDSlot;
import frc.lib.mechanisms.rotary.RotaryMechanism;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

public class Climber extends SubsystemBase {

    private final RotaryMechanism io;
    private final Trigger homed;
    private Debouncer homedDebouncer = new Debouncer(0.75, DebounceType.kRising);

    @RequiredArgsConstructor
    @Getter
    public enum Setpoint {
        HOME(Rotations.of(0)),
        PREP(Rotations.of(-1.4)),
        CLIMB(Rotations.of(-0.1));

        private final Angle setpoint;
    }


    public Climber(RotaryMechanism io)
    {
        this.io = io;
        this.homed = new Trigger(() -> homedDebouncer.calculate(
            io.getSupplyCurrent().gte(Amps.of(4.7))));
    }

    @Override
    public void periodic()
    {
        io.periodic();
    }

    public Command stop()
    {
        return this.runOnce(() -> io.runBrake());
    }

    public Command setSetpoint(Setpoint setpoint)
    {
        return this.runOnce(
            () -> io.runPosition(setpoint.getSetpoint(), ClimberConstants.CRUISE_VELOCITY,
                ClimberConstants.ACCELERATION, ClimberConstants.JERK,
                PIDSlot.SLOT_1));
    };

    public Command home()
    {
        return Commands.sequence(
            runOnce(() -> io.runVoltage(Volts.of(4))),
            Commands.waitUntil(homed),
            runOnce(() -> io.setEncoderPosition(Rotations.of(0))),
            setSetpoint(Setpoint.HOME));
    }
}
