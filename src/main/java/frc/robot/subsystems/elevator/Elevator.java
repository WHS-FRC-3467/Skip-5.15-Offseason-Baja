// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.io.motor.MotorIO.PIDSlot;
import frc.lib.mechanisms.linear.LinearMechanism;
import frc.lib.util.LoggedTunableNumber;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

public class Elevator extends SubsystemBase {
    private final LinearMechanism io;
    private Trigger homedTrigger;
    public Trigger launchHeightTrigger;
    private Debouncer homeDebouncer = new Debouncer(0.1, DebounceType.kRising);

    static LoggedTunableNumber launchHeight =
        new LoggedTunableNumber("Elevator/LaunchHeight", 3.8);

    @RequiredArgsConstructor
    @Getter
    public enum Setpoint {
        STOW(Rotations.of(0.09)),
        CORAL_INTAKE(Rotations.of(0.0)),
        LEVEL_1(Rotations.of(0.3)),
        LEVEL_2(Rotations.of(1.217)),
        LEVEL_3(Rotations.of(2.7)),
        LEVEL_4(Rotations.of(4.95)), // UNH settings
        // LEVEL_4(Rotations.of(5.02)), // Toyota settings
        CLIMB(Rotations.of(0.0)),
        ALGAE_LOW(Rotations.of(0.65)),
        ALGAE_HIGH(Rotations.of(2.1)),
        ALGAE_LOW_P(Rotations.of(2)),
        ALGAE_HIGH_P(Rotations.of(3.54)),
        ALGAE_GROUND(Rotations.of(0.05)),
        ALGAE_LOLLIPOP(Rotations.of(0.07)),
        ALGAE_STOW(Rotations.of(0.2)),
        PROCESSOR_SCORE(Rotations.of(0.05)),
        BARGE(Rotations.of(5.6));

        private final Angle setpoint;
    }

    public Elevator(LinearMechanism io)
    {
        this.io = io;
        homedTrigger =
            new Trigger(() -> homeDebouncer.calculate(io.getSupplyCurrent().gte(Amps.of(3))));
        launchHeightTrigger =
            new Trigger(() -> (io.getPosition().in(Rotations) >= launchHeight.getAsDouble()));
    }

    @Override
    public void periodic()
    {
        io.periodic();
    }

    public Command goToSetpoint(Setpoint setpoint)
    {
        return this
            .runOnce(() -> io.runPosition(setpoint.getSetpoint(), ElevatorConstants.CRUISE_VELOCITY,
                ElevatorConstants.ACCELERATION, ElevatorConstants.JERK, PIDSlot.SLOT_1));
    }

    public Command homeCommand()
    {
        return Commands.sequence(
            runOnce(() -> io.runVoltage(Volts.of(-2))),
            Commands.waitUntil(homedTrigger),
            runOnce(() -> io.setEncoderPosition(Setpoint.STOW.getSetpoint())),
            goToSetpoint(Setpoint.STOW));
    }
}
