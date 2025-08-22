// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.clawroller;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.io.motor.MotorIO.PIDSlot;
import frc.lib.mechanisms.flywheel.FlywheelMechanism;
import frc.robot.subsystems.elevator.ElevatorConstants;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

/** Add your docs here. */
public class ClawRoller extends SubsystemBase { // Don't extend if contained in superstructure
    private final FlywheelMechanism io;

    @RequiredArgsConstructor
    @Getter
    public enum Setpoint {
        SHUFFLE(Rotations.of(-0.1)),
        L4_RETRACT(Rotations.of(-0.5)),
        L1_SHUFFLE(Rotations.of(0.5)),
        HOLDCORAL(Rotations.of(-0.1));

        private final Angle setpoint;
    }

    public ClawRoller(FlywheelMechanism io)
    {
        this.io = io;

        // public final Trigger stalled =
        // new Trigger(
        // () -> (Math.abs(super.inputs.velocityRps) <= 0.02
        // && super.inputs.supplyCurrentAmps[0] >= 1));

        // public final Trigger stopped =
        // new Trigger(() -> (Math.abs(super.inputs.velocityRps) <= 0.02));

        // public final Trigger freeSpin =
        // new Trigger(() -> (Math.abs(super.inputs.velocityRps) >= 10));
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

    public Command intake()
    { // TODO: add duty cycle limiter to .35
        return this.runOnce(() -> io.runCurrent(Amps.of(80)));
    }

    public Command score()
    {
        return runOnce(() -> io.runVoltage(Volts.of(2.0)));
    }

    public Command L1score()
    { // TODO: add duty to .45
        return runOnce(() -> io.runCurrent(Amps.of(60)));
    }

    public Command algaeForward()
    {
        return runOnce(() -> io.runCurrent(Amps.of(100)));
    }

    public Command algaeReverse()
    {
        return runOnce(() -> io.runCurrent(Amps.of(-90)));
    }

    public Command runSetpoint(Setpoint setpoint)
    { // TODO: is "this" needed?
        return this
            .runOnce(
                () -> io.runPosition(setpoint.getSetpoint(), ClawRollerConstants.CRUISE_VELOCITY,
                    ClawRollerConstants.ACCELERATION, ElevatorConstants.JERK, PIDSlot.SLOT_1));
    }

    public Command zeroSensors()
    {
        return Commands.runOnce(() -> io.setEncoderPosition(Rotations.of(0)));
    }

    public Command shuffleCommand()
    {
        return Commands.sequence(
            zeroSensors(),
            runSetpoint(Setpoint.SHUFFLE));
    }

    public Command L4ShuffleCommand()
    {
        return Commands.sequence(
            zeroSensors(),
            runSetpoint(Setpoint.L4_RETRACT));
    }

    public Command L1ShuffleCommand()
    {
        return Commands.sequence(
            zeroSensors(),
            runSetpoint(Setpoint.L1_SHUFFLE));
    }

}
