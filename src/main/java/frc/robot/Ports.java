/*
 * Copyright (C) 2025 Windham Windup
 *
 * This program is free software: you can redistribute it and/or modify it under the terms of the
 * GNU General Public License as published by the Free Software Foundation, either version 3 of the
 * License, or any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without
 * even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with this program. If
 * not, see <https://www.gnu.org/licenses/>.
 */

package frc.robot;

import frc.lib.util.Device;

public class Ports {
    /*
     * LIST OF CHANNEL AND CAN IDS
     */

    public static final Device.CAN MITOCANDRIA = new Device.CAN(15, "rio");
    public static final Device.CAN CLAW_LASERCAN = new Device.CAN(16, "rio");
    public static final Device.CAN CLAW_ROLLER = new Device.CAN(17, "rio");

    public static final Device.CAN ARM_CANCODER = new Device.CAN(18, "rio");
    public static final Device.CAN ARM_MAIN = new Device.CAN(19, "rio");

    public static final Device.CAN ELEVATOR_CANDLE = new Device.CAN(20, "rio");

    public static final Device.CAN ELEVATOR_MAIN = new Device.CAN(22, "rio"); // Top Kraken
    public static final Device.CAN ELEVATOR_FOLLOWER = new Device.CAN(23, "rio"); // Bottom Kraken

    public static final Device.CAN CLIMBER = new Device.CAN(24, "Drivetrain");

    public static final Device.CAN PDH = new Device.CAN(25, "rio");

    public static final Device.CAN TOUNGE = new Device.CAN(30, "rio");

}
