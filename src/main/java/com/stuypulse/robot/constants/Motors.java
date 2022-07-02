/************************ PROJECT DORCAS ************************/
/* Copyright (c) 2022 StuyPulse Robotics. All rights reserved.  */
/* This work is licensed under the terms of the MIT license.    */
/****************************************************************/

package com.stuypulse.robot.constants;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

/*-
 * File containing all of the configurations that different motors require.
 *
 * Such configurations include:
 *  - If it is Inverted
 *  - The Idle Mode of the Motor
 *  - The Current Limit
 *  - The Open Loop Ramp Rate
 */
public interface Motors {

    public static TalonSRXConfig CLIMBER = new TalonSRXConfig(false, NeutralMode.Brake, 60);

    public static class TalonSRXConfig {
        public final boolean INVERTED;
        public final NeutralMode NEUTRAL_MODE;
        public final int PEAK_CURRENT_LIMIT_AMPS;
        public final double OPEN_LOOP_RAMP_RATE;

        public TalonSRXConfig(
            boolean inverted,
            NeutralMode neutralMode,
            int peakCurrentLimitAmps,
            double openLoopRampRate
        ) {
            this.INVERTED = inverted;
            this.NEUTRAL_MODE = neutralMode;
            this.PEAK_CURRENT_LIMIT_AMPS = peakCurrentLimitAmps;
            this.OPEN_LOOP_RAMP_RATE = openLoopRampRate;
        }

        public TalonSRXConfig(boolean inverted, NeutralMode neutralMode, int peakCurrentLimitAmps) {
            this(inverted, neutralMode, peakCurrentLimitAmps, 0.0);
        }

        public TalonSRXConfig(boolean inverted, NeutralMode neutralMode) {
            this(inverted, neutralMode, 80);
        }

        public void configure(WPI_TalonSRX motor) {
            motor.setInverted(INVERTED);
            motor.setNeutralMode(NEUTRAL_MODE);
            motor.configContinuousCurrentLimit(PEAK_CURRENT_LIMIT_AMPS - 10, 0);
            motor.configPeakCurrentLimit(PEAK_CURRENT_LIMIT_AMPS, 0);
            motor.configPeakCurrentDuration(100, 0);
            motor.enableCurrentLimit(true);
            motor.configOpenloopRamp(OPEN_LOOP_RAMP_RATE);
        }
    }
}
