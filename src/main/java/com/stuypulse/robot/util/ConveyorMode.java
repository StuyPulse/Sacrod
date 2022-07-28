package com.stuypulse.robot.util;

import java.util.function.Consumer;

import com.stuypulse.robot.subsystems.Conveyor;

public enum ConveyorMode {
    INDEXING(conveyor -> {
        if (conveyor.hasIntakeBall() && !conveyor.hasShooterBall()) {
            conveyor.runForward();       
        } else {
            conveyor.stop();
        }
    }),

    SHOOTING(conveyor -> { 
        conveyor.runForward();
    }),
    
    BRING_UP_BALLS(conveyor -> {
        if (!conveyor.hasShooterBall()) {
            conveyor.runForward();
        } else {
            conveyor.stop();
        }
    }),

    REVERSE(conveyor -> {
        conveyor.runReverse();
    }),
    
    STOP(conveyor -> {
        conveyor.stop();
    }),
    
    DEFAULT(INDEXING.action);
    
    private Consumer<Conveyor> action;

    private ConveyorMode(Consumer<Conveyor> action) {
        this.action = action;
    }

    public void run(Conveyor conveyor) {
        action.accept(conveyor);
    }
}