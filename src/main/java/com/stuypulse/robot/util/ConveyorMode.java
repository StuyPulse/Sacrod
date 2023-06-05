package com.stuypulse.robot.util;

import java.util.function.Consumer;

import com.stuypulse.robot.subsystems.Conveyor;
import com.stuypulse.robot.subsystems.IIntake;
import com.stuypulse.robot.subsystems.SwerveDrive;

public enum ConveyorMode {
    INDEXING(conveyor -> {
        var angle = SwerveDrive.getInstance().getAngle().getDegrees();
        var isShooting = angle > -90 && angle < 90;

        if (!conveyor.hasShooterBall() && isShooting) {
            conveyor.runForward();
        } else if (!conveyor.hasIntakeBall() && !isShooting) {
            conveyor.runReverse();
        } else {
            conveyor.stop();
        }
    }),

    SHOOTING(conveyor -> { 
        var angle = SwerveDrive.getInstance().getAngle().getDegrees();

        if (angle > -90 && angle < 90) {
            conveyor.runForward();
        } else {
            conveyor.runReverse();
            IIntake.getInstance().deacquire();
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