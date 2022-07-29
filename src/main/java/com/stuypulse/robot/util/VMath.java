package com.stuypulse.robot.util;

import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.math.Vector2D;

public class VMath {
    public static Vector2D spow(Vector2D v, double exp) {
        return new Vector2D(SLMath.spow(v.x, exp), SLMath.spow(v.y, exp));
    }
}
