package org.firstinspires.ftc.teamcode.pedro;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

/**
 * Marks an OpMode in which Pedro can't drive the robot (no path, hold or turn) until {@link BioBuzzConstants#isMeasured},
 * so it may build {@link BioBuzzConstants#create}'s follower before {@link BioBuzzConstants#measured} is set. Not inherited.
 */
@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.TYPE)
public @interface PedroNeverDrives {
}
