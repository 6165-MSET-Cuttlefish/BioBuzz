package org.firstinspires.ftc.teamcode.architecture.hardware.photon;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

import dev.frozenmilk.sinister.loading.Pinned;

/**
 * Marks an OpMode whose hubs {@link PhotonCore} takes over at INIT; only such an OpMode may call
 * {@link PhotonCore#enable()}. Hubs stay taken over for the rest of that hardware map's life.
 */
@Pinned
@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.TYPE)
public @interface Photon {
}
