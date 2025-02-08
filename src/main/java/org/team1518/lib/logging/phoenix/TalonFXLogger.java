package org.team1518.lib.logging.phoenix;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;

@CustomLoggerFor(TalonFX.class)
public class TalonFXLogger extends ClassSpecificLogger<TalonFX> {

    private static final Map<TalonFX, Consumer<EpilogueBackend>> cache = new HashMap<>();

    public TalonFXLogger() {
        super(TalonFX.class);
    }

    @Override
    public void update(EpilogueBackend backend, TalonFX talonFX) {
        cache
            .computeIfAbsent(talonFX, key -> {
                var acceleration = talonFX.getAcceleration(false);
                var closedLoopError = talonFX.getClosedLoopError(false);
                var closedLoopOutput = talonFX.getClosedLoopOutput(false);
                var closedLoopReference = talonFX.getClosedLoopReference(false);
                var deviceTemp = talonFX.getDeviceTemp(false);
                var motorVoltage = talonFX.getMotorVoltage(false);
                var position = talonFX.getPosition(false);
                var statorCurrent = talonFX.getStatorCurrent(false);
                var supplyCurrent = talonFX.getSupplyCurrent(false);
                var supplyVoltage = talonFX.getSupplyVoltage(false);
                var velocity = talonFX.getVelocity(false);

                BaseStatusSignal[] signals = {
                    acceleration,
                    closedLoopError,
                    closedLoopOutput,
                    closedLoopReference,
                    deviceTemp,
                    motorVoltage,
                    position,
                    statorCurrent,
                    supplyCurrent,
                    supplyVoltage,
                    velocity
                };

                return b -> {
                    BaseStatusSignal.refreshAll(signals);
                    b.log("acceleration", acceleration.getValueAsDouble());
                    b.log("closedLoopError", closedLoopError.getValueAsDouble());
                    b.log("closedLoopOutput", closedLoopOutput.getValueAsDouble());
                    b.log("closedLoopReference", closedLoopReference.getValueAsDouble());
                    b.log("deviceTemp", deviceTemp.getValueAsDouble());
                    b.log("motorVoltage", motorVoltage.getValueAsDouble());
                    b.log("position", position.getValueAsDouble());
                    b.log("statorCurrent", statorCurrent.getValueAsDouble());
                    b.log("supplyCurrent", supplyCurrent.getValueAsDouble());
                    b.log("supplyVoltage", supplyVoltage.getValueAsDouble());
                    b.log("velocity", velocity.getValueAsDouble());
                };
            })
            .accept(backend);
    }
}
