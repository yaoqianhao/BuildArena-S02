"""Sandbox / preset environment for the S02 rocket orbit-return example.

Same preset as the original rocket_orbit_return demo (HEAVY_LAUNCHER_POLICY
selects the controller phase: ``mission`` for the full closed loop,
``calibrate`` for an open-loop thrust / wheel pulse run used to re-fit
``MissionConfig.thrust_accel`` and the wheel torque axes).

S02 gates are enforced inside the controller (see CONTROL.md):
  * orbit sweep  >= 1080 deg (3 laps) with periapsis radius >= 1100
  * recovery integrity >= 0.90 and a settled landing held >= 10 s
"""

ENVIRONMENT = {
    # LONE ORB preset used by prepare_example / execute_prepared.
    "HEAVY_LAUNCHER_POLICY": "mission",
    "HEAVY_LAUNCHER_REQUIRE_SUCCESS": "1",
}

CLEAR_PREFIX = "BUILDARENA_"
