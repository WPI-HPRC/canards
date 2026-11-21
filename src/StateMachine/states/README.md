# Flight states

The seven flight states (docs/HANDOFF-flight-skeleton-2026-11-20.md,
section 6.1). Each is a long-lived object, built once by the FlightComputer
with references to the subsystems it uses. States touch the canards only
through Canards requests.

Every transition condition is debounced with a Persistence on sample time,
and every state except Init, PreFlight and Land has a timeout, so a failed
detection cannot strand the flight. Thresholds come from FlightConfig only.

There are no pyro charges on this rocket: no state fires anything.

`LaunchDetector` is the launch test shared by Init and PreFlight.
