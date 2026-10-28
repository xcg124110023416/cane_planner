# Slow-refresh correction

User steering: the actual updates look too slow. Original .5s default is slower
than the .1s control loop. A real manager regression with explicit .1s cadence
proves a 1ms late search shifts the next deadline and skips a control tick: at
620.217 expected three attempts, observed two. With .2s requested, an isolated
office baseline measured median publication gap .263s, maximum .293s.

Use default .2s (target 5Hz), retaining .1..5s configurability. Do not promise
10Hz: user's observed complete searches reach 160ms before corridor/MPPI work.
Schedule next deadline on a fixed ROS-time phase, advance once beyond current
time, skip missed slots without catch-up searches, reset both times on goal/
rollback. Keep original source/receipt freshness, transaction, geometry, MPPI
and final execution checks. Distinguish attempted searches from accepted route
publications. Retained old references cannot bypass current validation.

The preview probe found only three nonempty frames and a 2.2s gap with its old .5s schedule. Simply setting preview to .2s still left .8s gaps: job launches drifted into non-refresh cycles and returned after route revision changed. Align default preview period to .2s and anchor its next deadlines to the global search phase, retaining independent configuration and all original age/revision rejection. Bound preview period to .1..5s, matching global cadence validation. No LFPC timing change, reduced envelope,
unsafe candidate publication or user-owned node restart. A later root-obstructed
REFERENCE_OCCUPIED episode is a separate geometric STOP, not evidence of no A*.

Validate phase/default/no-catch-up and the full manager suite, then measure real
A* publication intervals under owned-master office and pedestrian replays.

A second production replay still had missing preview frames despite fixed phases.
Global eligibility was checked before MPPI but preview eligibility afterward: MPPI
could cross a shared deadline in a non-refresh cycle, causing a job to launch on
the old route just before the next update. Both now receive one scheduling epoch
captured after the observation mutex is acquired. This never replaces actual
source/receipt/CV/deadline/execution clocks; static preview creation remains actual
creation time. The real manager regression covers that within-cycle crossing, and
real /mpc/corridor_preview publication gaps are measured separately from paths.
