# Common controller manager test suite

This suite tests a ROS 2 controller manager (CM) against what any CM must guarantee, independent of
its design. It checks what a CM does, never how it does it.

| Package | Content |
|---|---|
| `cm_test_suite` | the rig plugins, the oracle, the fuzzer, the acceptance scripts and the `CmUnderTest` adapter interface |
| `cm_test_suite_legacy` | the adapter for the existing ros2_control CM, as baseline and as an example |

## 1. The rig

One fixed robot, complex enough to reach most failure combinations. Plugins find their
configuration in `rig.cpp` by name, so no parameters are needed.

```
hardware   arm (sync)       a1 a2 a3   position | velocity   (one mode per joint)
           base (async)     wl wr      velocity
           flipper (sync)   f1         position | velocity   keeps running without power
           imu (async)      imu/yaw    state only

teleop ─► whole_body ─┬─► arm_ik ─┐
                      ├─► wrist ──┴─► arm_vel ─► arm velocity     two writers on arm_vel
nav ──────────────────┼─► diff_drive ─► base_vel ─► base          nav and whole_body compete
                      └─► flipper_pos ─► flipper position
arm_pos ─► arm position          flipper_vel ─► flipper velocity  mode conflicts
jsb ◄ joint positions            monitor (no interfaces)
odom_pub ◄ diff_drive/odom (exported state), imu/yaw
```

Startup profile: all hardware active, all controllers loaded, active
`jsb arm_vel arm_ik wrist base_vel diff_drive flipper_pos odom_pub`.

With `CM_FUZZ_ALL_ASYNC=1` every component is async, so arm runs like Athena's arm in velocity mode
and flipper like its flippers in position mode, without a watchdog.

Every joint is limited to a velocity of 10^30 and, except the continuous wheels wl and wr, to a
position of ±2^53, and its states start at 0. Tokens lie within these limits, so enforcing them
changes no token.

## 2. What is checked

**Tokens.** Every command a controller writes is a token, a unique integer in the double. The truth
records who minted it, in which activation (epoch) and cycle, and for which interface. A chainable
controller derives each command token from the token in its reference, adding itself as a link. When
hardware applies a command, it walks the whole chain down to the original writer. Every link must be
active, in the same activation that minted it, at most `MAX_COMMAND_AGE_CYCLES` (5) old, and routed
to the right interface. NaN means no command and is always allowed. Async hardware may apply data
latched one cycle earlier, so for it a writer that stopped in the previous cycle still counts
(`CM_FUZZ_ASYNC_GRACE_CYCLES`, 1, and 0 for a CM that holds async hardware before its writer stops),
and since it runs on its own clock a value is stale only if it is also older than 5 ms (50 ms under
sanitizers). Otherwise the microsecond cycles of the stepped mode would count the scheduling delay
of the async thread as staleness. The 5 ms leave out the time the async thread waited preempted
since its last write, mostly its wakeup on a loaded machine, and the time the control loop waited
preempted in the cycle that minted the value, which delays its handoff to the async thread. The age
is taken at the start of the write, before the preemption, so a wait between the two is not age. So only
sleeping, blocking or an old command age a value. All run queue wait since the owner's last write
and in the whole minting cycle is excused, which includes read only cycles and the part of the
cycle before the mint. The two waits add up even where they overlap. Only the fuzz runs cover this
per write baseline. A token that dropped out of the ring of the last 65536 tokens is stale for sync
hardware. For async hardware it ages like the younger token in its slot, a lower bound, so a value
applied once may pass while one applied again and again is caught. Its chain is not walked.

**Hardware model** (`TraceSystem`, like a Hector drive without watchdog). A command interface is
applied only between a start and a stop of its mode in `perform_command_mode_switch`. A stop holds
it. `prepare` rejects two modes on one joint. Reads of inactive hardware are allowed, writes are
not.

**Limits.** Instead of every 16th token a controller writes 10^31 into a hardware command, beyond
every limit. Hardware must apply every command within the limits of its joint in the robot
description, so the probe only clamped onto a limit, or below it by a limiter that keeps the
position in range within one cycle. Such a value lies beyond every token and is not traced. The
velocity of an async joint with position limits is not probed, because a limiter may predict
such a joint from its last command, and a clamped probe would take it onto its position limit,
so the limiter would hold the next commands there. Under `CM_FUZZ_ALL_ASYNC=1` this leaves the
arm and flipper velocity limits unprobed, the configuration of Athena's arm in velocity mode, so
there only the CM's own tests cover the velocity limits.
A known blind spot follows: if a stopped writer's last command was a probe, the inactive writer
check cannot see that command applied.

**Rules every CM must follow**, beyond the rulebook text. These are the CM behaviours the oracle
encodes, the outcome checks only bound everything else:
- A pause of a running controller longer than 5 cycles requires its hardware to hold.
- Chained mode is set before the user is activated.
- A controller whose update failed is not updated again in the same activation.
- A best effort reader claims the states of every readable component when it starts. It may skip
  those of a component whose fault is still being handled, for example during a slow on_error, or
  whose call hangs.
- A controller may write no command in its first updates after an activation (`quiet`, 3
  updates). The hardware must never apply a command of an earlier activation meanwhile, also not
  through a chain.
- A controller whose on_deactivate refuses (FAILURE) counts as stopped. It is not updated any more,
  its interfaces are released and held, and it is reported inactive or finalized.
- A load is rejected while a callback of the previous instance of that controller still runs, for
  example an abandoned on_cleanup (rulebook 2.2.5).
- Inactive hardware is read while an active controller reads its states, so the controller keeps
  getting them. It is never written.
- A failure may stop at most the active unit around it, everything connected to the failed
  controller or hardware through controllers that were active before or started by the step.
  FORCE_AUTO may stop the active unit of what conflicts with the request, controllers claiming the
  same command interfaces or another mode of the same joint. STRICT and AUTO never stop anything
  that was not requested.
- A switch rejected only after its timeout may leave the request, the providers it claims and,
  under FORCE_AUTO, what conflicts with it stopped, and with them every active user and every
  active provider a stopped controller writes,
  because a rollback cut short at the deadline restores nothing more. A rejection never starts
  anything.
- A hardware recovery changes no controller, except that an accepted one that brought the hardware
  up from not active or faulted may start inactive controllers of the startup profile, as a
  startup switch after the recovery does. The rig has no protected controller, so the suite cannot
  see a recovery that restarts one.
- A switch and a shutdown return within the switch timeout plus 500 ms (2.5 s under sanitizers),
  even if a callback hangs, also while the CM handles a fault whose on_deactivate or on_error is
  slow or hangs. A `stall` stops the control loop until the switch or shutdown returned, a `hold`
  keeps it inside the read of sync hardware. Cycle time is not checked in such a step.
- `prepare` and `perform` may take longer than a cycle (30 ms, like Dynamixel's bus traffic). Only
  read and write of sync hardware run on the control loop (rulebook 2.2.1). The loop never waits for
  any other hardware call and skips the read and write of a sync component while such a call runs.
- A lifecycle callback may use up nearly all of the switch timeout (`late`, 975 ms of 1 s), so the
  deadline passes during the slow prepare or perform after it. The switch still returns in time, and
  nothing a rejected or rolled back switch leaves stopped may reach the hardware. A late callback
  counts as a hang for the order checks (rulebook 2.2.5).
- Async hardware may be slow and still make progress for 300 ms: read and write take 1 ms each
  (`slowio`), a random time up to 5 ms each (`slowio jitter`), or every write stalls the owner for
  50 ms, half of Hector's hang rule (`slowio stall`), or 80 ms, just below it (`slowio long_stall`).
  No fault fires, so the CM must not fault it or stop anything.
- Async hardware may hang in read or write (`hang`, 2.5 s), a write that hangs sends nothing. The CM
  may fault it and stop what uses it, like on an error.
- No control cycle takes longer than 20 ms (200 ms under sanitizers).

### Power cut

A power cut is the hard e-stop of rulebook section 5, and the CM sees it only as hardware faults. It
cuts the whole rig at once. arm, base and imu return OK for two more transfers and ERROR from the
third on, like a bus driver that tolerates a few failed transfers. flipper keeps running, so
everything on it, new activations included, must keep working. The controllers on the faulting
hardware must no longer be updated within 400 ms of the cut (2 s under sanitizers), also of a cut
during a switch except at cycle 0 of a switch that stalls or holds the control loop, and no fault
ever unloads a controller. Their lifecycle may lag behind a slow callback of another controller.
Once a transfer of their hardware returned ERROR, the consistency rule of the settled checks
requires them inactive. The `power_cut` check requires it as well, but runs only after a waited
cut. A controller still inside a hung or late lifecycle callback cannot be cut short, so its
400 ms start when that callback returned.

Restoring the power runs the recovery sequence of `hector_controller_spawner` through the adapter
once the faults settled. `recover_hardware` runs for every component reported neither active nor
finalized, then one AUTO switch for the startup controllers not reported active, STRICT for a CM
without the AUTO capability. Each call is checked as its own step, and afterwards every hardware
that is not finalized must be active. There are no retries, so a rejection the AUTO rules allow
leaves the startup set incomplete.

### Descriptions, events and settled checks

A robot description in which two components export the same interface must be rejected as a whole
and must never crash the CM. `<Name>Acceptance.duplicate_interfaces_rejected_{stepped,threaded}`
create the CM with it in a child process. A crash, an exception or a failed shutdown is a gate.

**Events** are detected by the plugins at the moment they happen (category `event:<kind>`).

| Kind | Meaning |
|---|---|
| `inactive_writer` | a command, reference or exported state from an inactive or restarted controller is applied or read |
| `stale_value` | a value older than 5 cycles is applied |
| `unknown_value`, `misrouted_value` | garbage in an interface, or a token minted for another interface |
| `limit_exceeded` | hardware applies a command outside the limits of its joint |
| `torn_command` | one write applies commands of one activation of a controller from different updates |
| `mode_conflict` | perform left two modes active on one joint |
| `io_while_inactive`, `io_after_fault` | write to inactive hardware, or read and write after a returned error before a lifecycle transition |
| `concurrent_hardware_call`, `concurrent_controller_call` | two threads inside one plugin at once |
| `update_while_inactive`, `update_after_failure`, `missing_interfaces` | update of an inactive or failed controller, or with the wrong interfaces |
| `activated_before_provider`, `activated_on_unchained_provider`, `activated_on_inactive_hardware` | activation order |
| `provider_stopped_under_user` | deactivation order. A user hanging in an injected callback is out of the running set, so its providers may stop |
| `shared_command_interface` | activated while another active controller claims one of its command or reference interfaces or another mode of a joint it commands. One hanging in an injected callback is out of the running set, one whose update failed is not, since it may still write in on_deactivate |
| `events_lost` | more events in one step than the truth holds, the checks after it may miss some |

**Settled checks.** After every step the harness waits until no injected hang sleeps in a callback,
then cycles until the truth (lifecycle states, chained modes, faults and applied interfaces) was
unchanged for 10 ms, no hardware is faulted, no failed controller is active, no lifecycle or mode
switch callback runs, the consistency checks pass and the hardware applies every claimed interface
that `liveness` checks. The wait is bounded by the longest hang, so an inconsistency between two
teardown steps or a mode switch a step after an activation is waited out for up to 200 ms (1 s under
sanitizers) and one that lasts longer is reported. Settling that is still not quiet when the
longest hang plus 1 s passed is a `settle` finding naming what kept it busy. Then:
- `consistency`: single claimer per command interface, active providers, chained mode equals "has
  an active writer", usable hardware, no failed controller or hardware still active, no hardware
  applying an interface without an active claimer
- `liveness`: over the last 10 cycles, extended by up to 100 ms for async hardware, every active
  hardware was read and written, every inactive hardware read by an active controller was read,
  every controller active in the same activation throughout was updated, and every claimed
  interface whose chain reaches an active writer was applied with a command. A provider that lost
  the writer of a reference may hold that output or write nothing. Hangs, stalls and holds end
  before this window, so none of them excuses a controller that is not updated.
- `status`: the list services equal the truth within 500 ms, nothing is reported in a transition,
  and every report sampled during the step shows a state the truth passed through
- `activity`: the same for the activity topic
- `outcome`: the step lies within the bounds of its semantics, see `oracle_steps.cpp`. A rejection
  changes nothing beyond what fired faults excuse, and nothing is reactivated because of a fault.
  Hardware changes only where a fault hit or the step targets it. A restart (`+x -x`) must change
  the epoch. The oracle predicts no answer, only `expect=accept|reject` in scripts is enforced. The
  acceptance scripts require valid requests of every strictness to be accepted, and the accept
  ratio in the run statistics shows a CM that rejects too much. A load while a callback of the
  previous instance still runs must be rejected.
- `power_cut`, `power_recovery`, `timeout` (also the 400 ms reaction to a power cut, 2 s under
  sanitizers), `rt_blocking` (a cycle over 20 ms, 200 ms under sanitizers, not counting the time
  other threads preempted it), `rt_allocation` (operator new inside the control cycle, not
  available under sanitizers), `shutdown` (nothing active, nothing applied), `hang`, `exception`
  (also of the shutdown that ends every script), `setup` (the startup profile exactly), `settle`

Only faults that actually fired and make their call fail or hang excuse anything: error, failure,
deactivate, throw, hang and late. A delay or a quiet activation excuses nothing. A hanging
on_activate gives up with FAILURE, so no plugin becomes active after the CM gave up on it.

### 2.1 Tiers

| Tier | Checks | Counts |
|---|---|---|
| gate | `consistency`, `shutdown`, `power_cut`, `exception`, `hang`, sanitizer reports and every `event:` except `stale_value`, `torn_command`, `activated_on_unchained_provider` and `events_lost` | right away |
| provisional | everything else | unless the referee rules it a suite bug |

Gates cover safety. Nothing of an inactive controller reaches hardware, chains are never
inconsistent, nothing crashes and errors are handled. `shared_command_interface` is a gate because
two writers on one interface are unsafe, and before it became one it found nothing on
hector_controller_manager. On upstream's CM that holds only for runs that skip the setup events,
because a plain legacy fuzz run never gets past setup. Provisional checks depend on oracle bounds or
time limits that may themselves be wrong, so dispute a finding you believe is wrong with the referee
instead of working around it (rulebook section 7). A gate that does not reproduce still counts if
its step settled. A report matching the truth does not prove that, because a CM may publish after
each internal step. Show instead that the flagged state is one teardown step short of the final one.

### 2.2 Reading a failure

```
[gate] [event:inactive_writer] step 3 'switch strict +teleop -nav': hardware base: value written
by an inactive or restarted controller in wl/velocity concerning nav (cycle 412 to 418, 7 times)
```

The tier comes first, then the category, then the phase and the message. The phase is `setup`,
`shutdown` or `step <n>` with the text of that script line, counted from 1. An event names the
plugin that saw it, the interface, the controller it concerns and the cycles in which it happened.
A fuzz failure is printed at once and followed by the shrunk script that still fails with the same
tier, category, phase and message, numbers and the entries of name lists aside. The shrinker
drops steps, then faults, stalls, holds, power cuts, `nowait` and single controllers of the
remaining steps, and counts a candidate as failing if one of two runs fails.
Save it and replay it with `CM_FUZZ_SCRIPT=<file>`.

## 3. Running it

```bash
colcon build --packages-select cm_test_suite cm_test_suite_legacy \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON
source install/setup.bash
export ROS_DOMAIN_ID=<free domain> ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
./build/cm_test_suite_legacy/test_legacy_cm --gtest_filter='Legacy*'
```

The plugins are found through pluginlib, hence the `source`. The legacy adapter registers twice,
as `Legacy` and as `LegacyAsap`, which switches with upstream's `activate_asap`. The filter
`Legacy*` runs both.

| Variable | Default | Meaning |
|---|---|---|
| `CM_FUZZ_SEED` | 0 | first seed |
| `CM_FUZZ_SEEDS` | 20 | stepped fuzz seeds |
| `CM_FUZZ_THREADED_SEEDS` | 5 | threaded fuzz seeds |
| `CM_FUZZ_STEPS` | 100 | steps per seed |
| `CM_FUZZ_SHRINK` | 1 | shrink a failing script, keeping the same tier, category, phase and message |
| `CM_FUZZ_SHRINK_SECONDS` | 60 | time budget of one shrink, the replay running at the end still finishes |
| `CM_FUZZ_SKIP` | | comma separated categories to ignore, for example `event:stale_value,rt_allocation` or `event` |
| `CM_FUZZ_TIER` | | `gates` reports only gate failures |
| `CM_FUZZ_REQUIRE_ALL` | 0 | `1` fails scripts that need a capability the CM lacks, instead of skipping them |
| `CM_FUZZ_SCRIPT` | | replay a script file in both modes |
| `CM_FUZZ_VERBOSE` | | keep ROS logging |
| `CM_FUZZ_ASYNC_GRACE_CYCLES` | 1 | cycles async hardware may still apply the command of a stopped writer |
| `CM_FUZZ_ALL_ASYNC` | 0 | `1` makes every rig component async |
| `CM_FUZZ_STATS` | 0 | `1` prints the statistics of each script run in one line |

Every script test records what its run exercised as test properties in the gtest XML: accepted
and rejected answers per step kind, armed and taken faults per hook and fault, steps with the power
cut, settling that hit its deadline and the longest checked cycle. Taken faults count every fault a
plugin took, delays included.

Skipping is for investigation only. Judging runs with nothing skipped, `CM_FUZZ_REQUIRE_ALL=1` and
both tiers.

**Isolation.** One test process runs its tests one after another, because the truth is process
wide. Runs in parallel need separate processes with separate `ROS_DOMAIN_ID`s, otherwise their
activity topics and services mix.

**Long runs.** Run the executable directly, not through ctest, whose timeout would stop it. Shard
seeds over processes, one domain ID each:

```bash
for shard in 0 1 2 3 4 5 6 7; do
  ROS_DOMAIN_ID=$((BASE_DOMAIN + shard)) CM_FUZZ_SEED=$((shard * 1250)) CM_FUZZ_SEEDS=1250 \
    CM_FUZZ_THREADED_SEEDS=0 CM_FUZZ_SHRINK=0 \
    ./build/<package>/<test> --gtest_filter='*Fuzz.stepped*' > fuzz_$shard.log 2>&1 &
done
wait
```

A hang that never ends aborts the process after 30 s, because its thread still references the CM.
The log names the step, the remaining seeds of that shard did not run.

**Sanitizers.** Build an ASan and a TSan tree of your packages and this suite, with
`-fsanitize=address -fno-omit-frame-pointer -O1 -g` or `-fsanitize=thread -fno-omit-frame-pointer
-O2 -g` as `CMAKE_CXX_FLAGS`, into separate build and install bases. TSan needs
`sudo sysctl vm.mmap_rnd_bits=28`. Run with `TSAN_OPTIONS="halt_on_error=0 report_signal_unsafe=0"`
and `ASAN_OPTIONS=detect_leaks=0`. The suite itself is TSan clean, so every report is in the CM or
its dependencies. A rare stall inside the sanitizer runtime can exceed the 200 ms cycle bound, so a
single `rt_blocking` under a sanitizer that does not replay is not a CM finding.

**Modes.** In the stepped mode the harness cycles only while an operation runs, as fast as
possible. In the threaded mode a thread cycles every millisecond like a real control loop.

**Scripts.** One step per line, see `script.hpp`. Put regression scripts into
`cm_test_suite/scripts/`, they run as acceptance tests for every CM.

```
switch strict +teleop -nav fault=hardware:base:write:error@3 stall|hold powercut@2 expect=accept
fault controller:arm_ik:update:throw [fault=controller:arm_ik:deactivate:hang ...] [nowait]
load nav | configure nav | cleanup nav | unload nav    [fault=...] [expect=accept|reject] [nowait]
recover arm                                             [fault=...] [expect=accept|reject]
powercut on [nowait] | powercut off
slowio [jitter|stall|long_stall]
shutdown                                                [fault=...] [stall]   (last step only)
```

## 4. Adding a controller manager

Implement `cm_test_suite::CmUnderTest` (`cm_under_test.hpp`) and register it in your test's main.
`cm_test_suite_legacy/test/test_legacy_cm.cpp` is a complete example of about 270 lines, and its
`CMakeLists.txt` shows the `ament_add_gtest` with a long `TIMEOUT` that links `cm_test_suite`.

```cpp
cm_test_suite::Capabilities capabilities;
capabilities.auto_switch = capabilities.force_auto_switch = true;
capabilities.startup = capabilities.recover_hardware = true;
cm_test_suite::register_suite("MyCm", capabilities, [](const cm_test_suite::CmConfig & config)
{ return std::make_unique<MyCm>(config); });
```

`cycle()` runs read, update and write once, and only one thread calls it at a time. Operations
block until done and may need cycles to complete. `reported()` may be called at any time from
another thread and returns what `list_controllers` and `list_hardware_components` return, and
`activity()` the state of the last message on the activity topic. Spin your nodes as
`ros2_control_node` would. `shutdown()` brings everything down as on process exit, while cycles may
still run. With the `startup` capability the CM loads and activates `CmConfig::startup` itself
within 10 s, otherwise the harness does it through the operations. Power cuts need
`recover_hardware`. Judging requires every capability.

Your CM library must not link `cm_test_suite`, only your test does. `reported()` must call your
real list services and `activity()` must read your real activity topic, and the CM must not behave
differently under test.

## 5. Baseline: the existing CM (lyrical-fix)

When the suite was added on 2026-11-02, every acceptance and fuzz test failed on the existing CM.
Its prepare and perform ran concurrently with read and write of the same hardware, users were
activated before their providers and deactivated after them on errors, joint limits allocated in
every update, and the commands of stopped, restarted or paused controllers kept being applied,
also through a provider. Async hardware wrote after a failed read, shutdown did not stop the
hardware, AUTO stopped the users of a deactivated controller instead of rejecting, and a switch
could not return while the control loop stalled.

The legacy adapter declares neither the AUTO nor the FORCE_AUTO capability, because upstream answers
both with BEST_EFFORT. Its runs skip those modes, and the power recovery restarts the startup
controllers with a STRICT switch.
