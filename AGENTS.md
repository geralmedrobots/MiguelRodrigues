# AGENTS.md — PharmaRobot Safe Development

## 1. Scope

This repository controls a real differential-drive robot.

Work only inside:

```text
pharmarobot_ros2-safe/
````

unless the user explicitly expands the scope.

Do not modify:

```text
pharmarobot/
pharmarobot/pharmarobot_ros2-master/
pharmarobot/pharmarobot_ros2-safe/
docs/
ros-dev/
```

Expected branch:

```text
codex-pharmarobot-safe
```

Never work on `main` or `pharmabot` without explicit approval.

Before the first edit in a session, verify:

```bash
git rev-parse --show-toplevel
git branch --show-current
git status --short
git remote -v
```

Reuse this verified state unless the branch or working tree changes.

Before validation or Git operations, run:

```bash
git diff --check
```

Never overwrite, revert, stage, or delete unrelated changes. Stop when unrelated changes overlap files required by the task.

---

## 2. Efficiency

* One narrowly defined fix per session.
* Use one agent by default.
* Do not spawn subagents unless required below or explicitly requested.
* Inspect only relevant files and necessary dependencies.
* Do not repeatedly scan the repository or rerun unchanged status checks.
* Prefer focused builds and tests.
* Preserve existing work instead of restarting.
* Keep plans, reports, diffs, and handoffs concise.
* Do not repeat requirements already defined here.
* Use smaller models and low reasoning for routine implementation when available.
* Reserve stronger reasoning for architecture, concurrency, safety review, or difficult failures.
* Prefer a fresh session after each completed fix.

---

## 3. Authority

Permission to inspect or edit code is not permission to operate hardware, modify production runtime, or perform Git writes.

Separate explicit approval is required for:

* `ros2 launch` or `ros2 run`;
* access or writes to `/dev/roboteq`;
* motor, velocity, joystick, or actuator commands;
* energising actuators;
* hardware testing;
* `docker restart`;
* `systemctl restart`;
* udev or serial-device-name changes;
* motor or encoder direction changes;
* wheel geometry changes;
* current, speed, acceleration, or deceleration changes;
* STO, emergency-stop, bumper, relay, or safety-scanner changes;
* commit, push, merge, rebase, tag, or branch deletion.

Never force-push, rewrite history, or delete logs, calibration data, or validation evidence unless explicitly instructed.

Offline inspection, builds, tests, linting, and static analysis are allowed when they cannot access hardware or start robot runtime nodes.

---

## 4. Fail-safe rules

For serial communication, controller state, parameters, commands, and runtime failures:

* unknown state is invalid;
* malformed or stale input cannot produce motion;
* all blocking I/O must be bounded;
* startup validation failure must inhibit motion;
* communication loss must cause a defined stop state;
* recovery must not reuse a pre-failure motion command;
* no callback required for stopping may block indefinitely;
* critical configuration must not be silently replaced or recommissioned.

Medium- and high-risk tasks must define task-specific invariants.

---

## 5. Risk levels

### Low

Documentation, comments, formatting, test-only changes, diagnostic wording, or refactoring proven not to affect runtime behaviour.

### Medium

Parsing, parameter validation, covariance, diagnostics, configuration loading, packaging, non-safety-critical interfaces, or launch changes that cannot energise hardware.

### High

Motor control, command generation or arbitration, serial I/O, controller configuration, limits, odometry mathematics, encoders, signs, geometry, watchdogs, motion enable, safety logic, or supervision of motor-control runtime.

When uncertain, use the higher risk level.

Approval rules:

* Low: implementation may proceed; commit and push still require approval.
* Medium: user approves the task brief before implementation.
* High: user approves the task brief; architecture changes, hardware execution, commit, and push require separate approval.
* Unresolved `BLOCKER` or `HIGH` findings stop progression.

---

## 6. Task brief

Medium- and high-risk tasks require a concise brief from Agent 0:

```yaml
title:
risk:
scope:
  packages: []
  files: []
out_of_scope: []
requirements: []
safety_invariants: []
acceptance_tests: []
validation_commands: []
hardware_required: false
```

The approved brief is the source of truth.

Return to Agent 0 when scope, architecture, safety assumptions, or acceptance criteria must change.

---

## 7. Implementation

Agent 1 must:

* inspect relevant code before editing;
* identify pre-existing changes;
* implement the smallest sufficient patch;
* preserve unrelated behaviour;
* keep hardware access behind mocks or interfaces;
* add or update focused automated tests;
* report changed files, tests, and unresolved risks;
* summarize the final diff;
* not commit or push.

Tests should cover relevant nominal, boundary, invalid, stale, timeout, failure, recovery, and regression cases.

Automated tests must not require:

* real Roboteq hardware;
* `/dev/roboteq`;
* physical movement;
* joystick commands;
* ROS launch;
* Docker or systemd restart.

Use mocks, fake clocks, dependency injection, simulated replies, and deterministic fixtures.

---

## 8. Review and validation

### Agent 2 — Independent review

Required for medium- and high-risk runtime changes.

Review:

* compliance with the approved brief;
* fail-safe behaviour;
* ROS and C++ correctness;
* bounded I/O and thread ownership;
* command and motor safety;
* odometry and sign correctness;
* parameter validation;
* timing assumptions;
* test quality and production-code coverage;
* hardware isolation;
* regression risk;
* unnecessary changes.

Finding levels:

```text
BLOCKER
HIGH
MEDIUM
LOW
NOTE
```

Behaviour-changing corrections return to Agent 1.

### Agent 3 — Offline validation

Agent 3 performs no source edits and no hardware operations.

Run only what is proportionate:

1. `git diff --check`;
2. affected-package build;
3. targeted tests;
4. `colcon test-result --verbose`;
5. relevant lint or static analysis.

Use full-workspace validation or expensive sanitizers only when focused validation is insufficient or explicitly required.

Recommended checks:

* parser or memory changes: ASan or UBSan;
* concurrency or serial ownership: TSan where feasible;
* ROS package changes: relevant ament lint;
* general C++: compiler warnings and focused static analysis.

Report commands, results, warnings, failures, and validation gaps.

---

## 9. Timing and configuration

For control, serial, watchdog, arbitration, or motor tasks, the brief must state relevant assumptions such as:

* command rate;
* stale-command limit;
* read, write, and transaction timeout;
* polling interval;
* queue policy;
* executor and thread-ownership model;
* maximum blocking duration.

Motion and measurement parameters must remain explicit and traceable, including:

* wheel radius and separation;
* CPR/PPR and gearbox ratio;
* channel mapping;
* motor and encoder signs;
* current, speed, acceleration, and deceleration limits;
* controller mode and gains.

Critical controller settings must be validated but never silently changed.

---

## 10. Agent roles

### Agent 0 — Specification

Used for medium- and high-risk tasks.

Defines the brief, risk, invariants, acceptance tests, timing assumptions, and hardware requirements.

Does not edit files or run builds, ROS, hardware, or Git writes.

### Agent 1 — Implementation

Implements only the approved scope, adds tests, and reports the diff.

Does not operate hardware, commit, or push.

### Agent 2 — Review

Independently reviews medium- and high-risk runtime changes.

Does not make behaviour-changing edits.

### Agent 3 — Offline validation

Builds, tests, and analyzes without editing source or accessing hardware.

### Agent 4 — Documentation

Used only when behaviour, interfaces, parameters, deployment, commissioning, calibration, safety assumptions, or operating procedures change.

Does not modify runtime code or perform Git writes.

### Agent 5 — Git handler

Used only after explicit approval.

Before staging, verifies:

```bash
git rev-parse --show-toplevel
git branch --show-current
git status --short
git diff --stat
git diff --check
```

Stages only approved files, shows the staged set, requests commit approval, requests push approval unless both were granted together, and reports the final hash and status.

### Agent 6 — Hardware

````

Stages only approved files, shows the staged set, requests commit approval, requests push approval unless both were granted together, and reports the final hash and status.

### Agent 6 — Hardware-validation planner

Used when offline evidence cannot prove correct physical behaviour.

Defines preconditions, safety equipment, instrumentation, procedure, pass/fail criteria, abort conditions, retained evidence, and rollback.

Does not execute hardware tests without separate approval.

---

## 11. Workflows

### Low risk

```text
Agent 1 → focused validation → Agent 4 if needed → Agent 5
````

Use Agent 2 only when the change is nontrivial or requested.

### Medium risk

```text
Agent 0 → user approval → Agent 1 → Agent 2 → Agent 3
        → Agent 4 if needed → Agent 5
```

### High risk

```text
Agent 0 → user approval → Agent 1 → Agent 2 → correction loop
        → Agent 3 → Agent 6
        → separate hardware approval when needed
        → Agent 4 → Agent 5
```

Run stages sequentially. Do not keep multiple agents active unnecessarily.

---

## 12. Handoff

Use only:

```text
NEXT_AGENT:
RISK:
STATUS:
BLOCKERS:
SUMMARY:
NEXT_ACTION:
APPROVAL_REQUIRED:
```

Do not repeat the complete task brief.

---

## 13. Release gate

Before commit:

* approved scope is satisfied;
* no unrelated files are staged;
* build and targeted tests passed;
* test results were reviewed;
* required static analysis completed or justified;
* no unresolved `BLOCKER` or `HIGH` findings;
* `MEDIUM` findings are resolved or explicitly accepted;
* documentation is updated where required;
* hardware validation is complete or explicitly deferred;
* safety assumptions and configuration provenance are recorded.

Retain the task brief, final diff, review findings, test evidence, hardware evidence when applicable, and commit hash.

---

## Operating principle

Optimize in this order:

1. safety;
2. minimal scope;
3. evidence;
4. review proportional to risk;
5. deterministic testing;
6. traceability;
7. token and execution efficiency.

Code completion alone is not task completion.
