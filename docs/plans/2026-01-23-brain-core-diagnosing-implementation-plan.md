# Brain Core Diagnosing State Management Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Centralize brain_core state transitions so QR → BLE flow is deterministic, audio does not override diagnosing, and BLE outcomes are announced consistently.

**Architecture:** Introduce a pure, testable coordinator that owns a single mode enum and outputs declarative actions (TTS, face emotion, robot state, BLE request). The main loop executes actions and keeps all ROS calls on the single-threaded runtime.

**Tech Stack:** Rust (Tokio current_thread), r2r ROS2, robot_interfaces messages/services.

### Task 1: Add a pure coordinator + unit tests

**Files:**
- Create: `src/brain_core/src/modules/coordinator.rs`
- Modify: `src/brain_core/src/modules/mod.rs`
- Test: `src/brain_core/src/modules/coordinator.rs` (module unit tests)

**Step 1: Write the failing tests**

Add tests for the MVP flow in `src/brain_core/src/modules/coordinator.rs`:

```rust
#[cfg(test)]
mod tests {
    use super::{Coordinator, Event, Action, Mode, BleRequest};

    fn sample_payload() -> super::NeuralLinkPayload {
        super::NeuralLinkPayload {
            t: "ble".to_string(),
            m: "D6:65:62:00:2A:7E".to_string(),
            s: Some("".to_string()),
            c: Some("".to_string()),
            d: Some("0A030000".to_string()),
            n: Some("设备".to_string()),
        }
    }

    #[test]
    fn qr_from_idle_starts_ble_flow() {
        let mut coord = Coordinator::new();
        let actions = coord.on_event(Event::VisionFound(sample_payload()));
        assert!(actions.iter().any(|a| matches!(a, Action::Speak(_))));
        assert!(actions.iter().any(|a| matches!(a, Action::RequestBle(_))));
        assert_eq!(coord.mode(), Mode::BleConnecting);
    }

    #[test]
    fn qr_ignored_while_busy() {
        let mut coord = Coordinator::new();
        coord.on_event(Event::VisionFound(sample_payload()));
        let actions = coord.on_event(Event::VisionFound(sample_payload()));
        assert!(actions.is_empty());
        assert_eq!(coord.mode(), Mode::BleConnecting);
    }

    #[test]
    fn ble_success_reports_and_returns_idle() {
        let mut coord = Coordinator::new();
        coord.on_event(Event::VisionFound(sample_payload()));
        let actions = coord.on_event(Event::BleResult { success: true, message: "OK".into() });
        assert!(actions.iter().any(|a| matches!(a, Action::Speak(_))));
        assert_eq!(coord.mode(), Mode::Idle);
    }

    #[test]
    fn ble_failure_reports_and_returns_idle() {
        let mut coord = Coordinator::new();
        coord.on_event(Event::VisionFound(sample_payload()));
        let actions = coord.on_event(Event::BleResult { success: false, message: "ERR".into() });
        assert!(actions.iter().any(|a| matches!(a, Action::Speak(_))));
        assert_eq!(coord.mode(), Mode::Idle);
    }
}
```

**Step 2: Run test to verify it fails**

Run: `cd src/brain_core && cargo test coordinator`
Expected: FAIL with unresolved items (Coordinator/Mode/Action not found)

**Step 3: Write minimal implementation**

Create `Coordinator` with:
- `Mode` enum: `Idle | BleConnecting`
- `Event` enum: `VisionFound(NeuralLinkPayload) | BleResult { success, message }`
- `Action` enum: `Speak(String) | SetEmotion(String) | SetRobotState { state: String, detail: String } | RequestBle(BleRequest)`
- `BleRequest` struct with mac/service/characteristic/command
- `Coordinator::on_event` that implements the MVP flow:
  - From `Idle`, `VisionFound` → `BleConnecting`, actions: Speak("已识别到设备，开始连接"), SetEmotion("happy"), SetRobotState(BUSY), RequestBle
  - From `BleConnecting`, `BleResult` → `Idle`, actions: Speak(success/fail), SetEmotion("neutral"), SetRobotState(IDLE)
  - Other events → no actions

**Step 4: Run test to verify it passes**

Run: `cd src/brain_core && cargo test coordinator`
Expected: PASS

**Step 5: Commit**

```bash
git add src/brain_core/src/modules/coordinator.rs src/brain_core/src/modules/mod.rs
git commit -m "feat: add brain core coordinator for qr ble flow"
```

### Task 2: Wire coordinator into brain_core event loop

**Files:**
- Modify: `src/brain_core/src/main.rs`

**Step 1: Write failing test**

Add a small unit test in `src/brain_core/src/modules/coordinator.rs` to assert that `Action::RequestBle` contains the expected payload fields (mac/service/characteristic/command). This will fail until wiring uses coordinator output.

**Step 2: Run test to verify it fails**

Run: `cd src/brain_core && cargo test coordinator`
Expected: FAIL with mismatched expected action details

**Step 3: Write minimal implementation**

- Instantiate `Coordinator` in `main.rs` and move it into the event loop task.
- Replace direct BLE handling in `VisionTargetFound` with:
  - `actions = coordinator.on_event(Event::VisionFound(payload))`
  - Execute actions in order: publish TTS, set face emotion, set robot state, and call BLE service.
- When BLE service returns, send `BrainEvent::BleResult { success, message }` to the event loop.
- Keep all ROS calls on the current-thread runtime.

**Step 4: Run test to verify it passes**

Run: `cd src/brain_core && cargo test coordinator`
Expected: PASS

**Step 5: Commit**

```bash
git add src/brain_core/src/main.rs src/brain_core/src/modules/coordinator.rs
git commit -m "feat: route qr ble flow through coordinator"
```

### Task 3: Gate audio while diagnosing

**Files:**
- Modify: `src/brain_core/src/main.rs`
- Modify: `src/brain_core/src/modules/coordinator.rs`

**Step 1: Write failing test**

Add test: when `Mode::BleConnecting`, `Event::AudioFinal` returns no actions and does not change mode.

**Step 2: Run test to verify it fails**

Run: `cd src/brain_core && cargo test coordinator`
Expected: FAIL

**Step 3: Write minimal implementation**

- Extend `Event` with `AudioFinal(String)`.
- In coordinator, ignore `AudioFinal` unless in `Idle`.
- In `main.rs`, change the audio loop to send `BrainEvent::AudioFinal(text)` to the event loop instead of triggering LLM/TTS directly.
- If `AudioFinal` is accepted in `Idle`, keep current LLM flow by spawning a task that sends `BrainEvent::AudioLlmResult(answer)` back to the event loop; event loop then performs TTS and state updates.

**Step 4: Run test to verify it passes**

Run: `cd src/brain_core && cargo test coordinator`
Expected: PASS

**Step 5: Commit**

```bash
git add src/brain_core/src/main.rs src/brain_core/src/modules/coordinator.rs
git commit -m "feat: gate audio events during diagnosing"
```

### Task 4: Manual ROS2 validation

**Files:**
- No code changes

**Step 1: Build**

Run on robot: `colcon build --packages-select brain_core`

**Step 2: Run full system**

Run: `./run.sh`

**Step 3: Validate flow**

- Scan QR once: expect TTS “已识别到设备，开始连接…”, BLE attempt, and final success/failure TTS.
- Scan again during connecting: expect no duplicate BLE requests.
- Speak during BLE flow: expect no LLM response until flow completes.

**Step 4: Commit any documentation updates**

If you adjust runbook or notes, commit with `docs:` prefix.
