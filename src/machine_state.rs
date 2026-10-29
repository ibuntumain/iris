//! Guest lifecycle state and monitor readiness.
//!
//! One value answers "what is the guest doing, and can I talk to it?" for the
//! monitor's `machine-start` acknowledgement, `machine-status`, and the CI
//! socket's `status` command. Keeping the state and its transitions here means
//! those three surfaces cannot report different things.

use std::fmt;
use std::net::SocketAddr;

/// Where the guest is in its start/stop lifecycle.
///
/// `Running` is derived from the CPU's own running flag, not stored: an
/// execution-control command (`start`, `stop`, a breakpoint) changes it without
/// going through a machine-level transition, and reporting a stale copy would
/// be worse than storing nothing at all.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GuestState {
    /// The machine has not been started: its peripherals are down and the
    /// monitor is not accepting commands.
    Idle,
    /// The machine is up — peripherals started and the monitor accepting
    /// commands — but the CPU is parked, waiting for `machine-start` or
    /// `start`. This is the state a `--ci` machine sits in until its harness
    /// releases the CPU.
    Starting,
    /// The CPU is executing.
    Running,
    /// The CPU was stopped by `machine-stop`, `reset`, or a guest power-off.
    /// The monitor still accepts commands.
    Halted,
}

impl GuestState {
    pub fn as_str(self) -> &'static str {
        match self {
            GuestState::Idle => "idle",
            GuestState::Starting => "starting",
            GuestState::Running => "running",
            GuestState::Halted => "halted",
        }
    }

    /// Move to `to`, refusing transitions the lifecycle does not have.
    ///
    /// Identical states are allowed so callers can apply an operation
    /// idempotently. The machine-level path is `Idle`/`Halted` -> `Starting`
    /// -> `Running`; reaching `Running` directly from `Idle` or `Halted` is a
    /// bug (it would claim the guest ran without a start), and a running guest
    /// cannot move back to `Starting` without being halted first.
    pub fn transition(self, to: GuestState) -> Result<GuestState, TransitionError> {
        use GuestState::*;
        let allowed = self == to
            || matches!(
                (self, to),
                (Idle, Starting)
                    | (Halted, Starting)
                    | (Starting, Running)
                    | (Starting, Halted)
                    | (Running, Halted)
            );
        if allowed {
            Ok(to)
        } else {
            Err(TransitionError { from: self, to })
        }
    }
}

impl fmt::Display for GuestState {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.as_str())
    }
}

/// A refused [`GuestState::transition`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TransitionError {
    pub from: GuestState,
    pub to: GuestState,
}

impl fmt::Display for TransitionError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "invalid guest state transition: {} -> {}",
            self.from.as_str(),
            self.to.as_str()
        )
    }
}

impl std::error::Error for TransitionError {}

/// What a machine-level start or stop did, for the command that asked.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TransitionAck {
    pub from: GuestState,
    /// An intermediate state the operation passed through, when it is worth
    /// showing. A start from `Idle` or `Halted` goes through `Starting`; a
    /// start from an already-`Starting` machine does not.
    pub via: Option<GuestState>,
    pub to: GuestState,
}

impl TransitionAck {
    pub fn changed(&self) -> bool {
        self.from != self.to
    }

    /// One line for a monitor client: `halted -> starting -> running`, or
    /// `running (no change)` when the operation was a no-op.
    pub fn text(&self) -> String {
        if !self.changed() {
            format!("{} (no change)", self.to.as_str())
        } else if let Some(via) = self.via {
            format!("{} -> {} -> {}", self.from.as_str(), via.as_str(), self.to.as_str())
        } else {
            format!("{} -> {}", self.from.as_str(), self.to.as_str())
        }
    }

    /// The same information for a JSON client.
    pub fn to_json(&self) -> serde_json::Value {
        serde_json::json!({
            "from": self.from.as_str(),
            "via": self.via.map(GuestState::as_str),
            "to": self.to.as_str(),
            "changed": self.changed(),
        })
    }
}

/// A snapshot of the guest's state plus whether the monitor is accepting
/// commands, produced only by `Workstation::status`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MachineStatus {
    pub state: GuestState,
    /// True while the CPU thread is executing. Always `state == Running`.
    pub cpu_running: bool,
    /// The address the monitor bound, or `None` until it is listening (or
    /// when it could not bind and is disabled for this run).
    pub monitor_addr: Option<SocketAddr>,
}

impl MachineStatus {
    pub fn monitor_ready(&self) -> bool {
        self.monitor_addr.is_some()
    }

    /// Three lines a person can read and a shell can grep.
    pub fn text(&self) -> String {
        let monitor = match self.monitor_addr {
            Some(addr) => format!("ready ({addr})"),
            None => "not ready".to_string(),
        };
        format!(
            "state:   {}\ncpu:     {}\nmonitor: {}",
            self.state.as_str(),
            if self.cpu_running { "running" } else { "stopped" },
            monitor
        )
    }

    pub fn to_json(&self) -> serde_json::Value {
        serde_json::json!({
            "state": self.state.as_str(),
            "cpu_running": self.cpu_running,
            "monitor_ready": self.monitor_ready(),
            "monitor_addr": self.monitor_addr.map(|a| a.to_string()),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn addr() -> SocketAddr {
        "127.0.0.1:8888".parse().unwrap()
    }

    /// The normal start path names all three states in the acknowledgement.
    #[test]
    fn start_from_idle_goes_through_starting_to_running() {
        let starting = GuestState::Idle.transition(GuestState::Starting).unwrap();
        let running = starting.transition(GuestState::Running).unwrap();
        assert_eq!(running, GuestState::Running);

        let ack = TransitionAck {
            from: GuestState::Idle,
            via: Some(GuestState::Starting),
            to: GuestState::Running,
        };
        assert_eq!(ack.text(), "idle -> starting -> running");
        assert!(ack.changed());
    }

    /// A second start on a running guest is a no-op, not an error and not a
    /// second `Starting` transition.
    #[test]
    fn duplicate_start_while_running_is_a_no_op() {
        let state = GuestState::Running.transition(GuestState::Running).unwrap();
        assert_eq!(state, GuestState::Running);

        let ack = TransitionAck {
            from: GuestState::Running,
            via: None,
            to: GuestState::Running,
        };
        assert!(!ack.changed());
        assert_eq!(ack.text(), "running (no change)");
    }

    /// Both a halt and a reset leave the machine at `Halted`, and a start from
    /// there is the same `halted -> starting -> running` path as an idle
    /// start.
    #[test]
    fn start_after_halt_and_after_reset() {
        let halted = GuestState::Running.transition(GuestState::Halted).unwrap();
        assert_eq!(halted, GuestState::Halted);
        assert_eq!(
            halted.transition(GuestState::Starting).unwrap().transition(GuestState::Running).unwrap(),
            GuestState::Running
        );

        // A halted machine stopped again stays halted.
        assert_eq!(halted.transition(GuestState::Halted).unwrap(), GuestState::Halted);
        let ack = TransitionAck {
            from: GuestState::Halted,
            via: Some(GuestState::Starting),
            to: GuestState::Running,
        };
        assert_eq!(ack.text(), "halted -> starting -> running");
    }

    #[test]
    fn start_from_an_already_started_machine_has_no_via() {
        let ack = TransitionAck {
            from: GuestState::Starting,
            via: None,
            to: GuestState::Running,
        };
        assert_eq!(ack.text(), "starting -> running");
    }

    /// Reaching `Running` without passing through `Starting`, or rewinding a
    /// running guest, is refused and names both ends.
    #[test]
    fn invalid_transitions_are_refused() {
        for (from, to) in [
            (GuestState::Idle, GuestState::Running),
            (GuestState::Idle, GuestState::Halted),
            (GuestState::Halted, GuestState::Running),
            (GuestState::Running, GuestState::Starting),
            (GuestState::Running, GuestState::Idle),
        ] {
            let err = from.transition(to).unwrap_err();
            assert_eq!(err.from, from);
            assert_eq!(err.to, to);
            assert_eq!(
                err.to_string(),
                format!("invalid guest state transition: {} -> {}", from.as_str(), to.as_str())
            );
        }
    }

    #[test]
    fn status_reports_running_exactly_when_the_cpu_runs() {
        let running = MachineStatus {
            state: GuestState::Running,
            cpu_running: true,
            monitor_addr: Some(addr()),
        };
        assert!(running.monitor_ready());
        assert!(running.text().contains("state:   running"));
        assert!(running.text().contains("monitor: ready (127.0.0.1:8888)"));

        let idle = MachineStatus {
            state: GuestState::Idle,
            cpu_running: false,
            monitor_addr: None,
        };
        assert!(!idle.monitor_ready());
        assert!(idle.text().contains("monitor: not ready"));
    }

    #[test]
    fn json_carries_state_and_monitor_readiness() {
        let json = MachineStatus {
            state: GuestState::Starting,
            cpu_running: false,
            monitor_addr: Some(addr()),
        }
        .to_json();
        assert_eq!(json["state"], "starting");
        assert_eq!(json["cpu_running"], false);
        assert_eq!(json["monitor_ready"], true);
        assert_eq!(json["monitor_addr"], "127.0.0.1:8888");

        let json = MachineStatus {
            state: GuestState::Idle,
            cpu_running: false,
            monitor_addr: None,
        }
        .to_json();
        assert_eq!(json["monitor_ready"], false);
        assert!(json["monitor_addr"].is_null());
    }

    #[test]
    fn ack_json_matches_ack_text() {
        let ack = TransitionAck {
            from: GuestState::Halted,
            via: Some(GuestState::Starting),
            to: GuestState::Running,
        };
        let json = ack.to_json();
        assert_eq!(json["from"], "halted");
        assert_eq!(json["via"], "starting");
        assert_eq!(json["to"], "running");
        assert_eq!(json["changed"], true);

        let ack = TransitionAck {
            from: GuestState::Running,
            via: None,
            to: GuestState::Running,
        };
        assert_eq!(ack.to_json()["changed"], false);
    }
}
