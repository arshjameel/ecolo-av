//! How much of a model a provider took: from ONNX Runtime's log, and from its profile.

use std::collections::BTreeMap;

use crate::Result;

/// A provider's share of the graph, as ONNX Runtime logs it when it splits the graph:
/// "number of partitions supported by CoreML: 2 number of nodes in the graph: 808 number
/// of nodes supported by CoreML: 802".
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Coverage {
    /// Nodes in the graph, after ONNX Runtime's own clean-up.
    pub nodes: usize,
    /// Nodes the provider took.
    pub taken: usize,
    /// Pieces it took them in: 1 is the whole graph at once; each extra piece is a hop
    /// between the provider and the CPU every run.
    pub partitions: usize,
}

impl Coverage {
    /// The share taken, 0 to 1 (1 for an empty graph).
    pub fn share(&self) -> f64 {
        if self.nodes == 0 {
            1.0
        } else {
            self.taken as f64 / self.nodes as f64
        }
    }

    /// Whether the provider took every node, in one piece.
    pub fn whole(&self) -> bool {
        self.taken == self.nodes && self.partitions <= 1
    }

    /// Read ONNX Runtime's line on a provider's share (Core ML's wording; the last such
    /// line if there are several), or `None` if there is none.
    pub fn from_log<S: AsRef<str>>(lines: &[S]) -> Option<Self> {
        lines.iter().rev().find_map(|l| Self::from_line(l.as_ref()))
    }

    fn from_line(line: &str) -> Option<Self> {
        let after = |key: &str| -> Option<usize> {
            let at = line.find(key)? + key.len();
            let rest = line[at..].trim_start_matches([':', ' ']);
            let end = rest
                .find(|c: char| !c.is_ascii_digit())
                .unwrap_or(rest.len());
            rest[..end].parse().ok()
        };
        Some(Self {
            partitions: after_provider(line, "number of partitions supported by")?,
            nodes: after("number of nodes in the graph")?,
            taken: after_provider(line, "number of nodes supported by")?,
        })
    }
}

/// The number after "<key> <Provider>:" (the provider's name between).
fn after_provider(line: &str, key: &str) -> Option<usize> {
    let at = line.find(key)? + key.len();
    let rest = &line[at..];
    let colon = rest.find(':')?;
    let rest = rest[colon + 1..].trim_start();
    let end = rest
        .find(|c: char| !c.is_ascii_digit())
        .unwrap_or(rest.len());
    rest[..end].parse().ok()
}

/// Which provider ran which nodes in one profiled run.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Placements {
    /// Nodes each provider ran (`CPUExecutionProvider`: 6). A provider that fuses part of
    /// the graph (Core ML, TensorRT) counts each fused piece as one node.
    pub by_provider: BTreeMap<String, usize>,
    /// The CPU's nodes by their operator (`Reshape`: 4): what fell back.
    pub cpu_ops: BTreeMap<String, usize>,
}

/// Read ONNX Runtime's profile (its JSON trace, `Session::end_profiling`'s file).
pub fn placements(trace: &str) -> Result<Placements> {
    let events: Vec<serde_json::Value> = serde_json::from_str(trace)?;
    let mut nodes: BTreeMap<String, (String, String)> = BTreeMap::new();
    for e in &events {
        if e["cat"] != "Node" {
            continue;
        }
        let args = &e["args"];
        if let (Some(name), Some(provider)) = (e["name"].as_str(), args["provider"].as_str()) {
            let op = args["op_name"].as_str().unwrap_or("?").to_owned();
            let node = name.trim_end_matches("_kernel_time").to_owned();
            nodes.insert(node, (provider.to_owned(), op));
        }
    }
    let mut out = Placements::default();
    for (provider, op) in nodes.into_values() {
        if provider == "CPUExecutionProvider" {
            *out.cpu_ops.entry(op).or_insert(0) += 1;
        }
        *out.by_provider.entry(provider).or_insert(0) += 1;
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reads_core_mls_line() {
        let lines = [
            "something else",
            "CoreMLExecutionProvider::GetCapability, number of partitions supported by \
             CoreML: 2 number of nodes in the graph: 808 number of nodes supported by \
             CoreML: 802",
        ];
        let c = Coverage::from_log(&lines).unwrap();
        assert_eq!(
            c,
            Coverage {
                nodes: 808,
                taken: 802,
                partitions: 2
            }
        );
        assert!((c.share() - 802.0 / 808.0).abs() < 1e-12);
        assert!(!c.whole());
        assert!(Coverage::from_log(&["no such line"]).is_none());
    }

    #[test]
    fn whole_is_every_node_in_one_piece() {
        let c = Coverage {
            nodes: 10,
            taken: 10,
            partitions: 1,
        };
        assert!(c.whole());
        assert_eq!(c.share(), 1.0);
    }

    #[test]
    fn counts_the_cpus_ops() {
        let trace = r#"[
          {"cat": "Session", "name": "model_run", "args": {}},
          {"cat": "Node", "name": "a_kernel_time", "args": {"provider": "CPUExecutionProvider", "op_name": "Reshape"}},
          {"cat": "Node", "name": "a_fence_before", "args": {"provider": "CPUExecutionProvider", "op_name": "Reshape"}},
          {"cat": "Node", "name": "b_kernel_time", "args": {"provider": "CPUExecutionProvider", "op_name": "Flatten"}},
          {"cat": "Node", "name": "c_kernel_time", "args": {"provider": "CoreMLExecutionProvider", "op_name": "CoreML_1"}}
        ]"#;
        let p = placements(trace).unwrap();
        assert_eq!(p.by_provider["CPUExecutionProvider"], 3);
        assert_eq!(p.by_provider["CoreMLExecutionProvider"], 1);
        assert_eq!(p.cpu_ops["Reshape"], 2);
        assert_eq!(p.cpu_ops["Flatten"], 1);
    }
}
