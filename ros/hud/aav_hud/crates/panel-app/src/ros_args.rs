//! The ROS arguments the node `hw_panel` gets when it runs inside another program (the
//! map window's HARDWARE page).
//!
//! A launch file names its node with `-r __node:=map`. Without a node's prefix that
//! renames every node in the process, so `hw_panel` would become a second `map`. Those
//! renames are dropped; everything else stays: the parameters files (each keyed by its
//! node's name, so `map:` reaches only the map and `hw_panel:` only the panel), `-p`,
//! the namespace and the log level. A rename aimed at `hw_panel:` itself is kept.

/// The names a node rename sets.
const RENAMES: [&str; 2] = ["__node", "__name"];

/// `args` without the renames of the host's own node. Our own options (before
/// `--ros-args`) pass through: rcl ignores them.
pub fn for_hosted_panel(args: &[String]) -> Vec<String> {
    let mut out = Vec::with_capacity(args.len());
    let mut in_ros = false;
    let mut it = args.iter();
    while let Some(arg) = it.next() {
        match arg.as_str() {
            "--ros-args" => in_ros = true,
            "--" => in_ros = false,
            "-r" | "--remap" if in_ros => {
                let Some(rule) = it.next() else {
                    out.push(arg.clone());
                    break;
                };
                if !renames_the_host(rule) {
                    out.push(arg.clone());
                    out.push(rule.clone());
                }
                continue;
            }
            _ => {}
        }
        out.push(arg.clone());
    }
    out
}

/// Whether a remap rule (`[node:]from:=to`) renames a node other than `hw_panel`.
fn renames_the_host(rule: &str) -> bool {
    let Some((from, _)) = rule.split_once(":=") else {
        return false;
    };
    let (node, name) = match from.split_once(':') {
        Some((node, name)) => (Some(node), name),
        None => (None, from),
    };
    RENAMES.contains(&name) && node != Some(panel_core::config::NODE_NAME)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn strings(list: &[&str]) -> Vec<String> {
        list.iter().copied().map(String::from).collect()
    }

    #[test]
    fn the_hosts_rename_goes_and_the_rest_stays() {
        let args = strings(&[
            "map",
            "--vehicle",
            "v.yaml",
            "--ros-args",
            "-r",
            "__node:=map",
            "--params-file",
            "map.yaml",
            "--params-file",
            "hw_panel.yaml",
            "-r",
            "/x:=/y",
            "-p",
            "theme:=latte",
        ]);
        let want = strings(&[
            "map",
            "--vehicle",
            "v.yaml",
            "--ros-args",
            "--params-file",
            "map.yaml",
            "--params-file",
            "hw_panel.yaml",
            "-r",
            "/x:=/y",
            "-p",
            "theme:=latte",
        ]);
        assert_eq!(for_hosted_panel(&args), want);
    }

    #[test]
    fn a_rename_for_the_panel_itself_stays() {
        let args = strings(&[
            "map",
            "--ros-args",
            "--remap",
            "map:__node:=big_map",
            "-r",
            "hw_panel:__node:=panel2",
            "-r",
            "__ns:=/car",
        ]);
        let want = strings(&[
            "map",
            "--ros-args",
            "-r",
            "hw_panel:__node:=panel2",
            "-r",
            "__ns:=/car",
        ]);
        assert_eq!(for_hosted_panel(&args), want);
    }

    #[test]
    fn our_own_options_are_never_read_as_ros_arguments() {
        // `-r` before --ros-args is not ROS's: left alone.
        let args = strings(&["map", "-r", "__node:=x"]);
        assert_eq!(for_hosted_panel(&args), args);
    }
}
