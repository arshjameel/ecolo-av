//! The stage spans' subscriber (TODO 196): where a tick's time goes, against its budget.
//!
//! Built in only with the feature `trace` (or `tracy`), and on only when the environment
//! says so: `AAV_TRACE=1` prints each stage's time as it closes (`time.busy`), to stderr;
//! `AAV_TRACE=tracy` sends them to a Tracy viewer (feature `tracy`).

/// Install the subscriber `AAV_TRACE` asks for; say what was chosen, or `None` for none.
#[cfg(feature = "trace")]
pub fn start() -> Option<&'static str> {
    use tracing_subscriber::fmt::format::FmtSpan;

    match std::env::var("AAV_TRACE").ok()?.as_str() {
        "" | "0" => None,
        #[cfg(feature = "tracy")]
        "tracy" => {
            use tracing_subscriber::layer::SubscriberExt as _;
            let subscriber =
                tracing_subscriber::registry().with(tracing_tracy::TracyLayer::default());
            tracing::subscriber::set_global_default(subscriber).ok()?;
            Some("tracy")
        }
        _ => {
            tracing_subscriber::fmt()
                .with_span_events(FmtSpan::CLOSE)
                .with_writer(std::io::stderr)
                .try_init()
                .ok()?;
            Some("stderr")
        }
    }
}

/// Built without the feature `trace`: no spans to time.
#[cfg(not(feature = "trace"))]
pub fn start() -> Option<&'static str> {
    None
}
