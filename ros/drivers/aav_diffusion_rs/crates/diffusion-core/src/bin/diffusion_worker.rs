//! diffusion_worker: the diffusion planner's model worker in Rust, a drop-in for
//! `aav_diffusion_planner/worker.py` (the node's `worker:=rust`; launch
//! `diffusion_impl:=rust`). No ROS: it talks only to the node, over its stdin and stdout.
//!
//! Reads the settings line (the model, the provider), then each request; writes a reply
//! for each (`diffusion_core::wire`). Exits when stdin closes. Messages go to stderr.
//! A provider that can't load the model, or loads it and can't run it: the CPU from then
//! on, as the Python worker does (that plan's reply says the model failed: the node takes
//! the lattice's path).

use std::io::{BufReader, BufWriter};
use std::time::Instant;

use diffusion_core::{wire, Model, Provider};

fn main() {
    if let Err(e) = serve() {
        eprintln!("diffusion worker (rust): {e}");
        std::process::exit(1);
    }
}

fn open(model: &str, provider: Provider) -> diffusion_core::Result<Model> {
    match Model::open(model, provider) {
        Err(e) if provider != Provider::Cpu => {
            eprintln!("diffusion worker (rust): {provider} can't load it ({e}); the CPU");
            Model::open(model, Provider::Cpu)
        }
        other => other,
    }
}

fn serve() -> diffusion_core::Result<()> {
    // Ctrl-C reaches the whole process group: the node closes the pipe, and that ends us.
    let mut input = BufReader::new(std::io::stdin().lock());
    let mut output = BufWriter::new(std::io::stdout().lock());
    let Some(settings) = wire::read_settings(&mut input)? else {
        return Ok(());
    };
    let provider = Provider::from_device(&settings.provider).unwrap_or_else(|e| {
        eprintln!("diffusion worker (rust): {e}; the CPU");
        Provider::Cpu
    });
    let start = Instant::now();
    let mut model = open(&settings.model, provider)?;
    let cover = model
        .coverage()
        .map(|c| format!(", {} of {} nodes", c.taken, c.nodes))
        .unwrap_or_default();
    eprintln!(
        "diffusion worker (rust): {} on {}{cover}, loaded in {:.1} s",
        settings.model,
        model.provider(),
        start.elapsed().as_secs_f64()
    );
    while let Some(request) = wire::read_request(&mut input)? {
        let t0 = Instant::now();
        let paths = match model.sample(&request.crop, &request.cond, &request.noise) {
            Ok(paths) => Some(paths),
            Err(e) => {
                eprintln!("diffusion worker (rust): the model failed: {e}");
                if model.provider() != Provider::Cpu {
                    model = Model::open(&settings.model, Provider::Cpu)?;
                    eprintln!("diffusion worker (rust): on the CPU from now on");
                }
                None
            }
        };
        let ms = t0.elapsed().as_secs_f32() * 1000.0;
        wire::write_reply(&mut output, request.seq, paths.as_deref(), ms)?;
    }
    Ok(())
}
