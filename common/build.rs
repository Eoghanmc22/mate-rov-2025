use vergen_gix::{Emitter, GixBuilder};

fn main() -> anyhow::Result<()> {
    let gitcl = GixBuilder::all_git()?;
    Emitter::default().add_instructions(&gitcl)?.emit()?;
    Ok(())
}
