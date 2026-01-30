use crate::error::BackendError;
use anyhow::{anyhow, Context};
use serde_json::{json, Value};
use std::{
    fs::{File, OpenOptions},
    io::{BufWriter, Write},
    path::Path,
    time::Instant,
};

pub struct EventWriter {
    writer: BufWriter<File>,
    version: String,
    run_id: String,
    t0: Instant,
    seq: u64,
}

impl EventWriter {
    pub fn new(trace_path: impl AsRef<Path>) -> Result<Self, BackendError> {
        let version = std::env::var("ORACLE_VERSION").unwrap_or_else(|_| "0.1".to_string());
        let run_id = std::env::var("ORACLE_RUN_ID").unwrap_or_else(|_| "run_local".to_string());

        let file = OpenOptions::new()
            .create(true)
            .append(true)
            .open(trace_path.as_ref())
            .with_context(|| format!("open trace {:?}", trace_path.as_ref()))
            .map_err(|e| BackendError::system(e))?;

        Ok(Self {
            writer: BufWriter::new(file),
            version,
            run_id,
            t0: Instant::now(),
            seq: 0,
        })
    }

    fn t_ns(&self) -> u64 {
        let ns = self.t0.elapsed().as_nanos();
        ns.min(u64::MAX as u128) as u64
    }

    pub fn emit(&mut self, mut ev: Value) -> Result<(), BackendError> {
        let obj = ev
            .as_object_mut()
            .ok_or_else(|| BackendError::system(anyhow!("internal: event must be an object")))?;

        obj.entry("version").or_insert(json!(self.version));
        obj.entry("run_id").or_insert(json!(self.run_id));
        obj.entry("t_ns").or_insert(json!(self.t_ns()));
        obj.entry("sequence").or_insert(json!(self.seq));
        self.seq += 1;

        serde_json::to_writer(&mut self.writer, &ev)
            .context("serialize event")
            .map_err(|e| BackendError::system(e))?;
        self.writer
            .write_all(b"\n")
            .context("write newline")
            .map_err(|e| BackendError::system(e))?;
        Ok(())
    }

    pub fn flush(&mut self) -> Result<(), BackendError> {
        self.writer
            .flush()
            .context("flush trace")
            .map_err(|e| BackendError::system(e))
    }
}
