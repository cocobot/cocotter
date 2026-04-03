use std::path::PathBuf;
use std::time::{Duration, Instant};

use anyhow::{bail, Context, Result};
use bluer::gatt::remote::Characteristic;
use bluer::{AdapterEvent, Device};
use clap::Parser;
use futures::StreamExt;
use indicatif::{ProgressBar, ProgressStyle};
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use uuid::Uuid;

// OTA BLE UUIDs — must match code/libs/ble/src/ota.rs
const OTA_SERVICE: Uuid = Uuid::from_u128(0x81872000_ffa5_4969_9ab4_e777ca411f95);

fn ota_char(target: u8) -> Uuid {
    Uuid::from_u128(0x81872001_ffa5_4969_9ab4_e777ca411f95_u128 + (target as u128) * (1u128 << 96))
}

#[derive(Parser)]
#[command(about = "Flash firmware over BLE OTA")]
struct Args {
    /// galipeur | pami[0-3] | picotter | <BLE device name>
    device: String,
    /// Firmware .bin file
    firmware: PathBuf,
    /// OTA target index (auto-detected from device preset)
    #[arg(short, long)]
    target: Option<u8>,
    /// Don't reboot after successful flash
    #[arg(long)]
    no_reboot: bool,
    /// BLE scan timeout in seconds
    #[arg(long, default_value_t = 10)]
    timeout: u64,
}

/// Resolve device preset to (BLE name, OTA target index).
fn resolve(name: &str) -> (String, u8) {
    match name.to_ascii_lowercase().as_str() {
        "galipeur" => ("Galipeur".into(), 0),
        "pami" | "pami0" => ("PAMI0".into(), 0),
        "pami1" => ("PAMI1".into(), 0),
        "pami2" => ("PAMI2".into(), 0),
        "pami3" => ("PAMI3".into(), 0),
        "picotter" => ("Galipeur".into(), 1),
        _ => (name.into(), 0),
    }
}

/// Find a GATT characteristic by UUID on a connected device.
async fn find_char(device: &Device, uuid: Uuid) -> Result<Characteristic> {
    for svc in device.services().await? {
        if svc.uuid().await? == OTA_SERVICE {
            for ch in svc.characteristics().await? {
                if ch.uuid().await? == uuid {
                    return Ok(ch);
                }
            }
        }
    }
    bail!("OTA characteristic not found (UUID {uuid})");
}

#[tokio::main]
async fn main() -> Result<()> {
    let args = Args::parse();
    let (ble_name, def_target) = resolve(&args.device);
    let target = args.target.unwrap_or(def_target);
    let chr_uuid = ota_char(target);

    // Read firmware
    let fw = std::fs::read(&args.firmware)
        .with_context(|| format!("reading {}", args.firmware.display()))?;
    let crc = crc32fast::hash(&fw);
    eprintln!(
        "{} — {} bytes, CRC32: 0x{crc:08x}",
        args.firmware.display(),
        fw.len()
    );

    // BlueZ session
    let session = bluer::Session::new().await?;
    let adapter = session.default_adapter().await?;
    adapter.set_powered(true).await?;

    // Scan for device
    eprintln!("Scanning for '{ble_name}'...");
    let addr = {
        let stream = adapter.discover_devices().await?;
        futures::pin_mut!(stream);
        let deadline = Instant::now() + Duration::from_secs(args.timeout);
        loop {
            let remaining = deadline.saturating_duration_since(Instant::now());
            if remaining.is_zero() {
                bail!("'{ble_name}' not found (timeout {}s)", args.timeout);
            }
            match tokio::time::timeout(remaining, stream.next()).await {
                Ok(Some(AdapterEvent::DeviceAdded(addr))) => {
                    let dev = adapter.device(addr)?;
                    if dev.name().await?.as_deref() == Some(ble_name.as_str()) {
                        break addr;
                    }
                }
                Ok(Some(_)) => {}
                Ok(None) | Err(_) => bail!("'{ble_name}' not found"),
            }
        }
    };

    // Connect
    let device = adapter.device(addr)?;
    eprintln!("Connecting to {addr}...");
    device.connect().await?;

    // Wait for services resolved
    {
        let deadline = Instant::now() + Duration::from_secs(10);
        while !device.is_services_resolved().await? {
            if Instant::now() > deadline {
                bail!("service discovery timeout");
            }
            tokio::time::sleep(Duration::from_millis(100)).await;
        }
    }

    // Find OTA characteristic and acquire I/O channels
    let chr = find_char(&device, chr_uuid).await?;
    let mut write_io = chr.write_io().await?;
    let write_mtu = write_io.mtu();
    let mut notify_io = chr.notify_io().await?;
    let notify_mtu = notify_io.mtu();

    let chunk = write_mtu.saturating_sub(5).max(20); // 5 bytes OTA header
    eprintln!("MTU write={write_mtu} notify={notify_mtu}, chunk={chunk} bytes");

    // --- OTA protocol ---

    // START [0x01, size:u32le, crc:u32le]
    let mut buf = vec![0x01u8];
    buf.extend_from_slice(&(fw.len() as u32).to_le_bytes());
    buf.extend_from_slice(&crc.to_le_bytes());
    write_io.write_all(&buf).await?;
    eprintln!("START sent, waiting for READY...");

    let rsp = recv(&mut notify_io, notify_mtu, 15).await?;
    anyhow::ensure!(rsp.first() == Some(&0x81), "expected READY, got {rsp:02x?}");
    match rsp.get(1) {
        Some(0) => {}
        Some(1) => bail!("device busy"),
        Some(2) => bail!("not enough space"),
        other => bail!("unexpected READY status: {other:?}"),
    }
    eprintln!("Device ready.");

    // DATA [0x02, offset:u32le, data...]
    let pb = ProgressBar::new(fw.len() as u64);
    pb.set_style(
        ProgressStyle::default_bar()
            .template("{bar:40.green} {bytes}/{total_bytes} {bytes_per_sec} [{eta}]")
            .unwrap(),
    );

    let mut offset = 0usize;
    while offset < fw.len() {
        let end = (offset + chunk).min(fw.len());
        let mut pkt = Vec::with_capacity(5 + end - offset);
        pkt.push(0x02);
        pkt.extend_from_slice(&(offset as u32).to_le_bytes());
        pkt.extend_from_slice(&fw[offset..end]);
        write_io.write_all(&pkt).await?;

        let ack = recv(&mut notify_io, notify_mtu, 10).await?;
        anyhow::ensure!(ack.first() == Some(&0x82), "expected ACK, got {ack:02x?}");
        offset = u32::from_le_bytes([ack[1], ack[2], ack[3], ack[4]]) as usize;
        pb.set_position(offset as u64);
    }
    pb.finish();

    // FINISH [0x03]
    write_io.write_all(&[0x03]).await?;
    eprintln!("Verifying...");

    let res = recv(&mut notify_io, notify_mtu, 30).await?;
    anyhow::ensure!(res.first() == Some(&0x83), "expected RESULT, got {res:02x?}");
    match res.get(1) {
        Some(0) => eprintln!("Flash OK!"),
        Some(1) => bail!("verification failed: CRC mismatch"),
        Some(2) => bail!("verification failed: size mismatch"),
        other => bail!("flash error: status {other:?}"),
    }

    // REBOOT [0x04]
    if !args.no_reboot {
        write_io.write_all(&[0x04]).await?;
        eprintln!("Reboot sent.");
    }

    drop(write_io);
    drop(notify_io);
    device.disconnect().await?;
    Ok(())
}

async fn recv(
    reader: &mut (impl tokio::io::AsyncRead + Unpin),
    mtu: usize,
    secs: u64,
) -> Result<Vec<u8>> {
    let mut buf = vec![0u8; mtu];
    let n = tokio::time::timeout(Duration::from_secs(secs), reader.read(&mut buf))
        .await
        .context("notification timeout")?
        .context("read failed")?;
    anyhow::ensure!(n > 0, "notification stream closed");
    buf.truncate(n);
    Ok(buf)
}
