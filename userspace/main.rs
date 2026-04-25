use std::fs::OpenOptions;
use std::io::{Read, Write};
use std::thread::sleep;
use std::time::Duration;

const DEVICE:      &str = "/dev/myleds";
const POLL_MS:     u64  = 100;

// speed to duty cycle mapping
// speed = presses per 10 seconds
// max useful speed = 20 alternating presses in 10s = fast
fn speed_to_duty(speed: i32) -> (i32, i32)
{
    // clamp speed 0..20
    let s = speed.max(0).min(20);

    // L1 scales from 10% to 100% across full speed range
    // L2 only lights up above half speed (10+ presses)
    let duty_l1: i32;
    let duty_l2: i32;

    if s == 0 {
        // no presses - L1 minimum, L2 off
        duty_l1 = 10;
        duty_l2 = 0;
    } else if s <= 10 {
        // low speed - L1 scales 10-55%, L2 off
        duty_l1 = 10 + (s * 45 / 10);
        duty_l2 = 0;
    } else {
        // high speed - L1 at 100%, L2 scales 0-100%
        duty_l1 = 100;
        duty_l2 = (s - 10) * 100 / 10;
    }

    (duty_l1, duty_l2)
}

fn read_status() -> Option<(i32, i32, i32)>
{
    let mut file = OpenOptions::new()
        .read(true)
        .open(DEVICE)
        .ok()?;

    let mut buf = String::new();
    file.read_to_string(&mut buf).ok()?;

    // parse "speed=N duty_l1=N duty_l2=N"
    let mut speed  = 0i32;
    let mut d_l1   = 0i32;
    let mut d_l2   = 0i32;

    for part in buf.trim().split_whitespace()
    {
        let kv: Vec<&str> = part.split('=').collect();
        if kv.len() != 2 { continue; }

        match kv[0] {
            "speed"   => speed = kv[1].parse().unwrap_or(0),
            "duty_l1" => d_l1  = kv[1].parse().unwrap_or(0),
            "duty_l2" => d_l2  = kv[1].parse().unwrap_or(0),
            _         => {}
        }
    }

    Some((speed, d_l1, d_l2))
}

fn write_duty(duty_l1: i32, duty_l2: i32)
{
    let mut file = OpenOptions::new()
        .write(true)
        .open(DEVICE)
        .expect("Failed to open device for write");

    let cmd = format!("duty_l1={} duty_l2={}", duty_l1, duty_l2);
    file.write_all(cmd.as_bytes())
        .expect("Failed to write duty cycle");
}

fn main()
{
    println!("RPi LED Speed Indicator");
    println!("Device:   {}", DEVICE);
    println!("Poll:     {}ms", POLL_MS);
    println!("Speed mapping:");
    println!("  0 presses/10s  -> L1=10%  L2=off");
    println!("  10 presses/10s -> L1=55%  L2=off");
    println!("  20 presses/10s -> L1=100% L2=100%");
    println!("---");
    println!("{:>6}  {:>12}  {:>8}  {:>8}  {:>8}  {:>8}",
             "iter", "speed(/10s)", "new_l1%", "new_l2%",
             "drv_l1%", "drv_l2%");

    // verify device exists
    if !std::path::Path::new(DEVICE).exists() {
        println!("ERROR: {} not found", DEVICE);
        println!("Load the driver: sudo insmod project.ko");
        return;
    }

    let mut iter = 0u64;

    loop
    {
        iter += 1;

        // read current state from driver
        match read_status()
        {
            Some((speed, drv_l1, drv_l2)) =>
            {
                // calculate desired duty cycles from speed
                let (new_l1, new_l2) = speed_to_duty(speed);

                // write new duty cycles to driver
                write_duty(new_l1, new_l2);

                println!("{:>6}  {:>12}  {:>8}  {:>8}  {:>8}  {:>8}",
                         iter, speed, new_l1, new_l2, drv_l1, drv_l2);
            }
            None =>
            {
                println!("iter={} ERROR: failed to read device", iter);
            }
        }

        sleep(Duration::from_millis(POLL_MS));
    }
}