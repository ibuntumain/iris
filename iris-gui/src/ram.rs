//! RAM bank helpers shared by the Memory tab, menus, and status readouts.

/// Quick preset totals (MB) for the Memory menu and new-machine dialog.
const RAM_PRESETS: &[u32] = &[32, 64, 96, 128, 192, 256, 384, 512];

const IP28_RAM_PRESETS: &[u32] = &[32, 64, 96, 128, 192, 256, 384, 512, 768, 1024];

/// An O2's kits are 32/64/128/256 MB per DIMM pair, four pairs at most, so the
/// reachable totals are the same sums the IP28 makes — but built out of those
/// kits rather than out of MC banks.
const O2_RAM_PRESETS: &[u32] = &[32, 64, 96, 128, 192, 256, 384, 512, 768, 1024];

const OCTANE_RAM_PRESETS: &[u32] = &[64, 128, 192, 256, 384, 512, 768, 1024, 2048, 4096, 8192];

pub fn ram_presets(profile: iris::config::MachineProfile) -> &'static [u32] {
    use iris::config::MachineProfile as P;
    match profile {
        P::OctaneIp30 => OCTANE_RAM_PRESETS,
        P::Indigo2Ip28 => IP28_RAM_PRESETS,
        P::O2Ip32 => O2_RAM_PRESETS,
        _ => RAM_PRESETS,
    }
}

/// The kits an O2 takes in one DIMM pair (MB): 32, 64, 128 or 256. SGI sold no
/// others, and `iris::dev::crime_mem::check_kits` rejects anything else.
const O2_KITS_MB: &[u32] = &[256, 128, 64, 32];

/// The DIMM-pair layout that makes `total` MB on an O2: the fewest kits, largest
/// first, filled from pair A and never increasing. Rounded down to the 32 MB
/// granule, capped at four 256 MB kits (1 GB), and never below one 32 MB kit —
/// an O2 with pair A empty does not pass `validate()`.
///
/// Used when a bank layout the O2 cannot express has to be replaced (switching
/// a config over from the Indy or the Indigo2), where the alternative is a
/// Start failure naming DIMM pairs the user never chose.
pub fn o2_kits_for(total: u32) -> [u32; iris::config::MAX_BANKS] {
    let target = (total / 32 * 32).clamp(32, 1024);
    let mut left = target;
    let mut out = [0u32; iris::config::MAX_BANKS];
    for slot in out.iter_mut().take(4) {
        for &kit in O2_KITS_MB {
            if left >= kit {
                *slot = kit;
                left -= kit;
                break;
            }
        }
    }
    out
}

/// Tooltip for a bank size `profile` cannot take.
pub fn bank_size_tip(profile: iris::config::MachineProfile) -> String {
    use iris::config::MachineProfile as P;
    match profile {
        P::OctaneIp30 => t!("ram-bank-octane-sizes"),
        P::O2Ip32 => t!("ram-bank-o2-sizes"),
        _ => t!("config-ram-bank-ip28-only"),
    }
}

pub fn active_banks(banks: &[u32]) -> usize {
    banks.iter().filter(|&&s| s > 0).count()
}

pub fn ram_summary(banks: &[u32]) -> String {
    let total: u32 = banks.iter().sum();
    let n = active_banks(banks);
    t!("ram-summary", total = total, banks = n)
}
