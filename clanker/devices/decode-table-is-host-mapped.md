# The decode table is host-mapped and never written in place

## Status: landed 2026-10-24 (`src/ppmem/devmap.rs`)

`Physical::device_map`, the 64 KB-slot array of `*const dyn BusDevice`, is
gone. The table is a `SparseDeviceMap`: a 256 MB `AddrSpace` reservation
covering 40 bits of physical address, mapped page by page from shared
constant pages (one per device pointer) and a pool of mixed pages. Design
record: `docs/devmap-design.md`.

## Rules

- **Change entries only through `fill` / `fill_bytes`.** Every table view is
  read-only; a store through `base()` segfaults. `fill` builds the new page
  off-line and swaps it in, which is what keeps concurrent DMA dispatch
  safe. Do not add a `*mut` path.
- **Set the default before anything can dispatch.** `set_default` creates
  the constant page the fault handler maps. Without it a fault on an
  untouched page prints "no default device" and chains to the previous
  handler, which kills the process. `Physical::init` does this first.
- **A partial write to an untouched page pads with the default.** Writing a
  device into a region nobody filled gives you that device there and the
  default in the rest of the 256 MB page. That is what a 40-bit machine
  wants; it means `fill` on an untouched page fails if no default is set.
- **Pages are 64 KB on every host** (`granularity().max(64 KB)`), so one
  table page covers 256 MB of guest space and every mapping is one page.
  That is the Windows constraint (placeholders split on 64 KB, views cannot
  be partially unmapped) applied everywhere, so Linux exercises the same
  shape Windows needs.
- **Compare entries with `entry_eq`, not `addr_eq`.** `UnmappedRam` is
  zero-sized and shares its address with the next field of `Physical`; only
  the vtable half of the fat pointer tells them apart. `addr_eq` would call
  it equal to `AliasBus`.
- **The fault handler must stay allocation-free.** It takes the table's
  spin lock (bounded spin, gives up with a message), maps one page, writes
  one line with `write(2)`. Keep it that way; a `println!` or `Vec` in there
  is a deadlock waiting for a fault on the thread that holds the lock.
- **Debuggers stop on SIGSEGV.** `handle SIGSEGV nostop noprint pass` (and
  `SIGBUS`) in gdb. Only faults above 4 GB reach it on an Indy, so this
  rarely matters until the Octane.

## Not done

- The `BusDevice` trait takes `u64` (2026-10-26) and `Physical` masks to
  `PHYS_MASK` (40 bits). `TranslateResult.phys` is `u64` too (2026-10-26),
  so the CPU reaches the bus with full physical addresses. Above 4 GB:
  - jitv2's `PfnMap` covers only the 32-bit space; `jitv2::pfn_of` returns
    `None` above it, `jitv2_track_pcp` leaves `pcp` null, and `step_jit`
    interprets. Code there is never compiled.
  - ppmem's window and bitmap cover 4 GB. Test the bitmap with
    `ppmem::bitmap_maps`, never `1 << (phys >> BITMAP_SHIFT)`: the shift
    overflows above 4 GB (wraps to region 0 in release).
  - `nutlb_fill` declines pages at or above 4 GB, because the JIT's inline
    `emit_tc_mapped` uses that wrapping shift. Those accesses take the
    callout.
  - LLAddr is `u64`, PA[PA_BITS-1:4] masked by `CpuModel::LLADDR_MASK`
    (PA_BITS 36 on R4x00/R5000, 40 on the R10000), on LL and on (D)MTC0.
  IP32 `Ip32Bus` still truncates to u32 and models the PCI high window and
  the no-ECC RAM alias folded into the low 4 GB; those can now be decoded
  at their real addresses.
- Windows backend written against the API, not run: same status as
  `map_windows.rs`. The devmap unit tests are the acceptance test.
- Shared pages are never freed (bounded by the number of distinct devices);
  pool pages are. Fine for now.
