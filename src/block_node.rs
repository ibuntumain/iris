//! Layered block-node interface for the disk backends.
//!
//! This module is the single block-device abstraction — [`BlockNode`] — with
//! one implementation per disk backend ([`RawNode`], [`CowNode`], [`ChdNode`],
//! [`ChdCdNode`]). It replaces the former `crate::scsi::DiskBackend` enum:
//! callers hold a `Box<dyn BlockNode>` and never match on a backend kind.
//! SCSI stays LBA- and sector-oriented at the top; a node is byte-addressed and
//! owns the backend-specific arithmetic.
//!
//! The shape follows QEMU's block graph: every image is a node with an optional
//! `backing` node, and the one query that makes copy-on-write consistent across
//! formats is [`BlockNode::block_status`] — "does this layer hold these bytes,
//! and if not, where do I read them from?". Commit / rollback / dirty-count are
//! methods on that same contract.
//!
//! Nodes are byte-addressed and operate on byte ranges. The raw and overlay
//! layers use 512-byte sectors internally; reads and writes may be unaligned,
//! and a partial-sector write copies the rest of the sector up through the
//! backing chain first (read-modify-write), exactly as a real overlay must.
//!
//! # Dirty, commit, and rollback
//!
//! A node may sit over an optional *divergence layer*: a sparse `.overlay` file
//! beside a raw base, or a MAME `.diff.chd` beside a CHD base. Writes land in
//! that layer and reads fall through it. **Dirty has exactly one meaning across
//! every backend: the node holds writes that are not yet part of its durable
//! base.** Precisely, a node is dirty when its divergence layer holds at least
//! one sector that shadows the base it was opened over.
//!
//! [`BlockNode::dirty_count`] reports the size of that divergence in the units
//! the backend tracks. A raw overlay tracks its sectors exactly, so it returns
//! the number of dirty 512-byte sectors. A CHD diff is only exposed by MAME as a
//! whole-image flag, not a per-sector map, so it returns `1` while any change is
//! held and `0` otherwise. Callers that need the predicate rather than a size
//! test `dirty_count() > 0`, or [`BlockNode::is_cow`] for "has a divergence
//! layer at all".
//!
//! [`BlockNode::commit`] folds the layer into the base — raw copies its dirty
//! sectors into the base in place; CHD rebuilds the base from the merged
//! parent+diff view — and leaves the node with an empty layer.
//! [`BlockNode::rollback`] discards the layer, reverting reads to the base. A
//! node with no divergence layer has nothing to fold or discard, so the trait
//! defaults are no-ops.

use std::collections::HashSet;
use std::fs::{File, OpenOptions};
use std::io::{self, Read, Seek, SeekFrom, Write};
use std::path::{Path, PathBuf};

use crate::chd_disk::{ChdCd, ChdHd};

/// Sector granularity shared by the raw and overlay layers. Matches the SCSI
/// disk sector size and [`crate::cow_disk`]'s private constant.
pub const SECTOR_BYTES: usize = 512;
/// [`SECTOR_BYTES`] as a `u64`, for offset arithmetic.
pub const SECTOR_SIZE: u64 = SECTOR_BYTES as u64;

/// What one layer knows about the allocation of a byte range.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BlockState {
    /// The range is allocated in this layer and holds real data.
    Data,
    /// The range is not held by this layer and there is no backing to fall
    /// through to, so it reads as zeroes.
    Zero,
    /// The range is not held by this layer; read it from [`BlockNode::backing`]
    /// at [`BlockStatus::backing_offset`].
    Allocated,
}

/// The answer to a [`BlockNode::block_status`] query.
///
/// A status describes a homogeneous run beginning at the queried offset, so the
/// answer always makes sense even when the requested range straddles an
/// allocation boundary: `len` is the number of bytes from the query offset for
/// which this `state` holds. Callers that need a whole range covered iterate
/// `offset += status.len` until they have consumed it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BlockStatus {
    /// The allocation state of the run.
    pub state: BlockState,
    /// For [`BlockState::Allocated`], the byte offset to read from in the
    /// backing node. `None` for `Data` and `Zero`.
    pub backing_offset: Option<u64>,
    /// Bytes from the query offset for which `state` holds. Never zero for a
    /// non-empty query.
    pub len: u64,
}

impl BlockStatus {
    /// A run of real data held by this layer.
    pub const fn data(len: u64) -> Self {
        Self { state: BlockState::Data, backing_offset: None, len }
    }

    /// A run that reads as zeroes because no layer holds it.
    pub const fn zero(len: u64) -> Self {
        Self { state: BlockState::Zero, backing_offset: None, len }
    }

    /// A run served by the backing node at `backing_offset`.
    pub const fn allocated(backing_offset: u64, len: u64) -> Self {
        Self { state: BlockState::Allocated, backing_offset: Some(backing_offset), len }
    }
}

/// A byte-addressed node in a layered block graph.
///
/// Implementations are the raw image, the copy-on-write overlay, the hard-disk
/// CHD and the CD CHD. A node owns its storage; it is `Send` (the SCSI worker
/// already moves its backend across threads) but not `Sync`.
///
/// See the module docs for the single definition of dirty, commit and rollback
/// that every backend shares.
pub trait BlockNode: Send {
    /// Capacity in bytes.
    fn size(&self) -> u64;

    /// Read exactly `buf.len()` bytes starting at byte `offset`.
    fn read(&mut self, offset: u64, buf: &mut [u8]) -> io::Result<()>;

    /// Write exactly `buf.len()` bytes starting at byte `offset`.
    fn write(&mut self, offset: u64, buf: &[u8]) -> io::Result<()>;

    /// Push buffered writes out to durable storage.
    fn flush(&mut self) -> io::Result<()>;

    /// Discard `len` bytes at `offset`, releasing the layer's claim on them.
    /// A later read falls through to the backing node (or zeroes, if none).
    fn discard(&mut self, offset: u64, len: u64) -> io::Result<()>;

    /// Report the allocation state of `[offset, offset + len)`.
    fn block_status(&self, offset: u64, len: u64) -> io::Result<BlockStatus>;

    /// The node that serves ranges this layer does not hold, if any.
    fn backing(&self) -> Option<&dyn BlockNode>;

    /// Size of this node's uncommitted divergence, in the units the backend
    /// tracks: exact dirty 512-byte sectors for a raw overlay, or a coarse
    /// `1`/`0` for a CHD diff. `0` when nothing is uncommitted or there is no
    /// divergence layer. See the module docs for the one dirty meaning.
    fn dirty_count(&self) -> usize {
        0
    }

    /// Whether this node writes through a divergence layer (a raw `.overlay` or
    /// a CHD `.diff.chd`) that [`BlockNode::commit`] or
    /// [`BlockNode::rollback`] can act on. `false` for a plainly-written image.
    fn is_cow(&self) -> bool {
        false
    }

    /// Fold this node's divergence into its durable base and empty the layer.
    /// Returns the number of dirty units applied (raw: sectors; CHD: `1` when a
    /// diff was folded, `0` when there was nothing to fold). A no-op for a node
    /// with no divergence layer.
    fn commit(&mut self) -> io::Result<usize> {
        Ok(0)
    }

    /// Discard this node's divergence, reverting reads to the base. A no-op for
    /// a node with no divergence layer.
    fn rollback(&mut self) -> io::Result<()> {
        Ok(())
    }

    /// `(base, overlay)` paths for this node's divergence layer, if any.
    fn overlay_paths(&self) -> Option<(PathBuf, PathBuf)> {
        None
    }

    /// `(base, overlay)` when a clean exit should auto-fold this node's diff
    /// back into the base. A CHD in copy-on-write mode keeps changes separate
    /// and returns `None`; a compressed CHD with COW off returns `Some`. Always
    /// `None` for a node with no divergence layer.
    fn pending_sync(&self) -> Option<(PathBuf, PathBuf)> {
        None
    }

    /// Copy this node's divergence layer to `dest` and return its dirty sector
    /// set (sorted), for snapshot capture. Raw overlays only; other backends
    /// leave no file and return an empty list.
    fn export_overlay(&mut self, _dest: &Path) -> io::Result<Vec<u64>> {
        Ok(Vec::new())
    }

    /// Replace this node's divergence layer with `source` and adopt `dirty` as
    /// its sector set, for snapshot restore. Raw overlays only; a no-op
    /// elsewhere.
    fn import_overlay(&mut self, _source: &Path, _dirty: Vec<u64>) -> io::Result<()> {
        Ok(())
    }
}

fn invalid(msg: &str) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidInput, msg.to_string())
}

/// Check that `[offset, offset + len)` lies within a node of `size` bytes.
fn check_range(size: u64, offset: u64, len: u64) -> io::Result<()> {
    let end = offset
        .checked_add(len)
        .ok_or_else(|| invalid("block range overflows u64"))?;
    if end > size {
        return Err(invalid("block range past end of node"));
    }
    Ok(())
}

/// The sector range `[first, last)` covering `[offset, offset + len)`.
fn sectors_for(offset: u64, len: u64) -> (u64, u64) {
    let first = offset / SECTOR_SIZE;
    let end = offset + len;
    let last = (end + SECTOR_SIZE - 1) / SECTOR_SIZE;
    (first, last)
}

/// A plain raw image file: every byte is real data held by this layer.
pub struct RawNode {
    file: File,
    size: u64,
}

impl RawNode {
    /// Open `path` read-write and use its current length as the capacity.
    pub fn open(path: &Path) -> io::Result<Self> {
        let file = OpenOptions::new().read(true).write(true).open(path)?;
        let size = file.metadata()?.len();
        Ok(Self { file, size })
    }

    /// Open `path` read-only. Used for removable read-only media (CD-ROM ISOs)
    /// and for a copy-on-write base, whose contents the overlay never mutates
    /// through this node.
    pub fn open_readonly(path: &Path) -> io::Result<Self> {
        let file = OpenOptions::new().read(true).open(path)?;
        let size = file.metadata()?.len();
        Ok(Self { file, size })
    }

    /// Wrap an already-open file handle. `size` is the node's capacity; the
    /// caller has the file positioned/opened as it wants.
    pub fn from_file(file: File, size: u64) -> Self {
        Self { file, size }
    }
}

impl BlockNode for RawNode {
    fn size(&self) -> u64 {
        self.size
    }

    fn read(&mut self, offset: u64, buf: &mut [u8]) -> io::Result<()> {
        check_range(self.size, offset, buf.len() as u64)?;
        self.file.seek(SeekFrom::Start(offset))?;
        self.file.read_exact(buf)
    }

    fn write(&mut self, offset: u64, buf: &[u8]) -> io::Result<()> {
        check_range(self.size, offset, buf.len() as u64)?;
        self.file.seek(SeekFrom::Start(offset))?;
        self.file.write_all(buf)
    }

    fn flush(&mut self) -> io::Result<()> {
        self.file.sync_all()
    }

    fn discard(&mut self, offset: u64, len: u64) -> io::Result<()> {
        check_range(self.size, offset, len)?;
        // Raw images do not track allocation, so a discard is a zero-fill: the
        // bytes are still "data" to this layer, they just read as zeroes.
        let zeros = vec![0u8; len as usize];
        self.write(offset, &zeros)
    }

    fn block_status(&self, offset: u64, len: u64) -> io::Result<BlockStatus> {
        check_range(self.size, offset, len)?;
        Ok(BlockStatus::data(len))
    }

    fn backing(&self) -> Option<&dyn BlockNode> {
        None
    }
}

/// A copy-on-write overlay: writes are redirected to a sparse overlay file and
/// unheld ranges read through to the backing node.
pub struct CowNode {
    backing: Option<Box<dyn BlockNode>>,
    /// Base image path, so [`CowNode::commit`] can merge the overlay back into
    /// it. Derived from a `.overlay` suffix unless set explicitly.
    base_path: Option<PathBuf>,
    overlay: File,
    overlay_path: PathBuf,
    dirty: HashSet<u64>,
    size: u64,
}

/// The base image path implied by a `….overlay` path, if any.
fn derive_base_path(overlay_path: &Path) -> Option<PathBuf> {
    overlay_path
        .to_str()
        .and_then(|s| s.strip_suffix(".overlay"))
        .map(PathBuf::from)
}

impl CowNode {
    /// Open (creating if needed) an overlay at `overlay_path` presenting `size`
    /// bytes over `backing`. The overlay starts with no held sectors, so every
    /// range reads through to the backing until first written.
    pub fn new(
        backing: Option<Box<dyn BlockNode>>,
        overlay_path: impl AsRef<Path>,
        size: u64,
    ) -> io::Result<Self> {
        let overlay_path = overlay_path.as_ref().to_path_buf();
        let overlay = OpenOptions::new()
            .read(true)
            .write(true)
            .create(true)
            .open(&overlay_path)?;
        // Recover the dirty set persisted by a previous run. The sidecar is the
        // load-bearing record of which sectors the host finished writing —
        // sparse bytes in the overlay alone cannot be trusted.
        let sidecar = crate::cow_disk::sidecar_path_for(&overlay_path);
        let dirty = crate::cow_disk::load_dirty_sidecar(&sidecar).unwrap_or_default();
        let base_path = derive_base_path(&overlay_path);
        let base_label = base_path
            .as_deref()
            .map(|p| p.display().to_string())
            .unwrap_or_else(|| "<backing>".to_string());
        eprintln!(
            "iris: COW overlay active (base: {}, overlay: {}, dirty sectors: {})",
            base_label,
            overlay_path.display(),
            dirty.len()
        );
        if dirty.is_empty() && std::fs::metadata(&overlay_path).map(|m| m.len()).unwrap_or(0) > 0 {
            eprintln!("iris: note: overlay file has data but no .dirty sidecar — prior writes are not in use");
        }
        eprintln!(
            "iris: to reset disk to clean state, delete {} and {}",
            overlay_path.display(),
            sidecar.display()
        );
        Ok(Self { backing, base_path, overlay, overlay_path, dirty, size })
    }

    /// Record the base image path explicitly. `add_device` knows it; `new`
    /// otherwise derives it from the `.overlay` suffix.
    pub fn with_base_path(mut self, base_path: impl Into<PathBuf>) -> Self {
        self.base_path = Some(base_path.into());
        self
    }

    /// Path of the overlay file (for diagnostics and, later, commit/export).
    pub fn overlay_path(&self) -> &Path {
        &self.overlay_path
    }

    fn sidecar_path(&self) -> PathBuf {
        crate::cow_disk::sidecar_path_for(&self.overlay_path)
    }

    /// Read one whole sector: the overlay if it holds it, else the backing, else
    /// zeroes.
    fn read_sector(&mut self, lba: u64) -> io::Result<[u8; SECTOR_BYTES]> {
        let mut buf = [0u8; SECTOR_BYTES];
        if self.dirty.contains(&lba) {
            self.overlay.seek(SeekFrom::Start(lba * SECTOR_SIZE))?;
            self.overlay.read_exact(&mut buf)?;
        } else if let Some(backing) = self.backing.as_mut() {
            backing.read(lba * SECTOR_SIZE, &mut buf)?;
        }
        Ok(buf)
    }

    /// The state of one sector, ignoring the requested range.
    fn sector_state(&self, lba: u64) -> BlockState {
        if self.dirty.contains(&lba) {
            BlockState::Data
        } else if self.backing.is_some() {
            BlockState::Allocated
        } else {
            BlockState::Zero
        }
    }
}

impl Drop for CowNode {
    fn drop(&mut self) {
        if let Err(e) = self.flush() {
            eprintln!(
                "iris: COW flush on drop failed for {}: {} (writes may be lost)",
                self.overlay_path.display(),
                e
            );
        }
    }
}

impl BlockNode for CowNode {
    fn size(&self) -> u64 {
        self.size
    }

    fn read(&mut self, offset: u64, buf: &mut [u8]) -> io::Result<()> {
        check_range(self.size, offset, buf.len() as u64)?;
        if buf.is_empty() {
            return Ok(());
        }
        let end = offset + buf.len() as u64;
        let (first, last) = sectors_for(offset, buf.len() as u64);
        for lba in first..last {
            let sector = self.read_sector(lba)?;
            let s_off = lba * SECTOR_SIZE;
            let start = offset.max(s_off);
            let stop = end.min(s_off + SECTOR_SIZE);
            let src = (start - s_off) as usize;
            let dst = (start - offset) as usize;
            let n = (stop - start) as usize;
            buf[dst..dst + n].copy_from_slice(&sector[src..src + n]);
        }
        Ok(())
    }

    fn write(&mut self, offset: u64, buf: &[u8]) -> io::Result<()> {
        check_range(self.size, offset, buf.len() as u64)?;
        if buf.is_empty() {
            return Ok(());
        }
        let (first, last) = sectors_for(offset, buf.len() as u64);
        for lba in first..last {
            // Copy-on-write does read-modify-write: pull the current sector up
            // through the chain so the untouched bytes survive, then hand the
            // whole sector to the overlay.
            let mut sector = self.read_sector(lba)?;
            let s_off = lba * SECTOR_SIZE;
            let start = offset.max(s_off);
            let stop = (offset + buf.len() as u64).min(s_off + SECTOR_SIZE);
            let src = (start - offset) as usize;
            let dst = (start - s_off) as usize;
            let n = (stop - start) as usize;
            sector[dst..dst + n].copy_from_slice(&buf[src..src + n]);

            self.overlay.seek(SeekFrom::Start(lba * SECTOR_SIZE))?;
            self.overlay.write_all(&sector)?;
            self.dirty.insert(lba);
        }
        Ok(())
    }

    fn flush(&mut self) -> io::Result<()> {
        self.overlay.sync_all()?;
        crate::cow_disk::save_dirty_sidecar(&self.sidecar_path(), &self.dirty)?;
        if let Some(backing) = self.backing.as_mut() {
            backing.flush()?;
        }
        Ok(())
    }

    fn discard(&mut self, offset: u64, len: u64) -> io::Result<()> {
        check_range(self.size, offset, len)?;
        if len == 0 {
            return Ok(());
        }
        let (first, last) = sectors_for(offset, len);
        for lba in first..last {
            self.dirty.remove(&lba);
        }
        Ok(())
    }

    fn block_status(&self, offset: u64, len: u64) -> io::Result<BlockStatus> {
        check_range(self.size, offset, len)?;
        if len == 0 {
            return Ok(BlockStatus::data(0));
        }
        let (first, _) = sectors_for(offset, len);
        let state = self.sector_state(first);
        // Extend the run while the next sector has the same state and still
        // overlaps the requested range.
        let end = offset + len;
        let mut lba = first;
        loop {
            let next = lba + 1;
            if next * SECTOR_SIZE >= end || self.sector_state(next) != state {
                break;
            }
            lba = next;
        }
        let run_end = ((lba + 1) * SECTOR_SIZE).min(end);
        let covered = run_end - offset;
        Ok(match state {
            BlockState::Data => BlockStatus::data(covered),
            BlockState::Zero => BlockStatus::zero(covered),
            BlockState::Allocated => BlockStatus::allocated(offset, covered),
        })
    }

    fn backing(&self) -> Option<&dyn BlockNode> {
        self.backing.as_deref()
    }

    fn dirty_count(&self) -> usize {
        self.dirty.len()
    }

    fn is_cow(&self) -> bool {
        true
    }

    /// Merge every dirty overlay sector into the base image, then truncate the
    /// overlay so it is empty. Mirrors [`crate::cow_disk::CowDisk::commit`].
    fn commit(&mut self) -> io::Result<usize> {
        let base_path = self.base_path.clone().ok_or_else(|| {
            io::Error::new(
                io::ErrorKind::Other,
                "cannot determine base path from overlay path",
            )
        })?;
        let mut base_rw = OpenOptions::new().read(true).write(true).open(&base_path)?;
        let mut buf = [0u8; SECTOR_BYTES];
        let mut committed = 0usize;
        for &lba in &self.dirty {
            self.overlay.seek(SeekFrom::Start(lba * SECTOR_SIZE))?;
            self.overlay.read_exact(&mut buf)?;
            base_rw.seek(SeekFrom::Start(lba * SECTOR_SIZE))?;
            base_rw.write_all(&buf)?;
            committed += 1;
        }
        base_rw.sync_all()?;
        self.dirty.clear();
        self.overlay.set_len(0)?;
        eprintln!("iris: COW committed {} sectors to {}", committed, base_path.display());
        Ok(committed)
    }

    /// Delete every overlay write and clear the on-disk dirty set. Mirrors
    /// [`crate::cow_disk::CowDisk::reset_overlay`].
    fn rollback(&mut self) -> io::Result<()> {
        self.dirty.clear();
        self.overlay.set_len(0)?;
        self.overlay.seek(SeekFrom::Start(0))?;
        let _ = std::fs::remove_file(self.sidecar_path());
        Ok(())
    }

    /// Copy the current overlay to `dest` and return the dirty sector list
    /// (sorted). Mirrors [`crate::cow_disk::CowDisk::export_overlay`].
    fn export_overlay(&mut self, dest: &Path) -> io::Result<Vec<u64>> {
        self.overlay.sync_all()?;
        crate::cow_disk::reflink_or_copy(&self.overlay_path, dest)?;
        let mut dirty: Vec<u64> = self.dirty.iter().copied().collect();
        dirty.sort_unstable();
        Ok(dirty)
    }

    /// Replace the overlay contents with `source` and adopt `dirty`. Mirrors
    /// [`crate::cow_disk::CowDisk::import_overlay`].
    fn import_overlay(&mut self, source: &Path, dirty: Vec<u64>) -> io::Result<()> {
        if source.exists() {
            crate::cow_disk::reflink_or_copy(source, &self.overlay_path)?;
        } else {
            std::fs::File::create(&self.overlay_path)?;
        }
        self.overlay = OpenOptions::new()
            .read(true)
            .write(true)
            .create(true)
            .open(&self.overlay_path)?;
        self.dirty = dirty.into_iter().collect();
        Ok(())
    }
}

/// A hard-disk CHD. The underlying [`ChdHd`] already presents the merged
/// parent + diff view, so this layer holds every byte it reports — there is no
/// backing node.
pub struct ChdNode {
    /// The CHD handle. `None` only transiently, while [`BlockNode::commit`] or
    /// [`BlockNode::rollback`] has closed it so `flatten_diff` can atomically
    /// rebuild the base; it is restored before the operation returns.
    hd: Option<ChdHd>,
    sector_size: u64,
}

impl ChdNode {
    /// Open a hard-disk CHD. `cow` is the per-disk copy-on-write toggle, threaded
    /// straight through to [`ChdHd::open`].
    pub fn open(path: &str, cow: bool) -> io::Result<Self> {
        let hd = ChdHd::open(path, cow)?;
        let sector_size = u64::from(hd.sector_size());
        Ok(Self { hd: Some(hd), sector_size })
    }

    fn hd(&self) -> &ChdHd {
        self.hd.as_ref().expect("CHD handle present outside commit/rollback")
    }

    fn hd_mut(&mut self) -> &mut ChdHd {
        self.hd.as_mut().expect("CHD handle present outside commit/rollback")
    }
}

impl BlockNode for ChdNode {
    fn size(&self) -> u64 {
        self.hd().size()
    }

    fn read(&mut self, offset: u64, buf: &mut [u8]) -> io::Result<()> {
        let ss = self.sector_size;
        if offset % ss != 0 || (buf.len() as u64) % ss != 0 {
            return Err(invalid("CHD reads must be sector-aligned"));
        }
        check_range(self.hd().size(), offset, buf.len() as u64)?;
        let data = self.hd_mut().read_blocks(offset / ss, buf.len() / (ss as usize), ss)?;
        buf.copy_from_slice(&data);
        Ok(())
    }

    fn write(&mut self, offset: u64, buf: &[u8]) -> io::Result<()> {
        let ss = self.sector_size;
        if offset % ss != 0 || (buf.len() as u64) % ss != 0 {
            return Err(invalid("CHD writes must be sector-aligned"));
        }
        check_range(self.hd().size(), offset, buf.len() as u64)?;
        self.hd_mut().write_sectors(offset / ss, buf)
    }

    fn flush(&mut self) -> io::Result<()> {
        // CHD sector writes go straight to the image (or its diff); there is no
        // user-space write-back cache to drain.
        Ok(())
    }

    fn discard(&mut self, offset: u64, len: u64) -> io::Result<()> {
        check_range(self.hd().size(), offset, len)?;
        let zeros = vec![0u8; len as usize];
        self.write(offset, &zeros)
    }

    fn block_status(&self, offset: u64, len: u64) -> io::Result<BlockStatus> {
        check_range(self.hd().size(), offset, len)?;
        Ok(BlockStatus::data(len))
    }

    fn backing(&self) -> Option<&dyn BlockNode> {
        None
    }

    fn dirty_count(&self) -> usize {
        usize::from(self.hd().diff_dirty())
    }

    fn is_cow(&self) -> bool {
        self.hd().overlay_paths().is_some()
    }

    fn overlay_paths(&self) -> Option<(PathBuf, PathBuf)> {
        self.hd().overlay_paths()
    }

    fn pending_sync(&self) -> Option<(PathBuf, PathBuf)> {
        self.hd().pending_sync()
    }

    /// Rebuild the base from the merged parent+diff view and reopen a fresh
    /// overlay. The handle is closed first — `flatten_diff` atomically renames
    /// the rebuilt base — and reopened afterwards, so the node stays usable. On
    /// a flatten error the base and diff are left intact and the diff is
    /// reattached.
    fn commit(&mut self) -> io::Result<usize> {
        if !self.hd().diff_dirty() {
            return Ok(0);
        }
        let (base, diff) = self
            .hd()
            .overlay_paths()
            .expect("a dirty CHD reports overlay paths");
        let cow = self.hd().is_cow();
        let base_str = base
            .to_str()
            .ok_or_else(|| invalid("non-UTF-8 CHD path"))?
            .to_string();

        self.hd = None; // close so flatten_diff can rename over the base
        let folded = crate::chd_disk::flatten_diff(&base, &diff, &mut |_| {}, &|| false);
        // base + diff are intact even when the fold failed, so either way the
        // disk can be reopened; surface the fold error after restoring the node.
        self.hd = Some(ChdHd::open(&base_str, cow)?);
        folded?;
        Ok(1)
    }

    /// Discard the diff (every uncommitted write) and reopen a fresh overlay
    /// over the untouched base. A CHD with no diff has nothing to roll back.
    fn rollback(&mut self) -> io::Result<()> {
        let Some((base, diff)) = self.hd().overlay_paths() else {
            return Ok(());
        };
        let cow = self.hd().is_cow();
        let base_str = base
            .to_str()
            .ok_or_else(|| invalid("non-UTF-8 CHD path"))?
            .to_string();

        self.hd = None; // close before reopening over the (untouched) base
        let _ = std::fs::remove_file(&diff);
        self.hd = Some(ChdHd::open(&base_str, cow)?);
        Ok(())
    }
}

/// A CD-ROM CHD (single-track MODE1). The cooked reader presents a flat,
/// byte-addressed, read-only stream — writes are rejected. It holds every byte
/// it reports, so there is no backing node.
pub struct ChdCdNode {
    reader: ChdCd,
    size: u64,
}

impl ChdCdNode {
    /// Open a CD CHD read-only.
    pub fn open(path: &str) -> io::Result<Self> {
        let reader = ChdCd::open(path)?;
        let size = reader.size();
        Ok(Self { reader, size })
    }
}

impl BlockNode for ChdCdNode {
    fn size(&self) -> u64 {
        self.size
    }

    fn read(&mut self, offset: u64, buf: &mut [u8]) -> io::Result<()> {
        check_range(self.size, offset, buf.len() as u64)?;
        self.reader.read_at(offset, buf)
    }

    fn write(&mut self, _offset: u64, _buf: &[u8]) -> io::Result<()> {
        Err(io::Error::new(io::ErrorKind::PermissionDenied, "CD CHD is read-only"))
    }

    fn flush(&mut self) -> io::Result<()> {
        Ok(())
    }

    fn discard(&mut self, _offset: u64, _len: u64) -> io::Result<()> {
        Err(io::Error::new(io::ErrorKind::PermissionDenied, "CD CHD is read-only"))
    }

    fn block_status(&self, offset: u64, len: u64) -> io::Result<BlockStatus> {
        check_range(self.size, offset, len)?;
        Ok(BlockStatus::data(len))
    }

    fn backing(&self) -> Option<&dyn BlockNode> {
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::{AtomicU64, Ordering};

    fn tmp_path(tag: &str) -> PathBuf {
        static N: AtomicU64 = AtomicU64::new(0);
        std::env::temp_dir().join(format!(
            "iris-blocknode-{}-{}-{}",
            tag,
            std::process::id(),
            N.fetch_add(1, Ordering::Relaxed)
        ))
    }

    fn raw_filled(byte: u8, len: usize) -> (RawNode, PathBuf) {
        let path = tmp_path("raw");
        std::fs::write(&path, vec![byte; len]).unwrap();
        (RawNode::open(&path).unwrap(), path)
    }

    /// Build a 256 KiB compressed hard-disk CHD full of `byte`, and return its
    /// path. MAME's v5 compression map asserts below one hunk, so keep it large.
    fn compressed_chd(byte: u8) -> PathBuf {
        use libchdman_rs::hd::{create_from_reader, HdCreateOptions};
        use libchdman_rs::CHD_CODEC_ZLIB;
        use std::io::Cursor;

        let base = tmp_path("chd").with_extension("chd");
        let logical = 256 * 1024u64;
        create_from_reader(
            Cursor::new(vec![byte; logical as usize]),
            &base,
            HdCreateOptions {
                logical_size: logical,
                hunk_size: 4096,
                unit_size: SECTOR_BYTES as u32,
                codecs: [CHD_CODEC_ZLIB, 0, 0, 0],
                geometry: None,
                ident: None,
            },
            &mut |_| {},
            &|| false,
        )
        .unwrap();
        base
    }

    fn cleanup(overlay: &Path) {
        let _ = std::fs::remove_file(overlay);
        let _ = std::fs::remove_file(crate::cow_disk::sidecar_path_for(overlay));
    }

    #[test]
    fn raw_roundtrips_and_reports_data() {
        let (mut node, path) = raw_filled(0x00, 4 * SECTOR_BYTES);

        node.write(SECTOR_SIZE, &[0x11u8; SECTOR_BYTES]).unwrap();
        let mut got = [0u8; SECTOR_BYTES];
        node.read(SECTOR_SIZE, &mut got).unwrap();
        assert_eq!(got, [0x11u8; SECTOR_BYTES]);

        let status = node.block_status(SECTOR_SIZE, SECTOR_SIZE).unwrap();
        assert_eq!(status.state, BlockState::Data);
        assert_eq!(status.backing_offset, None);
        assert_eq!(status.len, SECTOR_SIZE);
        assert!(node.backing().is_none(), "a raw image has no backing");

        node.discard(SECTOR_SIZE, SECTOR_SIZE).unwrap();
        node.read(SECTOR_SIZE, &mut got).unwrap();
        assert_eq!(got, [0x00u8; SECTOR_BYTES], "discard zero-fills a raw node");

        node.flush().unwrap();
        let _ = std::fs::remove_file(path);
    }

    #[test]
    fn raw_rejects_ranges_past_the_end() {
        let (mut node, path) = raw_filled(0, 2 * SECTOR_BYTES);
        let mut buf = [0u8; SECTOR_BYTES];
        assert!(node.read(SECTOR_SIZE, &mut buf).is_ok());
        assert!(node.read(2 * SECTOR_SIZE, &mut buf).is_err());
        assert!(node.block_status(SECTOR_SIZE, SECTOR_SIZE + 1).is_err());
        let _ = std::fs::remove_file(path);
    }

    #[test]
    fn cow_unwritten_range_reads_through_to_backing() {
        let (backing, backing_path) = raw_filled(0xAA, 4 * SECTOR_BYTES);
        let overlay_path = tmp_path("cow-overlay");
        let mut node =
            CowNode::new(Some(Box::new(backing)), &overlay_path, 4 * SECTOR_SIZE).unwrap();

        assert!(node.backing().is_some(), "the overlay sits on a backing node");
        assert_eq!(node.dirty_count(), 0);

        // An unwritten range reads through to the backing and reports that it is
        // allocated somewhere below this layer.
        let mut got = [0u8; SECTOR_BYTES];
        node.read(SECTOR_SIZE, &mut got).unwrap();
        assert_eq!(got, [0xAAu8; SECTOR_BYTES]);
        let status = node.block_status(SECTOR_SIZE, SECTOR_SIZE).unwrap();
        assert_eq!(status.state, BlockState::Allocated);
        assert_eq!(status.backing_offset, Some(SECTOR_SIZE));
        assert_eq!(status.len, SECTOR_SIZE);

        let _ = std::fs::remove_file(backing_path);
        let _ = std::fs::remove_file(overlay_path);
    }

    #[test]
    fn cow_written_range_reads_overlay_and_reports_data() {
        let (backing, backing_path) = raw_filled(0xAA, 4 * SECTOR_BYTES);
        let overlay_path = tmp_path("cow-overlay");
        let mut node =
            CowNode::new(Some(Box::new(backing)), &overlay_path, 4 * SECTOR_SIZE).unwrap();

        node.write(SECTOR_SIZE, &[0x55u8; SECTOR_BYTES]).unwrap();
        assert_eq!(node.dirty_count(), 1);

        let mut got = [0u8; SECTOR_BYTES];
        node.read(SECTOR_SIZE, &mut got).unwrap();
        assert_eq!(got, [0x55u8; SECTOR_BYTES], "the overlay shadows the backing");
        let status = node.block_status(SECTOR_SIZE, SECTOR_SIZE).unwrap();
        assert_eq!(status.state, BlockState::Data);
        assert_eq!(status.backing_offset, None);

        // A neighbouring range is still served by the backing.
        node.read(2 * SECTOR_SIZE, &mut got).unwrap();
        assert_eq!(got, [0xAAu8; SECTOR_BYTES]);

        let _ = std::fs::remove_file(backing_path);
        let _ = std::fs::remove_file(overlay_path);
    }

    #[test]
    fn cow_discard_releases_the_overlay_range() {
        let (backing, backing_path) = raw_filled(0xAA, 4 * SECTOR_BYTES);
        let overlay_path = tmp_path("cow-overlay");
        let mut node =
            CowNode::new(Some(Box::new(backing)), &overlay_path, 4 * SECTOR_SIZE).unwrap();

        node.write(SECTOR_SIZE, &[0x55u8; SECTOR_BYTES]).unwrap();
        node.discard(SECTOR_SIZE, SECTOR_SIZE).unwrap();
        assert_eq!(node.dirty_count(), 0);

        let mut got = [0u8; SECTOR_BYTES];
        node.read(SECTOR_SIZE, &mut got).unwrap();
        assert_eq!(got, [0xAAu8; SECTOR_BYTES], "discard falls back to the backing");
        let status = node.block_status(SECTOR_SIZE, SECTOR_SIZE).unwrap();
        assert_eq!(status.state, BlockState::Allocated);
        assert_eq!(status.backing_offset, Some(SECTOR_SIZE));

        let _ = std::fs::remove_file(backing_path);
        let _ = std::fs::remove_file(overlay_path);
    }

    #[test]
    fn cow_without_backing_reports_zero_and_zero_fills() {
        let overlay_path = tmp_path("cow-nobacking");
        let mut node = CowNode::new(None, &overlay_path, 2 * SECTOR_SIZE).unwrap();
        assert!(node.backing().is_none());

        let mut got = [0xFFu8; SECTOR_BYTES];
        node.read(0, &mut got).unwrap();
        assert_eq!(got, [0x00u8; SECTOR_BYTES], "no backing and no overlay reads zeroes");
        let status = node.block_status(0, SECTOR_SIZE).unwrap();
        assert_eq!(status.state, BlockState::Zero);
        assert_eq!(status.backing_offset, None);

        node.write(0, &[0x33u8; SECTOR_BYTES]).unwrap();
        assert_eq!(node.block_status(0, SECTOR_SIZE).unwrap().state, BlockState::Data);

        node.discard(0, SECTOR_SIZE).unwrap();
        assert_eq!(node.block_status(0, SECTOR_SIZE).unwrap().state, BlockState::Zero);

        let _ = std::fs::remove_file(overlay_path);
    }

    #[test]
    fn cow_range_straddling_allocated_and_data_reports_first_run() {
        let (backing, backing_path) = raw_filled(0xAA, 4 * SECTOR_BYTES);
        let overlay_path = tmp_path("cow-overlay");
        let mut node =
            CowNode::new(Some(Box::new(backing)), &overlay_path, 4 * SECTOR_SIZE).unwrap();

        // Sector 1 lives in the overlay; sectors 0 and 2 do not.
        node.write(SECTOR_SIZE, &[0x55u8; SECTOR_BYTES]).unwrap();

        let mut got = [0u8; 3 * SECTOR_BYTES];
        node.read(0, &mut got).unwrap();
        assert_eq!(&got[0..SECTOR_BYTES], &[0xAAu8; SECTOR_BYTES]);
        assert_eq!(&got[SECTOR_BYTES..2 * SECTOR_BYTES], &[0x55u8; SECTOR_BYTES]);
        assert_eq!(&got[2 * SECTOR_BYTES..], &[0xAAu8; SECTOR_BYTES]);

        let first = node.block_status(0, 3 * SECTOR_SIZE).unwrap();
        assert_eq!(first.state, BlockState::Allocated);
        assert_eq!(first.len, SECTOR_SIZE, "the run stops at the overlay boundary");

        // Stepping by `status.len` reaches the held sector.
        let second = node.block_status(first.len, 2 * SECTOR_SIZE).unwrap();
        assert_eq!(second.state, BlockState::Data);
        assert_eq!(second.len, SECTOR_SIZE);

        let _ = std::fs::remove_file(backing_path);
        let _ = std::fs::remove_file(overlay_path);
    }

    #[test]
    fn cow_partial_sector_write_preserves_neighbours() {
        let (backing, backing_path) = raw_filled(0xAA, 2 * SECTOR_BYTES);
        let overlay_path = tmp_path("cow-overlay");
        let mut node =
            CowNode::new(Some(Box::new(backing)), &overlay_path, 2 * SECTOR_SIZE).unwrap();

        // Overwrite 100 bytes in the middle of sector 0.
        node.write(100, &[0x11u8; 100]).unwrap();
        let mut got = [0u8; SECTOR_BYTES];
        node.read(0, &mut got).unwrap();
        assert_eq!(&got[0..100], &[0xAAu8; 100]);
        assert_eq!(&got[100..200], &[0x11u8; 100]);
        assert_eq!(&got[200..], &[0xAAu8; SECTOR_BYTES - 200]);

        let _ = std::fs::remove_file(backing_path);
        let _ = std::fs::remove_file(overlay_path);
    }

    #[test]
    fn chd_node_reads_writes_and_reports_data() {
        use libchdman_rs::hd::{create_from_reader, HdCreateOptions};
        use libchdman_rs::CHD_CODEC_ZLIB;
        use std::io::Cursor;

        let base = tmp_path("chd").with_extension("chd");
        // MAME's v5 compression map asserts on a logical size smaller than one
        // hunk, so use the same 256 KiB the chd_disk tests use.
        let logical = 256 * 1024u64;
        create_from_reader(
            Cursor::new(vec![0xABu8; logical as usize]),
            &base,
            HdCreateOptions {
                logical_size: logical,
                hunk_size: 4096,
                unit_size: SECTOR_BYTES as u32,
                codecs: [CHD_CODEC_ZLIB, 0, 0, 0],
                geometry: None,
                ident: None,
            },
            &mut |_| {},
            &|| false,
        )
        .unwrap();

        let mut node = ChdNode::open(base.to_str().unwrap(), false).unwrap();
        assert_eq!(node.size(), logical);
        assert!(node.backing().is_none(), "a CHD presents the merged view itself");

        let mut got = [0u8; SECTOR_BYTES];
        node.read(SECTOR_SIZE, &mut got).unwrap();
        assert_eq!(got, [0xABu8; SECTOR_BYTES]);
        assert_eq!(
            node.block_status(SECTOR_SIZE, SECTOR_SIZE).unwrap().state,
            BlockState::Data
        );

        node.write(SECTOR_SIZE, &[0x5Au8; SECTOR_BYTES]).unwrap();
        node.read(SECTOR_SIZE, &mut got).unwrap();
        assert_eq!(got, [0x5Au8; SECTOR_BYTES]);
        node.flush().unwrap();

        drop(node);
        let _ = std::fs::remove_file(&base);
        let _ = std::fs::remove_file(crate::chd_disk::diff_path_for(&base));
    }

    #[test]
    fn cow_commit_merges_dirty_into_base_and_clears_overlay() {
        let (backing, backing_path) = raw_filled(0xAA, 4 * SECTOR_BYTES);
        let overlay_path = tmp_path("cow-overlay");
        let mut node = CowNode::new(Some(Box::new(backing)), &overlay_path, 4 * SECTOR_SIZE)
            .unwrap()
            .with_base_path(&backing_path);

        node.write(SECTOR_SIZE, &[0x55u8; SECTOR_BYTES]).unwrap();
        assert_eq!(node.dirty_count(), 1);

        let committed = node.commit().unwrap();
        assert_eq!(committed, 1);
        assert_eq!(node.dirty_count(), 0);
        assert_eq!(
            node.block_status(SECTOR_SIZE, SECTOR_SIZE).unwrap().state,
            BlockState::Allocated,
            "after commit the sector is served by the base again"
        );

        // The base file on disk now holds the write...
        let on_disk = std::fs::read(&backing_path).unwrap();
        assert_eq!(&on_disk[SECTOR_BYTES..2 * SECTOR_BYTES], &[0x55u8; SECTOR_BYTES]);
        // ...and the node reads it back through the backing.
        let mut got = [0u8; SECTOR_BYTES];
        node.read(SECTOR_SIZE, &mut got).unwrap();
        assert_eq!(got, [0x55u8; SECTOR_BYTES]);

        node.flush().unwrap();
        let _ = std::fs::remove_file(&backing_path);
        cleanup(&overlay_path);
    }

    #[test]
    fn cow_reset_discards_overlay_and_reverts_reads() {
        let (backing, backing_path) = raw_filled(0xAA, 2 * SECTOR_BYTES);
        let overlay_path = tmp_path("cow-overlay");
        let mut node = CowNode::new(Some(Box::new(backing)), &overlay_path, 2 * SECTOR_SIZE)
            .unwrap()
            .with_base_path(&backing_path);

        node.write(0, &[0x11u8; SECTOR_BYTES]).unwrap();
        assert_eq!(node.dirty_count(), 1);
        node.rollback().unwrap();
        assert_eq!(node.dirty_count(), 0);

        let mut got = [0u8; SECTOR_BYTES];
        node.read(0, &mut got).unwrap();
        assert_eq!(got, [0xAAu8; SECTOR_BYTES], "reset falls back to the base");

        node.flush().unwrap();
        let _ = std::fs::remove_file(&backing_path);
        cleanup(&overlay_path);
    }

    #[test]
    fn cow_dirty_set_survives_reopen_via_sidecar() {
        let (backing, backing_path) = raw_filled(0xAA, 4 * SECTOR_BYTES);
        let overlay_path = tmp_path("cow-overlay");
        {
            let mut node =
                CowNode::new(Some(Box::new(backing)), &overlay_path, 4 * SECTOR_SIZE).unwrap();
            node.write(SECTOR_SIZE, &[0x55u8; SECTOR_BYTES]).unwrap();
            node.flush().unwrap(); // persists the .dirty sidecar
        }

        let backing2 = RawNode::open(&backing_path).unwrap();
        let mut node =
            CowNode::new(Some(Box::new(backing2)), &overlay_path, 4 * SECTOR_SIZE).unwrap();
        assert_eq!(node.dirty_count(), 1, "the sidecar restores the dirty set");
        let mut got = [0u8; SECTOR_BYTES];
        node.read(SECTOR_SIZE, &mut got).unwrap();
        assert_eq!(got, [0x55u8; SECTOR_BYTES], "overlay bytes are read back after reopen");

        let _ = std::fs::remove_file(&backing_path);
        cleanup(&overlay_path);
    }

    #[test]
    fn cow_export_and_import_roundtrip_the_overlay() {
        let (backing, backing_path) = raw_filled(0xAA, 2 * SECTOR_BYTES);
        let overlay_path = tmp_path("cow-overlay");
        let export_path = tmp_path("cow-export");
        let mut node =
            CowNode::new(Some(Box::new(backing)), &overlay_path, 2 * SECTOR_SIZE).unwrap();
        node.write(0, &[0x42u8; SECTOR_BYTES]).unwrap();

        let dirty = node.export_overlay(&export_path).unwrap();
        assert_eq!(dirty, vec![0u64]);
        assert!(export_path.exists(), "export writes the overlay bytes");

        // Roll back, then re-import the captured overlay.
        node.rollback().unwrap();
        assert_eq!(node.dirty_count(), 0);
        node.import_overlay(&export_path, dirty).unwrap();
        assert_eq!(node.dirty_count(), 1);
        let mut got = [0u8; SECTOR_BYTES];
        node.read(0, &mut got).unwrap();
        assert_eq!(got, [0x42u8; SECTOR_BYTES]);

        let _ = std::fs::remove_file(&backing_path);
        cleanup(&overlay_path);
        let _ = std::fs::remove_file(&export_path);
    }

    #[test]
    fn chd_node_reports_pending_sync_and_overlay_paths() {
        let base = compressed_chd(0xAB);
        let mut node = ChdNode::open(base.to_str().unwrap(), false).unwrap();

        // A compressed base cannot be written in place, so it gets a diff.
        assert!(node.overlay_paths().is_some());
        assert!(node.is_cow(), "a compressed base writes through a diff regardless of the COW flag");
        assert_eq!(node.dirty_count(), 0, "nothing written yet");
        assert!(node.pending_sync().is_none());

        node.write(SECTOR_SIZE, &[0x5Au8; SECTOR_BYTES]).unwrap();
        assert_eq!(node.dirty_count(), 1, "a CHD reports a coarse 1/0 dirtiness");
        assert!(node.pending_sync().is_some(), "COW off: the diff auto-folds on exit");

        drop(node);
        let _ = std::fs::remove_file(&base);
        let _ = std::fs::remove_file(crate::chd_disk::diff_path_for(&base));
    }

    /// The overlay contract — dirty_count / commit / rollback — is exercised
    /// through the `dyn BlockNode` the SCSI device actually holds, so no caller
    /// needs to match on a backend kind.
    #[test]
    fn cow_contract_operations_go_through_the_trait_object() {
        let (backing, backing_path) = raw_filled(0xAA, 4 * SECTOR_BYTES);
        let overlay_path = tmp_path("cow-dyn");
        let node = CowNode::new(Some(Box::new(backing)), &overlay_path, 4 * SECTOR_SIZE)
            .unwrap()
            .with_base_path(&backing_path);
        let mut node: Box<dyn BlockNode> = Box::new(node);

        assert!(node.is_cow());
        assert_eq!(node.dirty_count(), 0);

        node.write(SECTOR_SIZE, &[0x55u8; SECTOR_BYTES]).unwrap();
        assert_eq!(node.dirty_count(), 1);

        node.rollback().unwrap();
        assert_eq!(node.dirty_count(), 0);
        let mut got = [0u8; SECTOR_BYTES];
        node.read(SECTOR_SIZE, &mut got).unwrap();
        assert_eq!(got, [0xAAu8; SECTOR_BYTES], "rollback falls back to the base");

        node.write(SECTOR_SIZE, &[0x55u8; SECTOR_BYTES]).unwrap();
        assert_eq!(node.commit().unwrap(), 1);
        assert_eq!(node.dirty_count(), 0);
        let on_disk = std::fs::read(&backing_path).unwrap();
        assert_eq!(&on_disk[SECTOR_BYTES..2 * SECTOR_BYTES], &[0x55u8; SECTOR_BYTES]);

        let _ = std::fs::remove_file(&backing_path);
        cleanup(&overlay_path);
    }

    /// A CHD's commit folds the diff into the base (the flatten operation) and
    /// rollback discards it, both through the trait object and both leaving the
    /// node usable.
    #[test]
    fn chd_commit_and_rollback_go_through_the_trait_object() {
        let base = compressed_chd(0xAB);
        let mut node: Box<dyn BlockNode> =
            Box::new(ChdNode::open(base.to_str().unwrap(), false).unwrap());

        node.write(SECTOR_SIZE, &[0x5Au8; SECTOR_BYTES]).unwrap();
        assert_eq!(node.dirty_count(), 1, "a write dirties the diff");
        assert_eq!(node.commit().unwrap(), 1, "commit folds the diff (flatten)");
        assert_eq!(node.dirty_count(), 0, "the fold leaves nothing uncommitted");

        // The fold is visible through the reopened node.
        let mut got = [0u8; SECTOR_BYTES];
        node.read(SECTOR_SIZE, &mut got).unwrap();
        assert_eq!(got, [0x5Au8; SECTOR_BYTES]);

        // A second write, then a rollback, restores the folded base.
        node.write(SECTOR_SIZE, &[0x11u8; SECTOR_BYTES]).unwrap();
        assert_eq!(node.dirty_count(), 1);
        node.rollback().unwrap();
        assert_eq!(node.dirty_count(), 0);
        node.read(SECTOR_SIZE, &mut got).unwrap();
        assert_eq!(got, [0x5Au8; SECTOR_BYTES], "rollback restores the committed base");

        drop(node);
        let _ = std::fs::remove_file(&base);
        let _ = std::fs::remove_file(crate::chd_disk::diff_path_for(&base));
    }
}
