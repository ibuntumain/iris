# VNC frame capture and damage

Status: design notes for `src/vnc/`, 2026-10-29. Updated 2026-10-31 for
iris-gui's own renderers.

- The server reads only `GfxDisplay::screen().rgba` + `frame_seq`; no board
  draw path knows about it. Damage = 64x64 tiles whose pixels differ from
  the last published frame (row slices compared; per-tile generation
  numbers; viewers remember the generation they saw).
- GR2, IMPACT, O2 rewrite `rgba` every composed frame. Newport only when
  asked: `set_capture(true)` makes every full refresh read back, and forces
  the window's renderer to `SwCompositor` (`switch_compositor(false)` every
  frame is cheap and idempotent; `disp compositor gl` is refused). With no
  renderer (`--ci`) the REX3 refresh loop composes in software itself.
- **A front-end renderer replaces that Newport path.** GR2/IMPACT/O2 are
  unaffected (they own `rgba` themselves), but Newport with a renderer
  installed has nobody writing `rgba` — the board assumes the renderer
  does it. iris-gui's `CaptureRenderer`/`GlCaptureRenderer` therefore
  mirror the published band into `screen.rgba` when `need_readback` is
  true, and the whole frame when a frame was dropped (see
  `clanker/gui/gui-frame-sink-carries-only-the-damaged-band.md`).
  `need_readback` = `take_screenshot || want_frame`, so the mirror costs
  nothing until a viewer connects or a screenshot is asked for.
- `request_frame` is the one-shot version (CI screenshot, monitor `screen`):
  it does not write `screenshot_NNNN.png`, which `request_screenshot` does
  on Newport. `status_bar_only` is computed with `!frame_request`, so a
  request always forces a full refresh and cannot be starved by the
  heartbeat-only early return.
- O2 bumps `frame_seq` 60 times a second even when nothing changed; the
  tile compare then finds nothing and publishes nothing.
- An idle Newport desktop in `--ci` produces a `frame_seq` bump only on real
  change (fb/palette dirty), so a watched idle desktop costs a seq check per
  tick.
- `tools/vnc-check.py` is a minimal Python RFB client (Raw + ZRLE decode)
  for scripts; its ZRLE decode is slow (pure Python), the server is not.
