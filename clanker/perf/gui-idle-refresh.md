# Status-bar-only heartbeat refresh skips the compose, and iris-gui uploads
# only the damaged scanlines

When `fb_dirty` is false but the idle heartbeat fires (~10 Hz), REX3 sets
`screen.status_bar_only` and capture renderers skip `SwCompositor` / full GL
compose — only the status bar would be redrawn.

iris-gui bakes no status bar into its frames, so its renderers return early on
`status_bar_only` and publish nothing: the GUI keeps the texture it has and
goes back to sleep. `status_bar_only` is computed with `!frame_request`, so a
`request_frame` (a screenshot, a VNC `fresh_frame`) always forces a full
refresh and cannot be starved by the heartbeat case.

Full-frame work still runs whenever the guest draws to the framebuffer or
palette/cursor state changes — and then the GUI uploads only the band of
scanlines that actually differs from the frame on screen
(`gui/gui-frame-sink-carries-only-the-damaged-band.md`).
