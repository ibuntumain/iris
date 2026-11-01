# iris-gui gets one winit EventLoop, and it is eframe's

Symptom: `IRIS_GUI_GL=1` always printed
`iris-gui: IRIS_GUI_GL=1 but headless GL init failed — using CPU compositor`
and fell back, on hosts where GLX/EGL were demonstrably fine.

Cause: `HeadlessGl::new()` built its hidden window through
`glutin_winit::DisplayBuilder`, which needs a `&winit::EventLoop`. winit allows
exactly one event loop **per process**, enforced in
`third_party/winit-0.30.13/src/event_loop.rs`:

```rust
static EVENT_LOOP_CREATED: AtomicBool = AtomicBool::new(false);
...
if EVENT_LOOP_CREATED.swap(true, Ordering::Relaxed) {
    return Err(EventLoopError::RecreationAttempt);
}
```

iris-gui is an eframe app, and eframe's `run_native` creates that one event loop
on the main thread for the GUI window. `HeadlessGl` runs later, on the worker
thread, so `EventLoop::new()` returned `RecreationAttempt` before GL was ever
touched. Nothing about the drivers was wrong; it could never have worked.

Rule: **do not create a `winit::EventLoop` anywhere in iris-gui.** The GUI owns
the process's one event loop; anything else that wants a window or a display
handle must get it without winit. `Cargo.toml` already says this in the
`[patch.crates-io]` comment ("The GUI owns the process's event loop; the
embedded core does not create a second window") — the code disagreed with it.

Fix: build the display directly.
`glutin::display::Display::new(RawDisplayHandle, DisplayApiPreference)` is
public and needs no event loop, and `DisplayBuilder::build` only used the event
loop for two things — `glutin_display_handle()` and `finalize_window()` — so
neither was essential. Ask for the platform's *default* display
(`XlibDisplayHandle::new(None, 0)` on Linux, which `raw-window-handle`
documents as "request the default display when using EGL") and render into a
pbuffer. `GlCompositor` draws into its own FBO and reads that back, so it never
needed a window.

Verified on Mesa/Intel HD 4400: EGL 1.5 display, GL 4.6 (Core Profile), pbuffer
+ context + `make_current` all succeed. Prefer `DisplayApiPreference::Egl` on
Linux — GLX would need a real Xlib `Display` pointer, which is the thing being
deliberately not obtained.
