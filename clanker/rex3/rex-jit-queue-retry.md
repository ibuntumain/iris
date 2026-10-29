# Retry rejected REX JIT cold compile requests

The REX JIT compile queue (`src/dev/ng1/rex3_jit_queue.rs`) has two lanes. The
draw path uses the **hot** lane (`RexJit::request_compile`), which is never
rejected for a full queue; warm-up and best-effort prefetch use the **cold** lane
(`request_compile_cold` / `request_compile_blocking`), which holds 256 requests.

If a cold `send` fails, remove the shader key from the map; otherwise subsequent
draws see a phantom queued entry and never request compilation again. A
263-entry warm-up profile reproduced this failure and 30-second waits in
graphics tests. Warm-up itself uses `send_cold_blocking`, so it waits rather than
truncating the profile; the cold *drop* path exists so a non-blocking prefetch
caller can retry later.

Unit tests must start without loading the user's persistent REX profile. Test
warm-up explicitly with fixtures rather than depending on a host's `~/.iris`
state. `hot_draw_request_jumps_a_full_cold_queue` fills the cold lane and checks
that a hot draw request is still admitted and drained first;
`hot_request_is_admitted_ahead_of_a_full_cold_queue` exercises the same at the
queue level without starting the real compiler.
