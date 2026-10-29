# The Octane PROM's stores into its own flash stay in the cache

Status: rule (the model is an approximation: dirty lines are never evicted or
written back).

**Symptom.** The PROM bus-errored early on a plain store into the boot flash
window.

**What to know.** The IP30 PROM runs cached from the boot flash and keeps its
initialised data there. For example, a one-shot init flag at flash offset
0xc1148 is incremented with an ordinary cached `sw`. On an R10000 that store
write-allocates and the line stays dirty-exclusive, so it never reaches the
Bridge. Later cached loads hit the line. iris's shadow cache passes every
store straight to the bus, and the Bridge's flash port takes only 16-bit
writes with write-enable set, so the 32-bit store faulted.

**The model.** `CachedFlashLines` (`src/ip30.rs`) sits between the CPU and
the bus. The first cached store to a flash line copies the 128-byte line in;
from then on cached loads and stores of it use the copy. Uncached accesses
still reach the flash, bus masters never see the copies, reset empties them,
and everything outside the flash window goes straight to the bus.

**Why it is only an approximation.** The copies are never evicted or written
back. What the hardware does if such a line were written back is not modelled.

**How to apply.** If the PROM faults on a store to a flash address, check
that the access is cached before touching the Bridge's flash port: the port
is right to refuse it. Do not make the flash accept wider writes to get past
this. If a guest ever needs a write-back of one of these lines, that is the
behaviour to establish first, then model it here.
