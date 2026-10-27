# ARM PREEMPT_RT port

`preempt-rt-arm.patch` ports the official `5.10.158-rt77` patch to the pinned
vendor `5.10.160` kernel. The result is named `5.10.160-rt77-mohism` to distinguish
it from an unmodified upstream RT release. It does not update the kernel to a
newer LTS version. Only the ARM architecture is included in this port.

The patch retains upstream copyright and license notices. Its upstream URL and
SHA256 are recorded in `manifest.json`, together with the exact before/after
hashes of every changed file. The prepared tree is generated separately from
the vendor source by `scripts/prepare-rt.py`.

Port-specific resolutions:

- Preserve the vendor ARM `TIF_NOTIFY_SIGNAL` support, moving it to bit 9 so
  it does not collide with lazy rescheduling or seccomp. Keep the vendor
  low-16-bit return-to-user test in `entry-common.S`.
- Retain the vendor locked CPU-affinity helper and its CPU-possible-mask checks
  while integrating the RT migration flags and pending-affinity machinery.
- Preserve vendor wake-queue counters, cpuset mutex serialization, block hooks,
  memory allocation hooks and UART PTIME handling. Convert the cpuset callback
  lock and UART IER access together with their upstream RT changes.
- Replace the vendor softirq deferral path with the RT softirq mechanism,
  retaining the vendor active-softirq counters. Its separate
  `RT_SOFTINT_OPTIMIZATION` option is disabled for this profile.
- Use a normal field for `sk_peer_lock` on RT: an RT spinlock does not fit into
  the Android ABI's 64-bit reserve. The kernel ABI changes; all modules must be
  rebuilt. No compatibility with prebuilt non-RT modules is claimed.
- Add forward declarations for vendor mutex/rwsem trace-hook pointer types and
  make the public rwsem hooks visible with the RT implementation.
- Compile the system monitor's CPU-hotplug mutex only with CPU hotplug, and
  the cpufreq driver's remote-CPU wake operation only with SMP.
- Use the RT printk implementation and console printing threads. Vendor hooks
  attached to the replaced old printk/rtmutex paths are not carried into those
  new paths. No Android hook ABI compatibility is claimed.

The board already disables the FIQ debugger in its device tree. The RT config
also disables its driver, which uses the old printk dump interface. The regular
UART console and both USB ACM interfaces remain configured.

An ARM build is evidence of integration, not proof of real-time timing or
runtime driver correctness. Target boot, lock diagnostics, peripheral stress
and latency testing are described in `docs/realtime.md` at the repository root.
