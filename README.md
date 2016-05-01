# RiscVQemuPcat (QEMU RISC-V PC/AT port)
The QEMU RISC-V PC/AT port was originally cloned from https://github.com/riscv/riscv-qemu.
In order to support PC/AT platforms and also compliant with RISC-V MMIO mechanism, I added some PC/AT peripherals and modified
those I/O based devices to MMIO devices, such as CMOS, PCI, VGA, ACPI and etc.
QEMU RISC-V PC/AT port is a special version which support MMIO PC/AT devices. RISC-V PC/AT board is only used to emulate
RISC-V UEFI port on EDKII. This version of QEMU was diverged from official RISC-V QEMU (https://github.com/riscv/riscv-qemu).
It's hard to sync-up mine with the official one because lot of changes from official RISC-V QEMU and my changes for PC/AT in the past
8 months. One thing sad is I missed the original revision of official RISC-V QEMU. I will have to spend some time to migrate the
changes from these two repositories.

