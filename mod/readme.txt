Build:
make
Debug build:
make DEBUG_BUILD=1 

Features:
If an error occurs during initialization, but before attaching to the CAM
subsystem, the module can be unloaded. On 5.x systems it can be detached from
the CAM layer and unloaded at any time when not being busy.

Bugs:
The DELAYed i/o is way too slow, and that makes the device nearly unusable,
should you bump on a scratched CD, the system will hang for tens of minutes,
while trying to read the bad sector.
Something must be done to allow async operation of the parallel port-related
code. But that's to the FreeBSD 4.x architecture, as it doesn't provide any non-
busywaiting equivalent of DELAY(9), and we can't [at]sleep(9), as we run out of
any process' context, under CAM or even under timeout(9), simulating a real
hardware interrupt. 

Maybe add a callback for the drive lost situation.
Add some sysctls to tweak delay/modes.

The driver is derived from the Linux paride package by Grant R. Guenther.
http://www.torque.net/paride (grant@torque.net). Basically, it's simply a
stripped down version, converted to SCSI and ported to CAM. As it's the
CAM's responsibility to handle all the device bookkeeping, and the ppbus's
to handle the detection & initialization of the parallel port, we don't need
a stackable architecture in the driver. I intentionally leaved frpw.c nearly
'as is' (did not port it to microseq(9)) as it's easier to port it to other
UNIXes. The FreeBSD specific part is in ppcd.c. It also could be possible
to port other low-level drivers from the paride package to FreeBSD, but the
only hardware I have now is FreeCom PowerCD on a XILINX chip.
The FreeBSD specific part is heavily based on vpo(4) by Nicolas Sochu.
The SCSI support routine is taken from atapicam(4) by Thomas Quinot. 

You should also have the gboard.sys or a similar DOS device driver to be able
to use it, since there is microcode/FSM firmware from FreeCom GMBH. Otherwise,
you must recompile with NO_FIRMWARE=1 and start it from DOS/Windows to
initialize, then use under FreeBSD.

Theoretically, it could be possible to even burn the CDs, using, say,
cdrecord cdrtools package. However, one must do so with great care, on the
lowest recording speed possible, and with a bufer underrun protection switched
on. But I haven't tested it myself, as my writers behave wierdly, when attached
to the adapter.

It is also possible to set the speed, using cdcontrol(1).

If you happen to pull out the power supply unit while working, try doing a
camcontrol reset # for the ppcd bus. This should bring the drive back. Under
5.x you may consider kldreloading.

Compiling into the kernel for installation purposes.

Building without microcode.
make NO_FIRMWARE=1
However, doing so will make it impossible to totally reset the drive (e.g. by
power cycling it) without a reboot.

Setting the drive delay. According to Grant, some systems may need a delay,
when communicating with the parallel port. For those, you must add:
make DELAY=#
and supply a value in microseconds.
