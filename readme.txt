USB HID keyboard example.

This example project use the EFM32 USB Device protocol stack
and implements an USB HID class keyboard device (one button !).
The example is power optimized, when USB cable is plugged in and USB is active,
the current drawn from VMCU is less than 1.5 mA, when the cable is disconnected
or the device is suspended, the current is less than 5 uA.
Current drawn from VBUS (which powers the USB PHY) is approximately 460 uA.

PB0 is the keyboard button. Pressing this button will succesively write
"Silicon Labs - ".

Board:  Silicon Labs SLSTK3400A Starter Kit
Device: EFM32HG322F64
