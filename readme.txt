A touchless controller for the classic game, flappy bird! Also contains good examples on 
how to drive the MemoryLCD screen from sharp, and how to make a USB mouse on the EFM32 
Happy Gecko devices.

The hardware for this controller is the sensor expansion board from the Zero Gecko 
weather station kit, combined with the EFM32HG starter kit.

The software is using the drivers for the Si1147, which is included in emlib and the 
USB-stack, also included with the Simplicity Studio distribution. It is basically a 
usbdhidkbd-example,that is modified to be a USB HID mouse instead, and set up to output 
mouse clicks when you advance your hand towards the sensor. It displays the distance 
graphically on the screen, along with the threshold you have to cross to make the 
controller issue a click.

It has an adjustable threshold so you can fine tune the sensitivity on how much you need 
to move your hand for the application to register it, here's a small demonstration on 
how that works. Note that every time the filled circle crosses the not-filled circle, a 
click is issued over USB:
https://photos.google.com/photo/AF1QipNLJ46P17pgiQBeqU154fgQrSlBb0v8V8jiJGNY

To run this demo, create a folder in the examples-folder of the SLSTK3400A-folder in 
your Simplicity Studio distribution, and put all the files there:
C:\SiliconLabs\SimplicityStudio\v3\developer\sdks\efm32\v2\kits\SLSTK3400A_EFM32HG\examples
