# indi_cam84_ccd
Indi driver for DIY Cam84 CCD (Sony ICX453AQ)

See :

http://astroccd.org/2015/04/cam84/
http://www.cloudynights.com/topic/497530-diy-astro-ccd-16-bit-color-6mpx-camera
http://indilib.org

Now using libftdi instead of FTDI D2XX, unloading ftdi_sio is no more necessary :
New udev file unbinds just the necessary devices
BUT : libftdi seems more tricky to use with timings... many reads/writes fails

Now testing :
-2x2 binning
-Framing

