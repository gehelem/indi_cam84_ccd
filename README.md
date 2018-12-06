# indi_cam84_ccd
Indi driver for DIY Cam84 CCD (Sony ICX453AQ)

See :

http://astroccd.org/2015/04/cam84/
http://www.cloudynights.com/topic/497530-diy-astro-ccd-16-bit-color-6mpx-camera
http://indilib.org

Now using libftdi can use FTDI D2XX by doing a 
cmake . -DLIBFTDI=OFF
(default is -DLIBFTDI=ON which uses libftdi)

libftdi performance has been greatly improved by increasing the ftdi_write_data_set_chunksize back to the default value.

Framing now works
2x2 binning may not be ready
