# osx-pl2303

This is a driver for the many-and-various Prolific PL2303 serial-to-USB
adapters that are out there.I just updated it to work on OS X Lion (10.7)
and OS X Mountain Lion (10.).

I forked this project from [failberg/osx-pl2303](http://github.com/downloads/failberg/osx-pl2303).

A compiled kernelmodule and installation help can be found at:
<http://xbsd.nl/2011/07/pl2303-serial-usb-on-osx-lion.html>.

I'm happy to receive pull request for updates or patches.


### Additional information from origin.

> If you need to add additional device IDs or vendor IDs, those live in
> Info.plist; just clone the relevant section of the plist XML and create
> a new stanza - and be sure to send me a patch!

> Thank you to B.J. Arnoldus for writing the driver in the first place,
> Michael Haller for patching the driver to work under SL, and the various
> folks who contributed patches to the Sourceforge tracker, some of which
> I've applied here.

