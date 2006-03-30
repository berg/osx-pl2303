/*
 * osx_pl2303.cpp Prolific PL2303 USB to serial adaptor driver for OS X
 *
 * Copyright (c) 2006 BJA Electronics, Jeroen Arnoldus (opensource@bja-electronics.nl)
 * 
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 * Driver is inspired by the following projects:
 * - Linux kernel PL2303.c Copyright (C) 2001-2004 Greg Kroah-Hartman (greg@kroah.com)
 *                         Copyright (C) 2003 IBM Corp.
 * - Apple16x50Serial Copyright (c) 1997-2003 Apple Computer, Inc. All rights reserved.
 *                    Copyright (c) 1994-1996 NeXT Software, Inc.  All rights reserved.
 * - AppleRS232Serial Copyright (c) 2002 Apple Computer, Inc. All rights reserved.
 * - AppleUSBIrda Copyright (c) 2000 Apple Computer, Inc. All rights reserved.
 *
 * Tests:
 * - Driver only tested with ATEN UC-RS232A, but should support other PL2303 based USB to RS232 converters
 *
 * Todo:
 * - Implementation Powermanagement
 * - Implementation Flow-Control/Handshake
 * - Fix USBF: Could not open device: Strange error message
 * - Some small bugs
 *
 *
 *
 * The Linux Driver contains the following code in open():
 * 	if (priv->type != HX) {
 *		usb_clear_halt(serial->dev, port->write_urb->pipe);
 *		usb_clear_halt(serial->dev, port->read_urb->pipe);
 *	}
 *
 * This driver does not implement this clear_halt: The driver initialise the pipes when
 * the device is opened from a client. I assume that the pipe is clean and contains
 * no halt. I did not have the HX chip. I could not test this case.
 *
 * http://www.usb.org/developers/devclass_docs/usbcdc11.pdf
 */
 
#include <IOKit/IOLib.h>
#include <IOKit/IOTypes.h>
#include <IOKit/IOMessage.h>


#include "osx_pl2303.h"
#include <IOKit/serial/IOSerialKeys.h>
#include <IOKit/usb/IOUSBInterface.h>

#include <UserNotification/KUNCUserNotifications.h>

extern "C" {
#include <pexpert/pexpert.h>
}

#define DEBUG
#ifdef DEBUG
#define DEBUG_IOLog(args...)	IOLog (args)
#else
#define DEBUG_IOLog(args...)
#endif

#define DATALOG

#ifdef DATALOG
#define DATA_IOLog(args...)	IOLog (args)
#else
#define DATA_IOLog(args...)
#endif


#define super IOSerialDriverSync

OSDefineMetaClassAndStructors(nl_bjaelectronics_driver_PL2303, IOSerialDriverSync)


/****************************************************************************************************/
//
//      Function:   Asciify
//
//      Inputs:     i - the nibble
//
//      Outputs:    return byte - ascii byte
//
//      Desc:       Converts to ascii. 
//
/****************************************************************************************************/

static UInt8 Asciify(UInt8 i)
{
	
    i &= 0xF;
    if ( i < 10 )
		return( '0' + i );
    else return( 55  + i );
    
}/* end Asciify */

bool nl_bjaelectronics_driver_PL2303::init(OSDictionary *dict)
{
	bool res = super::init(dict);
	DEBUG_IOLog("%s(%p)::Initializing\n", getName(), this);
	return res;
}

void nl_bjaelectronics_driver_PL2303::free(void)
{
	DEBUG_IOLog("%s(%p)::Freeing\n", getName(), this);
	super::free();
}

IOService *nl_bjaelectronics_driver_PL2303::probe(IOService *provider, SInt32 *score)
{
	IOUSBDevice		*Provider;
	DEBUG_IOLog("%s(%p)::Probe\n", getName(), this);
	Provider = OSDynamicCast(IOUSBDevice, provider);   
	 if (!Provider) {
        IOLog("%s(%p)::Probe Attached to non-IOUSBDevice provider!  Failing probe()\n", getName(), this);
        return NULL;
    }
	IOService *res = super::probe(provider, score);
	DEBUG_IOLog("%s(%p)::Probe successful\n", getName(), this);
	return res;
}


bool nl_bjaelectronics_driver_PL2303::start(IOService *provider)
{
    enum pl2303_type type = type_0;

    fTerminate = false;     // Make sure we don't think we're being terminated
    fPort = NULL;
    fNub = NULL;
    fpInterface = NULL;
    
    fpinterruptPipeBuffer = NULL;
    fPipeInBuffer = NULL;
    fPipeOutBuffer = NULL;
    
    fpDevice = NULL;
    fpInPipe = NULL;
    fpOutPipe = NULL;
    fpInterruptPipe = NULL;
    
    fUSBStarted = false;            // set to true when start finishes up ok
    fSessions = 0;
    
    fReadActive = false;
	fWriteActive = false;
	
    DEBUG_IOLog("%s(%p)::start PL2303 Driver\n", getName(), this);
	
    if( !super::start( provider ) )
	{
		IOLog("%s(%p)::start - super failed\n", getName(), this);
        goto Fail;
    }	
	
    fpDevice = OSDynamicCast(IOUSBDevice, provider);
	
    if(!fpDevice) 
    {
        IOLog("%s(%p)::start - Provider isn't a USB device!!!\n", getName(), this);
        goto Fail;
    }
	
    if (fpDevice->GetNumConfigurations() < 1)
    {
        IOLog("%s(%p)::start - no composite configurations\n", getName(), this);
        goto Fail;
    }
		
    // make our nub (and fPort) now
    if( !createNub() ) goto Fail;
	
    // Now configure it (leaves device suspended)
    if( !configureDevice( fpDevice->GetNumConfigurations() ) ) goto Fail;
	
    // Finally create the bsd tty (serial stream) and leave it there until usb stop
	
    if( !createSerialStream() ) goto Fail;
		
    fWorkLoop = getWorkLoop();
    if (!fWorkLoop)
    {
        IOLog("%s(%p)::start - getWorkLoop failed\n", getName(), this);
        goto Fail;
    }
    
    fWorkLoop->retain();
    
    fCommandGate = IOCommandGate::commandGate(this);
    if (!fCommandGate)
    {
        IOLog("%s(%p)::start - create commandGate failed\n", getName(), this);
        goto Fail;
    }
    
    if (fWorkLoop->addEventSource(fCommandGate) != kIOReturnSuccess)
    {
        IOLog("%s(%p)::start - addEventSource fCommandGate to WorkLoop failed\n", getName(), this);
        goto Fail;
    }
	
    fCommandGate->enable();	

    OSNumber *	deviceClass = (OSNumber *) fpDevice->getProperty(kUSBDeviceClass);
	DEBUG_IOLog("%s(%p)::start - GetMaxPacketSize: %p DeviceClass %p\n", getName(), this, fpDevice->GetMaxPacketSize(), deviceClass->unsigned8BitValue() );
	if ( deviceClass->unsigned8BitValue() == 0x02)
		type = type_0;
	else if ( fpDevice->GetMaxPacketSize() == 0x40)
		type = HX;
	else if ( deviceClass->unsigned8BitValue() == 0x00)
		type = type_1;
	else if ( deviceClass->unsigned8BitValue() == 0xFF)
		type = type_1;
	IOLog("%s(%p)::start - device type: %d\n", getName(), this, type);
    fPort->type = type;

	fUSBStarted = true;  
	
	
	return true;
	
Fail:
	if (fNub) 
	{
		destroyNub();
	}
    if (fCommandGate)
    {
        fCommandGate->release();
        fCommandGate = NULL;
    }
    if (fWorkLoop)
    {
        fWorkLoop->release();
        fWorkLoop = NULL;
    }
    DEBUG_IOLog("%s(%p)::start - failed\n", getName(), this);
    stop( provider );
    return false;
    
}


/****************************************************************************************************/
//
//      Method:     nl_bjaelectronics_driver_PL2303::stop
//
//      Inputs:     provider - my provider
//
//      Outputs:    None
//
//      Desc:       Stops
//
/****************************************************************************************************/

void nl_bjaelectronics_driver_PL2303::stop( IOService *provider )
{
	DEBUG_IOLog("%s(%p)::Stopping\n", getName(), this);
    
    fUSBStarted = false;        // reset usb start/stop flag for CheckSerialState
    CheckSerialState();         // turn serial off, release resources
	DEBUG_IOLog("%s(%p)::stop  CheckSerialState succeed\n", getName(), this);
    
    if (fCommandGate)
    {
        fCommandGate->release();
        fCommandGate = NULL;
		DEBUG_IOLog("%s(%p)::stop Command gate destroyed\n", getName(), this);
    }
    if (fWorkLoop)
    {
        fWorkLoop->release();
        fWorkLoop = NULL;
		DEBUG_IOLog("%s(%p)::stop workloop destroyed\n", getName(), this);
    }	
	
    destroySerialStream();      // release the bsd tty
	
    destroyNub();               // delete the nubs and fPort
    
    if (fpInterface)  
	{
		fpInterface->release();     // retain done in ConfigureDevice
		fpInterface = NULL; 
		DEBUG_IOLog("%s(%p)::stop fpInterface destroyed\n", getName(), this);
	}
	
	// release our power manager state - NOT IMPLEMENTED
	//   PMstop();
    
    super::stop( provider );
    return;
    
}/* end stop */


/****************************************************************************************************/
//
//		Method:		nl_bjaelectronics_driver_PL2303::getWorkLoop
//
//		Inputs:	
//
//		Outputs:	
//
//		Desc:		create our own workloop if we don't have one already.
//
/****************************************************************************************************/
IOWorkLoop* nl_bjaelectronics_driver_PL2303::getWorkLoop() const
{
    IOWorkLoop *w;
    DEBUG_IOLog("%s(%p)::getWorkLoop\n", getName(), this);
    
    if (fWorkLoop) w = fWorkLoop;
		else  w = IOWorkLoop::workLoop();
    
    return w;
    
}/* end getWorkLoop */

/****************************************************************************************************/
//
//      Method:     nl_bjaelectronics_driver_PL2303::privateWatchState
//
//      Inputs:     port - the specified port, state - state watching for, mask - state mask (the specific bits)
//
//      Outputs:    IOReturn - kIOReturnSuccess, kIOReturnIOError or kIOReturnIPCError
//
//      Desc:       Wait for the at least one of the state bits defined in mask to be equal
//                  to the value defined in state. Check on entry then sleep until necessary.
//                  A return value of kIOReturnSuccess means that at least one of the port state
//                  bits specified by mask is equal to the value passed in by state.  A return
//                  value of kIOReturnIOError indicates that the port went inactive.  A return
//                  value of kIOReturnIPCError indicates sleep was interrupted by a signal. 
//
/****************************************************************************************************/

IOReturn nl_bjaelectronics_driver_PL2303::privateWatchState( PortInfo_t *port, UInt32 *state, UInt32 mask )
{
    unsigned            watchState, foundStates;
    bool                autoActiveBit   = false;
    IOReturn            rtn             = kIOReturnSuccess;
	
    DEBUG_IOLog("%s(%p)::privateWatchState\n", getName(), this);
	
    watchState              = *state;
	
	// hack to get around problem with carrier detection - Do we need this hack ??
	
    if ( *state | 0x40 )    /// mlj ??? PD_S_RXQ_FULL?
	{
		port->State |= 0x40;
	}
	
    if ( !(mask & (PD_S_ACQUIRED | PD_S_ACTIVE)) )
	{
		watchState &= ~PD_S_ACTIVE; // Check for low PD_S_ACTIVE
		mask       |=  PD_S_ACTIVE; // Register interest in PD_S_ACTIVE bit
		autoActiveBit = true;
	}
	
    for (;;)
    {
	    // Check port state for any interesting bits with watchState value
	    // NB. the '^ ~' is a XNOR and tests for equality of bits.
	    
		foundStates = (watchState ^ ~port->State) & mask;
		if ( foundStates )
		{
			*state = port->State;
			if ( autoActiveBit && (foundStates & PD_S_ACTIVE) )
			{
				rtn = kIOReturnIOError;
			} else {
				rtn = kIOReturnSuccess;
			}
			break;
		}
		port->WatchStateMask |= mask;
        
		retain();							// Just to make sure all threads are awake
		fCommandGate->retain();					// before we're released
        
		rtn = fCommandGate->commandSleep((void *)&port->State);		
        
		fCommandGate->retain();
		
		if (rtn == THREAD_TIMED_OUT)
		{
			rtn = kIOReturnTimeout;
			break;
		} else {
			if (rtn == THREAD_INTERRUPTED)
			{
				rtn = kIOReturnAborted;
				break;
			}
		}
		release();
		
    }/* end for */

	    // As it is impossible to undo the masking used by this
	    // thread, we clear down the watch state mask and wakeup
	    // every sleeping thread to reinitialize the mask before exiting.
	
    port->WatchStateMask = 0;
	fCommandGate->commandWakeup((void *)&port->State);
    
    return rtn;
    
}/* end privateWatchState */


/****************************************************************************************************/
//
//      Method:     nl_bjaelectronics_driver_PL2303::allocateResources
//
//      Inputs:     
//
//      Outputs:    return code - true (allocate was successful), false (it failed)
//
//      Desc:       Finishes up the rest of the configuration and gets all the endpoints open 
//
/****************************************************************************************************/

bool nl_bjaelectronics_driver_PL2303::allocateResources( void )
{
    IOUSBFindEndpointRequest    epReq;      // endPoint request struct on stack
    bool                        goodCall;   // return flag fm Interface call
	
	DEBUG_IOLog("%s(%p)::allocateResources\n", getName(), this);
	
    // Open all the end points
    if (!fpInterface) {
	    IOLog("%s(%p)::allocateResources failed - no fpInterface.\n", getName(), this);
		goto Fail;
	}
	
    goodCall = fpInterface->open( this );       // close done in releaseResources
    if ( !goodCall ){
		IOLog("%s(%p)::allocateResources - open data interface failed.\n", getName(), this);
		fpInterface->release();
		fpInterface = NULL;
		return false;
    }
	
    fpInterfaceNumber = fpInterface->GetInterfaceNumber();
    
    epReq.type          = kUSBBulk;
    epReq.direction     = kUSBIn;
    epReq.maxPacketSize = 0;
    epReq.interval      = 0;
    fpInPipe = fpInterface->FindNextPipe( 0, &epReq );
	if (!fpInPipe) {
	    IOLog("%s(%p)::allocateResources failed - no fpInPipe.\n", getName(), this);
		goto Fail;
	}
	
    epReq.direction = kUSBOut;
    fpOutPipe = fpInterface->FindNextPipe( 0, &epReq );
	if (!fpOutPipe) {
	    IOLog("%s(%p)::allocateResources failed - no fpOutPipe.\n", getName(), this);
		goto Fail;
	}
	
    epReq.type          = kUSBInterrupt;
    epReq.direction     = kUSBIn;
    fpInterruptPipe = fpInterface->FindNextPipe( 0, &epReq );
	if (!fpInterruptPipe) {
	    IOLog("%s(%p)::allocateResources failed - no fpInterruptPipe.\n", getName(), this);
		goto Fail;
	}
	
    // Allocate Memory Descriptor Pointer with memory for the interrupt-in pipe:
	
    fpinterruptPipeMDP = IOBufferMemoryDescriptor::withCapacity( INTERRUPT_BUFF_SIZE, kIODirectionIn );
	if (!fpinterruptPipeMDP) {
	    IOLog("%s(%p)::allocateResources failed - no fpinterruptPipeMDP.\n", getName(), this);
		goto Fail;
	}    
    fpinterruptPipeMDP->setLength( INTERRUPT_BUFF_SIZE );
    fpinterruptPipeBuffer = (UInt8*)fpinterruptPipeMDP->getBytesNoCopy();	
    // Allocate Memory Descriptor Pointer with memory for the data-in bulk pipe:
	
    fpPipeInMDP = IOBufferMemoryDescriptor::withCapacity( USBLapPayLoad, kIODirectionIn );
	if (!fpPipeInMDP) {
	    IOLog("%s(%p)::allocateResources failed - no fpPipeInMDP.\n", getName(), this);
		goto Fail;
	}        
    fpPipeInMDP->setLength( USBLapPayLoad );
    fPipeInBuffer = (UInt8*)fpPipeInMDP->getBytesNoCopy();
	
    // Allocate Memory Descriptor Pointer with memory for the data-out bulk pipe:
	
    fpPipeOutMDP = IOBufferMemoryDescriptor::withCapacity( MAX_BLOCK_SIZE, kIODirectionOut );
	if (!fpPipeOutMDP) {
	    DEBUG_IOLog("%s(%p)::allocateResources failed - no fpPipeOutMDP.\n", getName(), this);
		goto Fail;
	}          
    fpPipeOutMDP->setLength( MAX_BLOCK_SIZE );
    fPipeOutBuffer = (UInt8*)fpPipeOutMDP->getBytesNoCopy();
    
    // set up the completion info for all three pipes
    
	if (!fPort) {
	    IOLog("%s(%p)::allocateResources failed - no fPort.\n", getName(), this);
		goto Fail;
	}      
    finterruptCompletionInfo.target = this;
    finterruptCompletionInfo.action = interruptReadComplete;
    finterruptCompletionInfo.parameter  = fPort;
    
    fReadCompletionInfo.target  = this;
    fReadCompletionInfo.action  = dataReadComplete;
    fReadCompletionInfo.parameter   = fPort;
	
    fWriteCompletionInfo.target = this;
    fWriteCompletionInfo.action = dataWriteComplete;
    fWriteCompletionInfo.parameter  = fPort;
	
	if( SetSerialConfiguration() ){
		IOLog("%s(%p)::allocateResources SetSerialConfiguration failed\n", getName(), this);
		goto Fail;
	}
	
    DEBUG_IOLog("%s(%p)::allocateResources successful\n", getName(), this);
    return true;
	
Fail:
		return false;
    
} // allocateResources


/****************************************************************************************************/
//
//      Method:     nl_bjaelectronics_driver_PL2303::releaseResources
//
//      Inputs:     port - the Port
//
//      Outputs:    None
//
//      Desc:       Frees up the pipe resources allocated in allocateResources
//
/****************************************************************************************************/

void nl_bjaelectronics_driver_PL2303::releaseResources( void )
{
    DEBUG_IOLog("nl_bjaelectronics_driver_PL2303::releaseResources\n");
    
    if ( fpInterface ) { 
		fpInterface->close( this ); 
    }    
    
    if ( fpPipeOutMDP  ) { 
		fpPipeOutMDP->release();    
		fpPipeOutMDP    = 0; 
    }
    
    if ( fpPipeInMDP   ) { 
		fpPipeInMDP->release(); 
		fpPipeInMDP     = 0; 
    }
    
    if ( fpinterruptPipeMDP ) { 
		fpinterruptPipeMDP->release();  
		fpinterruptPipeMDP  = 0; 
    }
	
    return;
    
}/* end releaseResources */



//
// startSerial
//
// assumes createSerialStream is called once at usb start time
// calls allocateResources to open endpoints
//
bool nl_bjaelectronics_driver_PL2303::startSerial()
{
	IOUSBDevRequest request;
	char * buf;	
	IOReturn rtn;
	DEBUG_IOLog("%s(%p)::startSerial \n", getName(), this);
	
	buf = (char *) IOMalloc(10);
    if (!buf) {
		IOLog("%s(%p)::startSerial could not alloc memory for buf\n", getName(), this);
		goto	Fail;
	}

    if (!fNub) {
		IOLog("%s(%p)::startSerial fNub not available\n", getName(), this);
		goto	Fail;
	}

    // make chip as sane as can be
#define FISH(a,b,c,d)								\
	request.bmRequestType = a; \
    request.bRequest = b; \
	request.wValue =  c; \
	request.wIndex = d; \
	request.wLength = 1; \
	request.pData = buf; \
	rtn =  fpDevice->DeviceRequest(&request); \
	DEBUG_IOLog("%s(%p)::startSerial FISH 0x%x:0x%x:0x%x:0x%x  %d - %x\n", getName(), this,a,b,c,d,rtn,buf[0]);

#define SOUP(a,b,c,d)								\
	request.bmRequestType = a; \
    request.bRequest = b; \
	request.wValue =  c; \
	request.wIndex = d; \
	request.wLength = 0; \
	request.pData = NULL; \
	rtn =  fpDevice->DeviceRequest(&request); \
	DEBUG_IOLog("%s(%p)::startSerial SOUP 0x%x:0x%x:0x%x:0x%x  %d\n", getName(), this,a,b,c,d,rtn);


	FISH (VENDOR_READ_REQUEST_TYPE, VENDOR_READ_REQUEST, 0x8484, 0);
	SOUP (VENDOR_WRITE_REQUEST_TYPE, VENDOR_WRITE_REQUEST, 0x0404, 0);
	FISH (VENDOR_READ_REQUEST_TYPE, VENDOR_READ_REQUEST, 0x8484, 0);
	FISH (VENDOR_READ_REQUEST_TYPE, VENDOR_READ_REQUEST, 0x8383, 0);
	FISH (VENDOR_READ_REQUEST_TYPE, VENDOR_READ_REQUEST, 0x8484, 0);
	SOUP (VENDOR_WRITE_REQUEST_TYPE, VENDOR_WRITE_REQUEST, 0x0404, 1);
	FISH (VENDOR_READ_REQUEST_TYPE, VENDOR_READ_REQUEST, 0x8484, 0);
	FISH (VENDOR_READ_REQUEST_TYPE, VENDOR_READ_REQUEST, 0x8383, 0);
//	FISH (VENDOR_WRITE_REQUEST_TYPE, VENDOR_WRITE_REQUEST, 0x81, 1);
	SOUP (VENDOR_WRITE_REQUEST_TYPE, VENDOR_WRITE_REQUEST, 0, 1);
	SOUP (VENDOR_WRITE_REQUEST_TYPE, VENDOR_WRITE_REQUEST, 1, 0);

	if (fPort->type == HX) { 
		/* HX chip */
		SOUP (VENDOR_WRITE_REQUEST_TYPE, VENDOR_WRITE_REQUEST, 2, 0x44); 
		/* reset upstream data pipes */
         	SOUP (VENDOR_WRITE_REQUEST_TYPE, VENDOR_WRITE_REQUEST, 8, 0);
        	SOUP (VENDOR_WRITE_REQUEST_TYPE, VENDOR_WRITE_REQUEST, 9, 0);
	} else {
		SOUP (VENDOR_WRITE_REQUEST_TYPE, VENDOR_WRITE_REQUEST, 2, 0x24);
	}
	
    IOFree(buf, 10);
//	request.bmRequestType = USBmakebmRequestType(kUSBOut, kUSBClass, kUSBInterface);
//    request.bRequest = SET_LINE_REQUEST;
//	request.wValue =  0; 
//	request.wIndex = 0;
//	request.wLength = 7;
//	request.pData = buf;
//	rtn =  fpDevice->DeviceRequest(&request);
//	DEBUG_IOLog("%s(%p)::startSerial - chip clean return: %p \n", getName(), this,  rtn);
	
	// open the pipe endpoints
	if (!allocateResources() ) {
		IOLog("%s(%p)::startSerial Allocate resources failed\n", getName(), this);
		goto	Fail;
	}
    
    startPipes();                           // start reading on the usb pipes
    return true;
	
Fail:
		return false;
}

void nl_bjaelectronics_driver_PL2303::stopSerial()
{
	DEBUG_IOLog("%s(%p)::stopSerial\n", getName(), this);
    stopPipes();                            // stop reading on the usb pipes
	DEBUG_IOLog("%s(%p)::stopSerial stopPipes succeed\n", getName(), this);

    if (fpPipeOutMDP != NULL)               // better test for releaseResources?
    {
		releaseResources( );
    }
	DEBUG_IOLog("%s(%p)::stopSerial stopSerial succeed\n", getName(), this);
    
Fail:
		return;
}

/****************************************************************************************************/
//
//      Method:     nl_bjaelectronics_driver_PL2303::CheckSerialState
//
//      Inputs:     open session count (fSessions)
//                  usb start/stop (fStartStopUSB) -- replace with fTerminate?
//
//      Outputs:    
//
//      Desc:       Turns Serial on or off if appropriate
//
/****************************************************************************************************/

IOReturn nl_bjaelectronics_driver_PL2303::CheckSerialState( void )
{
    Boolean     newState = fUSBStarted &    // usb must have started, and 
//			(fPowerState == kIrDAPowerOnState) &   // powered on by the power manager, and
	(fSessions > 0); // one of the clients too
	
    DEBUG_IOLog("%s(%p)::CheckSerialState\n", getName(), this);    
    if ( newState ){       
		fTerminate = false;
		if ( !startSerial() )
		{
			fTerminate = true;
			IOLog("%s(%p)::CheckSerialState - StartSerial failed\n", getName(), this);
		} else {
			DEBUG_IOLog("%s(%p)::CheckSerialState - StartSerial successful\n", getName(), this);
		}
		
	} else if (!newState && !fTerminate)      // Turn Serial off if needed
    {
		DEBUG_IOLog("%s(%p)::CheckSerialState - StopSerial\n", getName(), this);
		fTerminate = true;              // Make it look like we've been terminated	    
		stopSerial();                     // stop irda and stop pipe i/o
		DEBUG_IOLog("%s(%p)::CheckSerialState - StopSerial successful\n", getName(), this);

    }
	return kIOReturnSuccess;  
	//    return ior;
}/* end CheckSerialState */


/****************************************************************************************************/
//
//      Method:     nl_bjaelectronics_driver_PL2303::configureDevice
//
//      Inputs:     numconfigs - number of configurations present
//
//      Outputs:    return Code - true (device configured), false (device not configured)
//
//      Desc:       Finds the configurations and then the appropriate interfaces etc.
//
/****************************************************************************************************/

bool nl_bjaelectronics_driver_PL2303::configureDevice( UInt8 numConfigs )
{
    IOUSBFindInterfaceRequest           req;            // device request Class on stack
    const IOUSBConfigurationDescriptor  *cd = NULL;     // configuration descriptor
    IOUSBInterfaceDescriptor            *intf = NULL;   // interface descriptor
    IOReturn                            ior;
    UInt8                               cval;
    UInt8                               config = 0;
	
    DEBUG_IOLog("%s(%p)::configureDevice\n", getName(), this);
	
    for (cval=0; cval<numConfigs; cval++)
    {	
		cd = fpDevice->GetFullConfigurationDescriptor(cval);
		if ( !cd )
		{
			IOLog("%s(%p)::configureDevice - Error getting the full configuration descriptor\n", getName(), this);
		} 
		
		else {
			
			// Find the first one - there may be more to go on in the future
			
			req.bInterfaceClass = kIOUSBFindInterfaceDontCare;
			req.bInterfaceSubClass  = kIOUSBFindInterfaceDontCare;
			req.bInterfaceProtocol  = kIOUSBFindInterfaceDontCare;
			req.bAlternateSetting   = kIOUSBFindInterfaceDontCare;
			
			ior = fpDevice->FindNextInterfaceDescriptor(cd, intf, &req, &intf);
			if ( ior == kIOReturnSuccess )
			{
				if ( intf ){
					config = cd->bConfigurationValue;
					DEBUG_IOLog("%s(%p)::configureDevice - Interface descriptor found\n", getName(), this);
					break;
				} else {
					DEBUG_IOLog("%s(%p)::configureDevice - That's weird the interface was null\n", getName(), this);
					cd = NULL;
				}
			} else {
				IOLog("%s(%p)::configureDevice - No CDC interface found this configuration\n", getName(), this);
				cd = NULL;
			}
		}
    }
	
    if ( !cd )
    {
		goto Fail;
    }
	
	// Now lets do it for real
	
    req.bInterfaceClass = kIOUSBFindInterfaceDontCare;
    req.bInterfaceSubClass  = kIOUSBFindInterfaceDontCare;
    req.bInterfaceProtocol  = kIOUSBFindInterfaceDontCare;
    req.bAlternateSetting   = kIOUSBFindInterfaceDontCare;
    
    fpInterface = fpDevice->FindNextInterface( NULL, &req );
    if ( !fpInterface )
	{
		DEBUG_IOLog("%s(%p)::configureDevice - Find next interface failed open device and reallocate objects\n", getName(), this);
		/* Dirty solution to configure device if the interface objects are not yet allocated */
		if (!fpDevice->open(fpDevice))
		{
			IOLog("%s(%p)::configureDevice - unable to open device for configuration \n", getName(), this);
			goto Fail;
		}	
		IOReturn rtn =  fpDevice->SetConfiguration(fpDevice, fpDevice->GetFullConfigurationDescriptor(0)->bConfigurationValue, true);
		if (rtn)
		{
			IOLog("%s(%p)::configureDevice - unable to set the configuration\n", getName(), this);
			goto Fail;
		}
		fpInterface = fpDevice->FindNextInterface( NULL, &req );
		if ( !fpInterface )
		{
			IOLog("%s(%p)::configureDevice - Find interface failed\n", getName(), this);
			goto Fail;
		} else {
			DEBUG_IOLog("%s(%p)::configureDevice Interface founded\n", getName(), this);
		}
	} else {
		DEBUG_IOLog("%s(%p)::configureDevice Interface founded\n", getName(), this);
	}
	
    fpInterface->retain();      // release done in stop()
    
    return true;
    
Fail:
		return false;
	
}/* end configureDevice */

/****************************************************************************************************/
//
//      Method:     nl_bjaelectronics_driver_PL2303::createNub
//
//      Inputs:
//
//      Outputs:    fNub  and fPort
//
//      Desc:       allocates and inits, but doesn't publish the BSD info on the nub yet
//              create serial stream finishes the job later.
//
/****************************************************************************************************/
bool
nl_bjaelectronics_driver_PL2303::createNub(void)
{
    DEBUG_IOLog("%s(%p)::createNub\n", getName(), this);

	if (fNub == NULL) {
		fNub = new IORS232SerialStreamSync;
    }
	
    if( !fNub ) goto Fail;
	
    if (fPort == NULL) {
		fPort = (PortInfo_t*)IOMalloc( sizeof(PortInfo_t) );
    }
	
    if( !fPort ) goto Fail;
	
    bzero(fPort, sizeof(PortInfo_t));
	
    if( !fNub->init(0, fPort ) ) goto Fail;
	
    if( !fNub->attach( this ) ) goto Fail;
	
    return true;
	
Fail:
	IOLog("%s(%p)::Createnub failed\n", getName(), this);
    // could try and clean up here, but let's start by just not crashing.
    return false;
}

void nl_bjaelectronics_driver_PL2303::destroyNub()
{
	DEBUG_IOLog("%s(%p)::destroyNub Try to destroy nub\n", getName(), this);
    if (fPort != NULL) {
		IOFree( fPort, sizeof(PortInfo_t) );
		fPort = NULL;
		DEBUG_IOLog("%s(%p)::destroyNub fPort reset \n", getName(), this);
		
    }
    
    if (fNub) {
		fNub->detach(this);
		fNub->release();    // crash boom?
		fNub = NULL;
		DEBUG_IOLog("%s(%p)::destroyNub Nub destroyed \n", getName(), this);
    }
}

/****************************************************************************************************/
//
//      Method:     nl_bjaelectronics_driver_PL2303::createSuffix
//
//      Inputs:     None
//
//      Outputs:    return Code - true (suffix created), false (suffix not create), sufKey - the key
//
//      Desc:       Creates the suffix key. It attempts to use the serial number string from the device
//                  if it's reasonable i.e. less than 8 bytes ascii. Remember it's stored in unicode 
//                  format. If it's not present or not reasonable it will generate the suffix based 
//                  on the location property tag. At least this remains the same across boots if the
//                  device is plugged into the same physical location. In the latter case trailing
//                  zeros are removed.
//
/****************************************************************************************************/

bool nl_bjaelectronics_driver_PL2303::createSuffix( unsigned char *sufKey )
{
    
    IOReturn                rc;
    UInt8                   serBuf[10];     // arbitrary size > 8
    OSNumber                *location;
    UInt32                  locVal;
    UInt8                   *rlocVal;
    UInt16                  offs, i, sig = 0;
    UInt8                   indx;
    bool                    keyOK = false;      
    DEBUG_IOLog("%s(%p)::createSuffix\n", getName(), this);
	
    indx = fpDevice->GetSerialNumberStringIndex();  
	DEBUG_IOLog("%s(%p)::createSuffix the index of string descriptor describing the device's serial number: %p\n", getName(), this, indx );
	
    if (indx != 0 )
    {   
		// Generate suffix key based on the serial number string (if reasonable <= 8 and > 0)   
		
		rc = fpDevice->GetStringDescriptor(indx, (char *)&serBuf, sizeof(serBuf));
		if ( !rc )
		{
			DEBUG_IOLog("%s(%p)::createSuffix serial number: %s\n", getName(), this, serBuf );
			
			if ( (strlen((char *)&serBuf) < 9) && (strlen((char *)&serBuf) > 0) )
			{
				strcpy( (char *)sufKey, (const char *)&serBuf);
				keyOK = true;
			}           
		} else {
			IOLog("%s(%p)::createSuffix error reading serial number string\n", getName(), this );
		} 
    }
    
    if ( !keyOK )
	{
		// Generate suffix key based on the location property tag
		
		location = (OSNumber *)fpDevice->getProperty(kUSBDevicePropertyLocationID);	
		DEBUG_IOLog("%s(%p)::createSuffix location number: %d\n", getName(), this, location );
		
		if ( location )
		{
			locVal = location->unsigned32BitValue();        
			offs = 0;
			rlocVal = (UInt8*)&locVal;
			for (i=0; i<4; i++)
			{
				sufKey[offs] = Asciify(rlocVal[i] >> 4);
				if ( sufKey[offs++] != '0')
					sig = offs;
				sufKey[offs] = Asciify(rlocVal[i]);
				if ( sufKey[offs++] != '0')
					sig = offs;
			}           
			sufKey[sig] = 0x00;
			keyOK = true;
		}
    }
    
	DEBUG_IOLog("%s(%p)::createSuffix the suffix: %s\n", getName(), this, sufKey );
	
    return keyOK;
	
}/* end createSuffix */


/****************************************************************************************************/
//
//      Method:     nl_bjaelectronics_driver_PL2303::SetStructureDefaults
//
//      Inputs:     port - the port to set the defaults, Init - Probe time or not
//
//      Outputs:    None
//
//      Desc:       Sets the defaults for the specified port structure
//
/****************************************************************************************************/

void nl_bjaelectronics_driver_PL2303::SetStructureDefaults( PortInfo_t *port, bool Init )
{
    UInt32  tmp;
    
    DEBUG_IOLog("%s(%p)::SetStructureDefaults\n", getName(), this);
	
	/* These are initialized when the port is created and shouldn't be reinitialized. */
    if ( Init )
	{
		port->FCRimage          = 0x00;
		port->IERmask           = 0x00;
		
		port->State             = ( PD_S_TXQ_EMPTY | PD_S_TXQ_LOW_WATER | PD_S_RXQ_EMPTY | PD_S_RXQ_LOW_WATER );
		port->WatchStateMask    = 0x00000000;              
//		port->serialRequestLock = 0;
    }
	
    port->BaudRate          = kDefaultBaudRate;         // 9600 bps
    port->CharLength        = 8;                        // 8 Data bits
    port->StopBits          = 2;                        // 1 Stop bit
    port->TX_Parity         = 1;                        // No Parity
    port->RX_Parity         = 1;                        // --ditto--
    port->MinLatency        = false;
    port->XONchar           = '\x11';
    port->XOFFchar          = '\x13';
    port->FlowControl       = 0x00000000;
    port->RXOstate          = IDLE_XO;
    port->TXOstate          = IDLE_XO;
    port->FrameTOEntry      = NULL;
	
    port->RXStats.BufferSize    = kMaxCirBufferSize;
    port->RXStats.HighWater     = (port->RXStats.BufferSize << 1) / 3;
    port->RXStats.LowWater      = port->RXStats.HighWater >> 1;
	
    port->TXStats.BufferSize    = kMaxCirBufferSize;
    port->TXStats.HighWater     = (port->RXStats.BufferSize << 1) / 3;
    port->TXStats.LowWater      = port->RXStats.HighWater >> 1;
    
    port->FlowControl           = (DEFAULT_AUTO | DEFAULT_NOTIFY);
	
    port->AreTransmitting   = FALSE;
	
    for ( tmp=0; tmp < (256 >> SPECIAL_SHIFT); tmp++ )
		port->SWspecial[ tmp ] = 0;

    DEBUG_IOLog("%s(%p)::SetStructureDefaults finished\n", getName(), this);

    return;
    
}/* end SetStructureDefaults */

/****************************************************************************************************/
//
//      Method:     nl_bjaelectronics_driver_PL2303::createSerialStream
//
//      Inputs:     None
//
//      Outputs:    return Code - true (created and initialilzed ok), false (it failed)
//
//      Desc:       Creates and initializes the nub and port structure
//
/****************************************************************************************************/

bool nl_bjaelectronics_driver_PL2303::createSerialStream()
{
    UInt8           indx;
    IOReturn            rc;
    unsigned char       rname[10];
    const char          *suffix = (const char *)&rname;
    DEBUG_IOLog("%s(%p)::createSerialStream\n", getName(), this);

    if (!fNub || !fPort) return false;
	
    SetStructureDefaults( fPort, true );            // init the Port structure
    
    // Allocate the request lock
    fPort->serialRequestLock = IOLockAlloc();   // init lock used to protect code on MP
    if ( !fPort->serialRequestLock ) 
	{
		return false;
    }
    
    // now the ring buffers
    if (!allocateRingBuffer(&(fPort->TX), fPort->TXStats.BufferSize) ||
		!allocateRingBuffer(&(fPort->RX), fPort->RXStats.BufferSize)) 
	{
		DEBUG_IOLog("%s(%p)::createSerialStream init ringbuffers  failed\n", getName(), this);
		return false;
	}
	
    if ( !fTerminate )
    {
		// Report the base name to be used for generating device nodes
		
		fNub->setProperty( kIOTTYBaseNameKey, baseName );
		
		// Create suffix key and set it
		
		if ( createSuffix( (unsigned char *)suffix ) )
		{       
			fNub->setProperty( kIOTTYSuffixKey, suffix );
		}
		
		
		// Save the Product String  (at least the first productNameLength's worth).
		
		indx = fpDevice->GetProductStringIndex();   
		if ( indx != 0 )
		{   
			rc = fpDevice->GetStringDescriptor( indx, (char *)&fProductName, sizeof(fProductName) );
			if ( !rc )
			{
				DEBUG_IOLog("%s(%p)::createSerialStream product name: %s\n", getName(), this, fProductName);
				if ( strlen((char *)fProductName) == 0 )        // believe it or not this sometimes happens (null string with an index defined???)
				{
					strcpy( (char *)fProductName, defaultName);
				}
				fNub->setProperty( (const char *)propertyTag, (const char *)fProductName );
			}
		}
	    
		fNub->registerService();
    }
    
    return true;
    
}/* end createSerialStream */

//
// release things created in createSerialStream
//
void
nl_bjaelectronics_driver_PL2303::destroySerialStream(void)
{
    DEBUG_IOLog("%s(%p)::destroySerialStream\n", getName(), this);
	if( !fPort ) goto Fail;
    
	
    if ( fPort->serialRequestLock )
	{
		IOLockFree( fPort->serialRequestLock ); // free the Serial Request Lock
		fPort->serialRequestLock = NULL;
	}
	
    // Remove all the buffers.
	
    freeRingBuffer( &fPort->TX );
    freeRingBuffer( &fPort->RX );
	
    removeProperty( (const char *)propertyTag );    // unhook from BSD
	DEBUG_IOLog("%s(%p)::destroySerialStream serial stream destroyed \n", getName(), this);
	
Fail:
		return;
}




//
// start reading on the pipes
//
bool nl_bjaelectronics_driver_PL2303::startPipes( void )
{
    IOReturn                    rtn;
    DEBUG_IOLog("%s(%p)::startPipes\n", getName(), this);
    
    if(!fPort) goto Fail;
    if(!fpinterruptPipeMDP) goto Fail;
    if(!fpPipeInMDP) goto Fail;
    if(!fpPipeOutMDP) goto Fail;

	// Read the data-in bulk pipe if not using interrupts
	rtn = fpInPipe->Read(fpPipeInMDP, &fReadCompletionInfo, NULL );

    if( !(rtn == kIOReturnSuccess) ) goto Fail;
    
    // is this really referenced by anyone??
    fReadActive = true;     // remember if we did a read
    DEBUG_IOLog("%s(%p)::startPipes pipes started\n", getName(), this);
    return true;
    
Fail:
		return false;
}/* end startPipes */

//
// stop i/o on the pipes
//
void nl_bjaelectronics_driver_PL2303::stopPipes()
{
    DEBUG_IOLog("%s(%p)::stopPipes fpInPipe %p\n", getName(), this, fpInPipe);
    if (fpInPipe){     
	    fpInPipe->Abort();}
	DEBUG_IOLog("%s(%p)::stopPipes fpOutPipe %p\n", getName(), this, fpOutPipe);

    if (fpOutPipe){        		
		fpOutPipe->Abort();}
	DEBUG_IOLog("%s(%p)::stopPipes fpInterruptPipe %p\n", getName(), this, fpInterruptPipe);

    if (fpInterruptPipe){    
		fpInterruptPipe->Abort();}
    DEBUG_IOLog("%s(%p)::stopPipes succeed\n", getName(), this);

}


/****************************************************************************************************/
//
//      Method:     nl_bjaelectronics_driver_PL2303::message
//
//      Inputs:     type - message type, provider - my provider, argument - additional parameters
//
//      Outputs:    return Code - kIOReturnSuccess
//
//      Desc:       Handles IOKit messages. 
//
/****************************************************************************************************/
enum {                                  // messageType for the callback routines
    kIrDACallBack_Status    = 0x1000,   // Status Information is coming
    kIrDACallBack_Unplug    = 0x1001    // USB Device is unplugged
};

IOReturn nl_bjaelectronics_driver_PL2303::message( UInt32 type, IOService *provider,  void *argument)
{
    DEBUG_IOLog("%s(%p)::message %p\n", getName(), this, type);

	switch ( type )
    {
		case kIOMessageServiceIsTerminated:
			DEBUG_IOLog("%s(%p)::message - kIOMessageServiceIsTerminated sessions: %p\n", getName(), this,fSessions);
			
			if ( fSessions ){
				stopSerial();         // stop serial now

				DEBUG_IOLog("%s(%p)::message - kIOMessageServiceIsTerminated fSessions\n", getName(), this);

				if ( (fPort != NULL) && (fPort->serialRequestLock != 0) ){
				    DEBUG_IOLog("%s(%p)::message - kIOMessageServiceIsTerminated changeState\n", getName(), this);
					changeState( fPort, 0, (UInt32)PD_S_ACTIVE );
				}
				DEBUG_IOLog("%s(%p)::message - kIOMessageServiceIsTerminated send KUNCUserNotificationDisplayNotice\n", getName(), this);

				KUNCUserNotificationDisplayNotice(
												  0,      // Timeout in seconds
												  0,      // Flags (for later usage)
												  "",     // iconPath (not supported yet)
												  "",     // soundPath (not supported yet)
												  "",     // localizationPath (not supported  yet)
												  "USB Serial Unplug Notice",       // the header
												  "The USB Serial Pod has been unplugged while an Application was still active. This can result in loss of data.",
												  "OK");
			} else {
				stopSerial();         // stop serial now
			
				if ( fpInterface ) {
					fpInterface->close( this ); 
					fpInterface->release();
					fpInterface = NULL; 
				}
			}
			
			DEBUG_IOLog("%s(%p)::message - kIOMessageServiceIsTerminated terminated\n", getName(), this);
				
			fTerminate = true;      // we're being terminated (unplugged)
			/* We need to disconnect the user client interface */
			break;
			
		case kIOMessageServiceIsSuspended:  
			DEBUG_IOLog("%s(%p)::message - kIOMessageServiceIsSuspended\n", getName(), this);
			break;
			
		case kIOMessageServiceIsResumed:    
			DEBUG_IOLog("%s(%p)::message - kIOMessageServiceIsResumed\n", getName(), this);
			break;
			
		case kIOMessageServiceIsRequestingClose: 
			DEBUG_IOLog("%s(%p)::message - kIOMessageServiceIsRequestingClose\n", getName(), this); 
			break;
			
		case kIOMessageServiceWasClosed:    
			DEBUG_IOLog("%s(%p)::message - kIOMessageServiceWasClosed\n", getName(), this); 
			break;
			
		case kIOMessageServiceBusyStateChange:  
			DEBUG_IOLog("%s(%p)::message - kIOMessageServiceBusyStateChange\n", getName(), this); 
			break;
			
		case kIOMessageServiceIsAttemptingOpen:
			DEBUG_IOLog("%s(%p)::received kIOMessageServiceIsAttemptingOpen with argument: %p \n", getName(), this, (int) argument );
			
			break;
			
		case kIOUSBMessagePortHasBeenResumed:   
			DEBUG_IOLog("%s(%p)::message - kIOUSBMessagePortHasBeenResumed\n", getName(), this);
			
			if ( !fTerminate )        
			{
				DEBUG_IOLog("%s(%p)::message - port already started \n", getName(), this);
				}
				else {                  // we're trying to resume, so start serial
				if ( !startSerial() )
				{
					fTerminate = true;
					DEBUG_IOLog("%s(%p)::message - startSerial failed\n", getName(), this);
				} 
				else {
					DEBUG_IOLog("%s(%p)::message - startSerial successful\n", getName(), this);
				}
			}	
			break;
			
		case kIOUSBMessageHubResumePort:
			DEBUG_IOLog("%s(%p)::message - kIOUSBMessageHubResumePort\n", getName(), this);
			if ( !fTerminate )        
			{
				DEBUG_IOLog("%s(%p)::message - port already started \n", getName(), this);
				}
				else {                  // we're trying to resume, so start serial
				if ( !startSerial() )
				{
					fTerminate = true;
					DEBUG_IOLog("%s(%p)::message - startSerial failed\n", getName(), this);
				    KUNCUserNotificationDisplayNotice(
					    0,      // Timeout in seconds
					    0,      // Flags (for later usage)
					    "",     // iconPath (not supported yet)
					    "",     // soundPath (not supported yet)
					    "",     // localizationPath (not supported  yet)
					    "USB Serial Problem Notice",      // the header
		    		    "The USB Serial Pod has experienced difficulties. To continue either replug the device (if external) or restart the computer",
					    "OK");

				} 
				else {
					DEBUG_IOLog("%s(%p)::message - startSerial successful\n", getName(), this);
				}
			}	
			break;			
			
		default:
			DEBUG_IOLog("%s(%p)::message - unknown message %p \n", getName(), this, type ); 
			break;
    }
    
    return kIOReturnSuccess;
}


/****************************************************************************************************/
//
//      Method:     nl_bjaelectronics_driver_PL2303::readPortState
//
//      Inputs:     port - the specified port
//
//      Outputs:    returnState - current state of the port
//
//      Desc:       Reads the current Port->State. 
//
/****************************************************************************************************/

UInt32 nl_bjaelectronics_driver_PL2303::readPortState( PortInfo_t *port )
{
    UInt32              returnState;
	DEBUG_IOLog("%s(%p)::readPortState\n", getName(), this, returnState );

    IOLockLock( port->serialRequestLock );
	DEBUG_IOLog("%s(%p)::readPortState port->State\n", getName(), this, returnState );

	returnState = port->State;
	IOLockUnlock( port->serialRequestLock);
	
	DEBUG_IOLog("%s(%p)::readPortState returnstate: %p \n", getName(), this, returnState );
	
    return returnState;
    
}/* end readPortState */

/****************************************************************************************************/
//
//      Method:     nl_bjaelectronics_driver_PL2303::changeState
//
//      Inputs:     port - the specified port, state - new state, mask - state mask (the specific bits)
//
//      Outputs:    None
//
//      Desc:       Change the current Port->State to state using the mask bits.
//                  if mask = 0 nothing is changed.
//                  delta contains the difference between the new and old state taking the
//                  mask into account and it's used to wake any waiting threads as appropriate. 
//
/****************************************************************************************************/

void nl_bjaelectronics_driver_PL2303::changeState( PortInfo_t *port, UInt32 state, UInt32 mask )
{
    UInt32              delta;
    DEBUG_IOLog("%s(%p)::changeState\n", getName(), this);
    	
    IOLockLock( port->serialRequestLock );
    state = (port->State & ~mask) | (state & mask); // compute the new state
    delta = state ^ port->State;                    // keep a copy of the diffs
    port->State = state;
	
	// Wake up all threads asleep on WatchStateMask
	
    if ( delta & port->WatchStateMask )
	{
		fCommandGate->commandWakeup((void *)&fPort->State);
	}
	
    IOLockUnlock( port->serialRequestLock );
    return;
    
}/* end changeState */


/****************************************************************************************************/
//
//		Method:		nl_bjaelectronics_driver_PL2303::acquirePort
//
//		Inputs:		sleep - true (wait for it), false (don't)
//				refCon - the Port (not used)
//
//		Outputs:	Return Code - kIOReturnSuccess, kIOReturnExclusiveAccess, kIOReturnIOError and various others
//
//		Desc:		Set up for gated acquirePort call.
//
/****************************************************************************************************/

IOReturn nl_bjaelectronics_driver_PL2303::acquirePort(bool sleep, void *refCon)
{
    IOReturn	ret;
    IOLog("%s(%p)::acquirePort ACQUIREPORT PLEASE SET BACK TO DEBUGLOG\n", getName(), this);

    retain();
    ret = fCommandGate->runAction(acquirePortAction, (void *)sleep, (void *)refCon);
    release();
    
    return ret;
	
}/* end acquirePort */

/****************************************************************************************************/
//
//		Method:		nl_bjaelectronics_driver_PL2303::acquirePortAction
//
//		Desc:		Dummy pass through for acquirePortGated.
//
/****************************************************************************************************/

IOReturn nl_bjaelectronics_driver_PL2303::acquirePortAction(OSObject *owner, void *arg0, void *arg1, void *, void *)
{
    DEBUG_IOLog("nl_bjaelectronics_driver_PL2303::acquirePortAction\n");

    return ((nl_bjaelectronics_driver_PL2303 *)owner)->acquirePortGated((bool)arg0, (void *)arg1);
    
}/* end acquirePortAction */

/****************************************************************************************************/
//
//		Method:		nl_bjaelectronics_driver_PL2303::acquirePortGated
//
//		Inputs:		sleep - true (wait for it), false (don't), refCon - the Port
//
//		Outputs:	Return Code - kIOReturnSuccess, kIOReturnExclusiveAccess, kIOReturnIOError and various others
//
//		Desc:		acquirePort tests and sets the state of the port object.  If the port was
//					available, then the state is set to busy, and kIOReturnSuccess is returned.
//					If the port was already busy and sleep is YES, then the thread will sleep
//					until the port is freed, then re-attempts the acquire.  If the port was
//					already busy and sleep is NO, then kIOReturnExclusiveAccess is returned.
//
//
/****************************************************************************************************/

IOReturn nl_bjaelectronics_driver_PL2303::acquirePortGated( bool sleep, void *refCon )
{
    PortInfo_t          *port = (PortInfo_t *) refCon;
    UInt32              busyState = 0;
    IOReturn            rtn = kIOReturnSuccess;
	
    DEBUG_IOLog("%s(%p)::acquirePortGated\n", getName(), this);
    
    if ( fTerminate ) {
		DEBUG_IOLog("%s(%p)::acquirePortGated Port is offline\n", getName(), this);
		
		//	    return kIOReturnOffline;
	}
    SetStructureDefaults( port, FALSE );    /* Initialize all the structures */
    
    for (;;)
	{
        DEBUG_IOLog("%s(%p)::acquirePortGated readportstate\n", getName(), this);

		busyState = readPortState( port ) & PD_S_ACQUIRED;
		if ( !busyState )
		{       
			// Set busy bit, and clear everything else
			changeState( port, (UInt32)PD_S_ACQUIRED | DEFAULT_STATE, (UInt32)STATE_ALL);
			break;
		} else {
			if ( !sleep )
			{
				IOLog("%s(%p)::acquirePortGated - Busy exclusive access\n", getName(), this);
				return kIOReturnExclusiveAccess;
			} else {
				busyState = 0;
				rtn = watchState( &busyState, PD_S_ACQUIRED, refCon );
				if ( (rtn == kIOReturnIOError) || (rtn == kIOReturnSuccess) )
				{
					continue;
				} else {
					IOLog("%s(%p)::acquirePortGated - Interrupted!\n", getName(), this);
					return rtn;
				}
			}
		}
	} /* end for */
    
    fSessions++;    //bump number of active sessions and turn on clear to send
    DEBUG_IOLog("%s(%p)::acquirePortGated change state\n", getName(), this);

    changeState( port, PD_RS232_S_CTS, PD_RS232_S_CTS);

    DEBUG_IOLog("%s(%p)::acquirePortGated check serial state\n", getName(), this);

	CheckSerialState();       // turn serial on/off if appropriate
    
    return rtn;
    
}/* end acquirePort */

/****************************************************************************************************/
//
//		Method:		nl_bjaelectronics_driver_PL2303::releasePort
//
//		Inputs:		refCon - the Port (not used)
//
//		Outputs:	Return Code - kIOReturnSuccess or kIOReturnNotOpen
//
//		Desc:		Set up for gated acquirePort call.
//
/****************************************************************************************************/

IOReturn nl_bjaelectronics_driver_PL2303::releasePort(void *refCon)
{
    IOReturn	ret;
    DEBUG_IOLog("%s(%p)::releasePort\n", getName(), this);
        
    retain();
    ret = fCommandGate->runAction(releasePortAction, (void *)refCon);
    release();
    
    return ret;
    
}/* end releasePort */

/****************************************************************************************************/
//
//		Method:		nl_bjaelectronics_driver_PL2303::releasePortAction
//
//		Desc:		Dummy pass through for releasePortGated.
//
/****************************************************************************************************/

IOReturn nl_bjaelectronics_driver_PL2303::releasePortAction(OSObject *owner, void *arg0, void *, void *, void *)
{
    DEBUG_IOLog("nl_bjaelectronics_driver_PL2303::releasePortAction\n");

    return ((nl_bjaelectronics_driver_PL2303 *)owner)->releasePortGated((void *) arg0);
}/* end releasePortAction */

/****************************************************************************************************/
//
//		Method:		nl_bjaelectronics_driver_PL2303::releasePortGated
//
//		Inputs:		refCon - the Port
//
//		Outputs:	Return Code - kIOReturnSuccess or kIOReturnNotOpen
//
//		Desc:		releasePort returns all the resources and does clean up.
//
//
//
/****************************************************************************************************/

IOReturn nl_bjaelectronics_driver_PL2303::releasePortGated( void *refCon )
{
    PortInfo_t          *port = (PortInfo_t *) refCon;
    UInt32              busyState;
    DEBUG_IOLog("%s(%p)::releasePortGated\n", getName(), this);
	
    
    busyState = (readPortState( port ) & PD_S_ACQUIRED);
    if ( !busyState )
	{
		IOLog("%s(%p)::releasePortGated - port not open\n", getName(), this);
		return kIOReturnNotOpen;	
	}
    
    changeState( port, 0, (UInt32)STATE_ALL );  // Clear the entire state word which also deactivates the port
	
    fSessions--;        // reduce number of active sessions
    CheckSerialState();   // turn irda off if appropriate
	
    if ((fTerminate) && (fSessions == 0))       // if it's the result of a terminate and session count is zero we also need to close things
	{
		if (0 && fpInterface )      // jdg - this was bogus
		{
			fpInterface->close( this ); 
			fpInterface->release();
			fpInterface = NULL; 
		}
        else DEBUG_IOLog("%s(%p)::releasePortGated - would have released fpInteface here\n", getName(), this);
    }
        
    return kIOReturnSuccess;
    
}/* end releasePort */


/****************************************************************************************************/
//
//		Method:		nl_bjaelectronics_driver_PL2303::setState
//
//		Inputs:		state - state to set
//					mask - state mask
//					refCon - the Port (not used)
//
//		Outputs:	Return Code - See setStateGated
//
//		Desc:		Set up for gated setState call.
//
/****************************************************************************************************/

IOReturn nl_bjaelectronics_driver_PL2303::setState(UInt32 state, UInt32 mask, void *refCon)
{
    IOReturn	ret;
    DEBUG_IOLog("%s(%p)::setState\n", getName(), this);
    	
	// Cannot acquire or activate via setState
    
    if (mask & (PD_S_ACQUIRED | PD_S_ACTIVE | (~EXTERNAL_MASK)))
    {		
        return kIOReturnBadArgument;
    }
	
	// ignore any bits that are read-only
	
	//    mask &= (~fPort.FlowControl & PD_RS232_A_MASK) | PD_S_MASK;
	//    if (mask)
	//    {
	retain();
	ret = fCommandGate->runAction(setStateAction, (void *)state, (void *)mask, (void *)refCon);
	release();
	
	return ret;
	//    }
	
	
    return kIOReturnSuccess;
    
}/* end setState */

/****************************************************************************************************/
//
//		Method:		nl_bjaelectronics_driver_PL2303::setStateAction
//
//		Desc:		Dummy pass through for setStateGated.
//
/****************************************************************************************************/

IOReturn nl_bjaelectronics_driver_PL2303::setStateAction(OSObject *owner, void *arg0, void *arg1, void *arg2, void *)
{
    DEBUG_IOLog("nl_bjaelectronics_driver_PL2303::setStateAction\n");

    return ((nl_bjaelectronics_driver_PL2303 *)owner)->setStateGated((UInt32)arg0, (UInt32)arg1, (void *)arg2);
    
}/* end setStateAction */

/****************************************************************************************************/
//
//      Method:     nl_bjaelectronics_driver_PL2303::setState
//
//      Inputs:     state - state to set, mask - state mask, refCon - the Port
//
//      Outputs:    Return Code - kIOReturnSuccess or kIOReturnBadArgument
//
//      Desc:       Set the state for the port device.  The lower 16 bits are used to set the
//                  state of various flow control bits (this can also be done by enqueueing a
//                  PD_E_FLOW_CONTROL event).  If any of the flow control bits have been set
//                  for automatic control, then they can't be changed by setState.  For flow
//                  control bits set to manual (that are implemented in hardware), the lines
//                  will be changed before this method returns.  The one weird case is if RXO
//                  is set for manual, then an XON or XOFF character may be placed at the end
//                  of the TXQ and transmitted later.
//
/****************************************************************************************************/

IOReturn nl_bjaelectronics_driver_PL2303::setStateGated( UInt32 state, UInt32 mask, void *refCon )
{
    PortInfo_t *port = (PortInfo_t *) refCon;
    DEBUG_IOLog("%s(%p)::setStateGated\n", getName(), this);
        
    if ( mask & (PD_S_ACQUIRED | PD_S_ACTIVE | (~EXTERNAL_MASK)) )
		return kIOReturnBadArgument;
	
    if ( readPortState( port ) & PD_S_ACQUIRED )
	{
	    // ignore any bits that are read-only
		mask &= (~port->FlowControl & PD_RS232_A_MASK) | PD_S_MASK;
		
		if ( mask)
			changeState( port, state, mask );
		
		return kIOReturnSuccess;
	}
	
	DEBUG_IOLog("%s(%p)::setStateGated port not open \n", getName(), this);
    return kIOReturnNotOpen;
    
}/* end setState */

/****************************************************************************************************/
//
//		Method:		nl_bjaelectronics_driver_PL2303::watchState
//
//		Inputs:		state - state to watch for
//				mask - state mask bits
//				refCon - the Port (not used)
//
//		Outputs:	Return Code - kIOReturnSuccess or value returned from ::watchState
//
//		Desc:		Set up for gated watchState call.
//
/****************************************************************************************************/

IOReturn nl_bjaelectronics_driver_PL2303::watchState(UInt32 *state, UInt32 mask, void *refCon)
{
    IOReturn 	ret;
    DEBUG_IOLog("%s(%p)::watchState\n", getName(), this);
	    
    if (!state) 
        return kIOReturnBadArgument;
	
    if (!mask)
        return kIOReturnSuccess;
	
    retain();
    ret = fCommandGate->runAction(watchStateAction, (void *)state, (void *)mask);
    release();
    return ret;
	
}/* end watchState */

/****************************************************************************************************/
//
//		Method:		nl_bjaelectronics_driver_PL2303::watchStateAction
//
//		Desc:		Dummy pass through for watchStateGated.
//
/****************************************************************************************************/

IOReturn nl_bjaelectronics_driver_PL2303::watchStateAction(OSObject *owner, void *arg0, void *arg1, void *, void *)
{
    DEBUG_IOLog("nl_bjaelectronics_driver_PL2303::watchStateAction\n");

    return ((nl_bjaelectronics_driver_PL2303 *)owner)->watchStateGated((UInt32 *)arg0, (UInt32)arg1);
    
}/* end watchStateAction */


/****************************************************************************************************/
//
//      Method:     nl_bjaelectronics_driver_PL2303::watchState
//
//      Inputs:     state - state to watch for, mask - state mask bits, refCon - the Port
//
//      Outputs:    Return Code - kIOReturnSuccess or value returned from ::watchState
//
//      Desc:       Wait for the at least one of the state bits defined in mask to be equal
//                  to the value defined in state. Check on entry then sleep until necessary,
//                  see watchState for more details.
//
/****************************************************************************************************/

IOReturn nl_bjaelectronics_driver_PL2303::watchStateGated( UInt32 *state, UInt32 mask)
{
    IOReturn    ret = kIOReturnNotOpen;
    DEBUG_IOLog("%s(%p)::watchStateGated\n", getName(), this);
	
	
    if ( readPortState( fPort ) & PD_S_ACQUIRED )
	{
		ret = kIOReturnSuccess;
		mask &= EXTERNAL_MASK;
		ret = privateWatchState( fPort, state, mask );
		*state &= EXTERNAL_MASK;
	}
    
    return ret;
    
}/* end watchState */

/****************************************************************************************************/
//
//      Method:     nl_bjaelectronics_driver_PL2303::nextEvent
//
//      Inputs:     refCon - the Port
//
//      Outputs:    Return Code - kIOReturnSuccess
//
//      Desc:       Not used by this driver.
//
/****************************************************************************************************/

UInt32 nl_bjaelectronics_driver_PL2303::nextEvent( void *refCon )
{
    UInt32      ret = kIOReturnSuccess;
    DEBUG_IOLog("%s(%p)::nextEvent\n", getName(), this);
		
    return ret;
    
}/* end nextEvent */


/****************************************************************************************************/
//
//		Method:		nl_bjaelectronics_driver_PL2303::executeEvent
//
//		Inputs:		event - The event
//				data - any data associated with the event
//				refCon - the Port (not used)
//
//		Outputs:	Return Code - kIOReturnSuccess, kIOReturnNotOpen or kIOReturnBadArgument
//
//		Desc:		Set up for gated executeEvent call.
//
/****************************************************************************************************/

IOReturn nl_bjaelectronics_driver_PL2303::executeEvent(UInt32 event, UInt32 data, void *refCon)
{
    IOReturn 	ret;
	DEBUG_IOLog("%s(%p)::executeEventAction\n", getName(), this);
       
    retain();
    ret = fCommandGate->runAction(executeEventAction, (void *)event, (void *)data, (void *)refCon);
    release();
	
    return ret;
    
}/* end executeEvent */

/****************************************************************************************************/
//
//		Method:		nl_bjaelectronics_driver_PL2303::executeEventAction
//
//		Desc:		Dummy pass through for executeEventGated.
//
/****************************************************************************************************/

IOReturn nl_bjaelectronics_driver_PL2303::executeEventAction(OSObject *owner, void *arg0, void *arg1, void *arg2, void *)
{
	DEBUG_IOLog("nl_bjaelectronics_driver_PL2303::executeEventAction\n");

	return ((nl_bjaelectronics_driver_PL2303 *)owner)->executeEventGated((UInt32)arg0, (UInt32)arg1, (void *)arg2);
    
}/* end executeEventAction */


/****************************************************************************************************/
//
//		Method:		nl_bjaelectronics_driver_PL2303::executeEventGated
//
//
//      Inputs:     event - The event, data - any data associated with the event, refCon - the Port
//
//      Outputs:    Return Code - kIOReturnSuccess, kIOReturnNotOpen or kIOReturnBadArgument
//
//      Desc:       executeEvent causes the specified event to be processed immediately.
//                  This is primarily used for channel control commands like START & STOP
//
/****************************************************************************************************/

IOReturn nl_bjaelectronics_driver_PL2303::executeEventGated( UInt32 event, UInt32 data, void *refCon )
{
    PortInfo_t  *port = (PortInfo_t *) refCon;
    IOReturn    ret = kIOReturnSuccess;
    UInt32      state, delta;
 
	DEBUG_IOLog("%s(%p)::executeEventGated\n", getName(), this);
	   
    delta = 0;
    state = readPortState( port );  

    
    if ( (state & PD_S_ACQUIRED) == 0 )
		return kIOReturnNotOpen;
	
    switch ( event )
	{
		case PD_RS232_E_XON_BYTE:
			DEBUG_IOLog("%s(%p)::executeEvent - PD_RS232_E_XON_BYTE\n", getName(), this );
			port->XONchar = data;
			break;
		case PD_RS232_E_XOFF_BYTE:
			DEBUG_IOLog("%s(%p)::executeEvent - PD_RS232_E_XOFF_BYTE\n", getName(), this );
			port->XOFFchar = data;
			break;
		case PD_E_SPECIAL_BYTE:
			DEBUG_IOLog("%s(%p)::executeEvent - PD_E_SPECIAL_BYTE\n", getName(), this );
			port->SWspecial[ data >> SPECIAL_SHIFT ] |= (1 << (data & SPECIAL_MASK));
			break;
			
		case PD_E_VALID_DATA_BYTE:
			DEBUG_IOLog("%s(%p)::executeEvent - PD_E_VALID_DATA_BYTE\n", getName(), this );
			port->SWspecial[ data >> SPECIAL_SHIFT ] &= ~(1 << (data & SPECIAL_MASK));
			break;
			
		case PD_E_FLOW_CONTROL:
			DEBUG_IOLog("%s(%p)::executeEvent - PD_E_FLOW_CONTROL\n", getName(), this );
			
			/*         old = fPort.FlowControl;				    // save old modes for unblock checks
            fPort.FlowControl = data & (CAN_BE_AUTO | CAN_NOTIFY);  // new values, trimmed to legal values
			
			// now cleanup if we've blocked RX or TX with the previous style flow control and we're switching to a different kind
			// we have 5 different flow control modes to check and unblock; 3 on rx, 2 on tx
			if (portOpened && old && (old ^ fPort.FlowControl))		// if had some modes, and some modes are different
	    {
#define SwitchingAwayFrom(flag) ((old & flag) && !(fPort.FlowControl & flag))
				
				// if switching away from rx xon/xoff and we've sent an xoff, unblock
				if (SwitchingAwayFrom(PD_RS232_A_RXO) && fPort.xOffSent)
				{
					AddBytetoQueue(&(fPort.TX), fPort.XONchar);
					fPort.xOffSent = false;
					SetUpTransmit(&fPort);
				}
				
				// if switching away from RTS flow control and we've lowered RTS, need to raise it to unblock
				if (SwitchingAwayFrom(PD_RS232_A_RTS) && !fPort.RTSAsserted)
				{
					fPort.RTSAsserted = true;
					SccSetRTS(&fPort, true);			    // raise RTS again
				}
				
				// if switching away from DTR flow control and we've lowered DTR, need to raise it to unblock
				if (SwitchingAwayFrom(PD_RS232_A_DTR) && !fPort.DTRAsserted)
				{
					fPort.DTRAsserted = true;
					SccSetDTR(&fPort, true);			    // raise DTR again
				}
				
				// If switching away from CTS and we've paused tx, continue it
				if (SwitchingAwayFrom(PD_RS232_S_CTS) && fPort.FlowControlState != CONTINUE_SEND)
				{
					fPort.FlowControlState = CONTINUE_SEND;
					IODBDMAContinue(fPort.TxDBDMAChannel.dmaBase);		// Continue transfer
				}
				
				// If switching away from TX xon/xoff and we've paused tx, continue it
				if (SwitchingAwayFrom(PD_RS232_S_TXO) && fPort.RXOstate == NEEDS_XON)
				{
					fPort.RXOstate = NEEDS_XOFF;
					fPort.FlowControlState = CONTINUE_SEND;
					IODBDMAContinue(fPort.TxDBDMAChannel.dmaBase);		// Continue transfer
				}
	    }
			*/
			break;
			
		case PD_E_ACTIVE:
			DEBUG_IOLog("%s(%p)::executeEvent - PD_E_ACTIVE\n", getName(), this );
			if ( (bool)data )
			{
				if ( !(state & PD_S_ACTIVE) )
				{
					SetStructureDefaults( port, FALSE );
					changeState( port, (UInt32)PD_S_ACTIVE, (UInt32)PD_S_ACTIVE ); // activate port
				}
			} else {
				if ( (state & PD_S_ACTIVE) )
				{
					changeState( port, 0, (UInt32)PD_S_ACTIVE );
				}
			}
			if( SetSerialConfiguration() ){
				DEBUG_IOLog("%s(%p)::executeEvent Set Serial Configuration failed\n", getName(), this);
			}
			break;
			
		case PD_E_DATA_LATENCY:
			DEBUG_IOLog("%s(%p)::executeEvent - PD_E_DATA_LATENCY\n", getName(), this );
			port->DataLatInterval = long2tval( data * 1000 );
			break;
			
		case PD_RS232_E_MIN_LATENCY:
			DEBUG_IOLog("%s(%p)::executeEvent - PD_RS232_E_MIN_LATENCY \n", getName(), this );
			port->MinLatency = bool( data );
			break;
			
		case PD_E_DATA_INTEGRITY:
			DEBUG_IOLog("%s(%p)::executeEvent - PD_E_DATA_INTEGRITY\n", getName(), this );
			if ( (data < PD_RS232_PARITY_NONE) || (data > PD_RS232_PARITY_SPACE))
			{
				ret = kIOReturnBadArgument;
			}
			else
			{
				port->TX_Parity = data;
				port->RX_Parity = PD_RS232_PARITY_DEFAULT;          
			}
			if( SetSerialConfiguration() ){
				DEBUG_IOLog("%s(%p)::executeEvent Set Serial Configuration failed\n", getName(), this);
			}
			break;
			
		case PD_E_DATA_RATE:
			DEBUG_IOLog("%s(%p)::executeEvent - PD_E_DATA_RATE \n", getName(), this );
			/* For API compatiblilty with Intel.    */
			data >>= 1;
			DEBUG_IOLog("%s(%p)::executeEvent - actual data rate baudrate: %d \n", getName(), this, data );
			if ( (data < kMinBaudRate) || (data > kMaxBaudRate) )       // Do we really care
				ret = kIOReturnBadArgument;
			else
			{
				port->BaudRate = data;
			}       
				if( SetSerialConfiguration() ){
					DEBUG_IOLog("%s(%p)::executeEvent Set Serial Configuration failed\n", getName(), this);
				}
				break;
			
		case PD_E_DATA_SIZE:
			/* For API compatiblilty with Intel.    */
			data >>= 1;
			DEBUG_IOLog("%s(%p)::executeEvent - PD_E_DATA_SIZE: %d \n", getName(), this, data );
			
			if ( (data < 5) || (data > 8) )
				ret = kIOReturnBadArgument;
			else
			{
				
				port->CharLength = data;            
			}
				if( SetSerialConfiguration() ){
					DEBUG_IOLog("%s(%p)::executeEvent Set Serial Configuration failed\n", getName(), this);
				}
				break;
			
		case PD_RS232_E_STOP_BITS:
			DEBUG_IOLog("%s(%p)::executeEvent - PD_RS232_E_STOP_BITS\n", getName(), this );
			if ( (data < 0) || (data > 20) )
				ret = kIOReturnBadArgument;
			else
			{
				port->StopBits = data;
			}
				if( SetSerialConfiguration() ){
					DEBUG_IOLog("%s(%p)::executeEvent Set Serial Configuration failed\n", getName(), this);
				}
				break;
			
		case PD_E_RXQ_FLUSH:
			DEBUG_IOLog("%s(%p)::executeEvent - PD_E_RXQ_FLUSH \n", getName(), this );
			break;
			
		case PD_E_RX_DATA_INTEGRITY:
			DEBUG_IOLog("%s(%p)::executeEvent - PD_E_RX_DATA_INTEGRITY\n", getName(), this );
			if ( (data != PD_RS232_PARITY_DEFAULT) &&  (data != PD_RS232_PARITY_ANY) )
				ret = kIOReturnBadArgument;
			else
				port->RX_Parity = data;
			break;
			
		case PD_E_RX_DATA_RATE:
			DEBUG_IOLog("%s(%p)::executeEvent - PD_E_RX_DATA_RATE\n", getName(), this );
			if ( data )
				ret = kIOReturnBadArgument;
			break;
			
		case PD_E_RX_DATA_SIZE:
			DEBUG_IOLog("%s(%p)::executeEvent - PD_E_RX_DATA_SIZE\n", getName(), this );
			if ( data )
				ret = kIOReturnBadArgument;
			break;
			
		case PD_RS232_E_RX_STOP_BITS:
			DEBUG_IOLog("%s(%p)::executeEvent - PD_RS232_E_RX_STOP_BITS \n", getName(), this );
			if ( data )
				ret = kIOReturnBadArgument;
			break;
			
		case PD_E_TXQ_FLUSH:
			DEBUG_IOLog("%s(%p)::executeEvent - PD_E_TXQ_FLUSH\n", getName(), this );
			break;
			
		case PD_RS232_E_LINE_BREAK:
			DEBUG_IOLog("%s(%p)::executeEvent - PD_RS232_E_LINE_BREAK\n", getName(), this );
			state &= ~PD_RS232_S_BRK;
			delta |= PD_RS232_S_BRK;
			break;
			
		case PD_E_DELAY:
			DEBUG_IOLog("%s(%p)::executeEvent - PD_E_DELAY\n", getName(), this );
			port->CharLatInterval = long2tval(data * 1000);
			break;
			
		case PD_E_RXQ_SIZE:
			DEBUG_IOLog("%s(%p)::executeEvent - PD_E_RXQ_SIZE\n", getName(), this );
			break;
			
		case PD_E_TXQ_SIZE:
			DEBUG_IOLog("%s(%p)::executeEvent - PD_E_TXQ_SIZE\n", getName(), this );
			break;
			
		case PD_E_RXQ_HIGH_WATER:
			DEBUG_IOLog("%s(%p)::executeEvent - PD_E_RXQ_HIGH_WATER \n", getName(), this );
			break;
			
		case PD_E_RXQ_LOW_WATER:
			DEBUG_IOLog("%s(%p)::executeEvent - PD_E_RXQ_LOW_WATER \n", getName(), this );
			break;
			
		case PD_E_TXQ_HIGH_WATER:
			DEBUG_IOLog("%s(%p)::executeEvent - PD_E_TXQ_HIGH_WATER \n", getName(), this );
			break;
			
		case PD_E_TXQ_LOW_WATER:
			DEBUG_IOLog("%s(%p)::executeEvent - PD_E_TXQ_LOW_WATER \n", getName(), this );
			break;
			
		default:
			DEBUG_IOLog("%s(%p)::executeEvent - unrecognized event \n", getName(), this );
			ret = kIOReturnBadArgument;
			break;
	}
	
    state |= state;/* ejk for compiler warnings. ?? */
		changeState( port, state, delta );
		
		return ret;
		
}/* end executeEvent */

/****************************************************************************************************/
//
//		Method:		nl_bjaelectronics_driver_PL2303::requestEvent
//
//		Inputs:		event - The event
//					refCon - the Port (not used)
//
//		Outputs:	Return Code - kIOReturnSuccess, kIOReturnBadArgument
//					data - any data associated with the event
//
//		Desc:		call requestEventGated through the command gate.
//
/****************************************************************************************************/

IOReturn nl_bjaelectronics_driver_PL2303::requestEvent(UInt32 event, UInt32 *data, void *refCon)
{
    IOReturn 	ret;
    
	DEBUG_IOLog("%s(%p)::requestEvent\n", getName(), this);
    
    retain();
    ret = fCommandGate->runAction(requestEventAction, (void *)event, (void *)data, (void *)refCon);
    release();
    
    return ret;
    
}/* end requestEvent */

/****************************************************************************************************/
//
//		Method:		nl_bjaelectronics_driver_PL2303::requestEventAction
//
//		Desc:		Dummy pass through for requestEventGated.
//
/****************************************************************************************************/

IOReturn nl_bjaelectronics_driver_PL2303::requestEventAction(OSObject *owner, void *arg0, void *arg1, void *arg2, void *)
{
	DEBUG_IOLog("nl_bjaelectronics_driver_PL2303::requestEventAction\n");

    return ((nl_bjaelectronics_driver_PL2303 *)owner)->requestEventGated((UInt32)arg0, (UInt32 *)arg1, (void *)arg2);
    
}/* end requestEventAction */

/****************************************************************************************************/
//
//      Method:     nl_bjaelectronics_driver_PL2303::requestEvent
//
//      Inputs:     event - The event, refCon - the Port
//
//      Outputs:    Return Code - kIOReturnSuccess, kIOReturnBadArgument, data - any data associated with the event
//
//      Desc:       requestEvent processes the specified event as an immediate request and
//                  returns the results in data.  This is primarily used for getting link
//                  status information and verifying baud rate and such.
//
//					Queue access requires this be on the command gate.
//
/****************************************************************************************************/

IOReturn nl_bjaelectronics_driver_PL2303::requestEventGated( UInt32 event, UInt32 *data, void *refCon )
{
    PortInfo_t  *port = (PortInfo_t *) refCon;
    IOReturn    returnValue = kIOReturnSuccess;
	
    DEBUG_IOLog("%s(%p)::requestEventGated\n", getName(), this);
	
    if ( data == NULL ) {
		DEBUG_IOLog("%s(%p)::requestEvent - data is null\n", getName(), this );
		returnValue = kIOReturnBadArgument;
	}
	else
	{
		switch ( event )
		{
			case PD_E_ACTIVE:
				DEBUG_IOLog("%s(%p)::requestEvent - PD_E_ACTIVE\n", getName(), this);
				*data = bool(readPortState( port ) & PD_S_ACTIVE);  
				break;
				
			case PD_E_FLOW_CONTROL:
				DEBUG_IOLog("%s(%p)::requestEvent - PD_E_FLOW_CONTROL\n", getName(), this);
				*data = port->FlowControl;                          
				break;
				
			case PD_E_DELAY:
				DEBUG_IOLog("%s(%p)::requestEvent - PD_E_DELAY\n", getName(), this);
				*data = tval2long( port->CharLatInterval )/ 1000;   
				break;
				
			case PD_E_DATA_LATENCY:
				DEBUG_IOLog("%s(%p)::requestEvent - PD_E_DATA_LATENCY\n", getName(), this);
				*data = tval2long( port->DataLatInterval )/ 1000;   
				break;
				
			case PD_E_TXQ_SIZE:
				DEBUG_IOLog("%s(%p)::requestEvent - PD_E_TXQ_SIZE\n", getName(), this);
				*data = GetQueueSize( &port->TX );  
				break;
				
			case PD_E_RXQ_SIZE:
				DEBUG_IOLog("%s(%p)::requestEvent - PD_E_RXQ_SIZE\n", getName(), this);
				*data = GetQueueSize( &port->RX );  
				break;
				
			case PD_E_TXQ_LOW_WATER:
				DEBUG_IOLog("%s(%p)::requestEvent - PD_E_TXQ_LOW_WATER\n", getName(), this);
				*data = 0; 
				returnValue = kIOReturnBadArgument; 
				break;
				
			case PD_E_RXQ_LOW_WATER:
				DEBUG_IOLog("%s(%p)::requestEvent - PD_E_RXQ_LOW_WATER\n", getName(), this);
				*data = 0; 
				returnValue = kIOReturnBadArgument; 
				break;
				
			case PD_E_TXQ_HIGH_WATER:
				DEBUG_IOLog("%s(%p)::requestEvent - PD_E_TXQ_HIGH_WATER\n", getName(), this);
				*data = 0; 
				returnValue = kIOReturnBadArgument; 
				break;
				
			case PD_E_RXQ_HIGH_WATER:
				DEBUG_IOLog("%s(%p)::requestEvent - PD_E_RXQ_HIGH_WATER\n", getName(), this);
				*data = 0; 
				returnValue = kIOReturnBadArgument; 
				break;
				
			case PD_E_TXQ_AVAILABLE:
				*data = FreeSpaceinQueue( &port->TX );   
				DEBUG_IOLog("%s(%p)::requestEvent - PD_E_TXQ_AVAILABLE size: %x\n", getName(), this, *data );
				break;
				
			case PD_E_RXQ_AVAILABLE:
				DEBUG_IOLog("%s(%p)::requestEvent - PD_E_RXQ_AVAILABLE\n", getName(), this);
				*data = UsedSpaceinQueue( &port->RX );  
				break;
				
			case PD_E_DATA_RATE:
				DEBUG_IOLog("%s(%p)::requestEvent - PD_E_DATA_RATE\n", getName(), this);
				*data = port->BaudRate << 1;        
				break;
				
			case PD_E_RX_DATA_RATE:
				DEBUG_IOLog("%s(%p)::requestEvent - PD_E_RX_DATA_RATE\n", getName(), this);
				*data = 0x00;                   
				break;
				
			case PD_E_DATA_SIZE:
				DEBUG_IOLog("%s(%p)::requestEvent - PD_E_DATA_SIZE\n", getName(), this);
				*data = port->CharLength << 1;  
				break;
				
			case PD_E_RX_DATA_SIZE:
				DEBUG_IOLog("%s(%p)::requestEvent - PD_E_RX_DATA_SIZE\n", getName(), this);
				*data = 0x00;                   
				break;
				
			case PD_E_DATA_INTEGRITY:
				DEBUG_IOLog("%s(%p)::requestEvent - PD_E_DATA_INTEGRITY\n", getName(), this);
				*data = port->TX_Parity;            
				break;
				
			case PD_E_RX_DATA_INTEGRITY:
				DEBUG_IOLog("%s(%p)::requestEvent - PD_E_RX_DATA_INTEGRITY\n", getName(), this);
				*data = port->RX_Parity;            
				break;
				
			case PD_RS232_E_STOP_BITS:
				DEBUG_IOLog("%s(%p)::requestEvent - PD_RS232_E_STOP_BITS\n", getName(), this);
				*data = port->StopBits << 1;        
				break;
				
			case PD_RS232_E_RX_STOP_BITS:
				DEBUG_IOLog("%s(%p)::requestEvent - PD_RS232_E_RX_STOP_BITS\n", getName(), this);
				*data = 0x00;                   
				break;
				
			case PD_RS232_E_XON_BYTE:
				DEBUG_IOLog("%s(%p)::requestEvent - PD_RS232_E_XON_BYTE\n", getName(), this);
				*data = port->XONchar;          
				break;
				
			case PD_RS232_E_XOFF_BYTE:
				DEBUG_IOLog("%s(%p)::requestEvent - PD_RS232_E_XOFF_BYTE\n", getName(), this);
				*data = port->XOFFchar;         
				break;
				
			case PD_RS232_E_LINE_BREAK:
				DEBUG_IOLog("%s(%p)::requestEvent - PD_RS232_E_LINE_BREAK\n", getName(), this);
				*data = bool(readPortState( port ) & PD_RS232_S_BRK);
				break;
				
			case PD_RS232_E_MIN_LATENCY:
				DEBUG_IOLog("%s(%p)::requestEvent - PD_RS232_E_MIN_LATENCY\n", getName(), this);
				*data = bool( port->MinLatency );       
				break;
				
			default:
				DEBUG_IOLog("%s(%p)::requestEvent - unrecognized event\n", getName(), this);
				returnValue = kIOReturnBadArgument;             
				break;
		}
    }
	
    return kIOReturnSuccess;
    
}/* end requestEvent */

/****************************************************************************************************/
//
//      Method:     nl_bjaelectronics_driver_PL2303::enqueueEvent
//
//      Inputs:     event - The event, data - any data associated with the event, 
//                                              sleep - true (wait for it), false (don't), refCon - the Port
//
//      Outputs:    Return Code - kIOReturnSuccess, kIOReturnNotOpen
//
//      Desc:       Not used by this driver.    
//
/****************************************************************************************************/

IOReturn nl_bjaelectronics_driver_PL2303::enqueueEvent( UInt32 event, UInt32 data, bool sleep, void *refCon)
{
	DEBUG_IOLog("%s(%p)::enqueueEvent\n", getName(), this);
 
	PortInfo_t *port = (PortInfo_t *) refCon;
    	
    if ( readPortState( port ) & PD_S_ACTIVE )
	{
		return kIOReturnSuccess;
	}
	
    return kIOReturnNotOpen;
    
}/* end enqueueEvent */


/****************************************************************************************************/
//
//      Method:     nl_bjaelectronics_driver_PL2303::dequeueEvent
//
//      Inputs:     sleep - true (wait for it), false (don't), refCon - the Port
//
//      Outputs:    Return Code - kIOReturnSuccess, kIOReturnNotOpen
//
//      Desc:       Not used by this driver.        
//
/****************************************************************************************************/

IOReturn nl_bjaelectronics_driver_PL2303::dequeueEvent( UInt32 *event, UInt32 *data, bool sleep, void *refCon )
{
	DEBUG_IOLog("%s(%p)::dequeueEvent\n", getName(), this);

    PortInfo_t *port = (PortInfo_t *) refCon;
    	
    if ( (event == NULL) || (data == NULL) )
		return kIOReturnBadArgument;
	
    if ( readPortState( port ) & PD_S_ACTIVE )
	{
		return kIOReturnSuccess;
	}
	
    return kIOReturnNotOpen;
    
}/* end dequeueEvent */

/****************************************************************************************************/
//
//		Method:		nl_bjaelectronics_driver_PL2303::enqueueData
//
//		Inputs:		buffer - the data
//					size - number of bytes
//					sleep - true (wait for it), false (don't)
//					refCon - the Port (not used)
//
//		Outputs:	Return Code - kIOReturnSuccess, kIOReturnBadArgument or value returned from watchState
//					count - bytes transferred  
//
//		Desc:		set up for enqueueDataGated call.	
//
/****************************************************************************************************/

IOReturn nl_bjaelectronics_driver_PL2303::enqueueData(UInt8 *buffer, UInt32 size, UInt32 *count, bool sleep, void *refCon)
{
    IOReturn 	ret;
		
    if (count == NULL || buffer == NULL)
        return kIOReturnBadArgument;
	
    retain();
	ret = fCommandGate->runAction(enqueueDataAction, (void *)buffer, (void *)size, (void *)count, (void *)sleep);
    release();
	
    return ret;
	
}/* end enqueueData */

/****************************************************************************************************/
//
//		Method:		nl_bjaelectronics_driver_PL2303::enqueueDatatAction
//
//		Desc:		Dummy pass through for equeueDataGated.
//
/****************************************************************************************************/

IOReturn nl_bjaelectronics_driver_PL2303::enqueueDataAction(OSObject *owner, void *arg0, void *arg1, void *arg2, void *arg3)
{
    return ((nl_bjaelectronics_driver_PL2303 *)owner)->enqueueDataGated((UInt8 *)arg0, (UInt32)arg1, (UInt32 *)arg2, (bool)arg3);
    
}/* end enqueueDataAction */

/****************************************************************************************************/
//
//
//      Method:     nl_bjaelectronics_driver_PL2303::enqueueData
//
//      Inputs:     buffer - the data, size - number of bytes, sleep - true (wait for it), false (don't),
//                                                                                      refCon - the Port
//
//      Outputs:    Return Code - kIOReturnSuccess or value returned from watchState, count - bytes transferred,  
//
//      Desc:       enqueueData will attempt to copy data from the specified buffer to
//                  the TX queue as a sequence of VALID_DATA events.  The argument
//                  bufferSize specifies the number of bytes to be sent.  The actual
//                  number of bytes transferred is returned in count.
//                  If sleep is true, then this method will sleep until all bytes can be
//                  transferred.  If sleep is false, then as many bytes as possible
//                  will be copied to the TX queue.
//                  Note that the caller should ALWAYS check the transferCount unless
//                  the return value was kIOReturnBadArgument, indicating one or more
//                  arguments were not valid.  Other possible return values are
//                  kIOReturnSuccess if all requirements were met.      
//
/****************************************************************************************************/

IOReturn nl_bjaelectronics_driver_PL2303::enqueueDataGated( UInt8 *buffer, UInt32 size, UInt32 *count, bool sleep)
{
    UInt32      state = PD_S_TXQ_LOW_WATER;
    IOReturn    rtn = kIOReturnSuccess;
	
    DEBUG_IOLog("%s(%p)::enqueueDataGated\n", getName(), this);
	
    if ( fTerminate ){
		IOLog("%s(%p)::enqueueDataGated fTerminate set\n", getName(), this);
		
		return kIOReturnOffline; }
	
    if ( count == NULL || buffer == NULL ){
		IOLog("%s(%p)::enqueueDataGated buffer empty\n", getName(), this);
		
		return kIOReturnBadArgument;}
	
    *count = 0;
	
    if ( !(readPortState( fPort ) & PD_S_ACTIVE) ){
		IOLog("%s(%p)::enqueueDataGated port not open\n", getName(), this);
		
		return kIOReturnNotOpen;
	}
    	
	/* OK, go ahead and try to add something to the buffer  */
    *count = AddtoQueue( &fPort->TX, buffer, size );
    CheckQueues( fPort );
	
	/* Let the tranmitter know that we have something ready to go   */
    SetUpTransmit( );
	
	/* If we could not queue up all of the data on the first pass and   */
	/* the user wants us to sleep until it's all out then sleep */
	
    while ( (*count < size) && sleep )
	{
		state = PD_S_TXQ_LOW_WATER;
		rtn = watchStateGated( &state, PD_S_TXQ_LOW_WATER );
		if ( rtn != kIOReturnSuccess )
		{
			IOLog("%s(%p)::enqueueDataGated - interrupted\n", getName(), this);
			return rtn;
		}
		
		*count += AddtoQueue( &fPort->TX, buffer + *count, size - *count );
		CheckQueues( fPort );
		
		/* Let the tranmitter know that we have something ready to go.  */
		
		SetUpTransmit( );
	}/* end while */

    DEBUG_IOLog("%s(%p)::enqueueDataGateda - Enqueue\n", getName(), this);

    return kIOReturnSuccess;
    
}/* end enqueueData */

/****************************************************************************************************/
//
//		Method:		nl_bjaelectronics_driver_PL2303::dequeueData
//
//		Inputs:		size - buffer size
//					min - minimum bytes required
//					refCon - the Port (not used)
//
//		Outputs:	buffer - data returned
//					min - number of bytes
//					Return Code - kIOReturnSuccess, kIOReturnBadArgument, kIOReturnNotOpen, or value returned from watchState
//
//		Desc:		set up for enqueueDataGated call.
//
/****************************************************************************************************/

IOReturn nl_bjaelectronics_driver_PL2303::dequeueData(UInt8 *buffer, UInt32 size, UInt32 *count, UInt32 min, void *refCon)
{
    IOReturn 	ret;
	DEBUG_IOLog("%s(%p)::dequeueData\n", getName(), this);
		
    if ((count == NULL) || (buffer == NULL) || (min > size))
        return kIOReturnBadArgument;
	
	retain();
    ret = fCommandGate->runAction(dequeueDataAction, (void *)buffer, (void *)size, (void *)count, (void *)min);
    release();
	
    return ret;
	
	
}/* end dequeueData */

/****************************************************************************************************/
//
//		Method:		nl_bjaelectronics_driver_PL2303::dequeueDatatAction
//
//		Desc:		Dummy pass through for equeueDataGated.
//
/****************************************************************************************************/

IOReturn nl_bjaelectronics_driver_PL2303::dequeueDataAction(OSObject *owner, void *arg0, void *arg1, void *arg2, void *arg3)
{
	DEBUG_IOLog("nl_bjaelectronics_driver_PL2303::dequeueDataAction\n");

    return ((nl_bjaelectronics_driver_PL2303 *)owner)->dequeueDataGated((UInt8 *)arg0, (UInt32)arg1, (UInt32 *)arg2, (UInt32)arg3);
    
}/* end dequeueDataAction */

 /****************************************************************************************************/
 //
 //      Method:     nl_bjaelectronics_driver_PL2303::dequeueData
 //
 //      Inputs:     size - buffer size, min - minimum bytes required, refCon - the Port
 //
 //      Outputs:    buffer - data returned, min - number of bytes
 //                  Return Code - kIOReturnSuccess, kIOReturnBadArgument, kIOReturnNotOpen, or value returned from watchState
 //
 //      Desc:       dequeueData will attempt to copy data from the RX queue to the
 //                  specified buffer.  No more than bufferSize VALID_DATA events
 //                  will be transferred. In other words, copying will continue until
 //                  either a non-data event is encountered or the transfer buffer
 //                  is full.  The actual number of bytes transferred is returned
 //                  in count.
 //                  The sleep semantics of this method are slightly more complicated
 //                  than other methods in this API. Basically, this method will
 //                  continue to sleep until either min characters have been
 //                  received or a non data event is next in the RX queue.  If
 //                  min is zero, then this method never sleeps and will return
 //                  immediately if the queue is empty.
 //                  Note that the caller should ALWAYS check the transferCount
 //                  unless the return value was kIOReturnBadArgument, indicating one or
 //                  more arguments were not valid.
 //
 /****************************************************************************************************/
 
 IOReturn nl_bjaelectronics_driver_PL2303::dequeueDataGated( UInt8 *buffer, UInt32 size, UInt32 *count, UInt32 min )
 {
	 IOReturn    rtn = kIOReturnSuccess;
	 UInt32      state = 0;
	 CirQueue *Queue;
	 
	 DEBUG_IOLog("%s(%p)::dequeueDataGated\n", getName(), this);
	 
	 /* Check to make sure we have good arguments.   */
	 if ( (count == NULL) || (buffer == NULL) || (min > size) )
		 return kIOReturnBadArgument;
	 
	 /* If the port is not active then there should not be any chars.    */
	 *count = 0;
	 if ( !(readPortState( fPort ) & PD_S_ACTIVE) )
		 return kIOReturnNotOpen;
	 
	 /* Get any data living in the queue.    */
	 *count = RemovefromQueue( &fPort->RX, buffer, size );
	 
	 CheckQueues( fPort );
	 while ( (min > 0) && (*count < min) )	
	 {
		 int count_read;
		 
		 /* Figure out how many bytes we have left to queue up */
		 state = 0;
		 Queue = &fPort->RX;
		 DEBUG_IOLog("%s(%p)::dequeueDataGated - min: %d count: %d size: %d SizeQueue: %d InQueue: %d \n", getName(), this,min,*count, (size - *count), Queue->Size, Queue->InQueue );
		 
		 rtn = watchStateGated( &state, PD_S_RXQ_EMPTY );
		 
		 if ( rtn != kIOReturnSuccess )
		 {
			 IOLog("%s(%p)::dequeueDataGated - INTERRUPTED\n", getName(), this );
			 //			LogData( kUSBIn, *count, buffer );
			 return rtn;
		 }
		 /* Try and get more data starting from where we left off */
		 count_read = RemovefromQueue( &fPort->RX, buffer + *count, (size - *count) );
		 
		 *count += count_read;
		 CheckQueues( fPort );
		 
	 }/* end while */

    DEBUG_IOLog("%s(%p)::dequeueDataGated -->Out Dequeue\n", getName(), this);

    return rtn;
    
 }/* end dequeueData */


/****************************************************************************************************/
//
//		Method:		nl_bjaelectronics_driver_PL2303::getState
//
//		Inputs:		refCon - the Port (not used)
//
//		Outputs:	Return value - port state
//
//		Desc:		Get the state for the port.
//
/****************************************************************************************************/

UInt32 nl_bjaelectronics_driver_PL2303::getState(void *refCon)
{    
	DEBUG_IOLog("%s(%p)::getState\n", getName(), this);

	PortInfo_t  *port = (PortInfo_t *) refCon;
    UInt32      state;
    
    CheckQueues( port );
	
    state = readPortState( port ) & EXTERNAL_MASK;
    
    DEBUG_IOLog("%s(%p)::getState-->State: %x\n", getName(), this, state );
    
    return state;
}/* end getState */




/****************************************************************************************************/
//
//      Method:     nl_bjaelectronics_driver_PL2303::StartTransmission
//
//      Inputs:     control_length - Length of control data
//                  control_buffer - Control data
//                  data_length - Length of raw data
//                  data_buffer - raw data
//
//      Outputs:    Return code - kIOReturnSuccess
//
//      Desc:       Start the transmisson. If both control and data length is zero then only
//                  the change byte will be sent.
//
/****************************************************************************************************/

IOReturn nl_bjaelectronics_driver_PL2303::StartTransmit(UInt32 control_length, UInt8 *control_buffer, UInt32 data_length, UInt8 *data_buffer)
{
    IOReturn    ior;
    
	DEBUG_IOLog("%s(%p)::StartTransmit\n", getName(), this);
	if ( data_length != 0 )
	{
		bcopy(data_buffer, &fPipeOutBuffer[0], data_length);		
	}
		
    // add up the total length to send off to the device
    fCount = control_length + data_length;
    fpPipeOutMDP->setLength( fCount );
	
    fWriteActive = true;
#ifdef DATALOG
	UInt8 *buf;
	UInt32 buflen;
	buflen = fCount;
	buf = &fPipeOutBuffer[0];
	IOLockLock( fPort->serialRequestLock );
	DATA_IOLog("nl_bjaelectronics_driver_PL2303: Send: ");
	while ( buflen ){
		unsigned char c = *buf;
		DATA_IOLog("[%02x] ",c);
		buf++;
		buflen--;
	}
	DATA_IOLog("\n");	
	IOLockUnlock( fPort->serialRequestLock );

#endif	
    ior = fpOutPipe->Write( fpPipeOutMDP, 1000, 1000, &fWriteCompletionInfo );  // 1 second timeouts
    
    return ior;
    
}/* end StartTransmission */

/****************************************************************************************************/
//
//      Method:     nl_bjaelectronics_driver_PL2303::dataWriteComplete
//
//      Inputs:     obj - me, param - parameter block(the Port), rc - return code, remaining - what's left
//
//      Outputs:    None
//
//      Desc:       BulkOut pipe (Data interface) write completion routine
//
/****************************************************************************************************/

void nl_bjaelectronics_driver_PL2303::dataWriteComplete( void *obj, void *param, IOReturn rc, UInt32 remaining )
{
	DEBUG_IOLog("nl_bjaelectronics_driver_PL2303::dataWriteComplete\n" );

    nl_bjaelectronics_driver_PL2303  *me = (nl_bjaelectronics_driver_PL2303*)obj;
    Boolean done = true;                // write really finished?
	
    me->fWriteActive = false;
    
    // in a transmit complete, but need to manually transmit a zero-length packet
    // if it's a multiple of the max usb packet size for the bulk-out pipe (64 bytes)
    if ( rc == kIOReturnSuccess )   /* If operation returned ok:    */
    {

		if ( me->fCount > 0 )                       // Check if it was not a zero length write
		{

			if ( (me->fCount % 64) == 0 )               // If was a multiple of 64 bytes then we need to do a zero length write
			{
			
				me->fWriteActive = true;
				me->fpPipeOutMDP->setLength( 0 );
				me->fCount = 0;
				me->fpOutPipe->Write( me->fpPipeOutMDP, &me->fWriteCompletionInfo );
				done = false;               // don't complete back to irda quite yet
			}
		}
    }
    
    return;
    
}/* end dataWriteComplete */



/****************************************************************************************************/
//
//      Method:     nl_bjaelectronics_driver_PL2303::interruptReadComplete
//
//      Inputs:     obj - me, param - parameter block(the Port), rc - return code, remaining - what's left
//                                                                                  (whose idea was that?)
//
//      Outputs:    None
//
//      Desc:       Interrupt pipe read. Interrupts are used for reading handshake signals. see linux driver
//                  NOT IMPLEMENTED
//
/****************************************************************************************************/

void nl_bjaelectronics_driver_PL2303::interruptReadComplete( void *obj, void *param, IOReturn rc, UInt32 remaining )
{
	DEBUG_IOLog("nl_bjaelectronics_driver_PL2303::interruptReadComplete\n" );

    nl_bjaelectronics_driver_PL2303  *me = (nl_bjaelectronics_driver_PL2303*)obj;
	//    PortInfo_t            *port = (PortInfo_t*)param;
	//    IOReturn    ior;
    UInt32      dLen;
	
//	DEBUG_IOLog("::interruptReadComplete INTERRUPT : %p \n", rc );
	
    if ( rc == kIOReturnSuccess )   /* If operation returned ok:    */
	{
		dLen = INTERRUPT_BUFF_SIZE - remaining;
//		DEBUG_IOLog("::interruptReadComplete WE hebben een OK INTERRUPT\n");		
    	if (dLen != 1)
		{
			//			XTRACE(kLogInterruptRead, 0xdead, 0xbeef);
//			DEBUG_IOLog("::interruptReadComplete interruptReadComplete - what was that?\n");
		} else {
			
			//			check(me->fReadActive == false);
			//				if (me->fReadActive == false) {
			//					if (ior != kIOReturnSuccess)
			//						{
			//						DEBUG_IOLog("%s(%p)::interrupt complete failed to start read\n", getName(), this);
			//						} else {
			//						me->fReadActive = true;
			//						}
			//				}
		}
		
	    /* Queue the next interrupt read:   */
		
		me->fpInterruptPipe->Read( me->fpinterruptPipeMDP, &me->finterruptCompletionInfo, NULL );
		
    }
    return;
    
}/* end interruptReadComplete */


/****************************************************************************************************/
//
//      Method:     nl_bjaelectronics_driver_PL2303::dataReadComplete
//
//      Inputs:     obj - me, param - parameter block(the Port), rc - return code, remaining - what's left
//
//      Outputs:    None
//
//      Desc:       BulkIn pipe (Data interface) read completion routine
//
/****************************************************************************************************/

void nl_bjaelectronics_driver_PL2303::dataReadComplete( void *obj, void *param, IOReturn rc, UInt32 remaining )
{
	DEBUG_IOLog("nl_bjaelectronics_driver_PL2303::dataReadComplete\n");    
    nl_bjaelectronics_driver_PL2303  *me = (nl_bjaelectronics_driver_PL2303*)obj;
    PortInfo_t      *port = (PortInfo_t*)param;
    UInt16          dtlength;
    IOReturn        ior = kIOReturnSuccess;
    if ( rc == kIOReturnSuccess )   /* If operation returned ok:    */
	{
		me->fReadActive = false;
		dtlength = USBLapPayLoad - remaining;
		if ( dtlength > 0 )
		{
#ifdef DATALOG
			IOLockLock( me->fPort->serialRequestLock );
			UInt8 *buf;
			UInt32 buflen;
			buflen = dtlength;
			buf = &me->fPipeInBuffer[0];
			DATA_IOLog("nl_bjaelectronics_driver_PL2303: Receive: ");
			while ( buflen ){
				unsigned char c = *buf;
				DATA_IOLog("[%02x] ",c);
				buf++;
				buflen--;
			}
			DATA_IOLog("\n");	
			IOLockUnlock( me->fPort->serialRequestLock );
#endif	
			ior = me->AddtoQueue( &me->fPort->RX, &me->fPipeInBuffer[0], dtlength );
		}
		
		/* Queue the next read 	*/		
		ior = me->fpInPipe->Read( me->fpPipeInMDP, &me->fReadCompletionInfo, NULL );
	    
		if ( ior == kIOReturnSuccess )
		{
			me->fReadActive = true;
			me->CheckQueues( port );
			return;
		} else {
			DEBUG_IOLog("nl_bjaelectronics_driver_PL2303::dataReadComplete dataReadComplete - queueing bulk read failed\n");
		}
		
	} else {
		
		/* Read returned with error */
		DEBUG_IOLog("nl_bjaelectronics_driver_PL2303::dataReadComplete - io err %x\n",rc );
		
	}
	
    return;
    
}/* end dataReadComplete */



/****************************************************************************************************/
//
//      Method:     nl_bjaelectronics_driver_PL2303::allocateRingBuffer
//
//      Inputs:     Queue - the specified queue to allocate, BufferSize - size to allocate
//
//      Outputs:    return Code - true (buffer allocated), false (it failed)
//
//      Desc:       Allocates resources needed by the queue, then sets up all queue parameters. 
//
/****************************************************************************************************/

bool nl_bjaelectronics_driver_PL2303::allocateRingBuffer( CirQueue *Queue, size_t BufferSize )
{
    UInt8       *Buffer;
	
	// Size is ignored and kMaxCirBufferSize, which is 4096, is used.
	
    DEBUG_IOLog("%s(%p)::allocateRingBuffer\n", getName(), this );
    Buffer = (UInt8*)IOMalloc( kMaxCirBufferSize );
	
    InitQueue( Queue, Buffer, kMaxCirBufferSize );
	
    if ( Buffer )
		return true;
	
    return false;
    
}/* end allocateRingBuffer */

/****************************************************************************************************/
//
//      Method:     nl_bjaelectronics_driver_PL2303::freeRingBuffer
//
//      Inputs:     Queue - the specified queue to free
//
//      Outputs:    None
//
//      Desc:       Frees all resources assocated with the queue, then sets all queue parameters 
//                  to safe values.
//
/****************************************************************************************************/

void nl_bjaelectronics_driver_PL2303::freeRingBuffer( CirQueue *Queue )
{
    DEBUG_IOLog("%s(%p)::freeRingBuffer\n", getName(), this );
    if( !(Queue->Start) )  goto Bogus;
    
    IOFree( Queue->Start, Queue->Size );
    CloseQueue( Queue );
	
Bogus:
		return;
    
}/* end freeRingBuffer */




/****************************************************************************************************/
//
//      Method:     nl_bjaelectronics_driver_PL2303::SetSpeed
//
//      Inputs:     brate - the requested baud rate
//
//      Outputs:    return word - baud coding
//
//      Desc:       Set the baudrate for the device 
//
/****************************************************************************************************/  

IOReturn nl_bjaelectronics_driver_PL2303::SetSerialConfiguration( void )
{
	IOReturn rtn;
	IOUSBDevRequest request;
	char buf[10];	
    DEBUG_IOLog("%s(%p)::SetSerialConfiguration baudrate: %d \n", getName(), this, fPort->BaudRate );
	
	memset(buf, 0x00, 0x07); //WARNING IS THIS ALLOWED IN KERNEL SPACE ?
    
    fCurrentBaud = fPort->BaudRate;
    
    switch (fPort->BaudRate){
		case 75: 
			fBaudCode = kLinkSpeed75;     // 0x01
			break;
		case 150: 
			fBaudCode = kLinkSpeed150;     // 0x01
			break;
		case 300: 
			fBaudCode = kLinkSpeed300;     // 0x01
			break;
		case 600: 
			fBaudCode = kLinkSpeed600;     // 0x01
			break;	
		case 1200:
			fBaudCode = kLinkSpeed1200;     // 0x01
			break;	 
		case 1800: 
			fBaudCode = kLinkSpeed1800;     // 0x01
			break;
		case 2400: 
			fBaudCode = kLinkSpeed2400;     // 0x01
			break;
		case 3600:
			fBaudCode = kLinkSpeed3600;     // 0x01
			break;	 
		case 4800:
			fBaudCode = kLinkSpeed4800;     // 0x01
			break;	 
		case 7200:
			fBaudCode = kLinkSpeed7200;     // 0x01
			break;	 	    
		case 9600: 
			fBaudCode = kLinkSpeed9600;     // 0x02
			break;	    
		case 19200: 
			fBaudCode = kLinkSpeed19200;    // 0x03
			break;	    
		case 38400: 
			fBaudCode = kLinkSpeed38400;    // 0x04
			break;	    
		case 57600: 
			fBaudCode = kLinkSpeed57600;    // 0x05
			break;	    
		case 115200:
			fBaudCode = kLinkSpeed115200;   // 0x06
			break;	    
		case 230400:
			fBaudCode = kLinkSpeed230400;   // 0x07
			break;	    
		case 460800:
			fBaudCode = kLinkSpeed460800;  // 0x08
			break;	    
			
			
		default:
			IOLog("%s(%p)::SetSerialConfiguration - Unsupported baud rate\n", getName(), this);
			fBaudCode = 0;
			break;
    }
	
	if(fBaudCode) {
		buf[0] = fBaudCode & 0xff;
		buf[1] = (fBaudCode >> 8) & 0xff;
		buf[2] = (fBaudCode >> 16) & 0xff;
		buf[3] = (fBaudCode >> 24) & 0xff;
	}
	
    switch (fPort->StopBits) {
        case 0:
            buf[4] = 0;
            break;
            
        case 2:
            buf[4] = 0; // 1 stop bit
            break;
            
        case 3:
            buf[4] = 1; // 1.5 stop bits
            break;
            
        case 4:
            buf[4] = 2; // 2 stop bits
            break;
            
        default:
            buf[4] = 0;
            break;
    }
	DEBUG_IOLog("%s(%p)::SetSerialConfiguration - StopBits: %d \n", getName(), this,  buf[4]);
	
	
    switch(fPort->TX_Parity)
    {
        case PD_RS232_PARITY_NONE:
            buf[5] = 0;
			DEBUG_IOLog("%s(%p)::SetSerialConfiguration - PARITY_NONE \n", getName(), this);
            break;
            
        case PD_RS232_PARITY_ODD:
            buf[5] = 1;
			DEBUG_IOLog("%s(%p)::SetSerialConfiguration - PARITY_ODD \n", getName(), this);
            break;
            
        case PD_RS232_PARITY_EVEN:
            buf[5] = 2;
			DEBUG_IOLog("%s(%p)::SetSerialConfiguration - PARITY_EVEN \n", getName(), this);
            break;
            
        case PD_RS232_PARITY_MARK:
			buf[5] = 3;
			DEBUG_IOLog("%s(%p)::SetSerialConfiguration - PARITY_MARK \n", getName(), this);
			break;
			
		case PD_RS232_PARITY_SPACE:
			buf[5] = 4;
			DEBUG_IOLog("%s(%p)::SetSerialConfiguration - PARITY_SPACE \n", getName(), this);
			break;
			
        default:
			buf[5] = 0;
			DEBUG_IOLog("%s(%p)::SetSerialConfiguration - PARITY_NONE \n", getName(), this);
    }
	
	if (fPort->CharLength >= 5 && fPort->CharLength <= 8){
		buf[6] = fPort->CharLength;
    }
	DEBUG_IOLog("%s(%p)::SetSerialConfiguration - Bits: %d \n", getName(), this,  buf[6]);
	
	request.bmRequestType = USBmakebmRequestType(kUSBOut, kUSBClass, kUSBInterface);
    request.bRequest = SET_LINE_REQUEST;
	request.wValue =  0; 
	request.wIndex = 0;
	request.wLength = 7;
	request.pData = buf;
	rtn =  fpDevice->DeviceRequest(&request);
	DEBUG_IOLog("%s(%p)::SetSerialConfiguration - return: %p \n", getName(), this,  rtn);
	
	return rtn;
}/* end SetSpeed */


/* QueuePrimatives  */

/****************************************************************************************************/
//
//      Method:     nl_bjaelectronics_driver_PL2303::AddBytetoQueue
//
//      Inputs:     Queue - the queue to be added to
//
//      Outputs:    Value - Byte to be added, Queue status - full or no error
//
//      Desc:       Add a byte to the circular queue.
//
/****************************************************************************************************/

QueueStatus nl_bjaelectronics_driver_PL2303::AddBytetoQueue( CirQueue *Queue, char Value )
{
    /* Check to see if there is space by comparing the next pointer,    */
    /* with the last, If they match we are either Empty or full, so     */
    /* check the InQueue of being zero.                 */
    DEBUG_IOLog("nl_bjaelectronics_driver_PL2303(%p)::AddBytetoQueue\n", this );
	
    if ( !(fPort && fPort->serialRequestLock ) ) goto Fail;
    IOLockLock( fPort->serialRequestLock );
	
    if ( (Queue->NextChar == Queue->LastChar) && Queue->InQueue ) {
		IOLockUnlock( fPort->serialRequestLock);
		return queueFull;
	}
	
    *Queue->NextChar++ = Value;
    Queue->InQueue++;
	
	/* Check to see if we need to wrap the pointer. */
	
    if ( Queue->NextChar >= Queue->End )
		Queue->NextChar =  Queue->Start;
	
    IOLockUnlock( fPort->serialRequestLock);
    return queueNoError;
    
Fail:
		return queueFull;       // for lack of a better error
    
}/* end AddBytetoQueue */

/****************************************************************************************************/
//
//      Method:     nl_bjaelectronics_driver_PL2303::GetBytetoQueue
//
//      Inputs:     Queue - the queue to be removed from
//
//      Outputs:    Value - where to put the byte, Queue status - empty or no error
//
//      Desc:       Remove a byte from the circular queue.
//
/****************************************************************************************************/

QueueStatus nl_bjaelectronics_driver_PL2303::GetBytetoQueue( CirQueue *Queue, UInt8 *Value )
{
    DEBUG_IOLog("%s(%p)::GetBytetoQueue\n", getName(), this );
	
    if( !(fPort && fPort->serialRequestLock) ) goto Fail;
    IOLockLock( fPort->serialRequestLock );
	
	/* Check to see if the queue has something in it.   */
	
    if ( (Queue->NextChar == Queue->LastChar) && !Queue->InQueue ) {
		IOLockUnlock(fPort->serialRequestLock);
		return queueEmpty;
	}
	
    *Value = *Queue->LastChar++;
    Queue->InQueue--;
	
	/* Check to see if we need to wrap the pointer. */
	
    if ( Queue->LastChar >= Queue->End )
		Queue->LastChar =  Queue->Start;
	
    IOLockUnlock(fPort->serialRequestLock);
    return queueNoError;
    
Fail:
		return queueEmpty;          // can't get to it, pretend it's empty
    
}/* end GetBytetoQueue */

/****************************************************************************************************/
//
//      Method:     nl_bjaelectronics_driver_PL2303::InitQueue
//
//      Inputs:     Queue - the queue to be initialized, Buffer - the buffer, size - length of buffer
//
//      Outputs:    Queue status - queueNoError.
//
//      Desc:       Pass a buffer of memory and this routine will set up the internal data structures.
//
/****************************************************************************************************/

QueueStatus nl_bjaelectronics_driver_PL2303::InitQueue( CirQueue *Queue, UInt8 *Buffer, size_t Size )
{
    DEBUG_IOLog("%s(%p)::InitQueue\n", getName(), this );

    Queue->Start    = Buffer;
    Queue->End      = (UInt8*)((size_t)Buffer + Size);
    Queue->Size     = Size;
    Queue->NextChar = Buffer;
    Queue->LastChar = Buffer;
    Queue->InQueue  = 0;
	
    IOSleep( 1 );
    
    return queueNoError ;
    
}/* end InitQueue */

/****************************************************************************************************/
//
//      Method:     nl_bjaelectronics_driver_PL2303::CloseQueue
//
//      Inputs:     Queue - the queue to be closed
//
//      Outputs:    Queue status - queueNoError.
//
//      Desc:       Clear out all of the data structures.
//
/****************************************************************************************************/

QueueStatus nl_bjaelectronics_driver_PL2303::CloseQueue( CirQueue *Queue )
{
    DEBUG_IOLog("%s(%p)::CloseQueue\n", getName(), this );
	
    Queue->Start    = 0;
    Queue->End      = 0;
    Queue->NextChar = 0;
    Queue->LastChar = 0;
    Queue->Size     = 0;
	
    return queueNoError;
    
}/* end CloseQueue */

/****************************************************************************************************/
//
//      Method:     nl_bjaelectronics_driver_PL2303::AddtoQueue
//
//      Inputs:     Queue - the queue to be added to, Buffer - data to add, Size - length of data
//
//      Outputs:    BytesWritten - Number of bytes actually put in the queue.
//
//      Desc:       Add an entire buffer to the queue.
//
/****************************************************************************************************/

size_t nl_bjaelectronics_driver_PL2303::AddtoQueue( CirQueue *Queue, UInt8 *Buffer, size_t Size )
{
    size_t      BytesWritten = 0;
    DEBUG_IOLog("%s(%p)::AddtoQueue\n", getName(), this );
	
    while ( FreeSpaceinQueue( Queue ) && (Size > BytesWritten) )
	{
		AddBytetoQueue( Queue, *Buffer++ );
		BytesWritten++;
	}
	
    return BytesWritten;
    
}/* end AddtoQueue */

/****************************************************************************************************/
//
//      Method:     nl_bjaelectronics_driver_PL2303::RemovefromQueue
//
//      Inputs:     Queue - the queue to be removed from, Size - size of buffer
//
//      Outputs:    Buffer - Where to put the data, BytesReceived - Number of bytes actually put in Buffer.
//
//      Desc:       Get a buffers worth of data from the queue.
//
/****************************************************************************************************/

size_t nl_bjaelectronics_driver_PL2303::RemovefromQueue( CirQueue *Queue, UInt8 *Buffer, size_t MaxSize )
{
    size_t      BytesReceived = 0;
    UInt8       Value;
    DEBUG_IOLog("%s(%p)::RemovefromQueue\n", getName(), this );
    
    while( (MaxSize > BytesReceived) && (GetBytetoQueue(Queue, &Value) == queueNoError) ) 
	{
		*Buffer++ = Value;
		BytesReceived++;
	}/* end while */
	
    return BytesReceived;
    
}/* end RemovefromQueue */

/****************************************************************************************************/
//
//      Method:     nl_bjaelectronics_driver_PL2303::FreeSpaceinQueue
//
//      Inputs:     Queue - the queue to be queried
//
//      Outputs:    Return Value - Free space left
//
//      Desc:       Return the amount of free space left in this buffer.
//
/****************************************************************************************************/

size_t nl_bjaelectronics_driver_PL2303::FreeSpaceinQueue( CirQueue *Queue )
{
    size_t  retVal = 0;
    DEBUG_IOLog("%s(%p)::FreeSpaceinQueue\n", getName(), this );
	
    if( !(fPort && fPort->serialRequestLock ) ) goto Fail;
	IOLockLock( fPort->serialRequestLock );
	
    retVal = Queue->Size - Queue->InQueue;
    
    IOLockUnlock(fPort->serialRequestLock);
    
Fail:
		return retVal;
    
}/* end FreeSpaceinQueue */

/****************************************************************************************************/
//
//      Method:     nl_bjaelectronics_driver_PL2303::UsedSpaceinQueue
//
//      Inputs:     Queue - the queue to be queried
//
//      Outputs:    UsedSpace - Amount of data in buffer
//
//      Desc:       Return the amount of data in this buffer.
//
/****************************************************************************************************/

size_t nl_bjaelectronics_driver_PL2303::UsedSpaceinQueue( CirQueue *Queue )
{
    DEBUG_IOLog("%s(%p)::UsedSpaceinQueue\n", getName(), this );

    return Queue->InQueue;
    
}/* end UsedSpaceinQueue */

/****************************************************************************************************/
//
//      Method:     nl_bjaelectronics_driver_PL2303::GetQueueSize
//
//      Inputs:     Queue - the queue to be queried
//
//      Outputs:    QueueSize - The size of the queue.
//
//      Desc:       Return the total size of the queue.
//
/****************************************************************************************************/

size_t nl_bjaelectronics_driver_PL2303::GetQueueSize( CirQueue *Queue )
{
    DEBUG_IOLog("%s(%p)::GetQueueSize\n", getName(), this );

    return Queue->Size;
    
}/* end GetQueueSize */

/****************************************************************************************************/
//
//      Method:     nl_bjaelectronics_driver_PL2303::GetQueueStatus
//
//      Inputs:     Queue - the queue to be queried
//
//      Outputs:    Queue status - full, empty or no error
//
//      Desc:       Returns the status of the circular queue.
//
/****************************************************************************************************/
/*
 QueueStatus nl_bjaelectronics_driver_PL2303::GetQueueStatus( CirQueue *Queue )
 {
	 if ( (Queue->NextChar == Queue->LastChar) && Queue->InQueue )
		 return queueFull;
	 else if ( (Queue->NextChar == Queue->LastChar) && !Queue->InQueue )
		 return queueEmpty;
	 
	 return queueNoError ;
	 
 }*/ /* end GetQueueStatus */

/****************************************************************************************************/
//
//      Method:     nl_bjaelectronics_driver_PL2303::CheckQueues
//
//      Inputs:     port - the port to check
//
//      Outputs:    None
//
//      Desc:       Checks the various queue's etc and manipulates the state(s) accordingly
//
/****************************************************************************************************/

void nl_bjaelectronics_driver_PL2303::CheckQueues( PortInfo_t *port )
{
    unsigned long   Used;
    unsigned long   Free;
    unsigned long   QueuingState;
    unsigned long   DeltaState;
    DEBUG_IOLog("%s(%p)::CheckQueues\n", getName(), this );
	
    // Initialise the QueueState with the current state.
	
    QueuingState = readPortState( port );
	
	/* Check to see if there is anything in the Transmit buffer. */
	
    Used = UsedSpaceinQueue( &port->TX );
    Free = FreeSpaceinQueue( &port->TX );
    if ( Free == 0 )
	{
		QueuingState |=  PD_S_TXQ_FULL;
		QueuingState &= ~PD_S_TXQ_EMPTY;
	}
	else if ( Used == 0 )
	{
		QueuingState &= ~PD_S_TXQ_FULL;
		QueuingState |=  PD_S_TXQ_EMPTY;
	}
	else
	{
		QueuingState &= ~PD_S_TXQ_FULL;
		QueuingState &= ~PD_S_TXQ_EMPTY;
	}
		
	
	/* Check to see if we are below the low water mark. */
    if ( Used < port->TXStats.LowWater )
		QueuingState |=  PD_S_TXQ_LOW_WATER;
	else QueuingState &= ~PD_S_TXQ_LOW_WATER;
	
    if ( Used > port->TXStats.HighWater )
		QueuingState |= PD_S_TXQ_HIGH_WATER;
	else QueuingState &= ~PD_S_TXQ_HIGH_WATER;
		
	
	/* Check to see if there is anything in the Receive buffer. */
    Used = UsedSpaceinQueue( &port->RX );
    Free = FreeSpaceinQueue( &port->RX );
	
    if ( Free == 0 )
	{
		QueuingState |= PD_S_RXQ_FULL;
		QueuingState &= ~PD_S_RXQ_EMPTY;
	}
	else if ( Used == 0 )
	{
		QueuingState &= ~PD_S_RXQ_FULL;
		QueuingState |= PD_S_RXQ_EMPTY;
	}
	else
	{
		QueuingState &= ~PD_S_RXQ_FULL;
		QueuingState &= ~PD_S_RXQ_EMPTY;
	}
	
	/* Check to see if we are below the low water mark. */
    if ( Used < port->RXStats.LowWater )
		QueuingState |= PD_S_RXQ_LOW_WATER;
	else QueuingState &= ~PD_S_RXQ_LOW_WATER;
	
    if ( Used > port->RXStats.HighWater )
		QueuingState |= PD_S_RXQ_HIGH_WATER;
	else QueuingState &= ~PD_S_RXQ_HIGH_WATER;
	
	/* Figure out what has changed to get mask.*/
    DeltaState = QueuingState ^ readPortState( port );
    changeState( port, QueuingState, DeltaState );
	//    		DEBUG_IOLog("%s(%p)::READY \n", getName(), this);
	
    return;
    
}/* end CheckQueues */


/****************************************************************************************************/
//
//      Method:     nl_bjaelectronics_driver_PL2303::SetUpTransmit
//
//      Inputs:
//
//      Outputs:    return code - true (transmit started), false (transmission already in progress)
//
//      Desc:       Setup and then start transmisson on the channel specified
//
/****************************************************************************************************/

bool nl_bjaelectronics_driver_PL2303::SetUpTransmit( void )
{	
    size_t      count = 0;
    size_t      data_Length;
    UInt8       *TempOutBuffer;
	
	DEBUG_IOLog("%s(%p)::SetUpTransmit\n", getName(), this);
    
	//  If we are already in the cycle of transmitting characters,
	//  then we do not need to do anything.
	
    if ( fPort->AreTransmitting == TRUE )
		return false;
	
	// First check if we can actually do anything, also if IrDA has no room we're done for now
	
    //if ( GetQueueStatus( &fPort->TX ) != queueEmpty )
    if (UsedSpaceinQueue(&fPort->TX) > 0)
	{
		//	data_Length = fIrDA->TXBufferAvailable();
		//	if ( data_Length == 0 )
		//	{
		//	    return false;
		//	}
		
		if ( data_Length > MAX_BLOCK_SIZE )
		{
			data_Length = MAX_BLOCK_SIZE;
		}
		
		TempOutBuffer = (UInt8*)IOMalloc( data_Length );
		if ( !TempOutBuffer )
		{
			//			DEBUG_IOLog("%s(%p)::SetUpTransmit - buffer allocation problem\n", getName(), this);
			return false;
		}
		bzero( TempOutBuffer, data_Length );
		
		// Fill up the buffer with characters from the queue
		
		count = RemovefromQueue( &fPort->TX, TempOutBuffer, data_Length );
		
		fPort->AreTransmitting = TRUE;
		changeState( fPort, PD_S_TX_BUSY, PD_S_TX_BUSY );
		
		StartTransmit(0, NULL, count, TempOutBuffer );      // do the "transmit" -- send to IrCOMM
		
		changeState( fPort, 0, PD_S_TX_BUSY );
		fPort->AreTransmitting = false;
		
		IOFree( TempOutBuffer, data_Length );
		//if ( tCount != count )
		//	{
		//	    ELG( tCount, count, 'IrW-', "SetUpTransmit - IrDA write problem, data has been dropped" );
		//	    return false;
		//	}
		
		// We potentially removed a bunch of stuff from the
		// queue, so see if we can free some thread(s)
		// to enqueue more stuff.
		
		CheckQueues( fPort );
    }
	
    return true;
    
}/* end SetUpTransmit */
