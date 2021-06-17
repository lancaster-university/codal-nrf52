/*
The MIT License (MIT)

Copyright (c) 2021 Lancaster University.

Permission is hereby granted, free of charge, to any person obtaining a
copy of this software and associated documentation files (the "Software"),
to deal in the Software without restriction, including without limitation
the rights to use, copy, modify, merge, publish, distribute, sublicense,
and/or sell copies of the Software, and to permit persons to whom the
Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
DEALINGS IN THE SOFTWARE.
*/
#include "CodalUSB.h"
#include "Timer.h"
#include "Event.h"

#if CONFIG_ENABLED(DEVICE_USB)
#include "CodalDmesg.h"
#include "nrfx_usbd.h"
#include "nrfx_power.h"
#include "nrfx_clock.h"
#include "nrfx_usbd_errata.h"

using namespace codal;

#define LOG(...) ((void)0)
// #define LOG DMESG
#define DBG(...) ((void)0)
// #define DBG DMESG

#define NUM_IN_EP NRF_USBD_EPIN_CNT
#define NUM_OUT_EP NRF_USBD_EPOUT_CNT

#define CHK(call)                                                                                  \
    do                                                                                             \
    {                                                                                              \
        int status = call;                                                                        \
        usb_assert(status == NRFX_SUCCESS);                                                              \
    } while (0)

#if 0
static UsbEndpointIn *findInEp(int ep)
{
    auto cusb = CodalUSB::usbInstance;
    ep &= 0x7f;
    if (ep == 0)
        return cusb->ctrlIn;
    for (auto iface = cusb->interfaces; iface; iface = iface->next)
    {
        if (iface->in && iface->in->ep == ep)
            return iface->in;
    }
    return NULL;
}

static UsbEndpointOut *findOutEp(int ep)
{
    auto cusb = CodalUSB::usbInstance;
    if (ep == 0)
        return cusb->ctrlOut;
    for (auto iface = cusb->interfaces; iface; iface = iface->next)
    {
        if (iface->out && iface->out->ep == ep)
            return iface->out;
    }
    return NULL;
}
#endif

inline void usb_pwr_detected()
{
    LOG("DETECT");
    if (NRF_USBD->ENABLE == 0)
    {
        NRF_USBD->EVENTCAUSE = USBD_EVENTCAUSE_READY_Msk;
        if ( nrfx_usbd_errata_187() )
        {
            if ( *((volatile uint32_t *) (0x4006EC00)) == 0x00000000 )
            {
                *((volatile uint32_t *) (0x4006EC00)) = 0x00009375;
                *((volatile uint32_t *) (0x4006ED14)) = 0x00000003;
                *((volatile uint32_t *) (0x4006EC00)) = 0x00009375;
            }
            else
            {
                *((volatile uint32_t *) (0x4006ED14)) = 0x00000003;
            }
        }

        if ( nrfx_usbd_errata_171() )
        {
            if ( *((volatile uint32_t *) (0x4006EC00)) == 0x00000000 )
            {
                *((volatile uint32_t *) (0x4006EC00)) = 0x00009375;
                *((volatile uint32_t *) (0x4006EC14)) = 0x000000C0;
                *((volatile uint32_t *) (0x4006EC00)) = 0x00009375;
            }
            else
            {
                *((volatile uint32_t *) (0x4006EC14)) = 0x000000C0;
            }
        }
        nrf_clock_event_clear(NRF_CLOCK, NRF_CLOCK_EVENT_HFCLKSTARTED);
        nrf_clock_task_trigger(NRF_CLOCK, NRF_CLOCK_TASK_HFCLKSTART);
        NRF_USBD->ENABLE = 1;
    }
    Event(CodalUSB::usbInstance->id, USB_EVT_CONNECTED);
}

inline void usb_pwr_ready()
{
    LOG("PWRREADY");
    NRF_USBD->EVENTCAUSE = USBD_EVENTCAUSE_READY_Msk;
    if ( nrfx_usbd_errata_187() )
    {
        if ( *((volatile uint32_t *) (0x4006EC00)) == 0x00000000 )
        {
            *((volatile uint32_t *) (0x4006EC00)) = 0x00009375;
            *((volatile uint32_t *) (0x4006ED14)) = 0x00000003;
            *((volatile uint32_t *) (0x4006EC00)) = 0x00009375;
        }
        else
        {
            *((volatile uint32_t *) (0x4006ED14)) = 0x00000003;
        }
    }

    if ( nrfx_usbd_errata_171() )
    {
        if ( *((volatile uint32_t *) (0x4006EC00)) == 0x00000000 )
        {
            *((volatile uint32_t *) (0x4006EC00)) = 0x00009375;
            *((volatile uint32_t *) (0x4006EC14)) = 0x000000C0;
            *((volatile uint32_t *) (0x4006EC00)) = 0x00009375;
        }
        else
        {
            *((volatile uint32_t *) (0x4006EC14)) = 0x000000C0;
        }
    }
    if ( nrfx_usbd_errata_166() )
    {
        *((volatile uint32_t *) (NRF_USBD_BASE + 0x800)) = 0x7E3;
        *((volatile uint32_t *) (NRF_USBD_BASE + 0x804)) = 0x40;

        __ISB(); __DSB();
    }

    nrf_usbd_int_enable(NRF_USBD,   NRF_USBD_INT_USBRESET_MASK     |
                                    NRF_USBD_INT_STARTED_MASK      |
                                    NRF_USBD_INT_ENDEPIN0_MASK     |
                                    NRF_USBD_INT_EP0DATADONE_MASK  |
                                    NRF_USBD_INT_ENDEPOUT0_MASK    |
                                    NRF_USBD_INT_USBEVENT_MASK     |
                                    NRF_USBD_INT_EP0SETUP_MASK     |
                                    NRF_USBD_INT_DATAEP_MASK);

    nrf_usbd_event_get_and_clear(NRF_USBD, NRF_USBD_EVENT_USBEVENT);

    NVIC_SetVector(USBD_IRQn, (uint32_t)USBD_IRQHandler);
    NVIC_ClearPendingIRQ(USBD_IRQn);
    NVIC_SetPriority(USBD_IRQn, 7);
    NVIC_EnableIRQ(USBD_IRQn);

    while(!nrf_clock_event_check(NRF_CLOCK, NRF_CLOCK_EVENT_HFCLKSTARTED));
    nrf_usbd_pullup_enable(NRF_USBD);

    Event(CodalUSB::usbInstance->id, USB_EVT_READY);
}

extern "C" void POWER_CLOCK_IRQHandler() {
    LOG("PCIRQ");
    if (nrf_power_event_get_and_clear(NRF_POWER, NRF_POWER_EVENT_USBDETECTED))
    {
        usb_pwr_detected();
    }

    if (nrf_power_event_get_and_clear(NRF_POWER, NRF_POWER_EVENT_USBREMOVED))
    {
        LOG("REMOVE");
        NRF_USBD->USBPULLUP = 0;
        NVIC_DisableIRQ(USBD_IRQn);
        NRF_USBD->INTENCLR = NRF_USBD->INTEN;
        NRF_USBD->ENABLE = 0;
        Event(CodalUSB::usbInstance->id, USB_EVT_REMOVED);
    }

    if (nrf_power_event_get_and_clear(NRF_POWER, NRF_POWER_EVENT_USBPWRRDY))
    {
        usb_pwr_ready();
    }
}

static volatile uint32_t ep_status = 0;

extern "C" void USBD_IRQHandler(void) {

    uint32_t enabled = nrf_usbd_int_enable_get(NRF_USBD);

    uint32_t set = 0;

    // walk inten and store active interrupts in set whilst clearing events.
    for (int i = 0; i < 25; i++) {
        if ((enabled & (1 << i)) && nrf_usbd_event_get_and_clear(NRF_USBD, (nrf_usbd_event_t)nrfx_bitpos_to_event(i)))
            set |= 1 << i;
    }

    LOG("SET %x ENABLED %x",set,enabled);

    if (set & USBD_INTEN_USBRESET_Msk) {
        LOG("RESET");
        CodalUSB::usbInstance->initEndpoints();
    }

    if (set & USBD_INTEN_USBEVENT_Msk) {
        uint32_t event = nrf_usbd_eventcause_get_and_clear(NRF_USBD);
        LOG("usb event %d", event);
        // if (event & NRF_USBD_EVENTCAUSE_ISOOUTCRC_MASK);
        if (event & NRF_USBD_EVENTCAUSE_SUSPEND_MASK)
        {
            LOG("USB suspend");
        }
        if (event & NRF_USBD_EVENTCAUSE_RESUME_MASK)
        {
            LOG("USB resume");
        }
        if (event & NRF_USBD_EVENTCAUSE_WUREQ_MASK)
        {
            LOG("USB wakeup");
        }
        if (event & NRF_USBD_EVENTCAUSE_READY_MASK)
        {
            LOG("USB READY");
        }
    }

    if (set & USBD_INTEN_EP0SETUP_Msk)
    {
        NRF_USBD->TASKS_EP0RCVOUT = 1;

        USBSetup stp;
        stp.bmRequestType = nrf_usbd_setup_bmrequesttype_get(NRF_USBD);
        stp.bRequest      = nrf_usbd_setup_brequest_get(NRF_USBD);
        uint16_t wValue = nrf_usbd_setup_wvalue_get(NRF_USBD);
        stp.wValueL       = wValue & 0xff;
        stp.wValueH       = (wValue >> 8);
        stp.wIndex        = nrf_usbd_setup_windex_get(NRF_USBD);
        stp.wLength       = nrf_usbd_setup_wlength_get(NRF_USBD);
        CodalUSB::usbInstance->setupRequest(stp);
    }

    if (set & USBD_INTEN_EPDATA_Msk)
    {
        ep_status = NRF_USBD->EPDATASTATUS;
        NRF_USBD->EPDATASTATUS = ep_status;

        LOG("EPDATASTATUS %d",ep_status);

        if (ep_status & 0xff0000)
        {
            LOG("EP READ!");
            CodalUSB::usbInstance->interruptHandler();
        }
    }
}

void usb_configure(uint8_t numEndpoints)
{
    LOG("USB_CFG");
    NRF_USBD->ENABLE = 0;

    // configure power interrupts for events when usb has been connected.
    NRF_POWER->INTENSET = NRF_POWER_INT_USBDETECTED_MASK |
                       NRF_POWER_INT_USBREMOVED_MASK  |
                       NRF_POWER_INT_USBPWRRDY_MASK;

    NVIC_SetVector(POWER_CLOCK_IRQn, (uint32_t)POWER_CLOCK_IRQHandler);
    NVIC_SetPriority(POWER_CLOCK_IRQn, NRFX_POWER_DEFAULT_CONFIG_IRQ_PRIORITY);
    NVIC_EnableIRQ(POWER_CLOCK_IRQn);

    // already connected... immediately configure usb...
    if (nrf_power_usbregstatus_vbusdet_get(NRF_POWER))
        usb_pwr_detected();
    if (nrf_power_usbregstatus_outrdy_get(NRF_POWER))
        usb_pwr_ready();
}

void usb_set_address(uint16_t wValue)
{
    // handled automatically by the hardware.
}

void usb_set_address_pre(uint16_t wValue)
{
    // handled automatically by the hardware.
}

int UsbEndpointIn::clearStall()
{
    LOG("clear stall IN %d", ep);
    nrf_usbd_ep_unstall(NRF_USBD, (nrfx_usbd_ep_t)(ep | NRF_USBD_EP_DIR_IN));
    nrf_usbd_dtoggle_set(NRF_USBD, (nrfx_usbd_ep_t)(ep | NRF_USBD_EP_DIR_IN), NRF_USBD_DTOGGLE_DATA0);
    wLength = 0;
    return DEVICE_OK;
}

int UsbEndpointIn::reset()
{
    LOG("reset IN %d", ep);
    // TODO?
    wLength = 0;
    return DEVICE_OK;
}

int UsbEndpointIn::stall()
{
    if (ep == 0)
        NRF_USBD->TASKS_EP0STALL = 1;
    else
        nrf_usbd_ep_stall(NRF_USBD, (nrfx_usbd_ep_t)(ep | NRF_USBD_EP_DIR_IN));

    wLength = 0;
    return DEVICE_OK;
}

int UsbEndpointOut::clearStall()
{
    nrf_usbd_ep_unstall(NRF_USBD, (nrfx_usbd_ep_t)ep);
    nrf_usbd_dtoggle_set(NRF_USBD, (nrfx_usbd_ep_t)ep, NRF_USBD_DTOGGLE_DATA0);
    NRF_USBD->SIZE.EPOUT[ep] = 0; // start accepting data again
    return DEVICE_OK;
}

int UsbEndpointOut::reset()
{
    LOG("reset OUT %d", ep);
    return DEVICE_OK;
}

int UsbEndpointOut::stall()
{
    if (NRF_USBD_EP_NR_GET(ep) == 0)
        NRF_USBD->TASKS_EP0STALL = 1;
    else
        nrf_usbd_ep_stall(NRF_USBD, (nrfx_usbd_ep_t)NRF_USBD_EP_NR_GET(ep));
    return DEVICE_OK;
}

UsbEndpointIn::UsbEndpointIn(uint8_t idx, uint8_t type, uint8_t size)
{
    usb_assert(size == 64);
    usb_assert(type <= USB_EP_TYPE_INTERRUPT);
    usb_assert(type != USB_EP_TYPE_ISOCHRONOUS); // iso not supported yet
    ep = idx;
    flags = 0;
    userdata = 0;

    if (type == USB_EP_TYPE_INTERRUPT)
        flags = USB_EP_FLAG_NO_AUTO_ZLP;

    NRF_USBD->EPINEN |= 0x1 << ep;
}

UsbEndpointOut::UsbEndpointOut(uint8_t idx, uint8_t type, uint8_t size)
{
    usb_assert(size == 64);
    usb_assert(type <= USB_EP_TYPE_INTERRUPT);
    usb_assert(type != USB_EP_TYPE_ISOCHRONOUS); // iso not supported yet
    ep = idx;
    userdata = 0;

    NRF_USBD->EPOUTEN |= 0x1 << ep;

    startRead();
}

int UsbEndpointOut::disableIRQ()
{
    userdata |= 0x8000;
    return DEVICE_OK;
}

int UsbEndpointOut::enableIRQ()
{
    userdata &= ~0x8000;
    return DEVICE_OK;
}

void UsbEndpointOut::startRead()
{
    // indicate we're ready to go!
    NRF_USBD->EPOUT[ep].PTR    = (uint32_t) buf;
    NRF_USBD->EPOUT[ep].MAXCNT = USB_MAX_PKT_SIZE;
    NRF_USBD->SIZE.EPOUT[ep] = 0;
}

int UsbEndpointOut::read(void *dst, int maxlen)
{
    usb_assert(this != NULL);

    if (ep != 0 && !(ep_status & (USBD_EPDATASTATUS_EPOUT1_Msk << (ep - 1))))
        return 0;

    NRF_USBD->EVENTS_ENDEPOUT[ep] = 0;
    NRF_USBD->TASKS_STARTEPOUT[ep] = 1;
    while(NRF_USBD->EVENTS_ENDEPOUT[ep] == 0);

    int packetSize = nrf_usbd_epout_size_get(NRF_USBD, ep);

    if (packetSize)
    {
        LOG("USBRead(%d) => %d bytes", ep, packetSize);
        userdata -= packetSize;
        // Note that we shall discard any excessive data
        if (packetSize > maxlen)
            packetSize = maxlen;
        memcpy(dst, buf, packetSize);
        // only re-start read on non-ctrl
        if (ep & 0x1f)
            startRead();
    }

    return packetSize;
}

static int writeEP(UsbEndpointIn* endpoint, uint8_t *data, int len)
{
    usb_assert(len <= USB_MAX_PKT_SIZE);
    NRF_USBD->EPIN[endpoint->ep].PTR    = (uint32_t) data;
    NRF_USBD->EPIN[endpoint->ep].MAXCNT = len;

    NRF_USBD->EVENTS_EP0DATADONE = 0;
    NRF_USBD->EVENTS_EPDATA = 0;
    NRF_USBD->EVENTS_ENDEPIN[endpoint->ep] = 0;

    NRF_USBD->TASKS_STARTEPIN[endpoint->ep] = 1;

    LOG("write: %p len=%d at IN %d", data, len, endpoint->ep);

    if (endpoint->ep == 0)
    {
        // no data stage (and therefore no EP0DATADONE event), initiate ep0status asap.
        if (len == 0)
            NRF_USBD->TASKS_EP0STATUS = 1;
        else
            while(NRF_USBD->EVENTS_EP0DATADONE == 0);
    }
    else {
        uint32_t timeout = 500000;
        while(NRF_USBD->EVENTS_EPDATA == 0 && timeout > 0)
            timeout--;

        if (timeout == 0)
            return DEVICE_INVALID_STATE;
    }

    return DEVICE_OK;
}

int UsbEndpointIn::write(const void *src, int len)
{
    LOG("outer write %p/%d %d", src, len, wLength);

    int transLen = len;

    // this happens when someone tries to write before USB is initialized
    usb_assert(this != NULL);

    int tlen = len;

    int zlp = !(flags & USB_EP_FLAG_NO_AUTO_ZLP);

    if (wLength)
    {
        if (len >= wLength)
        {
            len = wLength;
            // see
            // https://stackoverflow.com/questions/3739901/when-do-usb-hosts-require-a-zero-length-in-packet-at-the-end-of-a-control-read-t
            zlp = 0;
        }
        wLength = 0;
    }

    NVIC_DisableIRQ(USBD_IRQn);

    for (;;)
    {
        int n = len;
        if (n > USB_MAX_PKT_SIZE)
            n = USB_MAX_PKT_SIZE;
        memcpy(buf, src, n);

        int ret = writeEP(this, buf, n);

        if (ret < 0)
            return 0;

        len -= n;
        src = (const uint8_t *)src + n;

        if (!len)
            break;
    }

    LOG("zlp %d len %d cond %d", zlp, len, (len & (USB_MAX_PKT_SIZE - 1)));

    if (zlp && transLen && (transLen & (USB_MAX_PKT_SIZE - 1)) == 0)
        writeEP(this, buf, 0);
    else if (ep == 0 && tlen)
    {
        // unless it is a ZLP, we will need to initiate status stage ourselves here.
        // Figure 183 in the nrf52833 datasheet is really useful to understand
        // required IN transaction tasks.
        NRF_USBD->TASKS_EP0STATUS = 1;
    }

    NVIC_EnableIRQ(USBD_IRQn);

    return DEVICE_OK;
}

#endif