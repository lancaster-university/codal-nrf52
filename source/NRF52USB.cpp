/*
The MIT License (MIT)

Copyright (c) 2017 Lancaster University.

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

#if CONFIG_ENABLED(DEVICE_USB)
#include "CodalDmesg.h"
#include "nrfx_usbd.h"
#include "nrfx_power.h"
#include "nrfx_clock.h"
#include "nrfx_usbd_errata.h"

#define LOG DMESG
// #define DBG(...) ((void)0)
#define DBG DMESG

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
#endif

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

// void POWER_CLOCK_IRQHandler() {
//     while(1);
//     LOG("PCIRQ");
//     if (nrf_power_event_get_and_clear(NRF_POWER, NRF_POWER_EVENT_USBDETECTED))
//     {  
//         LOG("DETECT");
//         if (!NRF_USBD->ENABLE)
//         {
//             nrf_usbd_eventcause_clear(NRF_USBD, USBD_EVENTCAUSE_READY_Msk);
//             if ( nrfx_usbd_errata_187() )
//             {
//                 // CRITICAL_REGION_ENTER();
//                 if ( *((volatile uint32_t *) (0x4006EC00)) == 0x00000000 )
//                 {
//                     *((volatile uint32_t *) (0x4006EC00)) = 0x00009375;
//                     *((volatile uint32_t *) (0x4006ED14)) = 0x00000003;
//                     *((volatile uint32_t *) (0x4006EC00)) = 0x00009375;
//                 }
//                 else
//                 {
//                     *((volatile uint32_t *) (0x4006ED14)) = 0x00000003;
//                 }
//                 // CRITICAL_REGION_EXIT();
//             }

//             if ( nrfx_usbd_errata_171() )
//             {
//                 // CRITICAL_REGION_ENTER();
//                 if ( *((volatile uint32_t *) (0x4006EC00)) == 0x00000000 )
//                 {
//                     *((volatile uint32_t *) (0x4006EC00)) = 0x00009375;
//                     *((volatile uint32_t *) (0x4006EC14)) = 0x000000C0;
//                     *((volatile uint32_t *) (0x4006EC00)) = 0x00009375;
//                 }
//                 else
//                 {
//                     *((volatile uint32_t *) (0x4006EC14)) = 0x000000C0;
//                 }
//                 // CRITICAL_REGION_EXIT();
//             }
//             nrf_clock_event_clear(NRF_CLOCK, NRF_CLOCK_EVENT_HFCLKSTARTED);
//             nrf_clock_task_trigger(NRF_CLOCK, NRF_CLOCK_TASK_HFCLKSTART);
//             nrf_usbd_enable(NRF_USBD);
//         }
//     }

//     if (nrf_power_event_get_and_clear(NRF_POWER, NRF_POWER_EVENT_USBREMOVED))
//     {
//         LOG("REMOVE");
//         nrf_usbd_pullup_disable(NRF_USBD);
//     }
//     if (nrf_power_event_get_and_clear(NRF_POWER, NRF_POWER_EVENT_USBPWRRDY))
//     {
//         LOG("PWRREADY");
//         nrf_usbd_pullup_enable(NRF_USBD);
//     }
// }

void USBD_IRQHandler(void) {

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

    int ep_read = 0;
    for(int i = 1; i < DEVICE_USB_ENDPOINTS; i++)
        if (set & (1 << (USBD_INTEN_ENDEPOUT0_Pos + i))) {
            LOG("EP%d READ!", i);
            ep_read = 1;
        }
    if (ep_read)
        CodalUSB::usbInstance->interruptHandler();
}

void usb_configure(uint8_t numEndpoints)
{
    nrf_usbd_disable(NRF_USBD);

    while(!nrf_power_usbregstatus_vbusdet_get(NRF_POWER));

    uint32_t intmsk =
       NRF_USBD_INT_USBRESET_MASK     |
       NRF_USBD_INT_STARTED_MASK      |
       NRF_USBD_INT_ENDEPIN0_MASK     |
       NRF_USBD_INT_EP0DATADONE_MASK  |
       NRF_USBD_INT_ENDEPOUT0_MASK    |
       NRF_USBD_INT_USBEVENT_MASK     |
       NRF_USBD_INT_EP0SETUP_MASK     |
       NRF_USBD_INT_DATAEP_MASK;

//    if (enable_sof || nrfx_usbd_errata_104())
//    {
//        ints_to_enable |= NRF_USBD_INT_SOF_MASK;
//    }

   /* Enable all required interrupts */
    // nrf_power_int_enable(NRF_POWER, NRF_POWER_INT_USBDETECTED_MASK |
    //                                 NRF_POWER_INT_USBREMOVED_MASK  |
    //                                 NRF_POWER_INT_USBPWRRDY_MASK);
    // NVIC_SetVector(POWER_CLOCK_IRQn, (uint32_t)POWER_CLOCK_IRQHandler);
    // NVIC_SetPriority(POWER_CLOCK_IRQn, NRFX_POWER_DEFAULT_CONFIG_IRQ_PRIORITY);
    // NVIC_EnableIRQ(POWER_CLOCK_IRQn);

    nrf_usbd_enable(NRF_USBD);
    nrf_clock_event_clear(NRF_CLOCK, NRF_CLOCK_EVENT_HFCLKSTARTED);
    nrf_clock_task_trigger(NRF_CLOCK, NRF_CLOCK_TASK_HFCLKSTART);
    nrf_usbd_int_enable(NRF_USBD, intmsk);

    nrf_usbd_event_get_and_clear(NRF_USBD, NRF_USBD_EVENT_USBEVENT);

    NVIC_ClearPendingIRQ(USBD_IRQn);
    NVIC_SetPriority(USBD_IRQn, 7);
    NVIC_EnableIRQ(USBD_IRQn);

    nrf_usbd_pullup_enable(NRF_USBD);
}

void usb_set_address(uint16_t wValue) {}

void usb_set_address_pre(uint16_t wValue)
{
    // DBG("ctl=%p", USBx_OUTEP(0)->DOEPCTL);

    // LOG("set address %d", wValue);
    // CHK(HAL_PCD_SetAddress(&pcd, wValue));

    // DBG("ctl2=%p", USBx_OUTEP(0)->DOEPCTL);
    // auto cusb = CodalUSB::usbInstance;
    // cusb->ctrlOut->startRead();
    // for (auto iface = cusb->interfaces; iface; iface = iface->next)
    //     if (iface->out)
    //         iface->out->startRead();

    // DBG("ctl3=%p", USBx_OUTEP(0)->DOEPCTL);
}

int UsbEndpointIn::clearStall()
{
    LOG("clear stall IN %d", ep);

    nrf_usbd_ep_unstall(NRF_USBD, (nrfx_usbd_ep_t)NRF_USBD_EP_NR_GET(ep));
    nrf_usbd_dtoggle_set(NRF_USBD, (nrfx_usbd_ep_t)NRF_USBD_EP_NR_GET(ep), NRF_USBD_DTOGGLE_DATA0);
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
    if (NRF_USBD_EP_NR_GET(ep) == 0)
    {
        nrf_usbd_task_trigger(NRF_USBD, NRF_USBD_TASK_EP0STALL);
        nrf_usbd_task_trigger(NRF_USBD, NRF_USBD_TASK_EP0STATUS);
    }
    else
        nrf_usbd_ep_stall(NRF_USBD, (nrfx_usbd_ep_t)NRF_USBD_EP_NR_GET(ep));
    wLength = 0;
    return DEVICE_OK;
}

int UsbEndpointOut::clearStall()
{
    nrf_usbd_ep_unstall(NRF_USBD, (nrfx_usbd_ep_t)ep);
    return DEVICE_OK;
}

int UsbEndpointOut::reset()
{
    LOG("reset OUT %d", ep);
    return DEVICE_OK;
}

int UsbEndpointOut::stall()
{
    LOG("stall OUT %d", ep);
    nrf_usbd_ep_stall(NRF_USBD, (nrfx_usbd_ep_t)ep);
    return DEVICE_OK;
}

UsbEndpointIn::UsbEndpointIn(uint8_t idx, uint8_t type, uint8_t size)
{
    usb_assert(size == 64);
    usb_assert(type <= USB_EP_TYPE_INTERRUPT);
    ep = idx;
    flags = 0;
    userdata = 0;

    if (type == USB_EP_TYPE_INTERRUPT)
        flags = USB_EP_FLAG_NO_AUTO_ZLP;

    nrf_usbd_ep_enable(NRF_USBD, ep);
}

UsbEndpointOut::UsbEndpointOut(uint8_t idx, uint8_t type, uint8_t size)
{
    usb_assert(size == 64);
    usb_assert(type <= USB_EP_TYPE_INTERRUPT);
    ep = idx;
    userdata = 0;

    nrf_usbd_ep_enable(NRF_USBD, ep);
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
    if (ep == 0) {
        nrf_usbd_task_trigger(NRF_USBD, (nrf_usbd_task_t)NRF_USBD_TASK_EP0RCVOUT);
    }
    else 
        NRF_USBD->SIZE.EPOUT[ep] = 0;

    nrf_usbd_ep_easydma_set(NRF_USBD, ep, (uint32_t)buf, USB_MAX_PKT_SIZE);
    nrf_usbd_task_trigger(NRF_USBD, (nrf_usbd_task_t) (NRF_USBD_TASK_STARTEPOUT0 + NRF_USBD_EP_NR_GET(ep)));
    DBG("OUT %d start read", ep);
    while(!nrf_usbd_event_check(NRF_USBD, (nrf_usbd_event_t) (NRF_USBD_EVENT_ENDEPOUT0 + NRF_USBD_EP_NR_GET(ep))));

    if (nrf_usbd_epout_size_get(NRF_USBD, ep) == 0 && ep == 0)
        nrf_usbd_task_trigger(NRF_USBD, (nrf_usbd_task_t)NRF_USBD_TASK_EP0STATUS);
    DBG("OUT %d end read %d bytes", ep, nrf_usbd_epout_size_get(NRF_USBD, ep));
}

int UsbEndpointOut::read(void *dst, int maxlen)
{
    usb_assert(this != NULL);

    int packetSize = nrf_usbd_epout_size_get(NRF_USBD, ep);

    if (packetSize)
    {
        // LOG("USBRead(%d) => %d bytes", ep, packetSize);
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

static void writeEP(uint8_t *data, uint8_t ep, int len)
{
    usb_assert(len <= USB_MAX_PKT_SIZE);

    

    nrf_usbd_ep_easydma_set(NRF_USBD, ep | NRF_USBD_EP_DIR_IN, (uint32_t)data, len);
    DBG("write: %p len=%d at IN %d", data, len, NRF_USBD_EP_NR_GET(ep));
    for (int i =0 ;i<len; i++)
        DBG("%x,",data[i]);
    nrf_usbd_task_trigger(NRF_USBD, (nrf_usbd_task_t) (NRF_USBD_TASK_STARTEPIN0 + NRF_USBD_EP_NR_GET(ep)));
    while(!nrf_usbd_event_check(NRF_USBD, (nrf_usbd_event_t)(NRF_USBD_EVENT_ENDEPIN0 + NRF_USBD_EP_NR_GET(ep))));

    // there is a ZLP from the host coming
    if (len && NRF_USBD_EP_NR_GET(ep) == 0)
        CodalUSB::usbInstance->ctrlOut->startRead();
}

int UsbEndpointIn::write(const void *src, int len)
{
    DBG("outer write %p/%d", src, len);

    // this happens when someone tries to write before USB is initialized
    usb_assert(this != NULL);

    int zlp = !(flags & USB_EP_FLAG_NO_AUTO_ZLP);

    if (wLength)
    {
        DBG("WLENGTH SET %d", wLength);
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
        writeEP(buf, ep, n);
        len -= n;
        src = (const uint8_t *)src + n;
        if (!len)
            break;
    }

    // send ZLP manually if needed.
    if (zlp && len && (len & (USB_MAX_PKT_SIZE - 1)) == 0)
    {
        writeEP(buf, ep, 0);
    }

    NVIC_EnableIRQ(USBD_IRQn);

    return DEVICE_OK;
}

#endif