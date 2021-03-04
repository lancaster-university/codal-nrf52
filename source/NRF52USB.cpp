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



void USBD_IRQHandler(void) {

    const uint32_t enabled = nrf_usbd_int_enable_get(NRF_USBD);
    uint32_t to_process = enabled;
    uint32_t active = 0;

    /* Check all enabled interrupts */
    while (to_process)
    {
        uint8_t event_nr = __CLZ(__RBIT(to_process));
        if (nrf_usbd_event_get_and_clear(NRF_USBD,
                                         (nrf_usbd_event_t)nrfx_bitpos_to_event(event_nr)))
        {
            active |= 1UL << event_nr;
        }
        to_process &= ~(1UL << event_nr);
    }

    if (nrfx_usbd_errata_104())
    {
        /* Event correcting */
        if (/*(!m_dma_pending) &&*/ (0 != (active & (USBD_INTEN_SOF_Msk))))
        {
            uint8_t usbi, uoi, uii;
            /* Testing */
            *((volatile uint32_t *)(NRF_USBD_BASE + 0x800)) = 0x7A9;
            uii = (uint8_t)(*((volatile uint32_t *)(NRF_USBD_BASE + 0x804)));
            if (0 != uii)
            {
                uii &= (uint8_t)(*((volatile uint32_t *)(NRF_USBD_BASE + 0x804)));
            }

            *((volatile uint32_t *)(NRF_USBD_BASE + 0x800)) = 0x7AA;
            uoi = (uint8_t)(*((volatile uint32_t *)(NRF_USBD_BASE + 0x804)));
            if (0 != uoi)
            {
                uoi &= (uint8_t)(*((volatile uint32_t *)(NRF_USBD_BASE + 0x804)));
            }
            *((volatile uint32_t *)(NRF_USBD_BASE + 0x800)) = 0x7AB;
            usbi = (uint8_t)(*((volatile uint32_t *)(NRF_USBD_BASE + 0x804)));
            if (0 != usbi)
            {
                usbi &= (uint8_t)(*((volatile uint32_t *)(NRF_USBD_BASE + 0x804)));
            }
            /* Processing */
            *((volatile uint32_t *)(NRF_USBD_BASE + 0x800)) = 0x7AC;
            uii &= (uint8_t)*((volatile uint32_t *)(NRF_USBD_BASE + 0x804));
            if (0 != uii)
            {
                uint8_t rb;
                // m_simulated_dataepstatus |= ((uint32_t)uii) << NRFX_USBD_EPIN_BITPOS_0;
                *((volatile uint32_t *)(NRF_USBD_BASE + 0x800)) = 0x7A9;
                *((volatile uint32_t *)(NRF_USBD_BASE + 0x804)) = uii;
                rb = (uint8_t)*((volatile uint32_t *)(NRF_USBD_BASE + 0x804));
                // NRFX_USBD_LOG_PROTO1_FIX_PRINTF("   uii: 0x%.2x (0x%.2x)", uii, rb);
                (void)rb;
            }

            *((volatile uint32_t *)(NRF_USBD_BASE + 0x800)) = 0x7AD;
            uoi &= (uint8_t)*((volatile uint32_t *)(NRF_USBD_BASE + 0x804));
            if (0 != uoi)
            {
                uint8_t rb;
                // m_simulated_dataepstatus |= ((uint32_t)uoi) << NRFX_USBD_EPOUT_BITPOS_0;
                *((volatile uint32_t *)(NRF_USBD_BASE + 0x800)) = 0x7AA;
                *((volatile uint32_t *)(NRF_USBD_BASE + 0x804)) = uoi;
                rb = (uint8_t)*((volatile uint32_t *)(NRF_USBD_BASE + 0x804));
                // NRFX_USBD_LOG_PROTO1_FIX_PRINTF("   uoi: 0x%.2u (0x%.2x)", uoi, rb);
                (void)rb;
            }

            *((volatile uint32_t *)(NRF_USBD_BASE + 0x800)) = 0x7AE;
            usbi &= (uint8_t)*((volatile uint32_t *)(NRF_USBD_BASE + 0x804));
            if (0 != usbi)
            {
                uint8_t rb;
                if (usbi & 0x01)
                {
                    active |= USBD_INTEN_EP0SETUP_Msk;
                }
                if (usbi & 0x10)
                {
                    active |= USBD_INTEN_USBRESET_Msk;
                }
                *((volatile uint32_t *)(NRF_USBD_BASE + 0x800)) = 0x7AB;
                *((volatile uint32_t *)(NRF_USBD_BASE + 0x804)) = usbi;
                rb = (uint8_t)*((volatile uint32_t *)(NRF_USBD_BASE + 0x804));
                // NRFX_USBD_LOG_PROTO1_FIX_PRINTF("   usbi: 0x%.2u (0x%.2x)", usbi, rb);
                (void)rb;
            }

            // if (0 != (m_simulated_dataepstatus &
            //     ~((1U << NRFX_USBD_EPOUT_BITPOS_0) | (1U << NRFX_USBD_EPIN_BITPOS_0))))
            // {
            //     active |= enabled & NRF_USBD_INT_DATAEP_MASK;
            // }
            // if (0 != (m_simulated_dataepstatus &
            //     ((1U << NRFX_USBD_EPOUT_BITPOS_0) | (1U << NRFX_USBD_EPIN_BITPOS_0))))
            // {
            //     if (0 != (enabled & NRF_USBD_INT_EP0DATADONE_MASK))
            //     {
            //         m_simulated_dataepstatus &=
            //             ~((1U << NRFX_USBD_EPOUT_BITPOS_0) | (1U << NRFX_USBD_EPIN_BITPOS_0));
            //         active |= NRF_USBD_INT_EP0DATADONE_MASK;
            //     }
            // }
        }
    }

    /* Process the active interrupts */
    bool setup_active = 0 != (active & NRF_USBD_INT_EP0SETUP_MASK);
    active &= ~NRF_USBD_INT_EP0SETUP_MASK;
    int32_t ep_interrupt = -1;
    while (active)
    {
        uint8_t event_nr = __CLZ(__RBIT(active));
        int32_t event = -1;
        UsbEndpointOut* ep = NULL;

        switch(event_nr) {
            case USBD_INTEN_USBRESET_Pos:      /**< Reset condition on USB bus detected. */
                LOG("RESET"); 
                CodalUSB::usbInstance->initEndpoints();
                break;
            case USBD_INTEN_ENDEPOUT0_Pos:
                ep = findOutEp(0);
                ep->userdata = nrf_usbd_epout_size_get(NRF_USBD, 0);
                ep_interrupt = 1;
                break;
            case USBD_INTEN_ENDEPOUT1_Pos:
                ep = findOutEp(1);
                ep->userdata = nrf_usbd_epout_size_get(NRF_USBD, 1);
                ep_interrupt = 1;
                break;
            case USBD_INTEN_ENDEPOUT2_Pos:
                ep = findOutEp(2);
                ep->userdata = nrf_usbd_epout_size_get(NRF_USBD, 2);
                ep_interrupt = 1;
                break;
            case USBD_INTEN_ENDEPOUT3_Pos:
                ep = findOutEp(3);
                ep->userdata = nrf_usbd_epout_size_get(NRF_USBD, 3);
                ep_interrupt = 1;
                break;
            case USBD_INTEN_ENDEPOUT4_Pos:
                ep = findOutEp(4);
                ep->userdata = nrf_usbd_epout_size_get(NRF_USBD, 4);
                ep_interrupt = 1;
                break;
            case USBD_INTEN_ENDEPOUT5_Pos:
                ep = findOutEp(5);
                ep->userdata = nrf_usbd_epout_size_get(NRF_USBD, 5);
                ep_interrupt = 1;
                break;
            case USBD_INTEN_ENDEPOUT6_Pos:
                ep = findOutEp(6);
                ep->userdata = nrf_usbd_epout_size_get(NRF_USBD, 6);
                ep_interrupt = 1;
                break;
            case USBD_INTEN_ENDEPOUT7_Pos:
                ep = findOutEp(7);
                ep->userdata = nrf_usbd_epout_size_get(NRF_USBD, 7);
                ep_interrupt = 1;
                break;
            case USBD_INTEN_USBEVENT_Pos:
                event = nrf_usbd_eventcause_get_and_clear(NRF_USBD);
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
                break;

            case USBD_INTEN_STARTED_Pos:
            case USBD_INTEN_ENDEPIN0_Pos:
            case USBD_INTEN_ENDEPIN1_Pos:
            case USBD_INTEN_ENDEPIN2_Pos:
            case USBD_INTEN_ENDEPIN3_Pos:
            case USBD_INTEN_ENDEPIN4_Pos:
            case USBD_INTEN_ENDEPIN5_Pos:
            case USBD_INTEN_ENDEPIN6_Pos:
            case USBD_INTEN_ENDEPIN7_Pos:
            case USBD_INTEN_EP0DATADONE_Pos:
            case USBD_INTEN_ENDISOIN_Pos:
            case USBD_INTEN_ENDISOOUT_Pos:
            case USBD_INTEN_SOF_Pos:
                break;
        }

        active &= ~(1UL << event_nr);
    }

    if (ep_interrupt)
        CodalUSB::usbInstance->interruptHandler();


    if (setup_active)
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

        // CodalUSB::usbInstance->interruptHandler();
}

void usb_configure(uint8_t numEndpoints)
{
    nrf_usbd_enable(NRF_USBD);

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
    nrf_usbd_int_enable(NRF_USBD, intmsk);
    nrf_usbd_pullup_enable(NRF_USBD);
    NVIC_SetPriority(USBD_IRQn, 7);
    NVIC_EnableIRQ(USBD_IRQn);
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
    ep = idx | NRF_USBD_EP_DIR_IN;
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
    nrf_usbd_ep_easydma_set(NRF_USBD, ep, (uint32_t)buf, USB_MAX_PKT_SIZE);
    DBG("OUT %d start read", ep);
    while(!nrf_usbd_event_check(NRF_USBD, (nrf_usbd_event_t) (NRF_USBD_EVENT_ENDEPOUT0 + NRF_USBD_EP_NR_GET(ep))));
}

int UsbEndpointOut::read(void *dst, int maxlen)
{
    usb_assert(this != NULL);

    int packetSize = userdata & 0xff;

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
    uint32_t len32b = (len + 3) / 4;

    nrf_usbd_ep_easydma_set(NRF_USBD, ep, (uint32_t)data, len32b);
    DBG("write: %p len=%d at IN %d", data, len, ep);

    while(!nrf_usbd_event_check(NRF_USBD, (nrf_usbd_event_t)(NRF_USBD_EVENT_ENDEPIN0 + NRF_USBD_EP_NR_GET(ep))));

    // there is a ZLP from the host coming
    if (len && ep == 0)
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