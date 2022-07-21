#include "tusb_option.h"

#if TUSB_OPT_HOST_ENABLED && (CFG_TUSB_MCU == OPT_MCU_SWM341)

#include "host/hcd.h"
#include "SWM341.h"

//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM DECLARATION
//--------------------------------------------------------------------+

typedef struct
{
    uint8_t  dev_addr;
    uint8_t  ep_addr;
    uint16_t max_packet_size;
    union {
        uint8_t flags;
        struct {
            uint8_t data : 1;
            uint8_t xfer : 2;
        };
    };
    uint8_t *buffer;
    uint16_t length;
    uint16_t remaining;
} pipe_t;

typedef struct
{
    pipe_t   pipe[CFG_TUH_ENDPOINT_MAX * 2];    // OUT, IN
    uint32_t in_progress; // Bitmap. Each bit indicates that a transfer of the corresponding pipe is in progress
    uint32_t pending;     // Bitmap. Each bit indicates that a transfer of the corresponding pipe will be resume the next frame
} hcd_data_t;

static hcd_data_t _hcd;


//--------------------------------------------------------------------+
// INTERNAL FUNCTION DECLARATION
//--------------------------------------------------------------------+

static int find_pipe(uint8_t dev_addr, uint8_t ep_addr)
{
    for(int i = 0; i < CFG_TUH_ENDPOINT_MAX * 2; i++)
    {
        pipe_t *p = &_hcd.pipe[i];
        if((p->dev_addr == dev_addr) && (p->ep_addr == ep_addr))
            return i;
    }

    return -1;
}


static int select_next_pipenum(int pipenum)
{
    unsigned wip = _hcd.in_progress & ~_hcd.pending;    // pending: will be resume the next frame
    if(!wip)
        return -1;
    
    unsigned msk  = TU_GENMASK(31, pipenum);    // (0xFFFFFFFF << pipenum) & (0xFFFFFFFF >> (31 - 31))
    int      next = __builtin_ctz(wip & msk);
    if(next)
        return next;
    
    msk  = TU_GENMASK(pipenum, 0);              // (0xFFFFFFFF << 0) & (0xFFFFFFFF >> (31 - pipenum))
    next = __builtin_ctz(wip & msk);
    return next;
}


static bool continue_transfer(int pipenum)
{
    pipe_t *pipe = &_hcd.pipe[pipenum];

    int trsz = (USBH->SR & USBH_SR_TRSZ_Msk) >> USBH_SR_TRSZ_Pos;

    if(tu_edpt_dir(pipe->ep_addr) == TUSB_DIR_IN)
        USBH_ReadRxBuffer(pipe->buffer, trsz);
    
    pipe->buffer += trsz;
    pipe->remaining -= trsz;
    pipe->data ^= 1;

    if(pipe->remaining == 0)
        return false;

    const uint8_t size = pipe->remaining > pipe->max_packet_size ? pipe->max_packet_size : pipe->remaining;

    if(tu_edpt_dir(pipe->ep_addr) == TUSB_DIR_IN)
        while(USBH_SendInPacket(pipe->dev_addr, pipe->ep_addr, pipe->data, size) == 0) __NOP();
    else
        while(USBH_SendOutPacket(pipe->dev_addr, pipe->ep_addr, pipe->data, pipe->buffer, size) == 0) __NOP();

    return true;
}


static void suspend_transfer(int pipenum)
{
    pipe_t *pipe = &_hcd.pipe[pipenum];

    if((pipe->xfer == TUSB_XFER_INTERRUPT) || (pipe->xfer == TUSB_XFER_BULK))
    {
        _hcd.pending |= TU_BIT(pipenum);
        USBH->IF  = USBH_IF_SOF_Msk;
        USBH->IE |= USBH_IE_SOF_Msk;
    }
}


static bool resume_transfer(int pipenum)
{
    const unsigned ie = NVIC_GetEnableIRQ(USB_IRQn);
    NVIC_DisableIRQ(USB_IRQn);

    pipe_t *pipe = &_hcd.pipe[pipenum];

    const uint8_t size = pipe->remaining > pipe->max_packet_size ? pipe->max_packet_size : pipe->remaining;

    if(tu_edpt_dir(pipe->ep_addr) == TUSB_DIR_IN)
        while(USBH_SendInPacket(pipe->dev_addr, pipe->ep_addr, pipe->data, size) == 0) __NOP();
    else
        while(USBH_SendOutPacket(pipe->dev_addr, pipe->ep_addr, pipe->data, pipe->buffer, size) == 0) __NOP();

    if(ie) NVIC_EnableIRQ(USB_IRQn);
    
    return true;
}


static void process_rx(uint8_t rhport)
{
    (void)rhport;

    uint8_t dev_addr = (USBH->TOKEN & USBH_TOKEN_ADDR_Msk) >> USBH_TOKEN_ADDR_Pos;
    uint8_t ep_num   = (USBH->TOKEN & USBH_TOKEN_EPNR_Msk) >> USBH_TOKEN_EPNR_Pos;
    uint8_t ep_dir   =((USBH->TOKEN & USBH_TOKEN_TYPE_Msk) >> USBH_TOKEN_TYPE_Pos) == 9 ? USB_EP_IN : USB_EP_OUT;

    int pipenum = find_pipe(dev_addr, ep_num | ep_dir);
    
    xfer_result_t result;
    //TU_LOG2("USBH->SR: %d\n", (int)(USBH->SR & USBH_SR_RESP_Msk));
    switch(USBH->SR & USBH_SR_RESP_Msk)
    {
    case USBR_ACK:
        if(continue_transfer(pipenum))
            return;
        
        result = XFER_RESULT_SUCCESS;
        break;

    case USBR_NAK:
        suspend_transfer(pipenum);
        goto next_pipe;

    case USBR_STALL:
        result = XFER_RESULT_STALLED;
        break;
    
    default:
        result = XFER_RESULT_FAILED;
        break;
    }
    
    _hcd.in_progress &= ~TU_BIT(pipenum);

    pipe_t *pipe = &_hcd.pipe[pipenum];
    hcd_event_xfer_complete(pipe->dev_addr, pipe->ep_addr, pipe->length - pipe->remaining, result, true);
    
next_pipe:
    {
    int next_pipenum = select_next_pipenum(pipenum);
    if(0 <= next_pipenum)
        resume_transfer(next_pipenum);
    }
}


/*------------------------------------------------------------------*/
/* Host API
 *------------------------------------------------------------------*/
bool hcd_init(uint8_t rhport)
{
    (void)rhport;

    USBH_HW_Init();

    tu_memclr(&_hcd, sizeof(_hcd));

    USBH->PORTSR = USBH_PORTSR_CONNCHG_Msk | USBH_PORTSR_ENACHG_Msk |
                   USBH_PORTSR_SUSPCHG_Msk | USBH_PORTSR_RSTCHG_Msk;
    USBH->IF = USBH_IF_RXSTAT_Msk | USBH_IF_SOF_Msk;
    USBH->IE = USBH_IE_PORT_Msk | USBH_IE_RXSTAT_Msk;

    return true;
}

void hcd_int_enable(uint8_t rhport)
{
    (void)rhport;

    NVIC_EnableIRQ(USB_IRQn);
}

void hcd_int_disable(uint8_t rhport)
{
    (void)rhport;

    NVIC_DisableIRQ(USB_IRQn);
}

uint32_t hcd_frame_number(uint8_t rhport)
{
    (void)rhport;

    return USBH->FRAMENR;
}

/*--------------------------------------------------------------------+
 * Port API
 *--------------------------------------------------------------------+ */
bool hcd_port_connect_status(uint8_t rhport)
{
    (void)rhport;

    return USBH_IsDeviceConnected();
}

void hcd_port_reset(uint8_t rhport)
{
    (void)rhport;

    USBH_ResetPort();
}

void hcd_port_reset_end(uint8_t rhport)
{
    (void) rhport;
}

tusb_speed_t hcd_port_speed_get(uint8_t rhport)
{
    (void)rhport;

    return TUSB_SPEED_FULL;
}

void hcd_device_close(uint8_t rhport, uint8_t dev_addr)
{
    (void)rhport;

    const unsigned ie = NVIC_GetEnableIRQ(USB_IRQn);
    NVIC_DisableIRQ(USB_IRQn);

    pipe_t *p   = &_hcd.pipe[2];
    pipe_t *end = &_hcd.pipe[CFG_TUH_ENDPOINT_MAX * 2];
    for(;p != end; ++p)
    {
        if(p->dev_addr == dev_addr)
            tu_memclr(p, sizeof(*p));
    }

    if(ie) NVIC_EnableIRQ(USB_IRQn);
}

//--------------------------------------------------------------------+
// Endpoints API
//--------------------------------------------------------------------+
bool hcd_setup_send(uint8_t rhport, uint8_t dev_addr, uint8_t const setup_packet[8])
{
    (void)rhport;
    
    int pipenum = find_pipe(dev_addr, 0);
    if(pipenum < 0) return false;

    TU_ASSERT((_hcd.in_progress & TU_BIT(pipenum)) == 0);

    pipe_t *pipe = &_hcd.pipe[pipenum];
    pipe[0].data      = 0;
    pipe[0].buffer    = (uint8_t*)(uintptr_t)setup_packet;
    pipe[0].length    = 8;
    pipe[0].remaining = 8;
    pipe[1].data      = 1;

    _hcd.in_progress |= TU_BIT(pipenum);

    while(USBH_SendSetupPacket(dev_addr, (uint8_t *)setup_packet, 8) == 0) __NOP();

    return true;
}

bool hcd_edpt_open(uint8_t rhport, uint8_t dev_addr, tusb_desc_endpoint_t const * ep_desc)
{
    (void)rhport;

    uint8_t const ep_addr = ep_desc->bEndpointAddress;
    
    /* Find a free pipe */
    pipe_t *p = &_hcd.pipe[0];
    pipe_t *end = &_hcd.pipe[CFG_TUH_ENDPOINT_MAX * 2];
    if(dev_addr || ep_addr)
    {
        for(p += 2; p < end && (p->dev_addr || p->ep_addr); ++p) __NOP();
        if(p == end) return false;
    }

    p->dev_addr        = dev_addr;
    p->ep_addr         = ep_addr;
    p->max_packet_size = ep_desc->wMaxPacketSize;
    p->xfer            = ep_desc->bmAttributes.xfer;
    p->data            = 0;

    if(!ep_addr)
    {
        /* Open one more pipe for Control IN transfer */
        TU_ASSERT(TUSB_XFER_CONTROL == p->xfer);

        pipe_t *q = p + 1;
        q->dev_addr        = dev_addr;
        q->ep_addr         = tu_edpt_addr(ep_addr, TUSB_DIR_IN);
        q->max_packet_size = ep_desc->wMaxPacketSize;
        q->xfer            = ep_desc->bmAttributes.xfer;
        q->data            = 1;
    }
    
    return true;
}

bool hcd_edpt_xfer(uint8_t rhport, uint8_t dev_addr, uint8_t ep_addr, uint8_t * buffer, uint16_t buflen)
{
    (void)rhport;

    TU_LOG2("xfer %s %d\r\n", tu_edpt_dir(ep_addr) ? "IN" : "OUT", buflen);

    int pipenum = find_pipe(dev_addr, ep_addr);
    TU_ASSERT(pipenum >= 0);
    TU_ASSERT((_hcd.in_progress & TU_BIT(pipenum)) == 0);

    unsigned const ie  = NVIC_GetEnableIRQ(USB_IRQn);
    NVIC_DisableIRQ(USB_IRQn);

    pipe_t *pipe = &_hcd.pipe[pipenum];
    pipe->buffer      = buffer;
    pipe->length      = buflen;
    pipe->remaining   = buflen;
    _hcd.in_progress |= TU_BIT(pipenum);
    _hcd.pending     |= TU_BIT(pipenum); // Send at the next Frame
    USBH->IF  = USBH_IF_SOF_Msk;
    USBH->IE |= USBH_IE_SOF_Msk;

    if(ie) NVIC_EnableIRQ(USB_IRQn);

    return true;
}

bool hcd_edpt_clear_stall(uint8_t dev_addr, uint8_t ep_addr)
{
    if(!tu_edpt_number(ep_addr)) return true;

    int num = find_pipe(dev_addr, ep_addr);
    if(num < 0) return false;

    pipe_t *p = &_hcd.pipe[num];
    p->data = 0;    // Reset data toggle

    return true;
}


/*--------------------------------------------------------------------+
 * ISR
 *--------------------------------------------------------------------+*/
void hcd_int_handler(uint8_t rhport)
{
    uint32_t usb_if = USBH->IF & USBH->IE;  // 只查看使能的中断
    USBH->IF = USBH->IF;                    // 清除中断标志

    if(usb_if & USBH_IF_PORT_Msk)
    {
        if(USBH->PORTSR & USBH_PORTSR_CONNCHG_Msk)
        {
            USBH->PORTSR = USBH_PORTSR_CONNCHG_Msk;

            if(USBH->PORTSR & USBH_PORTSR_CONN_Msk)
            {
                hcd_port_reset(rhport);

                hcd_event_device_attach(rhport, true);
            }
            else
            {
                hcd_event_device_remove(rhport, true);
            }
        }

        if(USBH->PORTSR & USBH_PORTSR_RSTCHG_Msk)
        {
            USBH->PORTSR = USBH_PORTSR_RSTCHG_Msk;

            _hcd.in_progress = 0;
            _hcd.pending     = 0;
        }

        if(USBH->PORTSR & USBH_PORTSR_SUSPCHG_Msk)
        {
            USBH->PORTSR = USBH_PORTSR_SUSPCHG_Msk;
        }

        if(USBH->PORTSR & USBH_PORTSR_ENACHG_Msk)
        {
            USBH->PORTSR = USBH_PORTSR_ENACHG_Msk;
        }
    }

    if(usb_if & USBH_IF_SOF_Msk)
    {
        USBH->IE &= ~USBH_IE_SOF_Msk;

        if(_hcd.pending)
        {
            int pipenum = __builtin_ctz(_hcd.pending);
            _hcd.pending = 0;

            resume_transfer(pipenum);
        }
    }

    if(usb_if & USBH_IF_RXSTAT_Msk)
    {
        process_rx(rhport);
    }
}

#endif
