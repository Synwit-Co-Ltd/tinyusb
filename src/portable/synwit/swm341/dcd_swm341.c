#include "tusb_option.h"

#if TUSB_OPT_DEVICE_ENABLED && (CFG_TUSB_MCU == OPT_MCU_SWM341)

#include "device/dcd.h"
#include "SWM341.h"


/* RAM table needed to track ongoing transfers performed by dcd_edpt_xfer(), and the ISR */
static struct xfer_ctl_t
{
    uint8_t *data_ptr;                  // data_ptr tracks where to next copy data to (for OUT) or from (for IN)
    
    union {
        uint16_t in_remaining_bytes;    // for IN endpoints, we track how many bytes are left to transfer
        uint16_t out_bytes_so_far;      // for OUT endpoints, we track how many bytes we've transferred so far
    };

    uint16_t max_packet_size;
    uint16_t total_bytes;               // quantity needed to pass as argument to dcd_event_xfer_complete() (for IN endpoints)
} xfer_table[8];


/*
  local helper functions
*/

/* map ep_addr into peripheral endpoint register index */
static uint8_t ep_entry(uint8_t ep_addr, bool add)
{
    for(uint8_t ep_reg = 0; ep_reg < 8; ep_reg++)
    {
        if(add)
        {
            /* take first peripheral endpoint that is unused */
            if((USBD->EPCFG[ep_reg] & USBD_EPCFG_PKSZ_Msk) == 0)
                return ep_reg;
        }
        else
        {
            uint8_t ep_nbr = USBD->EPCFG[ep_reg] & USBD_EPCFG_EPNR_Msk;
            uint8_t ep_dir = (USBD->EPCFG[ep_reg] & USBD_EPCFG_DIR_Msk) ? 0x80 : 0x00;

            /* find a peripheral endpoint that matches ep_addr */
            if((ep_nbr | ep_dir) == ep_addr)
                return ep_reg;
        }
    }

    return 0;
}


/*
  SWM341 TinyUSB API driver implementation
*/

void dcd_init(uint8_t rhport)
{
    (void) rhport;

    SYS->USBCR |= (1 << SYS_USBCR_RST48M_Pos); __DSB();
	SYS->USBCR |= (1 << SYS_USBCR_RST12M_Pos); __DSB();
	SYS->USBCR |= (1 << SYS_USBCR_RSTPLL_Pos); __DSB();
	
	SYS->USBCR &= ~SYS_USBCR_ROLE_Msk;
	SYS->USBCR |= (3 << SYS_USBCR_ROLE_Pos);
	
	SYS->USBCR |= (1 << SYS_USBCR_VBUS_Pos);
	
	SYS->CLKEN0 |= (0x01 << SYS_CLKEN0_USB_Pos);
	
	USBD_EPConfig(0, 0, USB_EP_IN,  USB_EP_CTRL, 64, 0, 0, 0);	// Control In
	USBD_EPConfig(1, 0, USB_EP_OUT, USB_EP_CTRL, 64, 0, 0, 0);	// Control Out

    USBD->FFTHR = (0xFFF << 16) | (0x000 << 0);	 // 无用
	
	USBD->DEVCR = (3 << USBD_DEVCR_SPEED_Pos)  |
				  (1 << USBD_DEVCR_DEVICE_Pos) |
				  (1 << USBD_DEVCR_CSRDONE_Pos);
	
	USBD->DEVIE = (1 << USBD_DEVIE_RST_Pos)   |
				  (1 << USBD_DEVIE_SETUP_Pos) |
				  (1 << USBD_DEVIE_SETCFG_Pos);
	
	USBD->EPIE = 0x00010001;

    xfer_table[0].max_packet_size = 64;
    xfer_table[1].max_packet_size = 64;
}

void dcd_int_enable(uint8_t rhport)
{
    (void) rhport;

    NVIC_EnableIRQ(USB_IRQn);
}

void dcd_int_disable(uint8_t rhport)
{
    (void) rhport;

    NVIC_DisableIRQ(USB_IRQn);
}

void dcd_set_address(uint8_t rhport, uint8_t dev_addr)
{
    (void) rhport;
    (void) dev_addr;
}

void dcd_remote_wakeup(uint8_t rhport)
{
    (void) rhport;
}

bool dcd_edpt_open(uint8_t rhport, tusb_desc_endpoint_t const * p_endpoint_desc)
{
    (void) rhport;

    uint8_t ep_reg = ep_entry(p_endpoint_desc->bEndpointAddress, true);

    /* mine the data for the information we need */
    const int nbr = tu_edpt_number(p_endpoint_desc->bEndpointAddress);
    const int dir = tu_edpt_dir(p_endpoint_desc->bEndpointAddress);
    const int size = tu_edpt_packet_size(p_endpoint_desc);
    const int type = p_endpoint_desc->bmAttributes.xfer;

    USBD_EPConfig(ep_reg, nbr, dir, type, size, 1, 0, 0);

    USBD->EPIE |= (1 << nbr) << (dir ? 0 : 16);

    xfer_table[ep_reg].max_packet_size = size;

    return true;
}

void dcd_edpt_close_all(uint8_t rhport)
{
    (void) rhport;
}

bool dcd_edpt_xfer(uint8_t rhport, uint8_t ep_addr, uint8_t *buffer, uint16_t total_bytes)
{
    (void) rhport;

    uint8_t ep_nbr = tu_edpt_number(ep_addr);
    uint8_t ep_reg = ep_entry(ep_addr, false);

    struct xfer_ctl_t * xfer = &xfer_table[ep_reg];

    /* store away the information we'll needing now and later */
    xfer->data_ptr = buffer;
    xfer->total_bytes = total_bytes;

    if(tu_edpt_dir(ep_addr) == TUSB_DIR_IN)
    {
        xfer->in_remaining_bytes = total_bytes;

        USBD_TxWrite(ep_nbr, xfer->data_ptr, tu_min16(xfer->in_remaining_bytes, xfer->max_packet_size));
    }
    else
    {
        xfer->out_bytes_so_far = 0;

       USBD_RxReady(ep_nbr);
    }

    return true;
}

void dcd_edpt_stall(uint8_t rhport, uint8_t ep_addr)
{
    (void) rhport;

    if(tu_edpt_dir(ep_addr) == TUSB_DIR_IN)
        USBD_TxStall(ep_addr);
    else
        USBD_RxStall(ep_addr);
}

void dcd_edpt_clear_stall(uint8_t rhport, uint8_t ep_addr)
{
    (void) rhport;
    (void) ep_addr;
}

void dcd_int_handler(uint8_t rhport)
{
    (void) rhport;

    uint32_t devif = USBD->DEVIF;
    uint32_t epif  = USBD->EPIF;

    if(devif & USBD_DEVIF_RST_Msk)
    {
        USBD->DEVIF = USBD_DEVIF_RST_Msk;

        dcd_event_bus_reset(0, TUSB_SPEED_FULL, true);
    }
	else if(devif & USBD_DEVIF_SETCFG_Msk)
	{
		USBD->DEVIF = USBD_DEVIF_SETCFG_Msk;

        extern bool process_set_config(uint8_t rhport, uint8_t cfg_num);
        process_set_config(0, 1);
	}
    else if(devif & USBD_DEVIF_SETUP_Msk)
    {
		USBD->SETUPSR = USBD_SETUPSR_DONE_Msk;
		
        uint32_t SetupBuff[2];
		SetupBuff[0] = USBD->SETUPD1;
        SetupBuff[1] = USBD->SETUPD2;

        dcd_event_setup_received(0, (uint8_t *)SetupBuff, true);
    }
	else
    {
        for(uint8_t ep_reg = 0; ep_reg < 8; ep_reg++)
        {
            struct xfer_ctl_t * xfer = &xfer_table[ep_reg];

            uint8_t ep_dir = USBD->EPCFG[ep_reg] & USBD_EPCFG_DIR_Msk;
            uint8_t ep_nbr = USBD->EPCFG[ep_reg] & USBD_EPCFG_EPNR_Msk;
            
            uint8_t ep_addr = ep_nbr | (ep_dir ? TUSB_DIR_IN_MASK : 0x00);

            if(ep_dir)
            {
                if(epif & (1 << ep_nbr))
                {
                    if(USBD_TxSuccess(ep_nbr))
                    {
                        uint16_t size = tu_min16(xfer->in_remaining_bytes, xfer->max_packet_size);

                        /* update the bookkeeping to reflect the data that has now been sent to the PC */
                        xfer->in_remaining_bytes -= size;
                        xfer->data_ptr += size;

                        /* if more data to send, send it; otherwise, alert TinyUSB that we've finished */
                        if(xfer->in_remaining_bytes)
                            USBD_TxWrite(ep_nbr, xfer->data_ptr, tu_min16(xfer->in_remaining_bytes, xfer->max_packet_size));
                        else
                            dcd_event_xfer_complete(0, ep_addr, xfer->total_bytes, XFER_RESULT_SUCCESS, true);
                    }
                    USBD_TxIntClr(ep_nbr);

                    return;
                }
            }
            else
            {
                if(epif & (1 << (16 + ep_nbr)))
                {
                    USBD_RxIntClr();
                    if(USBD_RxSuccess())
                    {
                        uint16_t size = USBD_RxRead(xfer->data_ptr, xfer->max_packet_size);

                        xfer->data_ptr += size;
                        xfer->out_bytes_so_far += size;

                        /* when the transfer is finished, alert TinyUSB; otherwise, accept more data */
                        if((xfer->total_bytes == xfer->out_bytes_so_far) || (size < xfer->max_packet_size))
                            dcd_event_xfer_complete(0, ep_addr, xfer->out_bytes_so_far, XFER_RESULT_SUCCESS, true);
                        else
                            USBD_RxReady(ep_nbr);
                    }

                    return;
                }
            }
        }
    }
}

void dcd_disconnect(uint8_t rhport)
{
    (void) rhport;

    //USBD->DRVSE0 |= USBD_DRVSE0_DRVSE0_Msk;
}

void dcd_connect(uint8_t rhport)
{
    (void) rhport;
    
    //USBD->DRVSE0 &= ~USBD_DRVSE0_DRVSE0_Msk;
}

#endif
