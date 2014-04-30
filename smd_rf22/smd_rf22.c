#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/usb.h>
#include <linux/netdevice.h>
#include <uapi/linux/if_arp.h>
#include <uapi/linux/tty_flags.h>
#include <net/neighbour.h>
#include <linux/hoperfdevice.h>
#include <linux/if_hoperf.h>

#include "rf22_priv.h"

/*
 * Defines
 */
#define DRIVER_AUTHOR "Scott K Logan <logans@cottsay.net>"
#define DRIVER_DESC "RF22-based Networking Device"
#define DRIVER_NAME "smd_rf22"

#define SMD_RF22_VID 0x03EB /* Atmel VID */
#define SMD_RF22_PID 0x8888 /* Arbitrary Unused PID */

#define RF22_GET_REGISTER	0x01
#define RF22_SET_REGISTER	0x02
#define RF22_SET_REGISTER_TEMP	0x03

/*
 * Globals
 */
static __u16 vendor = SMD_RF22_VID;
static __u16 product = 0;
static int msg_level = -1;

/*
 * VID/PID Table
 */
static struct usb_device_id id_table [] =
{
	{ USB_DEVICE( SMD_RF22_VID, SMD_RF22_PID ) },
	{ },
	{ }
};

MODULE_DEVICE_TABLE( usb, id_table );

/*
 * Structures
 */
enum smd_rf22_flags
{
	SMD_RF22_NO_FLAGS = 0,
	SMD_RF22_TX_ACTIVE = ( 1 << 0 )
};

struct smd_rf22
{
	struct usb_device *usb_dev;
	struct usb_interface *usb_interface;
	struct net_device *net_dev;
	int flags;

	struct sk_buff_head rxq;
        struct sk_buff_head txq;
	struct sk_buff_head done;
	size_t rxq_max;
	size_t txq_max;

	unsigned char *bulk_in_buff;
	struct urb *read_urb;
	size_t bulk_in_buff_len;
	size_t bulk_out_buff_len;
	size_t control_buff_len;
	__u8 bulk_in_endpointAddr;
	__u8 bulk_out_endpointAddr;
	__u8 control_endpointAddr;

	int msg_enable;

	uint8_t device_mtu;
};

static struct device_type rf22_type =
{
	.name		= "RF22"
};

/*
 * Same as USBNET
 */
enum skb_state
{
	illegal = 0,
	tx_start,
	tx_done,
	rx_start,
	rx_done,
	rx_cleanup
};

struct skb_data
{
	struct urb *urb;
	struct smd_rf22 *dev;
	enum skb_state state;
	size_t length;
};

/*
 * USB Function Prototypes
 */
static int smd_rf22_probe( struct usb_interface *interface, const struct usb_device_id *id );
static void smd_rf22_disconnect( struct usb_interface *interface );
static int __init smd_rf22_init( void );
static void __exit smd_rf22_exit( void );
static int smd_rf22_probe_endpoints( struct usb_interface *interface );
static int smd_rf22_write_packet( struct usb_interface *interface, struct sk_buff *skb );
static void smd_rf22_write_complete( struct urb *urb );
static void smd_rf22_read_complete( struct urb *urb );

/*
 * Network Function Prototypes
 */
static int smd_rf22_open( struct net_device *net_dev );
static int smd_rf22_stop( struct net_device *net_dev );
static netdev_tx_t smd_rf22_xmit( struct sk_buff *skb, struct net_device *net_dev );
static void smd_rf22_free_skb( struct smd_rf22 *dev,
	struct sk_buff *skb, struct sk_buff_head *list );
static int smd_rf22_set_mac_addr( struct net_device *netdev, void *new_addr );
static int smd_rf22_change_mtu( struct net_device *net_dev, int new_mtu );
/*
 * Kernel Module
 */
static struct usb_driver smd_rf22_driver =
{
	.name		= DRIVER_NAME,
	.probe		= smd_rf22_probe,
	.disconnect	= smd_rf22_disconnect,
	.id_table	= id_table,
	.no_dynamic_id	= 1,
	.supports_autosuspend = 0,
};

static const struct net_device_ops smd_rf22_ndo =
{
	.ndo_open		= smd_rf22_open,
	.ndo_stop		= smd_rf22_stop,
	.ndo_start_xmit		= smd_rf22_xmit,
	.ndo_change_mtu		= smd_rf22_change_mtu,
	.ndo_set_mac_address	= smd_rf22_set_mac_addr,
	.ndo_validate_addr	= hoperf_validate_addr,
};

module_init( smd_rf22_init );
module_exit( smd_rf22_exit );

/*
 * Function Definitions
 */
static int smd_rf22_probe( struct usb_interface *interface, const struct usb_device_id *id )
{
	struct usb_device *usb_dev = interface_to_usbdev( interface );
	struct net_device *net_dev = NULL;
	struct smd_rf22 *dev = NULL;
	int retval = 0;

	/* Connect Net */
	net_dev = alloc_hoperfdev( sizeof( *dev ) );
	if( NULL == net_dev )
	{
		retval = -ENOMEM;
                goto err0;
	}

        SET_NETDEV_DEV( net_dev, &usb_dev->dev );

	dev = netdev_priv( net_dev );
	dev->net_dev = net_dev;
	dev->rxq_max = 4;
	dev->txq_max = 4;
	skb_queue_head_init( &dev->rxq );
	skb_queue_head_init( &dev->txq );
	skb_queue_head_init( &dev->done );
	dev->msg_enable = netif_msg_init( msg_level, NETIF_MSG_DRV | NETIF_MSG_PROBE | NETIF_MSG_LINK );
	net_dev->netdev_ops = &smd_rf22_ndo;
	//net_dev->flags |= IFF_NOARP;
	hoperf_random_addr( net_dev->dev_addr );
	SET_NETDEV_DEVTYPE( net_dev, &rf22_type );

	retval = register_netdev( net_dev );
	if( retval )
		goto err1;

	/* Connect USB */
	dev->usb_dev = usb_get_dev( usb_dev );
	dev->usb_interface = interface;
	usb_set_intfdata( interface, dev );
	retval = smd_rf22_probe_endpoints( interface );
	if( retval )
		goto err2;
	// TODO: More USB setup
	// TODO: Get dev_addr from device
	retval = usb_control_msg( usb_dev, usb_rcvctrlpipe( usb_dev, 0 ), RF22_GET_REGISTER, USB_DIR_IN | USB_TYPE_VENDOR, RF22_REG_3F_CHECK_HEADER3, 0x00, net_dev->dev_addr, 0x01, 100 );
	if( retval < 0 )
	{
		dev_err( &interface->dev, "Failed to fetch device address (%d)\n", retval );
		goto err2;
	}

	retval = usb_control_msg( usb_dev, usb_rcvctrlpipe( usb_dev, 0 ), RF22_GET_REGISTER, USB_DIR_IN | USB_TYPE_VENDOR, RF22_REG_41_CHECK_HEADER1, 0x00, &dev->device_mtu, 0x01, 100 );
	if( retval < 0 )
	{
		dev_err( &interface->dev, "Failed to fetch device MTU (%d)\n", retval );
		goto err2;
	}
	dev->device_mtu = HOPERF_DATA_LEN - dev->device_mtu;
	net_dev->mtu = dev->device_mtu;

	/* Ready to go */
	netif_info(dev, probe, net_dev,
		"register '%s' at usb-%s-%s\n",
		interface->dev.driver->name,
		usb_dev->bus->bus_name, usb_dev->devpath);

	netif_device_attach( net_dev );

	dev_dbg( &interface->dev, "New "DRIVER_NAME" device registered\n" );

	return 0;
err2:
	unregister_netdev( dev->net_dev );
err1:
	free_netdev( net_dev );
err0:
	return retval;
}

static void smd_rf22_disconnect( struct usb_interface *interface )
{
	struct smd_rf22 *dev = usb_get_intfdata( interface );

	/* Disconnect USB */
	usb_poison_urb( dev->read_urb );
	dev_kfree_skb_any( (struct sk_buff *)dev->read_urb->context );
	usb_free_urb( dev->read_urb );
	kfree( dev->bulk_in_buff );
	usb_set_intfdata( interface, NULL );
	if( NULL == dev )
		return;
	usb_put_dev( dev->usb_dev );

	/* Disconnect Net */
	unregister_netdev( dev->net_dev );
	free_netdev( dev->net_dev );

	dev_info( &interface->dev, DRIVER_NAME" device disconnected\n" );
}

static int __init smd_rf22_init( void )
{
	int retval = 0;

	if ( vendor > 0 && product > 0 )
	{
		pr_debug( DRIVER_NAME" adding vendor 0x%04X product 0x%04X", vendor, product );
		id_table[1].match_flags = USB_DEVICE_ID_MATCH_DEVICE;
		id_table[1].idVendor = vendor;
		id_table[1].idProduct = product;
        }

	retval = usb_register( &smd_rf22_driver );
	if( retval )
		pr_err( DRIVER_NAME": Failed to register USB driver: %d", retval );
	else
		pr_debug( DRIVER_NAME": Registered smd_rf22 driver" );

	return retval;
}

static void __exit smd_rf22_exit( void )
{
	usb_deregister( &smd_rf22_driver );
	pr_debug( DRIVER_NAME": Deregistered smd_rf22 driver" );
}

static int smd_rf22_open( struct net_device *net_dev )
{
	struct smd_rf22 *dev = netdev_priv( net_dev );
	uint8_t rx_mode = RF22_XTON | RF22_RXON | RF22_PLLON;
	int retval;

	// Go to RX mode
	retval = usb_control_msg( dev->usb_dev, usb_sndctrlpipe( dev->usb_dev, 0 ), RF22_SET_REGISTER_TEMP, USB_DIR_OUT | USB_TYPE_VENDOR, RF22_REG_07_OPERATING_MODE1, 0x00, &rx_mode, 0x01, 200 );
	if( retval < 0 )
	{
		dev_err( &dev->usb_interface->dev, "Failed to change to RX mode (%d)\n", retval );
		return -EIO;
	}

	// Setup USB RX
	usb_submit_urb( dev->read_urb, GFP_ATOMIC );

	// Start TX
        netif_start_queue( net_dev );
        netif_info( dev, ifup, net_dev,
		"open: enable queueing (rx %zu, tx %zu) mtu %d %s framing\n",
		dev->rxq_max, dev->txq_max, net_dev->mtu, "simple");

	// Carrier Up
	netif_carrier_on( net_dev );

	return 0;
}

static int smd_rf22_stop( struct net_device *net_dev )
{
	struct smd_rf22 *dev = netdev_priv( net_dev );
	uint8_t idle_mode = RF22_XTON;
	int retval = 0;

	netif_carrier_off( net_dev );

	// Stop TX
	netif_stop_queue( net_dev );
	dev->flags &= ~SMD_RF22_TX_ACTIVE;

	// Go to IDLE mode
	retval = usb_control_msg( dev->usb_dev, usb_sndctrlpipe( dev->usb_dev, 0 ), RF22_SET_REGISTER_TEMP, USB_DIR_OUT | USB_TYPE_VENDOR, RF22_REG_07_OPERATING_MODE1, 0x00, &idle_mode, 0x01, 100 );
	if( retval < 0 )
	{
		dev_err( &dev->usb_interface->dev, "Failed to change to IDLE mode (%d)\n", retval );
	}

	// Stop recv
	usb_unlink_urb( dev->read_urb );

        netif_info( dev, ifdown, net_dev,
		"stop stats: rx/tx %lu/%lu, errs %lu/%lu\n",
		net_dev->stats.rx_packets, net_dev->stats.tx_packets,
		net_dev->stats.rx_errors, net_dev->stats.tx_errors);

	return retval;
}

static netdev_tx_t smd_rf22_xmit( struct sk_buff *skb, struct net_device *net_dev )
{
	struct smd_rf22 *dev = netdev_priv( net_dev );
	struct skb_data *entry = (struct skb_data *)skb->cb;
	unsigned long flags = 0;
	int retval = 0;

	/*pr_debug( DRIVER_NAME": TX to: %02X from: %02X type: %02X size: %hhu total: %u",
		skb->data[0], skb->data[1], skb->data[3], skb->data[4], skb->len );*/

	spin_lock_irqsave( &dev->txq.lock, flags );
	retval = smd_rf22_write_packet( dev->usb_interface, skb );
	switch( retval )
	{
	case -ENOMEM:
		spin_unlock_irqrestore ( &dev->txq.lock, flags );
		goto drop;
		break;
	case -EPIPE:
		netif_stop_queue( net_dev );
		dev->flags &= ~SMD_RF22_TX_ACTIVE;
		//usbnet_defer_kevent( dev, EVENT_TX_HALT );
		pr_err( "tx: EPIPE" );
		break;
	default:
		netif_err( dev, tx_err, dev->net_dev,
			"tx: submit urb err %d\n", retval );
		break;
	case 0:
		__skb_queue_tail( &dev->txq, skb );
		entry->state = tx_start;
		if( dev->txq.qlen >= dev->txq_max )
		{
			netif_stop_queue( net_dev );
			dev->flags &= ~SMD_RF22_TX_ACTIVE;
			pr_info( "stopping queue" );
		}
		break;
	}
	spin_unlock_irqrestore ( &dev->txq.lock, flags );

	return NETDEV_TX_OK;

drop:
	pr_err( "Dropped a packet: %d", retval );
	net_dev->stats.tx_dropped++;

	if( skb )
		dev_kfree_skb_any( skb );
	return NETDEV_TX_OK;
}

static int smd_rf22_probe_endpoints( struct usb_interface *interface )
{
	struct smd_rf22 *dev = usb_get_intfdata( interface );
	struct usb_device *usb_dev = dev->usb_dev;
	struct usb_host_interface *iface_desc = interface->cur_altsetting;
	struct usb_endpoint_descriptor *endpoint = NULL;
	struct sk_buff *skb = NULL;
	struct skb_data *entry = NULL;
	int retval = 0;
	int i;

	dev->control_endpointAddr = usb_dev->ep0.desc.bEndpointAddress;
	dev->control_buff_len = usb_endpoint_maxp( &usb_dev->ep0.desc );

	for( i = 0; i < iface_desc->desc.bNumEndpoints; i++ )
	{
		endpoint = &iface_desc->endpoint[i].desc;

		if( usb_endpoint_is_bulk_in( endpoint ) )
		{
			dev->bulk_in_endpointAddr = endpoint->bEndpointAddress;
			dev->bulk_in_buff_len = usb_endpoint_maxp( endpoint );
			dev_dbg( &interface->dev, "found bulk in endpoint\n" );

			// urb
			dev->read_urb = usb_alloc_urb( 0, GFP_KERNEL );
			if( !dev->read_urb )
			{
				retval = -ENOMEM;
				goto err;
			}
			dev->bulk_in_buff = kmalloc( dev->bulk_in_buff_len, GFP_ATOMIC );
			if( !dev->bulk_in_buff )
			{
				retval = -ENOMEM;
				goto err;
			}
			skb = netdev_alloc_skb( dev->net_dev, HOPERF_FRAME_LEN );
			if( !skb )
			{
				retval = -ENOMEM;
				goto err;
			}
			entry = (struct skb_data *)skb->cb;
			entry->urb = dev->read_urb;
			entry->dev = dev;
			entry->length = 0;
			usb_fill_bulk_urb( dev->read_urb, usb_dev,
				usb_rcvbulkpipe( usb_dev, dev->bulk_in_endpointAddr ),
				dev->bulk_in_buff, dev->bulk_in_buff_len,
				smd_rf22_read_complete, skb );
		}
		else if( usb_endpoint_is_bulk_out( endpoint ) )
		{
			dev->bulk_out_endpointAddr = endpoint->bEndpointAddress;
			dev->bulk_out_buff_len = usb_endpoint_maxp( endpoint );
			dev_dbg( &interface->dev, "found bulk out endpoint\n" );
		}
		else if( usb_endpoint_is_int_in( endpoint ) )
			dev_dbg( &interface->dev, "found int in endpoint, ignoring\n" );
		else if( usb_endpoint_is_int_out( endpoint ) )
			dev_dbg( &interface->dev, "found int out endpoint, ignoring\n" );
		else
			dev_dbg( &interface->dev, "found unknown on endpoint %d, ignoring\n", i );
	}

	if( dev->bulk_in_endpointAddr == 0 )
	{
		dev_err( &interface->dev, "failed to find a bulk in endpoint" );
		retval = -EIO;
		goto err;
	}
	else if( dev->bulk_out_endpointAddr == 0 )
	{
		dev_err( &interface->dev, "failed to find a bulk out endpoint" );
		retval = -EIO;
		goto err;
	}

	return 0;

err:
	dev_kfree_skb_any( skb );
	usb_free_urb( dev->read_urb );
	kfree( dev->bulk_in_buff );
	return retval;
}

static int smd_rf22_write_packet( struct usb_interface *interface, struct sk_buff *skb )
{
	struct smd_rf22 *dev = usb_get_intfdata( interface );
	struct skb_data *entry;
	int retval = 0;
	struct urb *urb = NULL;

	/* verify that we actually have some data to write */
	if( !skb || skb->len <= 0 )
		goto out;

	skb_tx_timestamp( skb );

	urb = usb_alloc_urb( 0, GFP_ATOMIC );
	if( !urb )
	{
		retval = -ENOMEM;
		pr_err( "failed to allocate urb" );
		goto err;
	}

	entry = (struct skb_data *)skb->cb;
	entry->urb = urb;
	entry->dev = usb_get_intfdata( interface );
	entry->length = 0;

	/* initialize the urb properly */
	//usb_fill_bulk_urb( urb, dev->usb_dev,
	//		  usb_sndbulkpipe( dev->usb_dev, dev->bulk_out_endpointAddr ),
	//		  skb->data, ( skb->len < dev->bulk_out_buff_len ) ? skb->len : dev->bulk_out_buff_len, smd_rf22_write_complete, skb );
	usb_fill_bulk_urb( urb, dev->usb_dev,
			  usb_sndbulkpipe( dev->usb_dev, dev->bulk_out_endpointAddr ),
			  skb->data, skb->len, smd_rf22_write_complete, skb );
	//urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

	/* send the data out the bulk port */
	if( 0 == dev->txq.qlen )
	{
		retval = usb_submit_urb( urb, GFP_ATOMIC );
		if( retval )
		{
			pr_err( "failed to submit urb: %d", retval );
			goto err;
		}
	}

out:
	return retval;

err:
	usb_free_urb( urb );
	return retval;
}

static void smd_rf22_write_complete( struct urb *urb )
{
	struct sk_buff *skb = (struct sk_buff *)urb->context;
	struct skb_data *entry = (struct skb_data *)skb->cb;
        struct smd_rf22 *dev = entry->dev;
	unsigned long flags;
	int retval;

	if( urb->status == 0 )
	{
		if( urb->actual_length != 0 && urb->actual_length % dev->bulk_out_buff_len == 0 ) // Send ZLP
		{
			usb_fill_bulk_urb( urb, dev->usb_dev,
				usb_sndbulkpipe( dev->usb_dev, dev->bulk_out_endpointAddr ),
				0x00, 0x00, smd_rf22_write_complete, skb );
			retval = usb_submit_urb( urb, GFP_ATOMIC );
			if( retval )
			{
				pr_err( "failed to submit ZLP urb: %d", retval );
				dev->net_dev->stats.tx_errors++;
				smd_rf22_free_skb( dev, skb, &dev->txq );
			}
			return;
		}

		dev->net_dev->stats.tx_packets++;
		dev->net_dev->stats.tx_bytes += entry->length;
        }
	else
	{
		dev->net_dev->stats.tx_errors++;
		netif_dbg( dev, tx_err, dev->net_dev, "tx err %d\n", urb->status );
		switch( urb->status )
		{
		case -EPIPE:
		case -EPROTO:
		case -ETIME:
		case -EILSEQ:
			netif_stop_queue( dev->net_dev );
			dev->flags &= ~SMD_RF22_TX_ACTIVE;
		default:
			pr_info( "tx err %d\n", urb->status );
			break;
		}
	}

	smd_rf22_free_skb( dev, skb, &dev->txq );
	//pr_info( "tx complete txq %u", dev->txq.qlen );

	spin_lock_irqsave( &dev->txq.lock, flags );
	if( 0 != dev->txq.qlen )
	{
		entry = (struct skb_data *)dev->txq.next->cb;
		retval = usb_submit_urb( entry->urb, GFP_ATOMIC );
		if( retval )
			pr_err( "failed to submit urb 3: %d", retval );
	}
	if( !( dev->flags & SMD_RF22_TX_ACTIVE ) && dev->txq.qlen < dev->txq_max )
	{
		netif_wake_queue( dev->net_dev );
		dev->flags |= SMD_RF22_TX_ACTIVE;
	}
	spin_unlock_irqrestore ( &dev->txq.lock, flags );
}

static void smd_rf22_read_complete( struct urb *urb )
{
	struct sk_buff *skb = (struct sk_buff *)urb->context;
	struct skb_data *entry = (struct skb_data *)skb->cb;
        struct smd_rf22 *dev = entry->dev;
	struct net_device *net_dev = dev->net_dev;

	if( urb->actual_length )
		pr_info( "Got an msg of length %u\n", urb->actual_length );

	if( netif_running( net_dev ) )
	{
		if( urb->status )
		{
			pr_warn( "Urb with status: %d\n", urb->status );
			entry->length = 0;
			goto cont;
		}

		if( entry->length + urb->actual_length > HOPERF_FRAME_LEN )
		{
			// REALLY REALLY BAD
			pr_err( "SKB OVERRUN\n" );
			entry->length = 0;
			net_dev->stats.rx_errors++;
			goto cont;
		}

		memcpy( skb->data + entry->length, urb->transfer_buffer, urb->actual_length );
		entry->length += urb->actual_length;

		if( urb->actual_length != dev->bulk_in_buff_len && entry->length ) // Not a full urb, designates end-of-packet
		{
			struct hoperf_hdr *hrf;
			int retval;

			if( entry->length < HOPERF_ZLEN )
			{
				// ALSO REALLY REALLY BAD
				pr_err( "PACKET TOO SMALL: %zu\n", entry->length );
				entry->length = 0;
				net_dev->stats.rx_errors++;
				goto cont;
			}

			skb->len = entry->length;
			skb->protocol = hoperf_type_trans( skb, net_dev );
			hrf = hoperf_hdr( skb );

			if( entry->length != hrf->len + HOPERF_HLEN )
			{
				// ALSO REALLY REALLY BAD
				pr_err( "PACKET SIZE MISMATCH: header says %u, got %zu\n", hrf->len + HOPERF_HLEN, entry->length );
				entry->length = 0;
				net_dev->stats.rx_errors++;
				dev_kfree_skb_any( skb );
				goto cont_new;
			}
			else if( hrf->id != 0x00 || hrf->proto > 0x07 )
			{
				pr_err( "BAD PACKET HEADER: id %u proto %u\n", hrf->id, hrf->proto );
				entry->length = 0;
				net_dev->stats.rx_errors++;
				dev_kfree_skb_any( skb );
				goto cont_new;
			}

			pr_info( "entry->len += %u !!FINISH PACKET!! type: %02X size: %02X/%02X\n", urb->actual_length, hrf->proto, hrf->len, skb->len );
			net_dev->stats.rx_packets++;
			net_dev->stats.rx_bytes += skb->len + HOPERF_HLEN;

			retval = netif_rx( skb );
			if( retval != NET_RX_SUCCESS )
				pr_err( "netif_rx status %d\n", retval );

			// Make a new SKB
		cont_new:
			skb = netdev_alloc_skb( dev->net_dev, HOPERF_FRAME_LEN );
			if( !skb )
			{
				pr_err( "Failed to make a new skb!\n" );
				return;
			}
			entry = (struct skb_data *)skb->cb;
			entry->urb = urb;
			entry->dev = dev;
			urb->context = skb;
			entry->length = 0;
		}

	cont:
		usb_submit_urb( dev->read_urb, GFP_ATOMIC );
	}
}

static void smd_rf22_free_skb( struct smd_rf22 *dev,
	struct sk_buff *skb, struct sk_buff_head *list)
{
	unsigned long flags;
	struct skb_data *entry = (struct skb_data *)skb->cb;

	spin_lock_irqsave( &list->lock, flags );
	__skb_unlink( skb, list );
	spin_unlock_irqrestore( &list->lock, flags );
	usb_free_urb( entry->urb );
	dev_kfree_skb_any( skb );
}

static int smd_rf22_set_mac_addr( struct net_device *net_dev, void *p )
{
	struct sockaddr *addr = p;
	struct smd_rf22 *dev = netdev_priv( net_dev );
	struct usb_device *usb_dev = dev->usb_dev;
	int retval;

	retval = hoperf_prepare_mac_addr_change( net_dev, p );
	if( retval < 0 )
                return retval;
	retval = usb_control_msg( usb_dev, usb_sndctrlpipe( usb_dev, 0 ), RF22_SET_REGISTER_TEMP, USB_DIR_OUT | USB_TYPE_VENDOR, RF22_REG_3F_CHECK_HEADER3, 0x00, addr->sa_data, 0x01, 100 );
	if( retval < 0 )
		return retval;
	hoperf_commit_mac_addr_change( net_dev, p );

	return 0;
}

static int smd_rf22_change_mtu( struct net_device *net_dev, int new_mtu )
{
	struct smd_rf22 *dev = netdev_priv( net_dev );
	if( (unsigned)new_mtu > dev->device_mtu )
		return -EINVAL;
	return hoperf_change_mtu( net_dev, new_mtu );
}

MODULE_AUTHOR( DRIVER_AUTHOR );
MODULE_DESCRIPTION( DRIVER_DESC );
MODULE_LICENSE( "GPL" );

module_param( vendor, ushort, 0 );
MODULE_PARM_DESC( vendor, "User specified vendor ID (default="__MODULE_STRING(SMD_RF22_VID)")" );
module_param( product, ushort, 0 );
MODULE_PARM_DESC( product, "User specified product ID" );
module_param( msg_level, int, 0 );
MODULE_PARM_DESC( msg_level, "Override default message level" );
