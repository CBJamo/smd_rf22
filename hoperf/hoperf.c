//#include <uapi/linux/if_arp.h>
#include <linux/module.h>
#include <linux/hoperfdevice.h>
#include <linux/if_ether.h>
#include <linux/if_hoperf.h>
#include <net/arp.h>

__setup( "hoperf=", netdev_boot_setup );

static unsigned char eth_proto_to_hoperf( __be16 type )
{
	switch( type )
	{
	case ETH_P_IP:
		return HOPERF_P_IP;
	case ETH_P_ARP:
		return HOPERF_P_ARP;
	case ETH_P_RARP:
		return HOPERF_P_RARP;
	case ETH_P_PAUSE:
		return HOPERF_P_PAUSE;
	case ETH_P_1588:
		return HOPERF_P_1588;
	default:
		pr_err( "hoperf: Failed to convert %04X to HopeRF", type );
		return HOPERF_P_CONTROL;
	}
}

static __be16 hoperf_proto_to_eth( unsigned char type )
{
	switch( type )
	{
	default:
		pr_err( "hoperf: Failed to convert %02X to ETH proto", type );
	case HOPERF_P_IP:
		return ETH_P_IP;
	case HOPERF_P_ARP:
		return ETH_P_ARP;
	case HOPERF_P_RARP:
		return ETH_P_RARP;
	case HOPERF_P_PAUSE:
		return ETH_P_PAUSE;
	case HOPERF_P_1588:
		return ETH_P_1588;
	case HOPERF_P_MACSEC:
		return 0x88E5;
	case HOPERF_P_WOL:
		return 0x0842;
	}
}

int hoperf_header( struct sk_buff *skb, struct net_device *net_dev,
	unsigned short type,
	const void *daddr, const void *saddr, unsigned int len )
{
	struct hoperf_hdr *hrf = (struct hoperf_hdr *)skb_push( skb, HOPERF_HLEN );

	hrf->id = 0x00;
	hrf->proto = eth_proto_to_hoperf( type );
	hrf->len = len;

 	/*
	 *      Set the source hardware address.
	 */

	if( !saddr )
		saddr = net_dev->dev_addr;
	memcpy( hrf->src, saddr, HOPERF_ALEN );

	if( daddr )
	{
		memcpy( hrf->dst, daddr, HOPERF_ALEN );
			return HOPERF_HLEN;
	}

	/*
	 *      Anyway, the loopback-device should never use this function...
	 */

	if( net_dev->flags & ( IFF_LOOPBACK | IFF_NOARP ) )
	{
		memset( hrf->dst, 0, HOPERF_ALEN );
		return HOPERF_HLEN;
	}

	return -HOPERF_HLEN;
}
EXPORT_SYMBOL( hoperf_header );

int hoperf_rebuild_header( struct sk_buff *skb )
{
	struct hoperf_hdr *hrf = (struct hoperf_hdr *)skb->data;
	struct net_device *net_dev = skb->dev;
	int ret;

	switch( hrf->proto )
	{
	#ifdef CONFIG_INET
	case HOPERF_P_IP:
		ret = arp_find( hrf->dst, skb );
		pr_info( "%s: resolved arp: %d", net_dev->name, ret );
		return ret;
	#endif
	default:
		pr_info( "%s: unable to resolve type %02X addresses.\n",
			net_dev->name, hrf->proto );

		memcpy( hrf->src, net_dev->dev_addr, HOPERF_ALEN );
		break;
	}

	return 0;
}
EXPORT_SYMBOL( hoperf_rebuild_header );

__be16 hoperf_type_trans( struct sk_buff *skb, struct net_device *net_dev )
{
	struct hoperf_hdr *hrf;

	skb->dev = net_dev;
	skb_reset_mac_header( skb );
	skb_pull_inline( skb, HOPERF_HLEN );
	hrf = hoperf_hdr( skb );

	if( unlikely( is_broadcast_hoperf_addr( hrf->dst ) ) )
	{
		if( hoperf_addr_equal( hrf->dst, net_dev->broadcast ) )
			skb->pkt_type = PACKET_BROADCAST;
	}

	/*
	 *      This ALLMULTI check should be redundant by 1.4
	 *      so don't forget to remove it.
	 *
	 *      Seems, you forgot to remove it. All silly devices
	 *      seems to set IFF_PROMISC.
	 */
	else if( 1 /*dev->flags&IFF_PROMISC */ )
	{
		if( unlikely( !hoperf_addr_equal( hrf->dst, net_dev->dev_addr ) ) )
			skb->pkt_type = PACKET_OTHERHOST;
	}

	return htons( hoperf_proto_to_eth( hrf->proto ) );
}
EXPORT_SYMBOL( hoperf_type_trans );

int hoperf_header_parse( const struct sk_buff *skb, unsigned char *haddr )
{
	const struct hoperf_hdr *hrf = hoperf_hdr( skb );
	memcpy( haddr, hrf->src, HOPERF_ALEN);
	return HOPERF_ALEN;
}
EXPORT_SYMBOL( hoperf_header_parse );

/*int hoperf_header_cache( const struct neighbour *neigh, struct hh_cache *hh, __be16 type )
{
	struct hoperf_hdr *hrf;
	const struct net_device *net_dev = neigh->dev;

	hrf = (struct hoperf_hdr *)
		( ( (u8 *)hh->hh_data ) + ( HH_DATA_OFF( sizeof( *hrf ) ) ) );

	hrf->proto = eth_proto_to_hoperf( htons( type ) );
	memcpy( hrf->src, net_dev->dev_addr, HOPERF_ALEN );
	memcpy( hrf->dst, neigh->ha, HOPERF_ALEN );
	hh->hh_len = HOPERF_HLEN;
	return 0;
}
EXPORT_SYMBOL( hoperf_header_cache );*/

void hoperf_header_cache_update( struct hh_cache *hh,
	const struct net_device *net_dev,
	const unsigned char *haddr )
{
	memcpy( ( (u8 *)hh->hh_data) + HH_DATA_OFF( sizeof( struct hoperf_hdr ) ),
		haddr, HOPERF_ALEN );
}
EXPORT_SYMBOL( hoperf_header_cache_update );

int hoperf_prepare_mac_addr_change( struct net_device *net_dev, void *p )
{
	struct sockaddr *addr = p;

	if( !( net_dev->priv_flags & IFF_LIVE_ADDR_CHANGE) && netif_running( net_dev ) )
		return -EBUSY;
	if ( !is_valid_hoperf_addr( addr->sa_data ) )
		return -EADDRNOTAVAIL;
	return 0;
}
EXPORT_SYMBOL( hoperf_prepare_mac_addr_change );

void hoperf_commit_mac_addr_change( struct net_device *net_dev, void *p )
{
	struct sockaddr *addr = p;

	memcpy( net_dev->dev_addr, addr->sa_data, HOPERF_ALEN );
}
EXPORT_SYMBOL( hoperf_commit_mac_addr_change );

int hoperf_mac_addr( struct net_device *net_dev, void *p )
{
	int ret;

	ret = hoperf_prepare_mac_addr_change( net_dev, p );
	if( ret < 0 )
		return ret;
	hoperf_commit_mac_addr_change( net_dev, p );
	return 0;
}
EXPORT_SYMBOL( hoperf_mac_addr );

int hoperf_change_mtu( struct net_device *net_dev, int new_mtu )
{
	if( new_mtu < 0 || new_mtu > HOPERF_DATA_LEN )
		return -EINVAL;
	net_dev->mtu = new_mtu;
	return 0;
}
EXPORT_SYMBOL( hoperf_change_mtu );

int hoperf_validate_addr( struct net_device *net_dev )
{
	if( !is_valid_hoperf_addr( net_dev->dev_addr ) )
		return -EADDRNOTAVAIL;

	return 0;
}
EXPORT_SYMBOL( hoperf_validate_addr );

const struct header_ops hoperf_header_ops ____cacheline_aligned =
{
	.create		= hoperf_header,
	.parse		= hoperf_header_parse,
	.rebuild	= hoperf_rebuild_header,
//	.cache		= hoperf_header_cache,
	.cache_update	= hoperf_header_cache_update,
};

void hoperf_setup( struct net_device *net_dev )
{
	net_dev->header_ops		= &hoperf_header_ops;
	net_dev->type			= ARPHRD_VOID;
	net_dev->hard_header_len	= HOPERF_HLEN;
	net_dev->mtu			= HOPERF_DATA_LEN;
	net_dev->addr_len		= HOPERF_ALEN;
	net_dev->tx_queue_len		= 4;
	net_dev->flags			= IFF_BROADCAST;

	memset( net_dev->broadcast, 0xFF, HOPERF_ALEN );
}
EXPORT_SYMBOL( hoperf_setup );

struct net_device *alloc_hoperfdev_mqs( int sizeof_priv, unsigned int txqs,
	unsigned int rxqs )
{
	return alloc_netdev_mqs( sizeof_priv, "rf%d", NET_NAME_ENUM, hoperf_setup, txqs, rxqs );
}
EXPORT_SYMBOL( alloc_hoperfdev_mqs );

MODULE_LICENSE( "GPL" );

