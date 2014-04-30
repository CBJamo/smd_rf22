#ifndef _LINUX_IF_HOPERF_H
#define _LINUX_IF_HOPERF_H

#include <linux/skbuff.h>
#include <uapi/linux/if_hoperf.h>

static inline struct hoperf_hdr *hoperf_hdr( const struct sk_buff *skb )
{
	return (struct hoperf_hdr *)skb_mac_header( skb );
}

int hoperf_header_parse( const struct sk_buff *skb, unsigned char *haddr );

#endif /* _LINUX_IF_HOPERF_H */
