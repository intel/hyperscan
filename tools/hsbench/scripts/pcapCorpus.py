#!/usr/bin/env python

'''
Script to convert a pcap file containing UDP and TCP packets to a corpus file.
'''

import sys, getopt, pprint, os
from sqlite3 import dbapi2 as sqlite
import pcap
from optparse import OptionParser
from socket import AF_INET, IPPROTO_UDP, IPPROTO_TCP, inet_ntop, ntohs, ntohl, inet_ntoa
import struct
from CorpusBuilder import CorpusBuilder

ETHERTYPE_IP        = 0x0800    # IP protocol
ETHERTYPE_ARP       = 0x0806    # Addr. resolution protocol
ETHERTYPE_REVARP    = 0x8035    # reverse Addr. resolution protocol
ETHERTYPE_VLAN      = 0x8100    # IEEE 802.1Q VLAN tagging
ETHERTYPE_IPV6      = 0x86dd    # IPv6

#
# A dictionary of active TCP streams
#
tcp_streams = {}

#
# A dictionary of UDP streams
#
udp_streams = {}

#
# Current stream id
cur_stream_id = 0

def usage(exeName) :
    errmsg = "Usage: %s -i <pcap-file> -o <sqlite-file>"
    errmsg = errmsg % exeName
    print >> sys.stderr, errmsg
    sys.exit(-1)

class FiveTuple(object):
    def __init__(self, protocol, src_addr, src_port, dst_addr, dst_port):
        self.protocol = protocol
        self.src_addr = src_addr
        self.src_port = src_port
        self.dst_addr = dst_addr
        self.dst_port = dst_port

    def __str__(self):
        return "%d,%s,%d,%s,%d" % (self.protocol, self.src_addr, self.src_port, self.dst_addr, self.dst_port)

class UdpSegment:
    """Definition of a UDP segment
    """
    def __init__(self, five_tuple, header, payload):
        self.five_tuple = five_tuple
        self.udp_header = header
        self.udp_payload = payload

class TcpSegment:
    """Definition of a TCP segment
    """
    def __init__(self, five_tuple, header, payload):
        self.five_tuple = five_tuple
        self.tcp_header = header
        self.tcp_payload = payload
        self.tcp_sequence_number, self.tcp_acknowledgement_number = struct.unpack('!LL', header[4:12])

    def opt_isset_FIN(self):
        opts = ord(self.tcp_header[13]) & 0x3F
        return (opts & 0x01)

    def opt_isset_SYN(self):
        opts = ord(self.tcp_header[13]) & 0x3F
        return (opts & 0x02)

    def get_sequence_number(self):
        return self.tcp_sequence_number

    def __cmp__(self, other):
        return cmp(self.tcp_sequence_number, other.tcp_sequence_number)

class TcpStream:
    """Definition of a TCP stream.
    """
    TCP_STREAM_ACTIVE = 0x1
    TCP_STREAM_CLOSED = 0x02

    def __init__(self, five_tuple):
        self.five_tuple = five_tuple
        self.initial_sequence_number = 0
        self.segments = []

    def reset_stream(self):
        self.segments = []
        self.initial_sequence_number = 0

    def set_initial_sequence_number(self, sequence_number):
        self.initial_sequence_number = sequence_number

    def append_segment(self, tcp_segment):
        if len(self.segments) == 0:
             self.set_initial_sequence_number(tcp_segment.get_sequence_number())
        self.segments.append(tcp_segment)

    def get_segments_sorted(self):
        return sorted(self.segments)

class UdpStream:
    """A container for UDP packets that share the same 5-tuple
    """
    def __init__(self, five_tuple):
        self.five_tuple = five_tuple
        self.segments = []

    def append_segment(self, udp_segment):
        self.segments.append(udp_segment)


def newStream(five_tuple):
    '''
    Create a new stream using the arguments passed-in and return its ID.
    '''
    global cur_stream_id
    stream_id = cur_stream_id
    cur_stream_id += 1
    return stream_id

def process_tcp_segment(builder, segment):
    """Process a tcp segment. It checks for SYN and FIN segments are
    if set modifies the associated stream.
    """
    segment_id = str(segment.five_tuple)
    if segment_id in tcp_streams:
        m_tcp_stream = tcp_streams[segment_id]
        m_tcp_stream.append_segment(segment)
    else:
        m_tcp_stream = TcpStream(segment.five_tuple)
        m_tcp_stream.append_segment(segment)
        tcp_streams[segment_id] = m_tcp_stream


    if segment.opt_isset_SYN():
        m_tcp_stream.segments = []

    if segment.opt_isset_FIN():
        #
        # Finished with the stream - add the segments in the
        # stream to db allowing the stream to be reused.
        #
        db_add_tcp_stream_segments(builder, m_tcp_stream)
        del tcp_streams[segment_id]

def process_udp_segment(builder, segment):
    """ Process a UDP segment. Given the connectionless nature of the UDP
    protocol we simple accumulate the segment for later processing
    when all the packets have been read
    """
    segment_id = str(segment.five_tuple)
    if segment_id in udp_streams:
        m_udp_stream = udp_streams[segment_id]
        m_udp_stream.append_segment(segment)
    else:
        m_udp_stream = UdpStream(segment.five_tuple)
        m_udp_stream.append_segment(segment)
        udp_streams[segment_id] = m_udp_stream


def db_add_tcp_stream_segments(builder, tcp_stream):
    """Add the contents of a tcp stream to the database
    """
    tcp_segments = tcp_stream.get_segments_sorted()
    last_sequence_num = 0
    streamID = None

    for tcp_segment in tcp_segments:
        if (len(tcp_segment.tcp_payload) > 0) and (tcp_segment.tcp_sequence_number > last_sequence_num):
            #
            # Segment with an actual payload - add it to the stream's
            # list of chunks.
            #
            # Note: delay creating the stream until we have a via chunk to
            # commit to it
            #
            if streamID == None:
                streamID = newStream(tcp_stream.five_tuple)
            builder.add_chunk(streamID, tcp_segment.tcp_payload)
            last_sequence_num =  tcp_segment.tcp_sequence_number


def db_add_udp_stream_segments(builder, udp_stream):
    """Add the contents of a UDP stream to the database. Since UDP is
    connection-less, a UDP stream object is really just an accumulation
    of all the packets associated with a given 5-tuple.
    """
    udp_segments = udp_stream.segments
    streamID = None
    for udp_segment in udp_segments:
        if len(udp_segment.udp_payload) > 0:
            if streamID == None:
                streamID = newStream(udp_stream.five_tuple)
            builder.add_chunk(streamID, udp_segment.udp_payload)

def enchunk_pcap(pcapFN, sqliteFN):
    """Read the contents of a pcap file with name @pcapFN and produce
    a sqlite db with name @sqliteFN. It will contain chunks of data
    from TCP and UDP streams,
    """

    if not os.path.exists(pcapFN):
        print >> sys.stderr, "Input file '%s' does not exist. Exiting." % pcapFN
        sys.exit(-1)

    builder = CorpusBuilder(sqliteFN)

    #
    # Read in the contents of the pcap file, adding stream segments as found
    #
    pkt_cnt = 0
    ip_pkt_cnt = 0
    ip_pkt_off = 0
    unsupported_ip_protocol_cnt = 0
    pcap_ref = pcap.pcap(pcapFN)
    done = False

    while not done:
        try:
            ts, packet = pcap_ref.next()
        except:
            break

        pkt_cnt += 1

        linkLayerType = struct.unpack('!H', packet[(pcap_ref.dloff - 2):pcap_ref.dloff])[0]
        #
        # We're only interested in IP packets
        #
        if linkLayerType == ETHERTYPE_VLAN:
            linkLayerType = struct.unpack('!H', packet[(pcap_ref.dloff + 2):(pcap_ref.dloff + 4)])[0]
            if linkLayerType != ETHERTYPE_IP:
                continue
            else:
                ip_pkt_off = pcap_ref.dloff + 4
        elif linkLayerType == ETHERTYPE_IP:
            ip_pkt_off = pcap_ref.dloff
        else:
            continue

        ip_pkt_cnt += 1

        ip_pkt_total_len = struct.unpack('!H', packet[ip_pkt_off + 2: ip_pkt_off + 4])[0]
        ip_pkt = packet[ip_pkt_off:ip_pkt_off + ip_pkt_total_len]
        pkt_protocol = struct.unpack('B', ip_pkt[9])[0]

        if (pkt_protocol != IPPROTO_UDP) and (pkt_protocol != IPPROTO_TCP):
            #
            # we're only interested in UDP and TCP packets at the moment
            #
            continue

        pkt_src_addr = inet_ntoa(ip_pkt[12:16])
        pkt_dst_addr = inet_ntoa(ip_pkt[16:20])

        ip_hdr_len_offset = (ord(ip_pkt[0]) & 0x0f) * 4
        ip_payload = ip_pkt[ip_hdr_len_offset:len(ip_pkt)]

        pkt_src_port, pkt_dst_port = struct.unpack('!HH', ip_payload[0:4])
        five_tuple = FiveTuple(pkt_protocol, pkt_src_addr, pkt_src_port, pkt_dst_addr, pkt_dst_port)
        five_tuple_id = str(five_tuple)

        if pkt_protocol == IPPROTO_UDP:
            udp_payload_len = struct.unpack('!H', ip_payload[4:6])[0] - 8
            udp_header = ip_payload[0:8]
            udp_payload = ip_payload[8:len(ip_payload)]
            udp_segment = UdpSegment(five_tuple, udp_header, udp_payload)
            process_udp_segment(builder, udp_segment)
        elif pkt_protocol == IPPROTO_TCP:
            tcp_hdr_len = (ord(ip_payload[12]) >> 4) * 4
            tcp_header = ip_payload[0:tcp_hdr_len]
            tcp_payload = ip_payload[tcp_hdr_len:len(ip_payload)]
            segment = TcpSegment(five_tuple, tcp_header, tcp_payload)
            process_tcp_segment(builder, segment)

    #
    # Having read the contents of the pcap, we fill the database with any
    # remaining TCP and UDP segments
    #
    for tcp_stream in tcp_streams.itervalues():
        db_add_tcp_stream_segments(builder, tcp_stream)

    for udp_stream in udp_streams.itervalues():
        db_add_udp_stream_segments(builder, udp_stream)

    #
    # We've finished with the database
    #
    builder.finish()

if __name__ == '__main__' :

    args = getopt.getopt(sys.argv[1:], 'i:o:')
    args = dict(args[0])

    requiredKeys = [ '-i', '-o']
    for k in requiredKeys :
        if not args.has_key(k) :
            usage(os.path.basename(sys.argv[0]))

    fnArgs = tuple([ args[k] for k in requiredKeys ])
    enchunk_pcap(*fnArgs)
