#  __________
# |__  /  ___|
#   / /| |_
#  / /_|  _|
# /____|_| Friedrichshafen AG
#
# Copyright [2024] ZF Friedrichshafen AG
"""Script to replay UDP packages from a .pcap file. This is for debugging purposes only, as correct timing cannot be guaranteed."""

import os
import time
import socket
import argparse
from scapy.all import PcapReader
from typing import Union


def read_udp_packages_from_pcap(pcap_file: str, filter_pairs: [tuple] = None) -> (bytes, float, str, int, str, int):
    """
    generator function to read UDP packages from a .pcap file from specified IP and port

    :param pcap_file: pcap file path
    :param filter_pairs: list of tuples containing (src_ip, src_port, dst_ip, dst_port) pairs.
    :return: returns tuple with payload, stamp, src_ip, src_port, dst_ip, dst_port
    """
    for p in PcapReader(pcap_file):
        if p.payload.name == "IP" and p.payload.payload.name == "UDP":
            stamp = float(p.time)
            payload = bytes(p.payload.payload.payload.original)
            if filter_pairs is None:
                yield payload, stamp, p.payload.src, p.payload.payload.sport, p.payload.dst, p.payload.payload.dport
            else:
                for src_ip, src_port, dst_ip, dst_port in filter_pairs:
                    if ((src_ip is None or src_ip == p.payload.src) and
                        (src_port is None or src_port == p.payload.payload.sport) and
                        (dst_ip is None or dst_ip == p.payload.dst) and
                            (dst_port is None or dst_port == p.payload.payload.dport)):
                        yield payload, stamp, p.payload.src, p.payload.payload.sport, p.payload.dst, p.payload.payload.dport


def pcap_to_udp(pcap_file: str, filter_pair: tuple, dst_ip_mapping: Union[str, dict] = None, no_sleep=False):
    """
    read UDP packages from a .pcap file with specified source IP and Port and resend to specified IP and port
, 
    :param pcap_file: pcap file path
    :param filter_pair: (src_ip, src_port, dst_ip, dst_port) tuple
    :param source_ip: source IP address for sending UPD packages
    :param dst_ip_mapping: str or dict to map dst_ip to different ip for sending UPD packages
    """
    sock = socket.socket(socket.AF_INET,
                         socket.SOCK_DGRAM)

    start_stamp = -1
    start_time = time.time()

    for payload, stamp, src_ip, src_port, dst_ip, dst_port in read_udp_packages_from_pcap(pcap_file, [filter_pair]):

        if start_stamp == -1:
            start_stamp = stamp
        else:
            timediff = (stamp - start_stamp) - (time.time() - start_time)
            if not no_sleep and timediff > 0:
                time.sleep(timediff)

        if dst_ip_mapping is not None:
            if isinstance(dst_ip_mapping, str):
                target_ip = dst_ip_mapping
            else:
                target_ip = dst_ip_mapping[dst_ip]
        else:
            target_ip = dst_ip

        sock.sendto(payload, (target_ip, dst_port))


if __name__ == '__main__':
    parser = argparse.ArgumentParser("pcap_to_udp")
    parser.add_argument("pcap_file", help="pcap filepath", type=str)
    parser.add_argument('-sip', help="filter source IP address of UDP messages",
                        type=str, default=None)
    parser.add_argument('-dip', help="filter destination IP address of UDP messages",
                        type=str, default=None)
    parser.add_argument('-sport', help="filter source port of UDP messages",
                        type=int, default=None)
    parser.add_argument('-dport', help="filter destination port of UDP messages",
                        type=int, default=None)
    parser.add_argument('-oip', help="override destination IP to which UDP packages should be sent",
                        type=str, default="127.0.0.1")
    parser.add_argument('-ns', help="no sleep between packages, send out as fast as possible",
                        action='store_true')
    parser.set_defaults(ns=False)
    args = parser.parse_args()

    pcap_path = args.pcap_file
    src_ip = args.sip
    dst_ip = args.dip
    src_port = args.sport
    dst_port = args.dport
    dst_ip_overwrite = args.oip
    no_sleep = args.ns

    if not os.path.exists(pcap_path):
        print("pcap file not found")
        raise SystemExit(1)

    while True:
        print(f"playing {os.path.basename(pcap_path)}")
        pcap_to_udp(pcap_path, [src_ip, src_port,
                    dst_ip, dst_port], dst_ip_overwrite, no_sleep)
