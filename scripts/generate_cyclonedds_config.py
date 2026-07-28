#!/usr/bin/env python3
"""Generate a Cyclone DDS config without hard-coded host or interface names."""

import argparse
import ipaddress
import re
from pathlib import Path
import xml.etree.ElementTree as ET


CYCLONE_NAMESPACE = 'https://cdds.io/config'
TAG = f'{{{CYCLONE_NAMESPACE}}}'
HOSTNAME_PATTERN = re.compile(r'^[A-Za-z0-9](?:[A-Za-z0-9_.:-]*[A-Za-z0-9])?$')
INTERFACE_PATTERN = re.compile(r'^[A-Za-z0-9_.:-]+$')


def validate_peer(value):
    peer = value.strip()
    if not peer:
        raise argparse.ArgumentTypeError('peer addresses must not be empty')
    try:
        ipaddress.ip_address(peer.strip('[]'))
        return peer
    except ValueError:
        pass
    if not HOSTNAME_PATTERN.fullmatch(peer):
        raise argparse.ArgumentTypeError(
            f'invalid peer address or hostname: {value!r}'
        )
    return peer


def parse_peers(value):
    if not value.strip():
        return []
    return [validate_peer(item) for item in value.split(',')]


def build_config(base_path, peers, interface, allow_multicast):
    ET.register_namespace('', CYCLONE_NAMESPACE)
    tree = ET.parse(base_path)
    root = tree.getroot()

    general = root.find(f'.//{TAG}General')
    discovery = root.find(f'.//{TAG}Discovery')
    network_interface = root.find(f'.//{TAG}NetworkInterface')
    multicast = root.find(f'.//{TAG}AllowMulticast')
    if None in (general, discovery, network_interface, multicast):
        raise ValueError(f'{base_path} is missing required Cyclone DDS elements')

    multicast.text = allow_multicast
    if interface:
        if not INTERFACE_PATTERN.fullmatch(interface):
            raise ValueError(f'invalid interface name: {interface!r}')
        network_interface.attrib.pop('autodetermine', None)
        network_interface.set('name', interface)
    else:
        network_interface.attrib.pop('name', None)
        network_interface.set('autodetermine', 'true')

    existing_peers = discovery.find(f'{TAG}Peers')
    if existing_peers is not None:
        discovery.remove(existing_peers)
    if peers:
        peers_element = ET.Element(f'{TAG}Peers')
        for peer in peers:
            ET.SubElement(
                peers_element,
                f'{TAG}Peer',
                {'Address': peer},
            )
        discovery.insert(0, peers_element)

    ET.indent(tree, space='  ')
    return tree


def main():
    parser = argparse.ArgumentParser(
        description=(
            'Render a Cyclone DDS XML file with optional comma-separated peers. '
            'This command only writes the requested output file.'
        )
    )
    parser.add_argument('--base', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument(
        '--peers',
        default='',
        help='Comma-separated IP addresses or DNS/mDNS names.',
    )
    parser.add_argument(
        '--interface',
        default='',
        help='Optional network interface such as wlan0; autodetect when omitted.',
    )
    parser.add_argument(
        '--allow-multicast',
        choices=('true', 'false', 'spdp'),
        default='spdp',
    )
    args = parser.parse_args()

    peers = parse_peers(args.peers)
    tree = build_config(
        args.base,
        peers,
        args.interface.strip(),
        args.allow_multicast,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    tree.write(args.output, encoding='utf-8', xml_declaration=True)
    print(
        f'Wrote {args.output} with {len(peers)} explicit peer(s), '
        f'interface={args.interface or "autodetect"}, '
        f'allow_multicast={args.allow_multicast}.'
    )


if __name__ == '__main__':
    main()
