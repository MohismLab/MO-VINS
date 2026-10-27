#!/usr/bin/env python3
"""Connectivity gate for the proposed sync revision, not an electrical/timing signoff."""
import argparse
import json
from pathlib import Path
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--netlist',type=Path,default=Path('hardware/netlist.json'))
args=parser.parse_args()
expected=json.loads(Path('hardware/sync-required-changes.json').read_text())
actual={}
for component in json.loads(args.netlist.read_text())['components'].values():
    name=component['props'].get('Designator','')
    for pin in component.get('pinInfoMap',{}).values():
        actual[f"{name}.{pin['number']}"]=pin.get('net','')
failures=[]
for net,pins in {**expected['required_nets'],**expected['existing_preserved']}.items():
    for pin in pins:
        if actual.get(pin)!=net:
            failures.append(f'{pin}: expected {net}, got {actual.get(pin, "MISSING") or "UNCONNECTED"}')
for failure in failures:print('FAIL',failure)
if failures:
    print('NOT READY: proposed common-event wiring has not passed connectivity review')
    raise SystemExit(1)
print('PASS required pin/net assignments only; translator, DRC, timing and firmware qualification still required')
