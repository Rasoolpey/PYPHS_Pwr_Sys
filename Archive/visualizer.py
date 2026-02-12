import numpy as np
import pandas as pd

class PHSVisualizer:
    def __init__(self, builder):
        self.data = builder.system_data
        # Build bus idx to name mapping
        self.bus_idx_to_name = {}
        self.bus_name_to_idx = {}
        for bus in self.data.get('Bus', []):
            self.bus_idx_to_name[bus['idx']] = str(bus['name'])
            self.bus_name_to_idx[str(bus['name'])] = bus['idx']

    def extract_and_calculate_everything(self):
        print("\n" + "!"*30 + " FINAL VERIFIED DATA EXTRACTION " + "!"*30)

        # 1. BUS DATA
        print("\n[PART A: BUS/NODE DATA]")
        print(pd.DataFrame(self.data.get('Bus', [])).to_string(index=False))

        # 2. GENERATOR DATA
        print("\n[PART B: GENERATOR DATA (GENROU)]")
        print(pd.DataFrame(self.data.get('GENROU', [])).to_string(index=False))

        # 3. EXCITER DATA
        print("\n[PART C: EXCITER DATA (EXDC2 / AVR)]")
        print(pd.DataFrame(self.data.get('EXDC2', [])).to_string(index=False))

        # 4. GOVERNOR DATA
        print("\n[PART D: GOVERNOR DATA (TGOV1)]")
        print(pd.DataFrame(self.data.get('TGOV1', [])).to_string(index=False))

        # 5. TRANSFORMERS (Isolated)
        print("\n[PART E: TRANSFORMERS (Isolated)]")
        lines = self.data.get('Line', [])
        transformers = [l for l in lines if l['Vn1'] != l['Vn2']]
        if transformers:
            df_xfmr = pd.DataFrame(transformers)
            df_xfmr['Ratio'] = df_xfmr['Vn2'] / df_xfmr['Vn1']
            print(df_xfmr[['idx', 'bus1', 'bus2', 'Vn1', 'Vn2', 'Ratio', 'x']].to_string(index=False))

        # 6. TRANSMISSION LINES
        print("\n[PART F: TRANSMISSION LINES]")
        tx_lines = [l for l in lines if l['Vn1'] == l['Vn2']]
        print(pd.DataFrame(tx_lines)[['idx', 'bus1', 'bus2', 'r', 'x', 'b']].to_string(index=False))

        # 7. LOADS
        print("\n[PART G: LOADS (PQ)]")
        pq_loads = self.data.get('PQ', [])
        if pq_loads:
            print(pd.DataFrame(pq_loads)[['idx', 'bus', 'p0', 'q0']].to_string(index=False))

        # 8. Y-MATRIX / B-MATRIX
        nodes = sorted(list(set([str(l['bus1']) for l in lines] + [str(l['bus2']) for l in lines])), key=lambda x: int(x))
        n = len(nodes)
        idx = {name: i for i, name in enumerate(nodes)}
        Y = np.zeros((n, n), dtype=complex)
        for l in lines:
            b1, b2 = str(l['bus1']), str(l['bus2'])
            y_series = 1 / complex(l['r'], l['x'])
            y_shunt = complex(0, l['b'] / 2)
            i, j = idx[b1], idx[b2]
            Y[i, j] -= y_series; Y[j, i] -= y_series
            Y[i, i] += y_series + y_shunt; Y[j, j] += y_series + y_shunt
        print(f"\n[PART H: NUMERICAL B-MATRIX - {n}x{n}]")
        print(pd.DataFrame(np.imag(Y), index=nodes, columns=nodes).round(2))

    def build_phs_structure(self):
        """
        Build PHS interconnection structure with all component types.
        """
        print("\n" + "="*30 + " PHS INTERCONNECTION MAPPING " + "="*30)

        structure = {
            "storage_ports": [],     # Generators
            "exciter_ports": [],     # Exciters (AVR)
            "governor_ports": [],    # Governors
            "scaling_ports": [],     # Transformers
            "coupling_ports": [],    # Transmission Lines
            "load_ports": []         # Loads
        }

        # Build generator idx to bus mapping for exciter/governor lookup
        gen_to_bus = {}
        for g in self.data.get('GENROU', []):
            gen_to_bus[g['idx']] = g['bus']

        # Handle Generators
        gens = self.data.get('GENROU', [])
        for g in gens:
            gid = g['idx']
            structure["storage_ports"].append({
                "id": f"G{gid}",
                "gen_idx": gid,
                "at_bus": g['bus'],
                "M": g['M'],
                "xd": g['xd'],
                "xq": g['xq'],
                "Sn": g['Sn'],
                "type": "Synchronous Generator"
            })
            print(f"Logic: Generator G{gid} at Bus {g['bus']} (name={self.bus_idx_to_name.get(g['bus'], '?')}), M={g['M']}, Sn={g['Sn']}MVA")

        # Handle Exciters
        exciters = self.data.get('EXDC2', [])
        for e in exciters:
            syn_id = e['syn']
            bus_id = gen_to_bus.get(syn_id, None)
            structure["exciter_ports"].append({
                "id": f"AVR{e['idx']}",
                "syn": syn_id,
                "at_bus": bus_id,
                "KA": e['KA'],
                "TE": e['TE'],
                "type": "Exciter (EXDC2)"
            })
            print(f"Logic: Exciter AVR{e['idx']} -> Generator G{syn_id}, KA={e['KA']}")

        # Handle Governors
        governors = self.data.get('TGOV1', [])
        for gov in governors:
            syn_id = gov['syn']
            bus_id = gen_to_bus.get(syn_id, None)
            structure["governor_ports"].append({
                "id": f"GOV{gov['idx']}",
                "syn": syn_id,
                "at_bus": bus_id,
                "R": gov['R'],
                "T1": gov['T1'],
                "T3": gov['T3'],
                "type": "Governor (TGOV1)"
            })
            print(f"Logic: Governor GOV{gov['idx']} -> Generator G{syn_id}, R={gov['R']}")

        # Handle Transformers (Vn1 != Vn2)
        for l in self.data.get('Line', []):
            if l['Vn1'] != l['Vn2']:
                ratio = l['Vn2'] / l['Vn1']
                structure["scaling_ports"].append({
                    "id": l['idx'],
                    "buses": (l['bus1'], l['bus2']),
                    "ratio": ratio,
                    "x": l['x'],
                    "Vn1": l['Vn1'],
                    "Vn2": l['Vn2']
                })
                b1_name = self.bus_idx_to_name.get(l['bus1'], str(l['bus1']))
                b2_name = self.bus_idx_to_name.get(l['bus2'], str(l['bus2']))
                print(f"Logic: Transformer {l['idx']}: Bus {l['bus1']}({b1_name}) -> Bus {l['bus2']}({b2_name}), ratio={ratio:.2f}, X={l['x']}")

        # Handle Transmission Lines (Vn1 == Vn2)
        for l in self.data.get('Line', []):
            if l['Vn1'] == l['Vn2']:
                structure["coupling_ports"].append({
                    "id": l['idx'],
                    "buses": (l['bus1'], l['bus2']),
                    "r": l['r'],
                    "x": l['x'],
                    "b": l['b']
                })
                b1_name = self.bus_idx_to_name.get(l['bus1'], str(l['bus1']))
                b2_name = self.bus_idx_to_name.get(l['bus2'], str(l['bus2']))
                print(f"Logic: Line {l['idx']}: Bus {l['bus1']}({b1_name}) <-> Bus {l['bus2']}({b2_name}), X={l['x']}, R={l['r']}")

        # Handle Loads
        for pq in self.data.get('PQ', []):
            structure["load_ports"].append({
                "id": pq['idx'],
                "at_bus": pq['bus'],
                "p0": pq['p0'],
                "q0": pq['q0']
            })
            b_name = self.bus_idx_to_name.get(pq['bus'], str(pq['bus']))
            print(f"Logic: Load {pq['idx']} at Bus {pq['bus']}({b_name}), P={pq['p0']}pu, Q={pq['q0']}pu")

        return structure

    def draw(self, filename="kundur_system_figure.svg"):
        # 1. Standard Extraction
        self.extract_and_calculate_everything()

        # 2. Logical Routing
        phs_map = self.build_phs_structure()

        # 3. Define coordinates based on bus idx (not name)
        # Kundur Two-Area System Layout:
        # Area 1 (Left): Buses 1,2 (Gen), 5,6,7 (HV)
        # Area 2 (Right): Buses 3,4 (Gen), 8,9,10 (HV)
        coords = {
            # Generator buses (20kV) - Left side is Area 1, Right side is Area 2
            1: (150, 120),    # Bus 1 (name=1) - G1
            2: (150, 480),    # Bus 2 (name=2) - G2, G3
            3: (1350, 480),   # Bus 3 (name=12) - Area 2
            4: (1350, 120),   # Bus 4 (name=11) - G4
            # HV Transmission buses (230kV)
            5: (350, 120),    # Bus 5 (name=101)
            6: (350, 480),    # Bus 6 (name=102)
            7: (550, 300),    # Bus 7 (name=3) - tie point Area 1
            8: (950, 300),    # Bus 8 (name=13) - tie point Area 2
            9: (1150, 480),   # Bus 9 (name=112)
            10: (1150, 120),  # Bus 10 (name=111)
        }

        svg_width = 1550
        svg_height = 650
        svg = [f'<svg width="{svg_width}" height="{svg_height}" xmlns="http://www.w3.org/2000/svg">']
        svg.append('<defs>')
        # Arrow marker for flow direction
        svg.append('<marker id="arrowhead" markerWidth="10" markerHeight="7" refX="9" refY="3.5" orient="auto">')
        svg.append('<polygon points="0 0, 10 3.5, 0 7" fill="#666"/>')
        svg.append('</marker>')
        svg.append('</defs>')

        # Background
        svg.append('<rect width="100%" height="100%" fill="#fafafa"/>')

        # Title and Legend
        svg.append('<text x="50" y="35" font-family="Arial" font-size="22" font-weight="bold" fill="#1a237e">Kundur Two-Area Power System - PHS Topology</text>')

        # Area labels
        svg.append('<rect x="100" y="55" width="400" height="30" fill="#e3f2fd" stroke="#1976d2" rx="5"/>')
        svg.append('<text x="300" y="75" font-family="Arial" font-size="14" font-weight="bold" fill="#1976d2" text-anchor="middle">AREA 1</text>')
        svg.append('<rect x="1000" y="55" width="400" height="30" fill="#fce4ec" stroke="#c2185b" rx="5"/>')
        svg.append('<text x="1200" y="75" font-family="Arial" font-size="14" font-weight="bold" fill="#c2185b" text-anchor="middle">AREA 2</text>')

        # Legend
        legend_y = 580
        svg.append(f'<rect x="50" y="{legend_y}" width="1450" height="55" fill="#f5f5f5" stroke="#bdbdbd" rx="5"/>')
        svg.append(f'<text x="70" y="{legend_y+20}" font-family="Arial" font-size="12" font-weight="bold">Legend:</text>')
        # Generator
        svg.append(f'<circle cx="170" cy="{legend_y+25}" r="12" fill="#fff" stroke="#1976d2" stroke-width="2"/>')
        svg.append(f'<text x="165" y="{legend_y+30}" font-family="Arial" font-size="10" font-weight="bold" fill="#1976d2">G</text>')
        svg.append(f'<text x="190" y="{legend_y+30}" font-family="Arial" font-size="11">Generator</text>')
        # Exciter
        svg.append(f'<rect x="270" y="{legend_y+15}" width="20" height="20" fill="#fff3e0" stroke="#ff6f00" stroke-width="2" rx="3"/>')
        svg.append(f'<text x="275" y="{legend_y+30}" font-family="Arial" font-size="8" font-weight="bold" fill="#ff6f00">E</text>')
        svg.append(f'<text x="300" y="{legend_y+30}" font-family="Arial" font-size="11">Exciter</text>')
        # Governor
        svg.append(f'<rect x="380" y="{legend_y+15}" width="20" height="20" fill="#e8f5e9" stroke="#2e7d32" stroke-width="2" rx="3"/>')
        svg.append(f'<text x="385" y="{legend_y+30}" font-family="Arial" font-size="8" font-weight="bold" fill="#2e7d32">G</text>')
        svg.append(f'<text x="410" y="{legend_y+30}" font-family="Arial" font-size="11">Governor</text>')
        # Transformer
        svg.append(f'<circle cx="510" cy="{legend_y+20}" r="8" fill="none" stroke="#7b1fa2" stroke-width="2"/>')
        svg.append(f'<circle cx="520" cy="{legend_y+30}" r="8" fill="none" stroke="#7b1fa2" stroke-width="2"/>')
        svg.append(f'<text x="540" y="{legend_y+30}" font-family="Arial" font-size="11">Transformer</text>')
        # Line
        svg.append(f'<line x1="630" y1="{legend_y+25}" x2="680" y2="{legend_y+25}" stroke="#424242" stroke-width="3"/>')
        svg.append(f'<text x="690" y="{legend_y+30}" font-family="Arial" font-size="11">Transmission Line</text>')
        # Bus
        svg.append(f'<rect x="810" y="{legend_y+12}" width="8" height="26" fill="#212121" rx="2"/>')
        svg.append(f'<text x="830" y="{legend_y+30}" font-family="Arial" font-size="11">Bus</text>')
        # Load
        svg.append(f'<polygon points="900,{legend_y+35} 915,{legend_y+15} 930,{legend_y+35}" fill="#fff" stroke="#d32f2f" stroke-width="2"/>')
        svg.append(f'<text x="940" y="{legend_y+30}" font-family="Arial" font-size="11">Load</text>')

        # Group parallel lines by (bus1, bus2) pair
        line_groups = {}
        for cp in phs_map["coupling_ports"]:
            key = tuple(sorted(cp["buses"]))
            if key not in line_groups:
                line_groups[key] = []
            line_groups[key].append(cp)

        # DRAW TRANSMISSION LINES (with parallel line offset)
        for key, lines in line_groups.items():
            b1, b2 = key
            p1, p2 = coords.get(b1), coords.get(b2)
            if p1 and p2:
                num_lines = len(lines)
                # Calculate perpendicular offset for parallel lines
                dx = p2[0] - p1[0]
                dy = p2[1] - p1[1]
                length = np.sqrt(dx**2 + dy**2)
                if length > 0:
                    # Perpendicular unit vector
                    perp_x = -dy / length
                    perp_y = dx / length
                else:
                    perp_x, perp_y = 0, 1

                # Offset spacing
                spacing = 8
                start_offset = -spacing * (num_lines - 1) / 2

                for i, line in enumerate(lines):
                    offset = start_offset + i * spacing
                    x1 = p1[0] + perp_x * offset
                    y1 = p1[1] + perp_y * offset
                    x2 = p2[0] + perp_x * offset
                    y2 = p2[1] + perp_y * offset

                    svg.append(f'<line x1="{x1}" y1="{y1}" x2="{x2}" y2="{y2}" stroke="#424242" stroke-width="2"/>')

                # Add single impedance label at midpoint
                mid_x = (p1[0] + p2[0]) / 2
                mid_y = (p1[1] + p2[1]) / 2

                # Aggregate impedance info
                if num_lines == 1:
                    x_val = lines[0]['x']
                    label = f"X={x_val:.3f}"
                else:
                    x_vals = [l['x'] for l in lines]
                    x_eq = 1 / sum(1/x for x in x_vals)  # Parallel equivalent
                    label = f"{num_lines}x parallel, X_eq={x_eq:.4f}"

                # Position label with background
                label_offset = 15 if dy != 0 else -15
                svg.append(f'<rect x="{mid_x-45}" y="{mid_y+label_offset-10}" width="90" height="14" fill="#fff" fill-opacity="0.9" rx="2"/>')
                svg.append(f'<text x="{mid_x}" y="{mid_y+label_offset}" font-family="Arial" font-size="9" fill="#616161" text-anchor="middle">{label}</text>')

        # DRAW TRANSFORMERS
        for sp in phs_map["scaling_ports"]:
            b1, b2 = sp["buses"]
            p1, p2 = coords.get(b1), coords.get(b2)
            if p1 and p2:
                # Draw transformer symbol (two coupled circles)
                mid_x = (p1[0] + p2[0]) / 2
                mid_y = (p1[1] + p2[1]) / 2

                # Direction vector
                dx = p2[0] - p1[0]
                dy = p2[1] - p1[1]
                length = np.sqrt(dx**2 + dy**2)
                ux, uy = dx/length, dy/length

                # Line from bus1 to first circle
                svg.append(f'<line x1="{p1[0]}" y1="{p1[1]}" x2="{mid_x-ux*18}" y2="{mid_y-uy*18}" stroke="#7b1fa2" stroke-width="3"/>')
                # Line from second circle to bus2
                svg.append(f'<line x1="{mid_x+ux*18}" y1="{mid_y+uy*18}" x2="{p2[0]}" y2="{p2[1]}" stroke="#7b1fa2" stroke-width="3"/>')

                # Two overlapping circles (transformer symbol)
                svg.append(f'<circle cx="{mid_x-ux*10}" cy="{mid_y-uy*10}" r="12" fill="#f3e5f5" stroke="#7b1fa2" stroke-width="2"/>')
                svg.append(f'<circle cx="{mid_x+ux*10}" cy="{mid_y+uy*10}" r="12" fill="#f3e5f5" stroke="#7b1fa2" stroke-width="2"/>')

                # Label
                ratio = sp['ratio']
                x_val = sp['x']
                label_y_offset = 30 if mid_y < 300 else -25
                svg.append(f'<text x="{mid_x}" y="{mid_y+label_y_offset}" font-family="Arial" font-size="9" fill="#7b1fa2" text-anchor="middle">{sp["id"]}</text>')
                svg.append(f'<text x="{mid_x}" y="{mid_y+label_y_offset+11}" font-family="Arial" font-size="8" fill="#9c27b0" text-anchor="middle">X={x_val}, n={ratio:.1f}</text>')

        # DRAW GENERATORS with Exciter and Governor
        gen_positions = {}  # Track generator positions for exciter/governor placement
        for i, gp in enumerate(phs_map["storage_ports"]):
            bus_id = gp["at_bus"]
            p = coords.get(bus_id)
            if p:
                # Check if multiple generators at same bus
                same_bus_gens = [g for g in phs_map["storage_ports"] if g["at_bus"] == bus_id]
                gen_index = same_bus_gens.index(gp)

                # Position generator to left (Area 1) or right (Area 2) of bus
                if bus_id <= 2:  # Area 1
                    gx = p[0] - 80
                    vert_offset = (gen_index - len(same_bus_gens)/2 + 0.5) * 50
                else:  # Area 2
                    gx = p[0] + 80
                    vert_offset = (gen_index - len(same_bus_gens)/2 + 0.5) * 50

                gy = p[1] + vert_offset
                gen_positions[gp["gen_idx"]] = (gx, gy)

                # Draw connection line to bus
                svg.append(f'<line x1="{p[0]}" y1="{p[1]}" x2="{gx}" y2="{gy}" stroke="#1976d2" stroke-width="2"/>')

                # Draw generator circle
                svg.append(f'<circle cx="{gx}" cy="{gy}" r="22" fill="#e3f2fd" stroke="#1976d2" stroke-width="3"/>')
                svg.append(f'<text x="{gx}" y="{gy+5}" font-family="Arial" font-size="14" font-weight="bold" fill="#1976d2" text-anchor="middle">{gp["id"]}</text>')

                # Generator label
                label_x = gx - 35 if bus_id <= 2 else gx + 35
                svg.append(f'<text x="{label_x}" y="{gy-15}" font-family="Arial" font-size="9" fill="#1565c0" text-anchor="middle">M={gp["M"]}</text>')
                svg.append(f'<text x="{label_x}" y="{gy-3}" font-family="Arial" font-size="9" fill="#1565c0" text-anchor="middle">{gp["Sn"]:.0f}MVA</text>')

        # DRAW EXCITERS
        for ep in phs_map["exciter_ports"]:
            gen_idx = ep["syn"]
            if gen_idx in gen_positions:
                gx, gy = gen_positions[gen_idx]
                # Position exciter above generator
                ex, ey = gx, gy - 50

                # Connection line
                svg.append(f'<line x1="{gx}" y1="{gy-22}" x2="{ex}" y2="{ey+12}" stroke="#ff6f00" stroke-width="1.5" stroke-dasharray="3,2"/>')

                # Exciter box
                svg.append(f'<rect x="{ex-15}" y="{ey-12}" width="30" height="24" fill="#fff3e0" stroke="#ff6f00" stroke-width="2" rx="4"/>')
                svg.append(f'<text x="{ex}" y="{ey+4}" font-family="Arial" font-size="9" font-weight="bold" fill="#e65100" text-anchor="middle">AVR</text>')

        # DRAW GOVERNORS
        for gp in phs_map["governor_ports"]:
            gen_idx = gp["syn"]
            if gen_idx in gen_positions:
                gx, gy = gen_positions[gen_idx]
                # Position governor below generator
                govx, govy = gx, gy + 50

                # Connection line
                svg.append(f'<line x1="{gx}" y1="{gy+22}" x2="{govx}" y2="{govy-12}" stroke="#2e7d32" stroke-width="1.5" stroke-dasharray="3,2"/>')

                # Governor box
                svg.append(f'<rect x="{govx-15}" y="{govy-12}" width="30" height="24" fill="#e8f5e9" stroke="#2e7d32" stroke-width="2" rx="4"/>')
                svg.append(f'<text x="{govx}" y="{govy+4}" font-family="Arial" font-size="9" font-weight="bold" fill="#1b5e20" text-anchor="middle">GOV</text>')

        # DRAW LOADS
        for lp in phs_map["load_ports"]:
            bus_id = lp["at_bus"]
            p = coords.get(bus_id)
            if p:
                # Position load below bus
                lx, ly = p[0], p[1] + 50

                # Connection line
                svg.append(f'<line x1="{p[0]}" y1="{p[1]}" x2="{lx}" y2="{ly-15}" stroke="#d32f2f" stroke-width="2"/>')

                # Load triangle (arrow pointing down)
                svg.append(f'<polygon points="{lx},{ly+15} {lx-15},{ly-10} {lx+15},{ly-10}" fill="#ffebee" stroke="#d32f2f" stroke-width="2"/>')

                # Load label
                svg.append(f'<text x="{lx}" y="{ly+30}" font-family="Arial" font-size="9" fill="#c62828" text-anchor="middle">P={lp["p0"]}pu</text>')
                svg.append(f'<text x="{lx}" y="{ly+41}" font-family="Arial" font-size="9" fill="#c62828" text-anchor="middle">Q={lp["q0"]}pu</text>')

        # DRAW BUS BARS (after everything else for proper layering)
        for bus in self.data.get('Bus', []):
            bus_idx = bus['idx']
            pt = coords.get(bus_idx)
            if pt:
                bus_name = str(bus['name'])
                vn = bus['Vn']

                # Bus bar color based on voltage level
                if vn < 50:
                    bar_color = "#1565c0"  # LV - blue
                else:
                    bar_color = "#212121"  # HV - black

                # Draw bus bar
                svg.append(f'<rect x="{pt[0]-5}" y="{pt[1]-30}" width="10" height="60" fill="{bar_color}" rx="2"/>')

                # Bus label
                svg.append(f'<text x="{pt[0]}" y="{pt[1]-40}" font-family="Arial" font-size="12" font-weight="bold" text-anchor="middle">{bus_name}</text>')
                svg.append(f'<text x="{pt[0]}" y="{pt[1]+50}" font-family="Arial" font-size="10" fill="#757575" text-anchor="middle">{vn:.0f}kV</text>')

        svg.append('</svg>')

        with open(filename, 'w') as f:
            f.write("\n".join(svg))
        print(f"\n[COMPLETE] Comprehensive diagram generated: {filename}")
