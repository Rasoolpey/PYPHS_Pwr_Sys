"""
Auto-Layout Power System Visualizer

This module automatically positions power system components based on their
electrical distance (impedance) using graph layout algorithms. No hardcoded
coordinates are required - the layout is computed from the network topology.

Key Features:
- Impedance-weighted graph layout using force-directed algorithms
- Automatic separation by voltage levels
- Area-aware positioning when area data is available
- Supports any power system topology
"""

import numpy as np
import pandas as pd
from collections import defaultdict


class AutoPHSVisualizer:
    def __init__(self, builder):
        self.data = builder.system_data
        # Build bus idx to name mapping
        self.bus_idx_to_name = {}
        self.bus_name_to_idx = {}
        self.bus_info = {}
        for bus in self.data.get('Bus', []):
            self.bus_idx_to_name[bus['idx']] = str(bus['name'])
            self.bus_name_to_idx[str(bus['name'])] = bus['idx']
            self.bus_info[bus['idx']] = bus

    def compute_electrical_distance_matrix(self):
        """
        Compute electrical distance matrix based on line impedances.
        Distance = |Z| = sqrt(r² + x²) for direct connections, infinity otherwise.
        """
        buses = list(self.bus_info.keys())
        n = len(buses)
        bus_to_idx = {b: i for i, b in enumerate(buses)}

        # Initialize with infinity (no direct connection)
        dist_matrix = np.full((n, n), np.inf)
        np.fill_diagonal(dist_matrix, 0)

        # Fill in direct connections from lines
        for line in self.data.get('Line', []):
            b1, b2 = line['bus1'], line['bus2']
            if b1 in bus_to_idx and b2 in bus_to_idx:
                i, j = bus_to_idx[b1], bus_to_idx[b2]
                # Electrical distance = impedance magnitude
                z_mag = np.sqrt(line['r']**2 + line['x']**2)
                # For parallel lines, take minimum (they reduce effective impedance)
                dist_matrix[i, j] = min(dist_matrix[i, j], z_mag)
                dist_matrix[j, i] = dist_matrix[i, j]

        # Floyd-Warshall for shortest electrical path
        for k in range(n):
            for i in range(n):
                for j in range(n):
                    if dist_matrix[i, k] + dist_matrix[k, j] < dist_matrix[i, j]:
                        dist_matrix[i, j] = dist_matrix[i, k] + dist_matrix[k, j]

        return buses, dist_matrix

    def compute_layout_force_directed(self, scale=800, iterations=500):
        """
        Compute node positions using force-directed layout based on electrical distance.
        Uses a spring-electrical model where:
        - Springs pull connected nodes together (proportional to impedance)
        - Electrical repulsion pushes all nodes apart
        """
        buses, dist_matrix = self.compute_electrical_distance_matrix()
        n = len(buses)

        if n == 0:
            return {}

        # Initialize positions randomly
        np.random.seed(42)  # For reproducibility
        positions = np.random.rand(n, 2) * scale

        # Normalize distances for layout (avoid zero distances)
        dist_norm = dist_matrix.copy()
        dist_norm[dist_norm == np.inf] = np.max(dist_norm[dist_norm != np.inf]) * 2
        dist_norm[dist_norm == 0] = 0.01

        # Target distances for layout (scale impedances to reasonable pixel distances)
        max_dist = np.max(dist_norm[dist_norm != np.inf])
        target_dist = (dist_norm / max_dist) * (scale * 0.4) + 50

        # Force-directed iterations
        cooling = 1.0
        for iteration in range(iterations):
            forces = np.zeros((n, 2))

            for i in range(n):
                for j in range(i + 1, n):
                    # Vector from i to j
                    delta = positions[j] - positions[i]
                    distance = np.linalg.norm(delta)
                    if distance < 1:
                        distance = 1
                    direction = delta / distance

                    # Spring force (attractive for connected nodes)
                    if dist_matrix[i, j] != np.inf:
                        # Hooke's law: F = k * (d - d0)
                        target = target_dist[i, j]
                        spring_force = 0.1 * (distance - target)
                        forces[i] += spring_force * direction
                        forces[j] -= spring_force * direction

                    # Repulsive force (all nodes repel each other)
                    repulsion = -5000 / (distance ** 2)
                    forces[i] += repulsion * direction
                    forces[j] -= repulsion * direction

            # Apply forces with cooling
            positions += forces * cooling * 0.1
            cooling *= 0.995

            # Keep within bounds
            positions = np.clip(positions, 50, scale - 50)

        # Center the layout
        center = np.mean(positions, axis=0)
        positions -= center - np.array([scale / 2, scale / 2])

        # Create position dictionary
        coords = {buses[i]: (positions[i, 0], positions[i, 1]) for i in range(n)}

        return coords

    def compute_layout_hierarchical(self, width=1400, height=500):
        """
        Compute hierarchical layout based on voltage levels and areas.
        - Separates buses by voltage level (LV at edges, HV in middle)
        - Groups buses by area when available
        """
        buses = list(self.bus_info.keys())

        if not buses:
            return {}

        # Group buses by voltage level
        lv_buses = []  # Generator buses (low voltage)
        hv_buses = []  # Transmission buses (high voltage)

        for bus_idx in buses:
            bus = self.bus_info[bus_idx]
            if bus['Vn'] < 100:  # Assume < 100kV is LV (generator level)
                lv_buses.append(bus_idx)
            else:
                hv_buses.append(bus_idx)

        # Group by area if available
        area_buses = defaultdict(lambda: {'lv': [], 'hv': []})
        for bus_idx in buses:
            bus = self.bus_info[bus_idx]
            area = bus.get('area', 1)
            if bus_idx in lv_buses:
                area_buses[area]['lv'].append(bus_idx)
            else:
                area_buses[area]['hv'].append(bus_idx)

        coords = {}
        areas = sorted(area_buses.keys())
        num_areas = len(areas)

        # Compute connectivity to order buses within each group
        connectivity = self._compute_bus_connectivity()

        for area_idx, area in enumerate(areas):
            # Horizontal position based on area
            area_center_x = (area_idx + 0.5) * width / num_areas
            area_width = width / num_areas * 0.8

            # Position LV buses (top and bottom edges)
            lv_list = area_buses[area]['lv']
            for i, bus_idx in enumerate(lv_list):
                # Alternate top and bottom
                if i % 2 == 0:
                    y = 100
                else:
                    y = height - 100
                x = area_center_x + (i - len(lv_list) / 2) * 100
                coords[bus_idx] = (x, y)

            # Position HV buses (middle, ordered by connectivity)
            hv_list = sorted(area_buses[area]['hv'],
                           key=lambda b: sum(connectivity.get(b, {}).values()))

            # Order HV buses by their connections to create a sensible path
            if hv_list:
                ordered_hv = self._order_buses_by_path(hv_list, connectivity)
                for i, bus_idx in enumerate(ordered_hv):
                    x = area_center_x - area_width / 2 + (i + 0.5) * area_width / len(ordered_hv)
                    y = height / 2
                    coords[bus_idx] = (x, y)

        # Adjust LV bus positions to be near their connected HV buses
        for bus_idx in lv_buses:
            connected_hv = [b for b in connectivity.get(bus_idx, {}).keys() if b in hv_buses]
            if connected_hv:
                hv_x = np.mean([coords[b][0] for b in connected_hv])
                old_x, old_y = coords[bus_idx]
                coords[bus_idx] = (hv_x, old_y)

        return coords

    def _compute_bus_connectivity(self):
        """Compute which buses are directly connected and by what impedance."""
        connectivity = defaultdict(dict)
        for line in self.data.get('Line', []):
            b1, b2 = line['bus1'], line['bus2']
            z_mag = np.sqrt(line['r']**2 + line['x']**2)
            # Store minimum impedance for parallel lines
            if b2 not in connectivity[b1] or z_mag < connectivity[b1][b2]:
                connectivity[b1][b2] = z_mag
                connectivity[b2][b1] = z_mag
        return connectivity

    def _order_buses_by_path(self, bus_list, connectivity):
        """Order buses to minimize total path length (greedy approach)."""
        if len(bus_list) <= 1:
            return bus_list

        # Start from bus with fewest connections (likely an endpoint)
        remaining = set(bus_list)
        start = min(remaining, key=lambda b: len([x for x in connectivity.get(b, {}) if x in remaining]))

        ordered = [start]
        remaining.remove(start)

        while remaining:
            current = ordered[-1]
            # Find nearest connected bus
            neighbors = [(b, connectivity.get(current, {}).get(b, np.inf))
                        for b in remaining]
            if neighbors:
                next_bus = min(neighbors, key=lambda x: x[1])[0]
                ordered.append(next_bus)
                remaining.remove(next_bus)
            else:
                # No direct connection, just add any remaining
                ordered.append(remaining.pop())

        return ordered

    def build_phs_structure(self):
        """Build PHS interconnection structure with all component types."""
        print("\n" + "="*30 + " PHS INTERCONNECTION MAPPING " + "="*30)

        structure = {
            "storage_ports": [],
            "exciter_ports": [],
            "governor_ports": [],
            "scaling_ports": [],
            "coupling_ports": [],
            "load_ports": []
        }

        gen_to_bus = {}
        for g in self.data.get('GENROU', []):
            gen_to_bus[g['idx']] = g['bus']

        # Generators
        for g in self.data.get('GENROU', []):
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
            print(f"Generator G{gid} at Bus {g['bus']} ({self.bus_idx_to_name.get(g['bus'], '?')}), M={g['M']}")

        # Exciters
        for e in self.data.get('EXDC2', []):
            structure["exciter_ports"].append({
                "id": f"AVR{e['idx']}",
                "syn": e['syn'],
                "at_bus": gen_to_bus.get(e['syn']),
                "KA": e['KA'],
                "TE": e['TE']
            })
            print(f"Exciter AVR{e['idx']} -> Generator G{e['syn']}")

        # Governors
        for gov in self.data.get('TGOV1', []):
            structure["governor_ports"].append({
                "id": f"GOV{gov['idx']}",
                "syn": gov['syn'],
                "at_bus": gen_to_bus.get(gov['syn']),
                "R": gov['R']
            })
            print(f"Governor GOV{gov['idx']} -> Generator G{gov['syn']}")

        # Transformers
        for l in self.data.get('Line', []):
            if l['Vn1'] != l['Vn2']:
                ratio = l['Vn2'] / l['Vn1']
                structure["scaling_ports"].append({
                    "id": l['idx'],
                    "buses": (l['bus1'], l['bus2']),
                    "ratio": ratio,
                    "x": l['x'],
                    "r": l.get('r', 0),
                    "Vn1": l['Vn1'],
                    "Vn2": l['Vn2']
                })
                print(f"Transformer {l['idx']}: Bus {l['bus1']} -> {l['bus2']}, X={l['x']}")

        # Transmission Lines
        for l in self.data.get('Line', []):
            if l['Vn1'] == l['Vn2']:
                structure["coupling_ports"].append({
                    "id": l['idx'],
                    "buses": (l['bus1'], l['bus2']),
                    "r": l['r'],
                    "x": l['x'],
                    "b": l['b']
                })
                print(f"Line {l['idx']}: Bus {l['bus1']} <-> {l['bus2']}, X={l['x']}")

        # Loads
        for pq in self.data.get('PQ', []):
            structure["load_ports"].append({
                "id": pq['idx'],
                "at_bus": pq['bus'],
                "p0": pq['p0'],
                "q0": pq['q0']
            })
            print(f"Load {pq['idx']} at Bus {pq['bus']}, P={pq['p0']}, Q={pq['q0']}")

        return structure

    def draw(self, filename="auto_system_figure.svg", layout_method="hierarchical"):
        """
        Generate SVG visualization with automatic layout.

        Args:
            filename: Output SVG filename
            layout_method: One of "hierarchical" or "force_directed"
        """
        print(f"\n[AUTO-LAYOUT] Using {layout_method} layout algorithm...")

        # Build structure
        phs_map = self.build_phs_structure()

        # Compute layout based on method
        if layout_method == "force_directed":
            coords = self.compute_layout_force_directed(scale=1200, iterations=300)
            svg_width, svg_height = 1400, 800
        else:  # hierarchical (default)
            coords = self.compute_layout_hierarchical(width=1400, height=500)
            svg_width, svg_height = 1600, 700

        # Normalize coordinates to fit SVG with margins
        if coords:
            all_x = [c[0] for c in coords.values()]
            all_y = [c[1] for c in coords.values()]
            min_x, max_x = min(all_x), max(all_x)
            min_y, max_y = min(all_y), max(all_y)

            margin = 150
            scale_x = (svg_width - 2 * margin) / (max_x - min_x + 1)
            scale_y = (svg_height - 2 * margin - 100) / (max_y - min_y + 1)  # Extra space for legend

            for bus_idx in coords:
                x, y = coords[bus_idx]
                coords[bus_idx] = (
                    margin + (x - min_x) * scale_x,
                    margin + 50 + (y - min_y) * scale_y
                )

        # Start SVG
        svg = [f'<svg width="{svg_width}" height="{svg_height}" xmlns="http://www.w3.org/2000/svg">']
        svg.append('<defs>')
        svg.append('<marker id="arrowhead" markerWidth="10" markerHeight="7" refX="9" refY="3.5" orient="auto">')
        svg.append('<polygon points="0 0, 10 3.5, 0 7" fill="#666"/>')
        svg.append('</marker>')
        svg.append('</defs>')

        # Background
        svg.append('<rect width="100%" height="100%" fill="#fafafa"/>')

        # Title
        svg.append(f'<text x="50" y="35" font-family="Arial" font-size="20" font-weight="bold" fill="#1a237e">Auto-Layout Power System Visualization ({layout_method})</text>')
        svg.append('<text x="50" y="55" font-family="Arial" font-size="12" fill="#666">Positions computed from electrical distances (impedance-based)</text>')

        # Draw legend
        self._draw_legend(svg, svg_height - 70, svg_width)

        # Group parallel lines
        line_groups = {}
        for cp in phs_map["coupling_ports"]:
            key = tuple(sorted(cp["buses"]))
            if key not in line_groups:
                line_groups[key] = []
            line_groups[key].append(cp)

        # Draw transmission lines
        for key, lines in line_groups.items():
            b1, b2 = key
            p1, p2 = coords.get(b1), coords.get(b2)
            if p1 and p2:
                self._draw_parallel_lines(svg, p1, p2, lines)

        # Draw transformers
        for sp in phs_map["scaling_ports"]:
            b1, b2 = sp["buses"]
            p1, p2 = coords.get(b1), coords.get(b2)
            if p1 and p2:
                self._draw_transformer(svg, p1, p2, sp)

        # Draw generators, exciters, governors
        gen_positions = {}
        gen_at_bus = defaultdict(list)
        for gp in phs_map["storage_ports"]:
            gen_at_bus[gp["at_bus"]].append(gp)

        for bus_id, gens in gen_at_bus.items():
            p = coords.get(bus_id)
            if p:
                for i, gp in enumerate(gens):
                    gen_pos = self._draw_generator(svg, p, gp, i, len(gens), coords)
                    gen_positions[gp["gen_idx"]] = gen_pos

        # Draw exciters
        for ep in phs_map["exciter_ports"]:
            if ep["syn"] in gen_positions:
                self._draw_exciter(svg, gen_positions[ep["syn"]], ep)

        # Draw governors
        for gp in phs_map["governor_ports"]:
            if gp["syn"] in gen_positions:
                self._draw_governor(svg, gen_positions[gp["syn"]], gp)

        # Draw loads
        for lp in phs_map["load_ports"]:
            p = coords.get(lp["at_bus"])
            if p:
                self._draw_load(svg, p, lp)

        # Draw bus bars (last for proper layering)
        for bus in self.data.get('Bus', []):
            bus_idx = bus['idx']
            if bus_idx in coords:
                self._draw_bus(svg, coords[bus_idx], bus)

        svg.append('</svg>')

        with open(filename, 'w') as f:
            f.write("\n".join(svg))

        print(f"\n[COMPLETE] Auto-layout diagram generated: {filename}")
        return coords

    def _draw_legend(self, svg, y, width):
        """Draw legend at bottom of SVG."""
        svg.append(f'<rect x="50" y="{y}" width="{width-100}" height="55" fill="#f5f5f5" stroke="#bdbdbd" rx="5"/>')
        svg.append(f'<text x="70" y="{y+20}" font-family="Arial" font-size="12" font-weight="bold">Legend:</text>')

        # Generator
        svg.append(f'<circle cx="170" cy="{y+25}" r="12" fill="#e3f2fd" stroke="#1976d2" stroke-width="2"/>')
        svg.append(f'<text x="165" y="{y+30}" font-family="Arial" font-size="10" font-weight="bold" fill="#1976d2">G</text>')
        svg.append(f'<text x="190" y="{y+30}" font-family="Arial" font-size="11">Generator</text>')

        # Exciter
        svg.append(f'<rect x="270" y="{y+15}" width="20" height="20" fill="#fff3e0" stroke="#ff6f00" stroke-width="2" rx="3"/>')
        svg.append(f'<text x="300" y="{y+30}" font-family="Arial" font-size="11">Exciter</text>')

        # Governor
        svg.append(f'<rect x="380" y="{y+15}" width="20" height="20" fill="#e8f5e9" stroke="#2e7d32" stroke-width="2" rx="3"/>')
        svg.append(f'<text x="410" y="{y+30}" font-family="Arial" font-size="11">Governor</text>')

        # Transformer
        svg.append(f'<circle cx="510" cy="{y+20}" r="8" fill="#f3e5f5" stroke="#7b1fa2" stroke-width="2"/>')
        svg.append(f'<circle cx="520" cy="{y+30}" r="8" fill="#f3e5f5" stroke="#7b1fa2" stroke-width="2"/>')
        svg.append(f'<text x="540" y="{y+30}" font-family="Arial" font-size="11">Transformer</text>')

        # Line
        svg.append(f'<line x1="640" y1="{y+25}" x2="690" y2="{y+25}" stroke="#424242" stroke-width="3"/>')
        svg.append(f'<text x="700" y="{y+30}" font-family="Arial" font-size="11">Transmission Line</text>')

        # Bus
        svg.append(f'<rect x="830" y="{y+12}" width="8" height="26" fill="#212121" rx="2"/>')
        svg.append(f'<text x="850" y="{y+30}" font-family="Arial" font-size="11">Bus</text>')

        # Load
        svg.append(f'<polygon points="920,{y+35} 935,{y+15} 950,{y+35}" fill="#ffebee" stroke="#d32f2f" stroke-width="2"/>')
        svg.append(f'<text x="960" y="{y+30}" font-family="Arial" font-size="11">Load</text>')

    def _draw_parallel_lines(self, svg, p1, p2, lines):
        """Draw parallel transmission lines with offset."""
        num_lines = len(lines)
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        length = np.sqrt(dx**2 + dy**2)

        if length > 0:
            perp_x = -dy / length
            perp_y = dx / length
        else:
            perp_x, perp_y = 0, 1

        spacing = 6
        start_offset = -spacing * (num_lines - 1) / 2

        for i, line in enumerate(lines):
            offset = start_offset + i * spacing
            x1 = p1[0] + perp_x * offset
            y1 = p1[1] + perp_y * offset
            x2 = p2[0] + perp_x * offset
            y2 = p2[1] + perp_y * offset
            svg.append(f'<line x1="{x1:.1f}" y1="{y1:.1f}" x2="{x2:.1f}" y2="{y2:.1f}" stroke="#424242" stroke-width="2"/>')

        # Label
        mid_x = (p1[0] + p2[0]) / 2
        mid_y = (p1[1] + p2[1]) / 2

        if num_lines == 1:
            label = f"X={lines[0]['x']:.3f}"
        else:
            x_eq = 1 / sum(1/l['x'] for l in lines)
            label = f"{num_lines}x, X_eq={x_eq:.4f}"

        # Rotate label to follow line
        angle = np.degrees(np.arctan2(dy, dx))
        if angle > 90:
            angle -= 180
        elif angle < -90:
            angle += 180

        svg.append(f'<text x="{mid_x:.1f}" y="{mid_y-8:.1f}" font-family="Arial" font-size="9" fill="#616161" text-anchor="middle" transform="rotate({angle:.1f} {mid_x:.1f} {mid_y-8:.1f})">{label}</text>')

    def _draw_transformer(self, svg, p1, p2, sp):
        """Draw transformer symbol."""
        mid_x = (p1[0] + p2[0]) / 2
        mid_y = (p1[1] + p2[1]) / 2

        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        length = np.sqrt(dx**2 + dy**2)
        if length > 0:
            ux, uy = dx/length, dy/length
        else:
            ux, uy = 1, 0

        # Lines to transformer
        svg.append(f'<line x1="{p1[0]:.1f}" y1="{p1[1]:.1f}" x2="{mid_x-ux*15:.1f}" y2="{mid_y-uy*15:.1f}" stroke="#7b1fa2" stroke-width="3"/>')
        svg.append(f'<line x1="{mid_x+ux*15:.1f}" y1="{mid_y+uy*15:.1f}" x2="{p2[0]:.1f}" y2="{p2[1]:.1f}" stroke="#7b1fa2" stroke-width="3"/>')

        # Transformer circles
        svg.append(f'<circle cx="{mid_x-ux*8:.1f}" cy="{mid_y-uy*8:.1f}" r="10" fill="#f3e5f5" stroke="#7b1fa2" stroke-width="2"/>')
        svg.append(f'<circle cx="{mid_x+ux*8:.1f}" cy="{mid_y+uy*8:.1f}" r="10" fill="#f3e5f5" stroke="#7b1fa2" stroke-width="2"/>')

        # Label
        svg.append(f'<text x="{mid_x:.1f}" y="{mid_y+25:.1f}" font-family="Arial" font-size="8" fill="#7b1fa2" text-anchor="middle">X={sp["x"]}</text>')

    def _draw_generator(self, svg, bus_pos, gp, index, total, all_coords):
        """Draw generator and return its position."""
        # Determine placement direction (away from center of mass)
        center_x = np.mean([c[0] for c in all_coords.values()])

        # Place generator away from center
        if bus_pos[0] < center_x:
            gx = bus_pos[0] - 70
        else:
            gx = bus_pos[0] + 70

        # Offset for multiple generators at same bus
        vert_offset = (index - total/2 + 0.5) * 60
        gy = bus_pos[1] + vert_offset

        # Connection line
        svg.append(f'<line x1="{bus_pos[0]:.1f}" y1="{bus_pos[1]:.1f}" x2="{gx:.1f}" y2="{gy:.1f}" stroke="#1976d2" stroke-width="2"/>')

        # Generator circle
        svg.append(f'<circle cx="{gx:.1f}" cy="{gy:.1f}" r="20" fill="#e3f2fd" stroke="#1976d2" stroke-width="3"/>')
        svg.append(f'<text x="{gx:.1f}" y="{gy+5:.1f}" font-family="Arial" font-size="12" font-weight="bold" fill="#1976d2" text-anchor="middle">{gp["id"]}</text>')

        # Label
        label_x = gx - 30 if gx < bus_pos[0] else gx + 30
        svg.append(f'<text x="{label_x:.1f}" y="{gy-8:.1f}" font-family="Arial" font-size="8" fill="#1565c0" text-anchor="middle">M={gp["M"]}</text>')
        svg.append(f'<text x="{label_x:.1f}" y="{gy+3:.1f}" font-family="Arial" font-size="8" fill="#1565c0" text-anchor="middle">{gp["Sn"]:.0f}MVA</text>')

        return (gx, gy)

    def _draw_exciter(self, svg, gen_pos, ep):
        """Draw exciter connected to generator."""
        gx, gy = gen_pos
        ex, ey = gx, gy - 45

        svg.append(f'<line x1="{gx:.1f}" y1="{gy-20:.1f}" x2="{ex:.1f}" y2="{ey+10:.1f}" stroke="#ff6f00" stroke-width="1.5" stroke-dasharray="3,2"/>')
        svg.append(f'<rect x="{ex-12:.1f}" y="{ey-10:.1f}" width="24" height="20" fill="#fff3e0" stroke="#ff6f00" stroke-width="2" rx="3"/>')
        svg.append(f'<text x="{ex:.1f}" y="{ey+4:.1f}" font-family="Arial" font-size="8" font-weight="bold" fill="#e65100" text-anchor="middle">AVR</text>')

    def _draw_governor(self, svg, gen_pos, gp):
        """Draw governor connected to generator."""
        gx, gy = gen_pos
        govx, govy = gx, gy + 45

        svg.append(f'<line x1="{gx:.1f}" y1="{gy+20:.1f}" x2="{govx:.1f}" y2="{govy-10:.1f}" stroke="#2e7d32" stroke-width="1.5" stroke-dasharray="3,2"/>')
        svg.append(f'<rect x="{govx-12:.1f}" y="{govy-10:.1f}" width="24" height="20" fill="#e8f5e9" stroke="#2e7d32" stroke-width="2" rx="3"/>')
        svg.append(f'<text x="{govx:.1f}" y="{govy+4:.1f}" font-family="Arial" font-size="8" font-weight="bold" fill="#1b5e20" text-anchor="middle">GOV</text>')

    def _draw_load(self, svg, bus_pos, lp):
        """Draw load at bus."""
        lx = bus_pos[0]
        ly = bus_pos[1] + 45

        svg.append(f'<line x1="{bus_pos[0]:.1f}" y1="{bus_pos[1]:.1f}" x2="{lx:.1f}" y2="{ly-12:.1f}" stroke="#d32f2f" stroke-width="2"/>')
        svg.append(f'<polygon points="{lx:.1f},{ly+12:.1f} {lx-12:.1f},{ly-8:.1f} {lx+12:.1f},{ly-8:.1f}" fill="#ffebee" stroke="#d32f2f" stroke-width="2"/>')
        svg.append(f'<text x="{lx:.1f}" y="{ly+25:.1f}" font-family="Arial" font-size="8" fill="#c62828" text-anchor="middle">P={lp["p0"]}</text>')

    def _draw_bus(self, svg, pos, bus):
        """Draw bus bar."""
        bus_name = str(bus['name'])
        vn = bus['Vn']

        # Color by voltage level
        bar_color = "#1565c0" if vn < 100 else "#212121"

        svg.append(f'<rect x="{pos[0]-4:.1f}" y="{pos[1]-25:.1f}" width="8" height="50" fill="{bar_color}" rx="2"/>')
        svg.append(f'<text x="{pos[0]:.1f}" y="{pos[1]-32:.1f}" font-family="Arial" font-size="11" font-weight="bold" text-anchor="middle">{bus_name}</text>')
        svg.append(f'<text x="{pos[0]:.1f}" y="{pos[1]+40:.1f}" font-family="Arial" font-size="9" fill="#757575" text-anchor="middle">{vn:.0f}kV</text>')
