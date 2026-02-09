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

Recent Improvements (Inspired by VS Code Debug Visualizer):
- Fixed duplicate label rendering that caused text overlap
- Improved collision detection with simplified spiral search algorithm
- Enhanced text positioning using perpendicular offsets to minimize overlap
- Added white text backgrounds (paint-order: stroke) for better readability
- Fixed layout normalization to preserve aspect ratio and prevent cramming
- Increased margins and spacing for cleaner, more readable diagrams
- All labels now positioned strategically to avoid component overlap
"""

import numpy as np
import pandas as pd
from collections import defaultdict
import networkx as nx
try:
    import graphviz
    GRAPHVIZ_AVAILABLE = True
except ImportError:
    GRAPHVIZ_AVAILABLE = False
    print("[WARNING] graphviz package not available. Install with: pip install graphviz")
    print("[WARNING] Also ensure Graphviz is installed: https://graphviz.org/download/")


class AutoPHSVisualizer:
    def __init__(self, builder):
        self.data = builder.system_data
        # Build bus idx to name mapping
        self.bus_idx_to_name = {}
        self.bus_name_to_idx = {}
        self.bus_info = {}
        # Track component positions for collision detection
        self.component_positions = {}
        self.occupied_regions = []
        self.strict_component_clearance = False
        self.clearance_padding = 0.0
        for bus in self.data.get('Bus', []):
            self.bus_idx_to_name[bus['idx']] = str(bus['name'])
            self.bus_name_to_idx[str(bus['name'])] = bus['idx']
            self.bus_info[bus['idx']] = bus

    def compute_electrical_distance_matrix(self, scale_method='linear', min_distance=None, max_distance=None):
        """
        Compute electrical distance matrix based on line impedances.
        Distance = |Z| = sqrt(r² + x²) for direct connections, infinity otherwise.
        
        Args:
            scale_method: How to scale distances for better visual layout:
                - 'linear': Use raw impedance values (default)
                - 'log': Logarithmic scaling log(1 + Z) for compressed range
                - 'sqrt': Square root scaling sqrt(Z) for moderate compression
            min_distance: Minimum distance clamp (prevents too-close buses)
            max_distance: Maximum distance clamp (prevents too-far buses)
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
        
        # Apply distance scaling transformation
        if scale_method == 'log':
            # Logarithmic scaling: compresses large distances
            finite_mask = np.isfinite(dist_matrix)
            dist_matrix[finite_mask] = np.log1p(dist_matrix[finite_mask])  # log(1 + x)
        elif scale_method == 'sqrt':
            # Square root scaling: moderate compression
            finite_mask = np.isfinite(dist_matrix)
            dist_matrix[finite_mask] = np.sqrt(dist_matrix[finite_mask])
        # else: 'linear' - no transformation
        
        # Apply min/max distance clamping if specified
        if min_distance is not None or max_distance is not None:
            finite_mask = np.isfinite(dist_matrix)
            if min_distance is not None:
                dist_matrix[finite_mask] = np.maximum(dist_matrix[finite_mask], min_distance)
            if max_distance is not None:
                dist_matrix[finite_mask] = np.minimum(dist_matrix[finite_mask], max_distance)

        return buses, dist_matrix

    def compute_layout_force_directed(self, scale=800, iterations=500, spacing_factor=1.0):
        """
        Compute node positions using NetworkX's spring layout (force-directed).
        Uses proven Fruchterman-Reingold algorithm with automatic overlap prevention.
        
        Args:
            scale: Scale factor for the layout (larger = more spacing)
            iterations: Number of optimization iterations
            spacing_factor: Multiplier for bus spacing (default=1.0)
            
        Returns:
            Dictionary of bus coordinates
        """
        buses, dist_matrix = self.compute_electrical_distance_matrix()
        n = len(buses)

        if n == 0:
            return {}

        # Create NetworkX graph with weighted edges
        G = nx.Graph()
        for bus in buses:
            G.add_node(bus)
        
        # Add edges with impedance as weight
        for line in self.data.get('Line', []):
            b1, b2 = line['bus1'], line['bus2']
            if b1 in buses and b2 in buses:
                z_mag = np.sqrt(line['r']**2 + line['x']**2)
                # Use inverse impedance as weight (stronger connections = higher weight)
                weight = 1.0 / (z_mag + 0.01)
                G.add_edge(b1, b2, weight=weight)
        
        # Use NetworkX's spring layout with optimal spacing parameter
        # k = optimal distance between nodes (larger k = more spacing)
        k_param = (2.5 / np.sqrt(n)) * spacing_factor  # Apply spacing factor to k
        pos = nx.spring_layout(
            G, 
            k=k_param,           # Optimal distance between nodes with spacing factor
            iterations=iterations,
            weight='weight',      # Use impedance-based weights
            scale=scale * spacing_factor,  # Scale the layout with spacing factor
            seed=42              # Reproducible layout
        )
        
        # Convert to our coordinate format
        coords = {bus: (pos[bus][0], pos[bus][1]) for bus in buses}
        
        return coords

    def compute_layout_mds(self, width=1400, height=800, spacing_factor=1.0, 
                          scale_method='linear', min_distance=None, max_distance=None):
        """
        Compute node positions using spring layout with clamped edge weights.
        
        Uses Kamada-Kawai or spring layout with edge weights derived from 
        scaled and clamped electrical distances. This provides natural spacing
        while respecting distance constraints.
        
        Args:
            width: Canvas width
            height: Canvas height
            spacing_factor: Multiplier for bus spacing (default=1.0)
            scale_method: Distance scaling ('linear', 'log', 'sqrt')
            min_distance: Minimum distance clamp
            max_distance: Maximum distance clamp
            
        Returns:
            Tuple of (coordinates dict, bus list, distance matrix)
        """
        buses = list(self.bus_info.keys())
        n = len(buses)

        if n == 0:
            return {}, buses, np.array([])

        # Create NetworkX graph with weighted edges
        G = nx.Graph()
        for bus in buses:
            G.add_node(bus)
        
        # Build distance matrix for reference
        bus_to_idx = {b: i for i, b in enumerate(buses)}
        dist_matrix = np.full((n, n), np.inf)
        np.fill_diagonal(dist_matrix, 0)
        
        # Add edges with scaled and clamped distances
        for line in self.data.get('Line', []):
            b1, b2 = line['bus1'], line['bus2']
            if b1 in buses and b2 in buses:
                # Calculate raw impedance
                z_mag = np.sqrt(line['r']**2 + line['x']**2)
                
                # Apply scaling transformation
                if scale_method == 'log':
                    z_scaled = np.log1p(z_mag)
                elif scale_method == 'sqrt':
                    z_scaled = np.sqrt(z_mag)
                else:  # 'linear'
                    z_scaled = z_mag
                
                # Apply min/max clamping
                if min_distance is not None:
                    z_scaled = max(z_scaled, min_distance)
                if max_distance is not None:
                    z_scaled = min(z_scaled, max_distance)
                
                # Add edge with clamped distance as weight
                # For spring layout, use distance directly as the target length
                if G.has_edge(b1, b2):
                    # For parallel lines, use minimum (stronger connection)
                    G[b1][b2]['weight'] = min(G[b1][b2]['weight'], z_scaled)
                else:
                    G.add_edge(b1, b2, weight=z_scaled)
                
                # Update distance matrix
                i, j = bus_to_idx[b1], bus_to_idx[b2]
                dist_matrix[i, j] = min(dist_matrix[i, j], z_scaled)
                dist_matrix[j, i] = dist_matrix[i, j]
        
        # Try Kamada-Kawai layout first (respects edge weights as distances)
        try:
            print("[INFO] Using Kamada-Kawai layout for distance preservation...")
            pos = nx.kamada_kawai_layout(G, weight='weight', scale=1.0)
        except:
            # Fallback to spring layout with appropriate parameters
            print("[INFO] Kamada-Kawai failed, using spring layout...")
            # For spring layout, convert distances to spring constants
            # Shorter edges = stronger springs = higher weight
            for u, v, data in G.edges(data=True):
                distance = data['weight']
                # Inverse relationship: shorter distance = stronger spring
                data['spring_weight'] = 1.0 / (distance + 0.01)
            
            pos = nx.spring_layout(G, weight='spring_weight', k=0.5*spacing_factor, 
                                  iterations=100, seed=42, scale=1.0)
        
        # Scale positions to canvas size
        positions = np.array([pos[bus] for bus in buses])
        positions -= positions.mean(axis=0)
        
        # Apply canvas scaling
        scale_factor = min(width, height) * 0.35 * spacing_factor
        max_extent = np.max(np.abs(positions))
        if max_extent > 0:
            positions = positions / max_extent * scale_factor
        
        # Translate to canvas center with margins
        margin = 200
        positions[:, 0] += width / 2
        positions[:, 1] += height / 2
        
        # Ensure all positions are within bounds
        positions[:, 0] = np.clip(positions[:, 0], margin, width - margin)
        positions[:, 1] = np.clip(positions[:, 1], margin, height - margin)
        
        # Create coordinate dictionary
        coords = {buses[i]: (positions[i, 0], positions[i, 1]) for i in range(n)}

        return coords, buses, dist_matrix
        
        # Extract positions and apply scale factor for better spacing
        scale_factor = min(width, height) * 0.35 * spacing_factor  # Apply spacing factor to scale
        positions = np.array([pos[bus] for bus in buses])
        
        # Center and scale positions
        positions -= positions.mean(axis=0)
        positions *= scale_factor
        
        # Translate to canvas center with margins
        margin = 200
        positions[:, 0] += width / 2
        positions[:, 1] += height / 2
        
        # Ensure all positions are within bounds
        positions[:, 0] = np.clip(positions[:, 0], margin, width - margin)
        positions[:, 1] = np.clip(positions[:, 1], margin, height - margin)
        
        # Create coordinate dictionary
        coords = {buses[i]: (positions[i, 0], positions[i, 1]) for i in range(n)}

        return coords, buses, dist_matrix

    def compute_layout_hierarchical(self, width=1400, height=500, spacing_factor=1.0):
        """
        Compute hierarchical layout using NetworkX's multipartite layout.
        - Separates buses by voltage level (LV at edges, HV in middle)
        - Uses spring layout within each level for optimal spacing
        
        Args:
            width: Canvas width
            height: Canvas height
            spacing_factor: Multiplier for bus spacing (default=1.0)
        """
        buses = list(self.bus_info.keys())

        if not buses:
            return {}

        # Create NetworkX graph  
        G = nx.Graph()
        for bus in buses:
            G.add_node(bus)
        
        for line in self.data.get('Line', []):
            b1, b2 = line['bus1'], line['bus2']
            if b1 in buses and b2 in buses:
                G.add_edge(b1, b2)
        
        # Assign voltage level layers
        for bus_idx in buses:
            bus = self.bus_info[bus_idx]
            vn = bus['Vn']
            # Create layers based on voltage: 0=LV generators, 1=HV transmission, 2=MV distribution
            if vn < 50:  # Low voltage (generator level)
                layer = 0
            elif vn >= 200:  # High voltage (transmission)
                layer = 1  
            else:  # Medium voltage
                layer = 2
            G.nodes[bus_idx]['layer'] = layer
        
        # Use multipartite layout for hierarchical structure with spacing factor
        try:
            pos = nx.multipartite_layout(G, subset_key='layer', align='horizontal', 
                                        scale=width * 0.4 * spacing_factor)
        except:
            # Fallback to spring layout
            pos = nx.spring_layout(G, k=3.0 * spacing_factor, iterations=100, seed=42, 
                                  scale=width * 0.4 * spacing_factor)
        
        # Apply spacing and positioning with spacing factor
        coords = {}
        y_spacing = height * spacing_factor / max(3, len(set(G.nodes[b].get('layer', 1) for b in buses)))
        y_positions = {0: y_spacing * 0.5, 1: y_spacing * 1.5, 2: y_spacing * 2.5}
        
        for bus_idx in buses:
            x, y = pos[bus_idx]
            layer = G.nodes[bus_idx].get('layer', 1)
            
            # Scale X and use layer-based Y with spacing factor
            coords[bus_idx] = (
                (x + 1.0) * width / 2,  # Center and scale X
                y_positions.get(layer, height / 2)  # Fixed Y by layer
            )

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

    def _build_bus_adjacency(self):
        """Build adjacency list for buses based on lines."""
        adjacency = defaultdict(set)
        for line in self.data.get('Line', []):
            b1, b2 = line['bus1'], line['bus2']
            adjacency[b1].add(b2)
            adjacency[b2].add(b1)
        return adjacency

    def _estimate_bus_footprints(self, phs_map, spacing_factor=1.0, node_size_factor=1.0):
        """Estimate a per-bus radius needed to place attached components cleanly."""
        gen_at_bus = defaultdict(list)
        load_at_bus = defaultdict(list)
        for gp in phs_map.get("storage_ports", []):
            gen_at_bus[gp["at_bus"]].append(gp)
        for lp in phs_map.get("load_ports", []):
            load_at_bus[lp["at_bus"]].append(lp)

        bus_radii = {}
        for bus_idx in self.bus_info.keys():
            gen_count = len(gen_at_bus.get(bus_idx, []))
            load_count = len(load_at_bus.get(bus_idx, []))

            # Base offsets used by component placement
            gen_reach = 90 * spacing_factor + 30 * node_size_factor
            if gen_count > 1:
                gen_spacing = max(80, 40 + gen_count * 10) * spacing_factor
                gen_vertical_extent = gen_spacing * (gen_count - 1) / 2
            else:
                gen_vertical_extent = 0

            if load_count > 0:
                load_reach = (60 if gen_count == 0 else 80 + gen_count * 20) * spacing_factor + 20 * node_size_factor
            else:
                load_reach = 60 * spacing_factor

            base_radius = max(gen_reach, load_reach, 60 * node_size_factor)
            bus_radii[bus_idx] = base_radius + gen_vertical_extent

        return bus_radii

    def _relax_bus_clearance(self, coords, bus_radii, clearance=40.0, iterations=60):
        """Iteratively push buses apart to satisfy minimum clearance."""
        buses = list(coords.keys())
        if len(buses) < 2:
            return coords

        for _ in range(iterations):
            moved = False
            for i in range(len(buses)):
                for j in range(i + 1, len(buses)):
                    bi, bj = buses[i], buses[j]
                    xi, yi = coords[bi]
                    xj, yj = coords[bj]
                    dx, dy = xj - xi, yj - yi
                    dist = np.hypot(dx, dy)
                    min_dist = bus_radii.get(bi, 0) + bus_radii.get(bj, 0) + clearance

                    if dist < min_dist:
                        if dist < 1e-6:
                            ux, uy = 1.0, 0.0
                        else:
                            ux, uy = dx / dist, dy / dist
                        push = (min_dist - dist) / 2
                        coords[bi] = (xi - ux * push, yi - uy * push)
                        coords[bj] = (xj + ux * push, yj + uy * push)
                        moved = True
            if not moved:
                break

        return coords

    def _spread_edge_angles(self, coords, adjacency, spacing_factor=1.0, strength=0.25, iterations=30):
        """Spread incident edges around high-degree buses to maximize angular separation."""
        if not coords:
            return coords

        min_radius = 80 * spacing_factor
        buses = list(coords.keys())

        for _ in range(iterations):
            for bus in buses:
                neighbors = list(adjacency.get(bus, []))
                if len(neighbors) < 3:
                    continue

                bx, by = coords[bus]
                angles = []
                radii = []
                for nb in neighbors:
                    nx, ny = coords.get(nb, (bx, by))
                    dx, dy = nx - bx, ny - by
                    r = np.hypot(dx, dy)
                    if r < 1e-6:
                        r = min_radius
                    radii.append(max(r, min_radius))
                    angles.append(np.arctan2(dy, dx))

                # Circular mean for base angle
                sin_sum = np.sum(np.sin(angles))
                cos_sum = np.sum(np.cos(angles))
                base_angle = np.arctan2(sin_sum, cos_sum)
                delta = 2 * np.pi / len(neighbors)
                target_angles = [base_angle + i * delta for i in range(len(neighbors))]

                order = np.argsort(angles)
                for idx, nb_idx in enumerate(order):
                    nb = neighbors[nb_idx]
                    r = radii[nb_idx]
                    tx = bx + r * np.cos(target_angles[idx])
                    ty = by + r * np.sin(target_angles[idx])
                    nx, ny = coords[nb]
                    coords[nb] = (nx + (tx - nx) * strength, ny + (ty - ny) * strength)

        return coords

    def _improve_bus_layout(self, coords, phs_map, spacing_factor=1.0, node_size_factor=1.0,
                             angle_spread=True, strict_component_clearance=False, bus_clearance=40.0):
        """Improve bus spacing and angular separation before component placement."""
        if not coords:
            return coords

        adjacency = self._build_bus_adjacency()
        bus_radii = self._estimate_bus_footprints(phs_map, spacing_factor, node_size_factor)
        clearance = bus_clearance * spacing_factor

        if angle_spread:
            coords = self._spread_edge_angles(coords, adjacency, spacing_factor=spacing_factor)

        if strict_component_clearance or bus_clearance > 0:
            clearance_iters = 90 if strict_component_clearance else 50
            coords = self._relax_bus_clearance(coords, bus_radii, clearance=clearance, iterations=clearance_iters)

        # Second pass to stabilize after clearance adjustment
        if angle_spread:
            coords = self._spread_edge_angles(coords, adjacency, spacing_factor=spacing_factor, strength=0.18, iterations=15)

        return coords

    def _seed_bus_occupied_regions(self, coords, node_size_factor=1.0, spacing_factor=1.0):
        """Reserve space around bus bars to prevent component overlap."""
        pad = 12 * spacing_factor
        bar_width = 8 * node_size_factor + pad * 2
        bar_height = 50 * node_size_factor + pad * 2
        for bus in self.data.get('Bus', []):
            bus_idx = bus['idx']
            if bus_idx in coords:
                x, y = coords[bus_idx]
                self.occupied_regions.append((x - bar_width / 2, y - bar_height / 2,
                                              x + bar_width / 2, y + bar_height / 2))

    def draw_graphviz(self, filename="power_system_graphviz", layout_engine="dot", 
                     format="svg", show_impedance=True, show_voltage=True, 
                     load_scale=1.0, generator_scale=1.0, transformer_scale=1.0,
                     exciter_scale=1.0, governor_scale=1.0):
        """
        Generate professional graph visualization using Graphviz (same as VS Code Debug Visualizer).
        
        This method delegates ALL layout, spacing, and rendering to Graphviz - the same
        professional graph layout engine used by VS Code Debug Visualizer.
        
        Args:
            filename: Output filename (without extension)
            layout_engine: Graphviz layout engine:
                - "dot" (default): Hierarchical, good for directed graphs
                - "neato": Spring model, good for undirected graphs
                - "fdp": Force-directed placement
                - "sfdp": Scalable force-directed (best for large graphs)
                - "circo": Circular layout
                - "twopi": Radial layout
            format: Output format ("svg", "png", "pdf")
            show_impedance: Show impedance values on lines
            show_voltage: Show voltage levels on buses
            load_scale: Scale factor for load triangles (default=1.0)
            generator_scale: Scale factor for generator ellipses (default=1.0)
            transformer_scale: Scale factor for transformer diamonds (default=1.0)
            exciter_scale: Scale factor for exciter boxes (default=1.0)
            governor_scale: Scale factor for governor boxes (default=1.0)
        """
        if not GRAPHVIZ_AVAILABLE:
            print("[ERROR] Graphviz not available. Install with: pip install graphviz")
            return
        
        print(f"\n[GRAPHVIZ] Using {layout_engine} layout engine (professional automatic layout)...")
        print(f"[SCALE] Load: {load_scale}x | Generator: {generator_scale}x | Transformer: {transformer_scale}x")
        print(f"[SCALE] Exciter: {exciter_scale}x | Governor: {governor_scale}x")
        
        # Build structure
        phs_map = self.build_phs_structure()
        
        # Create Graphviz graph
        graph = graphviz.Graph(
            name='power_system',
            engine=layout_engine,
            graph_attr={
                'rankdir': 'LR',  # Left to right
                'ranksep': '1.5',  # Spacing between ranks
                'nodesep': '1.0',  # Spacing between nodes
                'splines': 'true',  # Curved edges
                'overlap': 'false',  # Prevent node overlap
                'bgcolor': '#fafafa',
                'fontname': 'Arial',
                'pad': '0.5',  # Padding around graph
            },
            node_attr={
                'fontname': 'Arial',
                'fontsize': '11',
                'margin': '0.2,0.1',
            },
            edge_attr={
                'fontname': 'Arial',
                'fontsize': '9',
            }
        )
        
        # Add buses as nodes
        for bus in self.data.get('Bus', []):
            bus_idx = bus['idx']
            bus_name = str(bus['name'])
            vn = bus['Vn']
            
            # Color by voltage level
            if vn < 100:
                color = '#1565c0'
                fillcolor = '#e3f2fd'
            else:
                color = '#f57f17'
                fillcolor = '#fff9c4'
            
            label = f"{bus_name}"
            if show_voltage:
                label += f"\\n{vn:.0f} kV"
            
            graph.node(
                str(bus_idx),
                label=label,
                shape='box',
                style='filled,rounded',
                fillcolor=fillcolor,
                color=color,
                penwidth='2.5'
            )
        
        # Add generators as separate nodes connected to buses (REDUCED SIZE)
        for gp in phs_map["storage_ports"]:
            gen_id = gp["id"]
            bus_idx = gp["at_bus"]
            
            label = f"{gen_id}\\nM={gp['M']:.1f}\\n{gp['Sn']:.0f} MVA"
            
            graph.node(
                f"gen_{gen_id}",
                label=label,
                shape='ellipse',
                style='filled',
                fillcolor='#e3f2fd',
                color='#1976d2',
                penwidth='1.5',
                width=f'{0.6 * generator_scale:.2f}',
                height=f'{0.5 * generator_scale:.2f}',
                fixedsize='true',
                fontsize='9'
            )
            
            graph.edge(
                str(bus_idx),
                f"gen_{gen_id}",
                color='#1976d2',
                penwidth='1.5'
            )
        
        # Add exciters as small nodes
        for ep in phs_map["exciter_ports"]:
            exc_id = ep["id"]
            gen_idx = ep["syn"]
            model = ep.get('model', 'AVR')
            
            graph.node(
                f"exc_{exc_id}",
                label=f"{model}",
                shape='box',
                style='filled,rounded',
                fillcolor='#fff3e0',
                color='#ff6f00',
                penwidth='1.5',
                width=f'{0.5 * exciter_scale:.2f}',
                height=f'{0.3 * exciter_scale:.2f}',
                fixedsize='true',
                fontsize='8'
            )
            
            graph.edge(
                f"gen_G{gen_idx}",
                f"exc_{exc_id}",
                color='#ff6f00',
                style='dashed',
                penwidth='1.5'
            )
        
        # Add governors as small nodes
        for gp in phs_map["governor_ports"]:
            gov_id = gp["id"]
            gen_idx = gp["syn"]
            model = gp.get('model', 'GOV')
            
            graph.node(
                f"gov_{gov_id}",
                label=f"{model}",
                shape='box',
                style='filled,rounded',
                fillcolor='#e8f5e9',
                color='#2e7d32',
                penwidth='1.5',
                width=f'{0.5 * governor_scale:.2f}',
                height=f'{0.3 * governor_scale:.2f}',
                fixedsize='true',
                fontsize='8'
            )
            
            graph.edge(
                f"gen_G{gen_idx}",
                f"gov_{gov_id}",
                color='#2e7d32',
                style='dashed',
                penwidth='1.5'
            )
        
        # Add loads as triangle nodes (MUCH SMALLER SIZE)
        for lp in phs_map["load_ports"]:
            load_id = lp["id"]
            bus_idx = lp["at_bus"]
            
            label = f"L\\n{lp['p0']:.1f}"
            
            graph.node(
                f"load_{load_id}",
                label=label,
                shape='triangle',
                style='filled',
                fillcolor='#ffebee',
                color='#d32f2f',
                penwidth='1.0',
                width=f'{0.25 * load_scale:.2f}',
                height=f'{0.25 * load_scale:.2f}',
                fixedsize='true',
                fontsize='8'
            )
            
            graph.edge(
                str(bus_idx),
                f"load_{load_id}",
                color='#d32f2f',
                penwidth='1.0'
            )
        
        # Add transmission lines (coupling ports)
        line_counts = defaultdict(int)
        for cp in phs_map["coupling_ports"]:
            b1, b2 = cp["buses"]
            key = tuple(sorted([b1, b2]))
            line_counts[key] += 1
            
            label = ""
            if show_impedance:
                label = f"X={cp['x']:.4f}"
            
            graph.edge(
                str(b1),
                str(b2),
                label=label,
                color='#424242',
                penwidth='2.5',
                style='solid'
            )
        
        # Add transformers (scaling ports) - WITH VISIBLE DIAMOND NODES
        for idx, sp in enumerate(phs_map["scaling_ports"]):
            b1, b2 = sp["buses"]
            xfmr_id = f"xfmr_{idx}"
            
            # Create transformer node (diamond/square)
            label = f"T\\n{sp['Vn1']:.0f}/{sp['Vn2']:.0f}"
            if show_impedance:
                label = f"T\\n{sp['Vn1']:.0f}/\\n{sp['Vn2']:.0f}\\nX={sp['x']:.3f}"
            
            graph.node(
                xfmr_id,
                label=label,
                shape='diamond',
                style='filled',
                fillcolor='#f3e5f5',
                color='#7b1fa2',
                penwidth='2.5',
                width=f'{0.5 * transformer_scale:.2f}',
                height=f'{0.5 * transformer_scale:.2f}',
                fixedsize='true',
                fontsize='8',
                fontcolor='#4a148c'
            )
            
            # Connect bus1 -> transformer
            graph.edge(
                str(b1),
                xfmr_id,
                color='#7b1fa2',
                penwidth='2.5'
            )
            
            # Connect transformer -> bus2
            graph.edge(
                xfmr_id,
                str(b2),
                color='#7b1fa2',
                penwidth='2.5'
            )
        
        # Render the graph
        try:
            output_path = graph.render(filename, format=format, cleanup=True)
            print(f"[COMPLETE] Graphviz visualization generated: {output_path}")
            print(f"[INFO] Layout engine: {layout_engine} | Format: {format}")
            return output_path
        except Exception as e:
            print(f"[ERROR] Failed to render graph: {e}")
            print("[TIP] Make sure Graphviz is installed: https://graphviz.org/download/")
            return None

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

        # Exciters (all types)
        for exc_type in ['EXDC2', 'EXST1', 'ESST3A', 'IEEEX1']:
            for e in self.data.get(exc_type, []):
                structure["exciter_ports"].append({
                    "id": f"AVR{e['idx']}",
                    "syn": e['syn'],
                    "at_bus": gen_to_bus.get(e['syn']),
                    "model": exc_type
                })
                print(f"Exciter {exc_type} AVR{e['idx']} -> Generator G{e['syn']}")

        # Governors (all types)
        for gov_type in ['TGOV1', 'IEEEG1']:
            for gov in self.data.get(gov_type, []):
                structure["governor_ports"].append({
                    "id": f"GOV{gov['idx']}",
                    "syn": gov['syn'],
                    "at_bus": gen_to_bus.get(gov['syn']),
                    "model": gov_type
                })
                print(f"Governor {gov_type} GOV{gov['idx']} -> Generator G{gov['syn']}")

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

    def _calculate_bus_space_requirements(self, bus_idx, phs_map):
        """Calculate space requirements for all components at a bus."""
        requirements = {
            'left': 0, 'right': 0, 'top': 0, 'bottom': 0,
            'generators': [], 'loads': [], 'components': []
        }
        
        # Count generators at this bus
        for gp in phs_map["storage_ports"]:
            if gp["at_bus"] == bus_idx:
                requirements['generators'].append(gp)
                
        # Count loads at this bus
        for lp in phs_map["load_ports"]:
            if lp["at_bus"] == bus_idx:
                requirements['loads'].append(lp)
                
        # Calculate space needed
        num_generators = len(requirements['generators'])
        num_loads = len(requirements['loads'])
        
        if num_generators > 0:
            requirements['left'] = max(requirements['left'], 100)
            requirements['right'] = max(requirements['right'], 100)
            requirements['top'] = max(requirements['top'], num_generators * 30)
            requirements['bottom'] = max(requirements['bottom'], num_generators * 30)
            
        if num_loads > 0:
            requirements['bottom'] = max(requirements['bottom'], 60)
            
        return requirements

    def _find_non_overlapping_position(self, base_pos, width, height, avoid_regions):
        """Find a position that doesn't overlap with existing components."""
        x, y = base_pos
        pad = self.clearance_padding if self.strict_component_clearance else 5.0
        
        # First, check if base position is clear
        test_region = (x - width/2 - pad, y - height/2 - pad,
                      x + width/2 + pad, y + height/2 + pad)
        if not self._overlaps_with_regions(test_region, avoid_regions):
            return base_pos
        
        # If strict clearance is enabled, do spiral search
        if self.strict_component_clearance:
            ring_step = max(width, height) * 0.8 + 15
            angle_steps = 16  # Increased for finer granularity
            max_rings = 8
            for ring in range(1, max_rings + 1):
                radius = ring * ring_step
                for k in range(angle_steps):
                    theta = 2 * np.pi * k / angle_steps
                    test_x = x + radius * np.cos(theta)
                    test_y = y + radius * np.sin(theta)
                    test_region = (test_x - width/2 - pad, test_y - height/2 - pad,
                                  test_x + width/2 + pad, test_y + height/2 + pad)
                    if not self._overlaps_with_regions(test_region, avoid_regions):
                        return (test_x, test_y)
        else:
            # Simple offset strategy - try common directions
            step = max(width, height) + 25
            offsets = [
                (0, step), (0, -step),      # Up/Down
                (step, 0), (-step, 0),       # Right/Left
                (step, step), (-step, step), # Diagonals
                (step, -step), (-step, -step)
            ]
            for dx, dy in offsets:
                test_x, test_y = x + dx, y + dy
                test_region = (test_x - width/2 - pad, test_y - height/2 - pad,
                              test_x + width/2 + pad, test_y + height/2 + pad)
                if not self._overlaps_with_regions(test_region, avoid_regions):
                    return (test_x, test_y)
        
        # Fallback: return position with safe offset
        return (x + width + 50, y + height + 50)

    def _overlaps_with_regions(self, region1, regions_list):
        """Check if region1 overlaps with any region in regions_list."""
        x1, y1, x2, y2 = region1
        for rx1, ry1, rx2, ry2 in regions_list:
            if not (x2 < rx1 or x1 > rx2 or y2 < ry1 or y1 > ry2):
                return True
        return False

    def draw(self, filename="auto_system_figure.svg", layout_method="hierarchical", spacing_factor=1.0,
             node_size_factor=1.0, angle_spread=False, strict_component_clearance=False, bus_clearance=0.0):
        """
        Generate SVG visualization with automatic layout.

        Args:
            filename: Output SVG filename
            layout_method: One of "hierarchical" or "force_directed"
            spacing_factor: Multiplier for component spacing (default=1.0, try 1.5-2.0 for more spread)
            node_size_factor: Multiplier for node/component sizes (default=1.0, try 0.6-0.8 for compact)
            angle_spread: Spread incident edges around high-degree buses (default=False)
            strict_component_clearance: Enforce no overlaps between all components (default=False)
            bus_clearance: Extra spacing between buses (default=0.0)
        """
        print(f"\n[AUTO-LAYOUT] Using {layout_method} layout algorithm...")

        # Build structure
        phs_map = self.build_phs_structure()

        # Compute layout based on method
        if layout_method == "force_directed":
            coords = self.compute_layout_force_directed(scale=1400, iterations=300, spacing_factor=spacing_factor)
            svg_width, svg_height = 1600, 1000  # Increased height for better spacing
        else:  # hierarchical (default)
            coords = self.compute_layout_hierarchical(width=1600, height=800, spacing_factor=spacing_factor)
            svg_width, svg_height = 1800, 1000  # Increased size for better spacing

        # Improve spacing and angular separation before normalization (opt-in)
        if angle_spread or strict_component_clearance or bus_clearance > 0:
            coords = self._improve_bus_layout(
                coords,
                phs_map,
                spacing_factor=spacing_factor,
                node_size_factor=node_size_factor,
                angle_spread=angle_spread,
                strict_component_clearance=strict_component_clearance,
                bus_clearance=bus_clearance
            )

        # Normalize coordinates to fit SVG with margins - preserve aspect ratio
        if coords:
            all_x = [c[0] for c in coords.values()]
            all_y = [c[1] for c in coords.values()]
            min_x, max_x = min(all_x), max(all_x)
            min_y, max_y = min(all_y), max(all_y)

            # Generous margins for labels and components
            margin = 200
            legend_space = 120
            
            # Calculate range with minimum to avoid division by zero
            x_range = max(max_x - min_x, 100)
            y_range = max(max_y - min_y, 100)
            
            # Calculate scales but preserve aspect ratio to avoid distortion
            available_width = svg_width - 2 * margin
            available_height = svg_height - 2 * margin - legend_space
            
            scale_x = available_width / x_range
            scale_y = available_height / y_range
            
            # Use the smaller scale to preserve aspect ratio (avoid stretching)
            scale = min(scale_x, scale_y) * 0.8  # 0.8 factor for extra breathing room
            
            for bus_idx in coords:
                x, y = coords[bus_idx]
                # Center the layout in available space
                x_centered = (x - min_x - x_range/2) * scale + svg_width/2
                y_centered = (y - min_y - y_range/2) * scale + svg_height/2 - legend_space/2
                coords[bus_idx] = (x_centered, y_centered)

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

        # Reset collision tracking for each diagram
        self.occupied_regions = []
        self.strict_component_clearance = strict_component_clearance
        self.clearance_padding = 8 * spacing_factor * node_size_factor if strict_component_clearance else 0.0
        if strict_component_clearance:
            self._seed_bus_occupied_regions(coords, node_size_factor=node_size_factor, spacing_factor=spacing_factor)
        
        # Draw generators, exciters, governors with collision avoidance
        gen_positions = {}
        gen_at_bus = defaultdict(list)
        for gp in phs_map["storage_ports"]:
            gen_at_bus[gp["at_bus"]].append(gp)

        for bus_id, gens in gen_at_bus.items():
            p = coords.get(bus_id)
            if p:
                bus_requirements = self._calculate_bus_space_requirements(bus_id, phs_map)
                for i, gp in enumerate(gens):
                    gen_pos = self._draw_generator_with_avoidance(svg, p, gp, i, len(gens), coords, bus_requirements, spacing_factor, node_size_factor)
                    gen_positions[gp["gen_idx"]] = gen_pos

        # Draw exciters with collision avoidance  
        exciter_positions = {}
        for ep in phs_map["exciter_ports"]:
            if ep["syn"] in gen_positions:
                bus_pos = coords.get(ep.get("at_bus"))
                if bus_pos:
                    exciter_pos = self._draw_exciter_with_avoidance(svg, gen_positions[ep["syn"]], bus_pos, ep, spacing_factor, node_size_factor)
                    exciter_positions[ep["syn"]] = exciter_pos

        # Draw governors with collision avoidance
        for gp in phs_map["governor_ports"]:
            if gp["syn"] in gen_positions:
                bus_pos = coords.get(gp.get("at_bus"))
                if bus_pos:
                    self._draw_governor_with_avoidance(svg, gen_positions[gp["syn"]], bus_pos, gp, spacing_factor, node_size_factor)

        # Draw loads with collision avoidance
        for lp in phs_map["load_ports"]:
            p = coords.get(lp["at_bus"])
            if p:
                bus_requirements = self._calculate_bus_space_requirements(lp["at_bus"], phs_map)
                self._draw_load_with_avoidance(svg, p, lp, bus_requirements, spacing_factor, node_size_factor)

        # Draw bus bars (last for proper layering)
        for bus in self.data.get('Bus', []):
            bus_idx = bus['idx']
            if bus_idx in coords:
                self._draw_bus(svg, coords[bus_idx], bus, node_size_factor)

        svg.append('</svg>')

        with open(filename, 'w') as f:
            f.write("\n".join(svg))

        print(f"\n[COMPLETE] Auto-layout diagram generated: {filename}")
        return coords

    def draw_electrical_distance(self, filename="electrical_distance.svg", svg_width=1600, svg_height=1000,
                                 spacing_factor=1.0, node_size_factor=1.0, angle_spread=True,
                                 strict_component_clearance=False, bus_clearance=40.0,
                                 distance_scale='log', min_distance=0.02, max_distance=1.0):
        """
        Generate SVG visualization where bus positions are determined by MDS
        on the electrical distance matrix, and lines are color-coded by impedance.

        Features:
        - Bus spacing is proportional to electrical distance (impedance)
        - Line color: green (low Z) -> yellow -> red (high Z)
        - Line thickness: thick (low Z, strong link) -> thin (high Z, weak link)
        - Impedance color bar legend
        - Distance labels on every connection
        
        Args:
            spacing_factor: Multiplier for component spacing (default=1.0, try 1.5-2.0 for more spread)
            node_size_factor: Multiplier for node/component sizes (default=1.0, try 0.6-0.8 for compact)
            angle_spread: Spread incident edges around high-degree buses (default=True)
            strict_component_clearance: Enforce no overlaps between all components (default=False)
            bus_clearance: Extra spacing between buses (default=40.0)
            distance_scale: Distance scaling method - 'linear', 'log' (default), or 'sqrt'
            min_distance: Minimum distance clamp (default=0.02 for log scale, prevents buses too close)
            max_distance: Maximum distance clamp (default=1.0 for log scale, prevents buses too far)
                         Note: For 'linear' scale use larger values (0.01-0.1 min, 1.0-5.0 max)
                               For 'log' scale use 0.02-0.05 min, 0.8-1.5 max
                               For 'sqrt' scale use 0.01-0.03 min, 0.5-1.0 max
        """
        print(f"\n[ELECTRICAL DISTANCE] Computing MDS layout from impedance matrix...")
        print(f"  Distance scaling: {distance_scale} | Min: {min_distance} | Max: {max_distance}")

        # Build structure
        phs_map = self.build_phs_structure()

        # MDS layout with spacing factor and distance scaling
        coords, buses, dist_matrix = self.compute_layout_mds(
            width=svg_width, height=svg_height, spacing_factor=spacing_factor,
            scale_method=distance_scale, min_distance=min_distance, max_distance=max_distance)

        if not coords:
            print("[ERROR] No buses found.")
            return

        # Collect all direct impedances for color mapping
        all_z = []
        for line in self.data.get('Line', []):
            z_mag = np.sqrt(line['r']**2 + line['x']**2)
            all_z.append(z_mag)
        if not all_z:
            all_z = [1.0]
        z_min, z_max = min(all_z), max(all_z)
        if z_max - z_min < 1e-9:
            z_max = z_min + 1.0

        # Improve spacing and angular separation before rendering
        coords = self._improve_bus_layout(
            coords,
            phs_map,
            spacing_factor=spacing_factor,
            node_size_factor=node_size_factor,
            angle_spread=angle_spread,
            strict_component_clearance=strict_component_clearance,
            bus_clearance=bus_clearance
        )

        # Start SVG
        svg = [f'<svg width="{svg_width}" height="{svg_height}" xmlns="http://www.w3.org/2000/svg">']

        # Gradient definitions for color bar
        svg.append('<defs>')
        svg.append('<linearGradient id="zGradient" x1="0%" y1="0%" x2="100%" y2="0%">')
        svg.append('<stop offset="0%" style="stop-color:#2e7d32;stop-opacity:1" />')
        svg.append('<stop offset="50%" style="stop-color:#f9a825;stop-opacity:1" />')
        svg.append('<stop offset="100%" style="stop-color:#c62828;stop-opacity:1" />')
        svg.append('</linearGradient>')
        svg.append('<marker id="arrowhead2" markerWidth="10" markerHeight="7" refX="9" refY="3.5" orient="auto">')
        svg.append('<polygon points="0 0, 10 3.5, 0 7" fill="#666"/>')
        svg.append('</marker>')
        svg.append('</defs>')

        # Background
        svg.append('<rect width="100%" height="100%" fill="#f5f5f5"/>')

        # Title
        system_name = self.data.get('system', {}).get('name', 'Power System')
        n_buses = len(buses)
        svg.append(f'<text x="50" y="35" font-family="Arial" font-size="20" font-weight="bold" fill="#1a237e">Electrical Distance Map ({n_buses}-Bus System)</text>')
        svg.append(f'<text x="50" y="55" font-family="Arial" font-size="12" fill="#666">Bus positions from MDS on impedance matrix | Line color = impedance magnitude |Z|</text>')

        # Draw impedance color bar
        self._draw_impedance_colorbar(svg, z_min, z_max, svg_width)

        # Draw connections: lines and transformers with impedance coloring
        drawn_connections = set()
        for line in self.data.get('Line', []):
            b1, b2 = line['bus1'], line['bus2']
            p1, p2 = coords.get(b1), coords.get(b2)
            if p1 and p2:
                z_mag = np.sqrt(line['r']**2 + line['x']**2)
                color = self._impedance_to_color(z_mag, z_min, z_max)
                # Thicker lines = lower impedance (stronger connection)
                thickness = max(1.5, 6.0 - 4.5 * (z_mag - z_min) / (z_max - z_min + 1e-9))
                is_transformer = line.get('Vn1', 0) != line.get('Vn2', 0)

                conn_key = (min(b1, b2), max(b1, b2))
                is_parallel = conn_key in drawn_connections

                # Compute perpendicular offset for parallel lines
                dx = p2[0] - p1[0]
                dy = p2[1] - p1[1]
                length = np.sqrt(dx**2 + dy**2)
                if length > 0:
                    perp_x, perp_y = -dy / length, dx / length
                else:
                    perp_x, perp_y = 0, 1

                offset = 8 if is_parallel else 0

                x1 = p1[0] + perp_x * offset
                y1 = p1[1] + perp_y * offset
                x2 = p2[0] + perp_x * offset
                y2 = p2[1] + perp_y * offset

                if is_transformer:
                    # Transformer: dashed line with circles
                    mid_x = (x1 + x2) / 2
                    mid_y = (y1 + y2) / 2
                    svg.append(f'<line x1="{x1:.1f}" y1="{y1:.1f}" x2="{x2:.1f}" y2="{y2:.1f}" stroke="{color}" stroke-width="{thickness:.1f}" stroke-dasharray="8,4" opacity="0.8"/>')
                    svg.append(f'<circle cx="{mid_x-5:.1f}" cy="{mid_y-5:.1f}" r="6" fill="none" stroke="{color}" stroke-width="2"/>')
                    svg.append(f'<circle cx="{mid_x+5:.1f}" cy="{mid_y+5:.1f}" r="6" fill="none" stroke="{color}" stroke-width="2"/>')
                else:
                    # Transmission line: solid line
                    svg.append(f'<line x1="{x1:.1f}" y1="{y1:.1f}" x2="{x2:.1f}" y2="{y2:.1f}" stroke="{color}" stroke-width="{thickness:.1f}" opacity="0.8"/>')

                # Impedance label at midpoint with perpendicular offset and white background
                mid_x = (x1 + x2) / 2
                mid_y = (y1 + y2) / 2
                angle = np.degrees(np.arctan2(dy, dx))
                if angle > 90:
                    angle -= 180
                elif angle < -90:
                    angle += 180
                
                # Place label perpendicular to line for better readability
                if length > 0:
                    perp_x = -dy / length
                    perp_y = dx / length
                else:
                    perp_x, perp_y = 0, 1
                label_offset = 12 + offset  # Combine parallel and perpendicular offsets
                label_x = mid_x + perp_x * label_offset
                label_y = mid_y + perp_y * label_offset

                label = f"|Z|={z_mag:.4f}"
                svg.append(f'<text x="{label_x:.1f}" y="{label_y:.1f}" font-family="Arial" font-size="8" fill="{color}" text-anchor="middle" font-weight="bold" stroke="white" stroke-width="2.5" paint-order="stroke" transform="rotate({angle:.1f} {label_x:.1f} {label_y:.1f})">{label}</text>')

                drawn_connections.add(conn_key)

        # Reset collision tracking for each diagram
        self.occupied_regions = []
        self.strict_component_clearance = strict_component_clearance
        self.clearance_padding = 8 * spacing_factor * node_size_factor if strict_component_clearance else 0.0
        if strict_component_clearance:
            self._seed_bus_occupied_regions(coords, node_size_factor=node_size_factor, spacing_factor=spacing_factor)

        # Draw generators, exciters, governors with collision avoidance
        gen_positions = {}
        gen_at_bus = defaultdict(list)
        for gp in phs_map["storage_ports"]:
            gen_at_bus[gp["at_bus"]].append(gp)

        for bus_id, gens in gen_at_bus.items():
            p = coords.get(bus_id)
            if p:
                bus_requirements = self._calculate_bus_space_requirements(bus_id, phs_map)
                for i, gp in enumerate(gens):
                    gen_pos = self._draw_generator_with_avoidance(svg, p, gp, i, len(gens), coords, bus_requirements, spacing_factor, node_size_factor)
                    gen_positions[gp["gen_idx"]] = gen_pos

        # Draw exciters with collision avoidance  
        exciter_positions = {}
        for ep in phs_map["exciter_ports"]:
            if ep["syn"] in gen_positions:
                bus_pos = coords.get(ep.get("at_bus"))
                if bus_pos:
                    exciter_pos = self._draw_exciter_with_avoidance(svg, gen_positions[ep["syn"]], bus_pos, ep, spacing_factor, node_size_factor)
                    exciter_positions[ep["syn"]] = exciter_pos

        # Draw governors with collision avoidance
        for gp in phs_map["governor_ports"]:
            if gp["syn"] in gen_positions:
                bus_pos = coords.get(gp.get("at_bus"))
                if bus_pos:
                    self._draw_governor_with_avoidance(svg, gen_positions[gp["syn"]], bus_pos, gp, spacing_factor, node_size_factor)

        # Draw loads with collision avoidance
        for lp in phs_map["load_ports"]:
            p = coords.get(lp["at_bus"])
            if p:
                bus_requirements = self._calculate_bus_space_requirements(lp["at_bus"], phs_map)
                self._draw_load_with_avoidance(svg, p, lp, bus_requirements, spacing_factor, node_size_factor)

        # Draw bus nodes (on top)
        for bus in self.data.get('Bus', []):
            bus_idx = bus['idx']
            if bus_idx in coords:
                self._draw_bus_node(svg, coords[bus_idx], bus, node_size_factor)

        # Draw component legend at bottom
        self._draw_legend(svg, svg_height - 70, svg_width)

        svg.append('</svg>')

        with open(filename, 'w') as f:
            f.write("\n".join(svg))

        print(f"[COMPLETE] Electrical distance diagram generated: {filename}")
        return coords

    def _impedance_to_color(self, z, z_min, z_max):
        """Map impedance to a green-yellow-red color."""
        t = (z - z_min) / (z_max - z_min + 1e-9)
        t = max(0.0, min(1.0, t))

        if t < 0.5:
            # Green -> Yellow
            s = t * 2
            r = int(46 + (249 - 46) * s)
            g = int(125 + (168 - 125) * s)
            b = int(50 + (37 - 50) * s)
        else:
            # Yellow -> Red
            s = (t - 0.5) * 2
            r = int(249 + (198 - 249) * s)
            g = int(168 + (40 - 168) * s)
            b = int(37 + (40 - 37) * s)

        return f"#{r:02x}{g:02x}{b:02x}"

    def _draw_impedance_colorbar(self, svg, z_min, z_max, svg_width):
        """Draw impedance color bar at top-right."""
        bar_x = svg_width - 350
        bar_y = 20
        bar_w = 200
        bar_h = 16

        # Background
        svg.append(f'<rect x="{bar_x-10}" y="{bar_y-5}" width="{bar_w+80}" height="{bar_h+30}" fill="white" stroke="#bdbdbd" rx="4" opacity="0.9"/>')

        # Label
        svg.append(f'<text x="{bar_x}" y="{bar_y+10}" font-family="Arial" font-size="10" font-weight="bold" fill="#333">|Z| (pu):</text>')

        # Gradient bar
        cb_x = bar_x + 60
        svg.append(f'<rect x="{cb_x}" y="{bar_y}" width="{bar_w-60}" height="{bar_h}" fill="url(#zGradient)" stroke="#999" rx="2"/>')

        # Min/Max labels
        svg.append(f'<text x="{cb_x}" y="{bar_y+bar_h+12}" font-family="Arial" font-size="9" fill="#333" text-anchor="middle">{z_min:.4f}</text>')
        svg.append(f'<text x="{cb_x+bar_w-60}" y="{bar_y+bar_h+12}" font-family="Arial" font-size="9" fill="#333" text-anchor="middle">{z_max:.4f}</text>')

        # Low/High labels
        svg.append(f'<text x="{cb_x-5}" y="{bar_y+10}" font-family="Arial" font-size="8" fill="#2e7d32" text-anchor="end">Low</text>')
        svg.append(f'<text x="{cb_x+bar_w-55}" y="{bar_y+10}" font-family="Arial" font-size="8" fill="#c62828">High</text>')

    def _draw_bus_node(self, svg, pos, bus, node_size_factor=1.0):
        """Draw bus as a labeled circle node (for electrical distance layout)."""
        bus_name = str(bus['name'])
        vn = bus['Vn']

        # Color by voltage level
        if vn < 100:
            fill = "#e3f2fd"
            stroke = "#1565c0"
        else:
            fill = "#fff9c4"
            stroke = "#f57f17"

        # Circle node with size factor
        bus_radius = 16 * node_size_factor
        svg.append(f'<circle cx="{pos[0]:.1f}" cy="{pos[1]:.1f}" r="{bus_radius:.1f}" fill="{fill}" stroke="{stroke}" stroke-width="{2.5*node_size_factor:.1f}"/>')
        
        # Position bus name ABOVE the circle with white background to avoid overlap
        name_y = pos[1] - bus_radius - 5
        svg.append(f'<text x="{pos[0]:.1f}" y="{name_y:.1f}" font-family="Arial" font-size="{10*node_size_factor:.0f}" font-weight="bold" text-anchor="middle" stroke="white" stroke-width="3" paint-order="stroke" fill="{stroke}">{bus_name}</text>')
        
        # Position voltage label BELOW the circle with white background
        voltage_y = pos[1] + bus_radius + 14*node_size_factor
        svg.append(f'<text x="{pos[0]:.1f}" y="{voltage_y:.1f}" font-family="Arial" font-size="{8*node_size_factor:.0f}" fill="#757575" text-anchor="middle" stroke="white" stroke-width="3" paint-order="stroke">{vn:.0f}kV</text>')

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

        # Rotate label to follow line (keep text readable)
        angle = np.degrees(np.arctan2(dy, dx))
        if angle > 90:
            angle -= 180
        elif angle < -90:
            angle += 180
            
        # Place label perpendicular to line to minimize overlap
        # Use perpendicular offset instead of parallel offset
        perp_offset = 12  # Perpendicular distance from line
        if length > 0:
            perp_x = -dy / length  # Perpendicular direction
            perp_y = dx / length
        else:
            perp_x, perp_y = 0, 1
            
        # Position label above line (perpendicular offset)
        label_x = mid_x + perp_x * perp_offset
        label_y = mid_y + perp_y * perp_offset
        
        # Add white background to label for readability
        svg.append(f'<text x="{label_x:.1f}" y="{label_y:.1f}" font-family="Arial" font-size="9" fill="#616161" text-anchor="middle" stroke="white" stroke-width="3" paint-order="stroke" transform="rotate({angle:.1f} {label_x:.1f} {label_y:.1f})">{label}</text>')

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

        # Label with white background for readability
        # Perpendicular offset for better placement
        if length > 0:
            perp_x = -dy / length
            perp_y = dx / length
        else:
            perp_x, perp_y = 0, 1
        label_x = mid_x + perp_y * 22  # Offset perpendicular to line
        label_y = mid_y - perp_x * 22
        svg.append(f'<text x="{label_x:.1f}" y="{label_y:.1f}" font-family="Arial" font-size="9" fill="#7b1fa2" text-anchor="middle" stroke="white" stroke-width="3" paint-order="stroke">X={sp["x"]:.3f}</text>')

    def _draw_generator_with_avoidance(self, svg, bus_pos, gp, index, total, all_coords, bus_requirements, spacing_factor=1.0, node_size_factor=1.0):
        """Draw generator with collision avoidance and return its position."""
        # Determine placement direction (away from center of mass)
        center_x = np.mean([c[0] for c in all_coords.values()])
        
        # Calculate base position with spacing factor
        base_offset_x = 90 * spacing_factor  # Scale horizontal offset
        if bus_pos[0] < center_x:
            base_gx = bus_pos[0] - base_offset_x
        else:
            base_gx = bus_pos[0] + base_offset_x

        # Improved vertical spacing for multiple generators
        if total == 1:
            vert_offset = 0
        else:
            spacing = max(80, 40 + total * 10) * spacing_factor  # Scale vertical spacing
            vert_offset = (index - total/2 + 0.5) * spacing
        
        base_gy = bus_pos[1] + vert_offset
        
        # Find non-overlapping position (collision region based on size factor)
        gen_radius = 30 * node_size_factor
        gen_pos = self._find_non_overlapping_position(
            (base_gx, base_gy), gen_radius * 2, gen_radius * 2, self.occupied_regions)
        gx, gy = gen_pos
        
        # Register occupied region with size factor
        self.occupied_regions.append((gx - gen_radius, gy - gen_radius, gx + gen_radius, gy + gen_radius))

        # Connection line
        svg.append(f'<line x1="{bus_pos[0]:.1f}" y1="{bus_pos[1]:.1f}" x2="{gx:.1f}" y2="{gy:.1f}" stroke="#1976d2" stroke-width="{2*node_size_factor:.1f}"/>')

        # Generator circle with size factor
        gen_circle_radius = 20 * node_size_factor
        svg.append(f'<circle cx="{gx:.1f}" cy="{gy:.1f}" r="{gen_circle_radius:.1f}" fill="#e3f2fd" stroke="#1976d2" stroke-width="{3*node_size_factor:.1f}"/>')
        # Position generator ID INSIDE the circle (centered)
        svg.append(f'<text x="{gx:.1f}" y="{gy+5*node_size_factor:.1f}" font-family="Arial" font-size="{12*node_size_factor:.0f}" font-weight="bold" fill="#1976d2" text-anchor="middle">{gp["id"]}</text>')

        # Label positioned BELOW the generator to avoid overlap
        label_y_offset = 30 * node_size_factor
        label_font_size = 8 * node_size_factor
        svg.append(f'<text x="{gx:.1f}" y="{gy+label_y_offset:.1f}" font-family="Arial" font-size="{label_font_size:.0f}" fill="#1565c0" text-anchor="middle">M={gp["M"]}</text>')
        svg.append(f'<text x="{gx:.1f}" y="{gy+label_y_offset+10*node_size_factor:.1f}" font-family="Arial" font-size="{label_font_size:.0f}" fill="#1565c0" text-anchor="middle">{gp["Sn"]:.0f}MVA</text>')
        
        # Register label region below generator
        label_region_width = 30 * node_size_factor
        self.occupied_regions.append((gx - label_region_width, gy + 20*node_size_factor, gx + label_region_width, gy + 50*node_size_factor))

        return (gx, gy)

    def _draw_exciter_with_avoidance(self, svg, gen_pos, bus_pos, ep, spacing_factor=1.0, node_size_factor=1.0):
        """Draw exciter connected to generator with collision avoidance."""
        gx, gy = gen_pos
        bx, by = bus_pos
        model = ep.get('model', 'AVR')

        # Place exciter at +120 degrees from bus direction for 3-way separation
        base_angle = np.arctan2(by - gy, bx - gx)
        exc_angle = base_angle + (2 * np.pi / 3)

        exciter_offset = 55 * spacing_factor
        base_ex = gx + exciter_offset * np.cos(exc_angle)
        base_ey = gy + exciter_offset * np.sin(exc_angle)
        exciter_width = 40 * node_size_factor
        exciter_height = 25 * node_size_factor
        exciter_pos = self._find_non_overlapping_position(
            (base_ex, base_ey), exciter_width, exciter_height, self.occupied_regions)
        ex, ey = exciter_pos
        
        # Register occupied region with size factor
        self.occupied_regions.append((ex - exciter_width/2, ey - exciter_height/2, ex + exciter_width/2, ey + exciter_height/2))

        # Calculate connection point on generator circle edge (dynamic based on exciter position)
        gen_radius = 20 * node_size_factor
        angle_to_exciter = np.arctan2(ey - gy, ex - gx)
        gen_conn_x = gx + gen_radius * np.cos(angle_to_exciter)
        gen_conn_y = gy + gen_radius * np.sin(angle_to_exciter)
        
        svg.append(f'<line x1="{gen_conn_x:.1f}" y1="{gen_conn_y:.1f}" x2="{ex:.1f}" y2="{ey:.1f}" stroke="#ff6f00" stroke-width="{1.5*node_size_factor:.1f}" stroke-dasharray="3,2"/>')
        svg.append(f'<rect x="{ex-18*node_size_factor:.1f}" y="{ey-10*node_size_factor:.1f}" width="{36*node_size_factor:.1f}" height="{20*node_size_factor:.1f}" fill="#fff3e0" stroke="#ff6f00" stroke-width="{2*node_size_factor:.1f}" rx="3"/>')
        # Position model text slightly ABOVE center to avoid line overlap
        svg.append(f'<text x="{ex:.1f}" y="{ey+2*node_size_factor:.1f}" font-family="Arial" font-size="{7*node_size_factor:.0f}" font-weight="bold" fill="#e65100" text-anchor="middle">{model}</text>')
        
        return (ex, ey)

    def _draw_governor_with_avoidance(self, svg, gen_pos, bus_pos, gp, spacing_factor=1.0, node_size_factor=1.0):
        """Draw governor connected to generator with collision avoidance."""
        gx, gy = gen_pos
        bx, by = bus_pos
        model = gp.get('model', 'GOV')

        # Place governor at -120 degrees from bus direction for 3-way separation
        base_angle = np.arctan2(by - gy, bx - gx)
        gov_angle = base_angle - (2 * np.pi / 3)

        governor_offset = 55 * spacing_factor
        base_govx = gx + governor_offset * np.cos(gov_angle)
        base_govy = gy + governor_offset * np.sin(gov_angle)

        governor_width = 40 * node_size_factor
        governor_height = 25 * node_size_factor
        governor_pos = self._find_non_overlapping_position(
            (base_govx, base_govy), governor_width, governor_height, self.occupied_regions)
        govx, govy = governor_pos
        
        # Register occupied region with size factor
        self.occupied_regions.append((govx - governor_width/2, govy - governor_height/2, govx + governor_width/2, govy + governor_height/2))

        # Calculate connection point on generator circle edge (dynamic based on governor position)
        gen_radius = 20 * node_size_factor
        angle_to_governor = np.arctan2(govy - gy, govx - gx)
        gen_conn_x = gx + gen_radius * np.cos(angle_to_governor)
        gen_conn_y = gy + gen_radius * np.sin(angle_to_governor)
        
        svg.append(f'<line x1="{gen_conn_x:.1f}" y1="{gen_conn_y:.1f}" x2="{govx:.1f}" y2="{govy:.1f}" stroke="#2e7d32" stroke-width="{1.5*node_size_factor:.1f}" stroke-dasharray="3,2"/>')
        svg.append(f'<rect x="{govx-18*node_size_factor:.1f}" y="{govy-10*node_size_factor:.1f}" width="{36*node_size_factor:.1f}" height="{20*node_size_factor:.1f}" fill="#e8f5e9" stroke="#2e7d32" stroke-width="{2*node_size_factor:.1f}" rx="3"/>')
        # Position model text slightly ABOVE center to avoid line overlap
        svg.append(f'<text x="{govx:.1f}" y="{govy+2*node_size_factor:.1f}" font-family="Arial" font-size="{7*node_size_factor:.0f}" font-weight="bold" fill="#1b5e20" text-anchor="middle">{model}</text>')

    def _draw_load_with_avoidance(self, svg, bus_pos, lp, bus_requirements, spacing_factor=1.0, node_size_factor=1.0):
        """Draw load at bus with collision avoidance."""
        # Calculate load position avoiding generators with spacing factor
        num_generators = len(bus_requirements['generators'])
        base_offset = (60 if num_generators == 0 else 80 + num_generators * 20) * spacing_factor
        
        base_lx = bus_pos[0]
        base_ly = bus_pos[1] + base_offset
        
        # Find non-overlapping position with size factor
        load_width = 30 * node_size_factor
        load_height = 40 * node_size_factor
        load_pos = self._find_non_overlapping_position(
            (base_lx, base_ly), load_width, load_height, self.occupied_regions)
        lx, ly = load_pos
        
        # Register occupied region with size factor
        self.occupied_regions.append((lx - load_width/2, ly - load_height/2, lx + load_width/2, ly + load_height/2))

        svg.append(f'<line x1="{bus_pos[0]:.1f}" y1="{bus_pos[1]:.1f}" x2="{lx:.1f}" y2="{ly-12*node_size_factor:.1f}" stroke="#d32f2f" stroke-width="{2*node_size_factor:.1f}"/>')
        svg.append(f'<polygon points="{lx:.1f},{ly+12*node_size_factor:.1f} {lx-12*node_size_factor:.1f},{ly-8*node_size_factor:.1f} {lx+12*node_size_factor:.1f},{ly-8*node_size_factor:.1f}" fill="#ffebee" stroke="#d32f2f" stroke-width="{2*node_size_factor:.1f}"/>')
        # Position load label BELOW the load triangle with white background
        label_y = ly + 24*node_size_factor
        svg.append(f'<text x="{lx:.1f}" y="{label_y:.1f}" font-family="Arial" font-size="{8*node_size_factor:.0f}" fill="#c62828" text-anchor="middle" stroke="white" stroke-width="3" paint-order="stroke">P={lp["p0"]:.2f}</text>')

    def _draw_bus(self, svg, pos, bus, node_size_factor=1.0):
        """Draw bus bar with clear, non-overlapping labels."""
        bus_name = str(bus['name'])
        vn = bus['Vn']

        # Color by voltage level
        bar_color = "#1565c0" if vn < 100 else "#212121"

        bar_width = 8 * node_size_factor
        bar_height = 50 * node_size_factor
        svg.append(f'<rect x="{pos[0]-bar_width/2:.1f}" y="{pos[1]-bar_height/2:.1f}" width="{bar_width:.1f}" height="{bar_height:.1f}" fill="{bar_color}" rx="2"/>')
        
        # Bus name above the bar with white background
        name_y = pos[1] - bar_height/2 - 7
        svg.append(f'<text x="{pos[0]:.1f}" y="{name_y:.1f}" font-family="Arial" font-size="{11*node_size_factor:.0f}" font-weight="bold" text-anchor="middle" stroke="white" stroke-width="3" paint-order="stroke" fill="{bar_color}">{bus_name}</text>')
        
        # Voltage label below the bar with white background
        voltage_y = pos[1] + bar_height/2 + 15*node_size_factor
        svg.append(f'<text x="{pos[0]:.1f}" y="{voltage_y:.1f}" font-family="Arial" font-size="{9*node_size_factor:.0f}" fill="#757575" text-anchor="middle" stroke="white" stroke-width="3" paint-order="stroke">{vn:.0f}kV</text>')
