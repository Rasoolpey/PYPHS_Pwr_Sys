"""
3D visualization for three-phase power grid graph with coupling.
"""

import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.patches import FancyBboxPatch
from matplotlib.lines import Line2D
import networkx as nx
from typing import Dict, Optional, List, Tuple
import sys
import os

# Add parent directory to path for imports
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.graph_base import PowerGridGraph, PhaseType


class ThreePhaseGraphPlotter:
    """Visualize three-phase power grid graph"""
    
    def __init__(self, graph: PowerGridGraph):
        """
        Initialize plotter with graph.
        
        Args:
            graph: PowerGridGraph object
        """
        self.graph = graph
        self.pos = self._compute_layout()
        self.phase_colors = {
            PhaseType.A: '#FF1744',  # Bright Red - Phase A
            PhaseType.B: '#00E676',  # Bright Green - Phase B  
            PhaseType.C: '#2196F3'   # Bright Blue - Phase C
        }
        self.phase_z = {
            PhaseType.A: 2.0,
            PhaseType.B: 1.0,
            PhaseType.C: 0.0
        }
    
    def _compute_layout(self) -> Dict[str, np.ndarray]:
        """Compute 2D layout for nodes"""
        # Create networkx graph for layout
        G = nx.Graph()
        for node_id in self.graph.nodes:
            G.add_node(node_id)
        
        for edge_id, phases in self.graph.edges.items():
            edge = phases[PhaseType.A]  # Use any phase for topology
            G.add_edge(edge.from_node_id, edge.to_node_id)
        
        # Use spring layout
        pos = nx.spring_layout(G, k=2, iterations=50, seed=42)
        
        # Convert to numpy arrays
        pos_array = {}
        for node, (x, y) in pos.items():
            pos_array[node] = np.array([x, y])
        
        return pos_array
    
    def plot_3d_coupled_graph(self, show_coupling: bool = True, 
                             coupling_threshold: float = 0.1) -> Tuple[plt.Figure, plt.Axes]:
        """
        Plot 3D visualization of three-phase graph with coupling.
        
        Args:
            show_coupling: Show inter-phase coupling lines
            coupling_threshold: Minimum coupling strength to show
        
        Returns:
            (figure, axes)
        """
        fig = plt.figure(figsize=(14, 10))
        ax = fig.add_subplot(111, projection='3d')
        
        # Plot nodes for each phase
        for phase in PhaseType:
            self._plot_phase_nodes_3d(ax, phase)
        
        # Plot edges for each phase
        for phase in PhaseType:
            self._plot_phase_edges_3d(ax, phase)
        
        # Plot coupling connections
        if show_coupling:
            self._plot_coupling_connections_3d(ax, coupling_threshold)
        
        # Set labels and title
        ax.set_xlabel('X Position')
        ax.set_ylabel('Y Position')
        ax.set_zlabel('Phase')
        ax.set_title('Three-Phase Power Grid with Coupling', fontsize=16, fontweight='bold')
        
        # Set z-axis labels
        ax.set_zticks([0, 1, 2])
        ax.set_zticklabels(['Phase C', 'Phase B', 'Phase A'])
        
        # Add legend
        legend_elements = [
            Line2D([0], [0], marker='o', color='w', markerfacecolor='#ff6b6b', 
                   markersize=10, label='Phase A'),
            Line2D([0], [0], marker='o', color='w', markerfacecolor='#4ecdc4', 
                   markersize=10, label='Phase B'),
            Line2D([0], [0], marker='o', color='w', markerfacecolor='#45b7d1', 
                   markersize=10, label='Phase C'),
            Line2D([0], [0], marker='s', color='w', markerfacecolor='gray', 
                   markersize=10, label='Generator'),
            Line2D([0], [0], marker='o', color='w', markerfacecolor='gray', 
                   markersize=10, label='Load'),
            Line2D([0], [0], marker='^', color='w', markerfacecolor='gray', 
                   markersize=10, label='Bus')
        ]
        
        if show_coupling:
            legend_elements.append(
                Line2D([0], [0], color='purple', linestyle=':', linewidth=2, 
                       label='Coupling')
            )
        
        ax.legend(handles=legend_elements, loc='upper right')
        
        # Adjust viewing angle
        ax.view_init(elev=20, azim=45)
        
        return fig, ax
    
    def _plot_phase_nodes_3d(self, ax, phase: PhaseType):
        """Plot nodes for a specific phase in 3D"""
        z = self.phase_z[phase]
        color = self.phase_colors[phase]
        
        for node_id, phases in self.graph.nodes.items():
            node = phases[phase]
            x, y = self.pos[node_id]
            
            # Determine marker based on node type
            if node.node_type == 'generator':
                marker = 's'  # Square
                size = 100
            elif node.node_type == 'load':
                marker = 'o'  # Circle
                size = 80
            else:
                marker = '^'  # Triangle
                size = 60
            
            # Plot node
            ax.scatter(x, y, z, c=color, marker=marker, s=size, 
                      edgecolors='black', linewidths=1.5, alpha=0.8)
    
    def _plot_phase_edges_3d(self, ax, phase: PhaseType):
        """Plot edges for a specific phase in 3D"""
        z = self.phase_z[phase]
        color = self.phase_colors[phase]
        
        for edge_id, phases in self.graph.edges.items():
            edge = phases[phase]
            
            # Get positions
            x1, y1 = self.pos[edge.from_node_id]
            x2, y2 = self.pos[edge.to_node_id]
            
            # Draw edge
            ax.plot([x1, x2], [y1, y2], [z, z], 
                   color=color, linewidth=2, alpha=0.6)
    
    def _plot_coupling_connections_3d(self, ax, threshold: float):
        """Plot coupling connections between phases"""
        # Node couplings
        for node_id, coupling_matrix in self.graph.node_couplings.items():
            x, y = self.pos[node_id]
            matrix = coupling_matrix.matrix
            
            # Plot coupling between phases
            for i, phase_i in enumerate(PhaseType):
                for j, phase_j in enumerate(PhaseType):
                    if i < j:  # Avoid duplicates
                        coupling_strength = abs(matrix[i, j])
                        if coupling_strength > threshold:
                            z1 = self.phase_z[phase_i]
                            z2 = self.phase_z[phase_j]
                            
                            # Draw vertical coupling line
                            ax.plot([x, x], [y, y], [z1, z2],
                                   color='purple', linestyle=':', 
                                   linewidth=1 + 3 * coupling_strength,
                                   alpha=0.5)
        
        # Edge couplings (show at midpoint)
        for edge_id, coupling_matrix in self.graph.edge_couplings.items():
            edge = self.graph.edges[edge_id][PhaseType.A]
            
            # Get midpoint
            x1, y1 = self.pos[edge.from_node_id]
            x2, y2 = self.pos[edge.to_node_id]
            x_mid = (x1 + x2) / 2
            y_mid = (y1 + y2) / 2
            
            matrix = coupling_matrix.matrix
            
            # Plot coupling at midpoint
            for i, phase_i in enumerate(PhaseType):
                for j, phase_j in enumerate(PhaseType):
                    if i < j:
                        coupling_strength = abs(matrix[i, j])
                        if coupling_strength > threshold:
                            z1 = self.phase_z[phase_i]
                            z2 = self.phase_z[phase_j]
                            
                            ax.plot([x_mid, x_mid], [y_mid, y_mid], [z1, z2],
                                   color='orange', linestyle='--',
                                   linewidth=1 + 3 * coupling_strength,
                                   alpha=0.5)
    
    def _compute_spectral_layout(self) -> Dict[str, np.ndarray]:
        """Compute spectral embedding layout using graph Laplacian eigenvectors"""
        # Create networkx graph for spectral embedding
        G = nx.Graph()
        for node_id in self.graph.nodes:
            G.add_node(node_id)
        
        for edge_id, phases in self.graph.edges.items():
            edge = phases[PhaseType.A]  # Use any phase for topology
            G.add_edge(edge.from_node_id, edge.to_node_id)
        
        # Compute spectral layout (uses eigenvectors of Laplacian)
        try:
            # spectral_layout uses the 2nd and 3rd smallest eigenvectors
            pos = nx.spectral_layout(G, dim=2, weight=None)
        except:
            # Fallback to spring layout if spectral fails (disconnected components)
            pos = nx.spring_layout(G, k=2, iterations=50, seed=42)
        
        # Convert to numpy arrays and scale
        pos_array = {}
        scale_factor = 5.0  # Scale to make visualization clearer
        for node, (x, z) in pos.items():
            pos_array[node] = np.array([x * scale_factor, z * scale_factor])
        
        return pos_array
    
    def plot_spectral_3d_graph(self, coupling_color_nodes: bool = True, 
                              coupling_color_edges: bool = True,
                              show_node_labels: bool = False,
                              view_elevation: float = 35,
                              view_azimuth: float = -45) -> Tuple[plt.Figure, plt.Axes]:
        """
        Plot 3D spectral visualization with each phase on separate Y-plane.
        
        Uses spectral embedding (Laplacian eigenvectors) for X-Z coordinates,
        and separates phases along Y-axis for clean 3D perspective.
        
        Args:
            coupling_color_nodes: Color nodes by coupling strength
            coupling_color_edges: Color edges by coupling strength
            show_node_labels: Show node names as text labels
            view_elevation: Elevation angle for 3D view (-90 to 90 degrees)
            view_azimuth: Azimuth angle for 3D view (0 to 360 degrees)
            
        Returns:
            (figure, axes)
        """
        fig = plt.figure(figsize=(16, 12))
        ax = fig.add_subplot(111, projection='3d')
        
        # Compute spectral layout
        spectral_pos = self._compute_spectral_layout()
        
        # Phase Y-positions (separated along Y-axis)
        phase_y_positions = {
            PhaseType.A: 4.0,   # Phase A at Y=4
            PhaseType.B: 2.0,   # Phase B at Y=2  
            PhaseType.C: 0.0    # Phase C at Y=0
        }
        
        # Plot edges first (so they appear behind nodes)
        for phase in PhaseType:
            self._plot_spectral_phase_edges_3d(ax, phase, spectral_pos, 
                                             phase_y_positions[phase], 
                                             coupling_color_edges)
        
        # Plot nodes
        for phase in PhaseType:
            self._plot_spectral_phase_nodes_3d(ax, phase, spectral_pos, 
                                             phase_y_positions[phase],
                                             coupling_color_nodes, show_node_labels)
        
        # Set labels and styling
        ax.set_xlabel('X Position (Spectral)', fontsize=12)
        ax.set_ylabel('Phase Separation', fontsize=12)
        ax.set_zlabel('Z Position (Spectral)', fontsize=12)
        ax.set_title('Three-Phase Power Grid - Spectral 3D Layout', 
                    fontsize=16, fontweight='bold')
        
        # Set Y-axis labels for phases
        ax.set_yticks([0, 2, 4])
        ax.set_yticklabels(['Phase C', 'Phase B', 'Phase A'])
        
        # Create legend
        legend_elements = [
            Line2D([0], [0], marker='o', color='w', markerfacecolor='#FF1744', 
                   markersize=12, label='Phase A', markeredgecolor='black'),
            Line2D([0], [0], marker='o', color='w', markerfacecolor='#00E676', 
                   markersize=12, label='Phase B', markeredgecolor='black'),
            Line2D([0], [0], marker='o', color='w', markerfacecolor='#2196F3', 
                   markersize=12, label='Phase C', markeredgecolor='black'),
            Line2D([0], [0], marker='s', color='w', markerfacecolor='gray', 
                   markersize=10, label='Generator', markeredgecolor='black'),
            Line2D([0], [0], marker='o', color='w', markerfacecolor='gray', 
                   markersize=10, label='Load', markeredgecolor='black'),
            Line2D([0], [0], marker='^', color='w', markerfacecolor='gray', 
                   markersize=10, label='Bus', markeredgecolor='black')
        ]
        
        ax.legend(handles=legend_elements, loc='upper left', fontsize=10)
        
        # Set viewing angle for best perspective
        ax.view_init(elev=view_elevation, azim=view_azimuth)
        
        # Improve grid and styling
        ax.grid(True, alpha=0.3)
        ax.xaxis.pane.fill = False
        ax.yaxis.pane.fill = False
        ax.zaxis.pane.fill = False
        
        return fig, ax
    
    def plot_spectral_3d_multiple_views(self, coupling_color_nodes: bool = True, 
                                       coupling_color_edges: bool = True) -> Dict[str, Tuple[plt.Figure, plt.Axes]]:
        """
        Generate spectral 3D plots from multiple viewing angles.
        
        Returns a dictionary with different view presets.
        """
        views = {
            'default': {'elev': 15, 'azim': -45, 'description': 'Default angled view'},
            'side_view': {'elev': 0, 'azim': 0, 'description': 'Side view (XZ plane)'},
            'top_view': {'elev': 90, 'azim': 0, 'description': 'Top view (XY plane)'},
            'front_view': {'elev': 0, 'azim': -90, 'description': 'Front view (YZ plane)'},
            'isometric': {'elev': 35, 'azim': -45, 'description': 'Isometric view'},
            'phase_focus': {'elev': 10, 'azim': 0, 'description': 'Focus on phase separation'},
            'angled_up': {'elev': 30, 'azim': -60, 'description': 'Angled view from above'},
            'low_angle': {'elev': 5, 'azim': -30, 'description': 'Low angle dramatic view'}
        }
        
        figures = {}
        
        for view_name, view_params in views.items():
            fig, ax = self.plot_spectral_3d_graph(
                coupling_color_nodes=coupling_color_nodes,
                coupling_color_edges=coupling_color_edges,
                show_node_labels=False,
                view_elevation=view_params['elev'],
                view_azimuth=view_params['azim']
            )
            
            # Update title to include view description
            ax.set_title(f'Three-Phase Power Grid - {view_params["description"]}', 
                        fontsize=16, fontweight='bold')
            
            figures[view_name] = (fig, ax)
        
        return figures
    
    def save_multiple_view_plots(self, output_dir: str = './plots'):
        """Save spectral 3D plots from multiple viewing angles"""
        import os
        os.makedirs(output_dir, exist_ok=True)
        
        figures = self.plot_spectral_3d_multiple_views()
        
        for view_name, (fig, ax) in figures.items():
            filename = f'{output_dir}/spectral_3d_{view_name}.png'
            fig.savefig(filename, dpi=300, bbox_inches='tight')
            plt.close(fig)
            print(f"   ✓ Saved {view_name} view")
        
        print(f"✓ All spectral view plots saved to {output_dir}/")
    
    def _plot_spectral_phase_nodes_3d(self, ax, phase: PhaseType, spectral_pos: Dict, 
                                     y_pos: float, coupling_coloring: bool, 
                                     show_labels: bool):
        """Plot nodes for a specific phase in spectral 3D layout"""
        color = self.phase_colors[phase]
        
        for node_id, phases in self.graph.nodes.items():
            if node_id not in spectral_pos:
                continue
                
            node = phases[phase]
            x, z = spectral_pos[node_id]
            y = y_pos
            
            # Determine marker and base size based on node type
            if node.node_type == 'generator':
                marker = 's'  # Square
                base_size = 120
                edge_color = 'darkred'
            elif node.node_type == 'load':
                marker = 'o'  # Circle
                base_size = 100
                edge_color = 'darkblue'
            else:  # bus
                marker = '^'  # Triangle
                base_size = 80
                edge_color = 'darkgreen'
            
            # Modify size based on voltage if available
            if hasattr(node, 'voltage_pu'):
                voltage_factor = abs(node.voltage_pu)
                size = base_size * (0.5 + voltage_factor)  # Size scales with voltage
            else:
                size = base_size
            
            # Color modification based on coupling if enabled
            node_color = color
            alpha = 0.8
            
            if coupling_coloring and node_id in self.graph.node_couplings:
                coupling_matrix = self.graph.node_couplings[node_id].matrix
                # Average off-diagonal coupling strength
                off_diag_sum = (abs(coupling_matrix[0,1]) + abs(coupling_matrix[0,2]) + 
                               abs(coupling_matrix[1,2])) / 3
                
                if off_diag_sum > 0.01:  # If significant coupling exists
                    # Intensity based on coupling strength
                    intensity = min(off_diag_sum * 2, 1.0)
                    # Blend with orange for high coupling
                    r, g, b = matplotlib.colors.to_rgb(color)
                    orange_r, orange_g, orange_b = matplotlib.colors.to_rgb('orange')
                    node_color = (r * (1-intensity) + orange_r * intensity,
                                 g * (1-intensity) + orange_g * intensity,
                                 b * (1-intensity) + orange_b * intensity)
                    alpha = 0.9
            
            # Plot the node
            ax.scatter(x, y, z, c=[node_color], marker=marker, s=size, 
                      edgecolors=edge_color, linewidths=1.5, alpha=alpha)
            
            # Add label if requested
            if show_labels:
                ax.text(x, y, z, node_id, fontsize=8, ha='center', va='bottom')
    
    def _plot_spectral_phase_edges_3d(self, ax, phase: PhaseType, spectral_pos: Dict,
                                     y_pos: float, coupling_coloring: bool):
        """Plot edges for a specific phase in spectral 3D layout"""
        base_color = self.phase_colors[phase]
        
        for edge_id, phases in self.graph.edges.items():
            edge = phases[phase]
            
            # Check if both nodes exist in spectral position
            if (edge.from_node_id not in spectral_pos or 
                edge.to_node_id not in spectral_pos):
                continue
            
            # Get positions
            x1, z1 = spectral_pos[edge.from_node_id]
            x2, z2 = spectral_pos[edge.to_node_id]
            y1 = y2 = y_pos
            
            # Determine line properties based on edge type
            if edge.edge_type == 'line':
                linestyle = '-'
                base_width = 2
            elif edge.edge_type == 'transformer':
                linestyle = '--'
                base_width = 3
            else:
                linestyle = ':'
                base_width = 1
            
            # Color and width modification for better phase differentiation
            edge_color = base_color
            linewidth = base_width * 1.0  # Slimmer lines for cleaner look
            alpha = 0.9  # Higher alpha for more vibrant colors
            
            if coupling_coloring and edge_id in self.graph.edge_couplings:
                coupling_matrix = self.graph.edge_couplings[edge_id].matrix
                # Average off-diagonal coupling strength
                off_diag_sum = (abs(coupling_matrix[0,1]) + abs(coupling_matrix[0,2]) + 
                               abs(coupling_matrix[1,2])) / 3
                
                if off_diag_sum > 0.01:  # If significant coupling exists
                    # For high coupling, make lines slightly thicker but keep them slim
                    linewidth = base_width * 1.3
                    alpha = 1.0  # Full opacity for coupled lines
                    # Add a subtle glow effect by keeping the phase color but making it brighter
                    r, g, b = matplotlib.colors.to_rgb(base_color)
                    # Brighten the color slightly for coupled edges
                    edge_color = (min(r * 1.2, 1.0), min(g * 1.2, 1.0), min(b * 1.2, 1.0))
            
            # Draw the edge
            ax.plot([x1, x2], [y1, y2], [z1, z2],
                   color=edge_color, linestyle=linestyle, 
                   linewidth=linewidth, alpha=alpha)
    
    def plot_single_phase(self, phase: PhaseType) -> Tuple[plt.Figure, plt.Axes]:
        """Plot single phase in 2D"""
        fig, ax = plt.subplots(figsize=(10, 8))
        
        # Draw edges
        for edge_id, phases in self.graph.edges.items():
            edge = phases[phase]
            x1, y1 = self.pos[edge.from_node_id]
            x2, y2 = self.pos[edge.to_node_id]
            
            # Color by edge type
            if edge.edge_type == 'line':
                color = 'blue'
                style = '-'
            else:
                color = 'green'
                style = '--'
            
            ax.plot([x1, x2], [y1, y2], color=color, linestyle=style, 
                   linewidth=2, alpha=0.6)
        
        # Draw nodes
        for node_id, phases in self.graph.nodes.items():
            node = phases[phase]
            x, y = self.pos[node_id]
            
            # Determine appearance
            if node.node_type == 'generator':
                marker = 's'
                color = '#ff6b6b'
                size = 200
            elif node.node_type == 'load':
                marker = 'o'
                color = '#4ecdc4'
                size = 150
            else:
                marker = '^'
                color = '#95e1d3'
                size = 100
            
            ax.scatter(x, y, marker=marker, c=color, s=size,
                      edgecolors='black', linewidths=2, zorder=5)
            
            # Add labels
            ax.text(x, y, node_id, fontsize=8, ha='center', va='center')
        
        ax.set_title(f'Phase {phase.value.upper()} Network', fontsize=14, fontweight='bold')
        ax.set_xlabel('X Position')
        ax.set_ylabel('Y Position')
        ax.grid(True, alpha=0.3)
        
        return fig, ax
    
    def plot_voltage_profile(self, phase: PhaseType) -> Tuple[plt.Figure, plt.Axes]:
        """Plot voltage magnitude profile"""
        fig, ax = plt.subplots(figsize=(10, 6))
        
        node_names = []
        voltages = []
        colors = []
        
        for node_id, phases in self.graph.nodes.items():
            node = phases[phase]
            node_names.append(node_id)
            voltages.append(abs(node.voltage_pu))
            
            # Color by node type
            if node.node_type == 'generator':
                colors.append('#ff6b6b')
            elif node.node_type == 'load':
                colors.append('#4ecdc4')
            else:
                colors.append('#95e1d3')
        
        # Create bar plot
        x_pos = np.arange(len(node_names))
        bars = ax.bar(x_pos, voltages, color=colors, edgecolor='black', linewidth=1)
        
        # Add reference lines
        ax.axhline(y=1.0, color='green', linestyle='--', label='Nominal (1.0 pu)')
        ax.axhline(y=0.95, color='orange', linestyle='--', label='Lower limit (0.95 pu)')
        ax.axhline(y=1.05, color='orange', linestyle='--', label='Upper limit (1.05 pu)')
        
        # Labels
        ax.set_xlabel('Bus')
        ax.set_ylabel('Voltage Magnitude (pu)')
        ax.set_title(f'Voltage Profile - Phase {phase.value.upper()}', 
                    fontsize=14, fontweight='bold')
        ax.set_xticks(x_pos)
        ax.set_xticklabels(node_names, rotation=45, ha='right')
        ax.legend()
        ax.grid(True, axis='y', alpha=0.3)
        
        return fig, ax
    
    def plot_coupling_strength(self) -> Tuple[plt.Figure, plt.Axes]:
        """Plot coupling strength heatmap"""
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))
        
        # Node coupling heatmap
        n_nodes = len(self.graph.nodes)
        node_coupling_avg = np.zeros((n_nodes, n_nodes))
        node_list = list(self.graph.nodes.keys())
        
        for i, node_id in enumerate(node_list):
            if node_id in self.graph.node_couplings:
                matrix = self.graph.node_couplings[node_id].matrix
                # Average off-diagonal coupling
                coupling_strength = (abs(matrix[0,1]) + abs(matrix[0,2]) + abs(matrix[1,2])) / 3
                node_coupling_avg[i, i] = coupling_strength
        
        im1 = ax1.imshow(node_coupling_avg, cmap='hot', aspect='auto')
        ax1.set_title('Node Coupling Strength')
        ax1.set_xlabel('Node Index')
        ax1.set_ylabel('Node Index')
        fig.colorbar(im1, ax=ax1)
        
        # Edge coupling strength
        edge_list = list(self.graph.edges.keys())
        edge_coupling_strengths = []
        
        for edge_id in edge_list:
            if edge_id in self.graph.edge_couplings:
                matrix = self.graph.edge_couplings[edge_id].matrix
                # Average off-diagonal coupling
                coupling_strength = (abs(matrix[0,1]) + abs(matrix[0,2]) + abs(matrix[1,2])) / 3
                edge_coupling_strengths.append(coupling_strength)
            else:
                edge_coupling_strengths.append(0.0)
        
        # Bar plot for edge coupling
        x_pos = np.arange(len(edge_list))
        ax2.bar(x_pos, edge_coupling_strengths, color='orange', edgecolor='black')
        ax2.set_xlabel('Edge')
        ax2.set_ylabel('Average Coupling Strength')
        ax2.set_title('Edge Coupling Strength')
        ax2.set_xticks(x_pos)
        ax2.set_xticklabels([f"E{i}" for i in range(len(edge_list))], rotation=45)
        
        plt.suptitle('Inter-Phase Coupling Analysis', fontsize=16, fontweight='bold')
        plt.tight_layout()
        
        return fig, (ax1, ax2)
    
    def plot_three_phase_overview(self) -> Tuple[plt.Figure, plt.Axes]:
        """Plot all three phases side by side"""
        fig, axes = plt.subplots(1, 3, figsize=(18, 6))
        
        for i, phase in enumerate(PhaseType):
            ax = axes[i]
            
            # Draw edges
            for edge_id, phases in self.graph.edges.items():
                edge = phases[phase]
                x1, y1 = self.pos[edge.from_node_id]
                x2, y2 = self.pos[edge.to_node_id]
                
                # Get impedance magnitude for line thickness
                if hasattr(edge, 'impedance'):
                    z_mag = abs(edge.impedance)
                    linewidth = max(1, min(5, 10 / (1 + z_mag)))
                else:
                    linewidth = 2
                
                ax.plot([x1, x2], [y1, y2], 
                       color=self.phase_colors[phase], 
                       linewidth=linewidth, alpha=0.6)
            
            # Draw nodes
            for node_id, phases_dict in self.graph.nodes.items():
                node = phases_dict[phase]
                x, y = self.pos[node_id]
                
                # Voltage magnitude for size
                v_mag = abs(node.voltage_pu)
                size = 100 * v_mag
                
                # Node type for shape
                if node.node_type == 'generator':
                    marker = 's'
                elif node.node_type == 'load':
                    marker = 'o'
                else:
                    marker = '^'
                
                ax.scatter(x, y, s=size, marker=marker,
                          c=self.phase_colors[phase],
                          edgecolors='black', linewidths=1.5)
            
            ax.set_title(f'Phase {phase.value.upper()}')
            ax.set_aspect('equal')
            ax.grid(True, alpha=0.3)
        
        plt.suptitle('Three-Phase System Overview', fontsize=16, fontweight='bold')
        plt.tight_layout()
        
        return fig, axes
    
    def plot_system_statistics(self) -> plt.Figure:
        """Plot system statistics dashboard"""
        fig = plt.figure(figsize=(12, 8))
        
        # Create grid
        gs = fig.add_gridspec(3, 2, hspace=0.3, wspace=0.3)
        
        # 1. Node type distribution
        ax1 = fig.add_subplot(gs[0, 0])
        node_types = {}
        for node_id, phases in self.graph.nodes.items():
            node_type = phases[PhaseType.A].node_type
            node_types[node_type] = node_types.get(node_type, 0) + 1
        
        ax1.pie(node_types.values(), labels=node_types.keys(), autopct='%1.1f%%',
                colors=['#ff6b6b', '#4ecdc4', '#95e1d3'])
        ax1.set_title('Node Type Distribution')
        
        # 2. Voltage histogram
        ax2 = fig.add_subplot(gs[0, 1])
        voltages = []
        for phases in self.graph.nodes.values():
            for node in phases.values():
                voltages.append(abs(node.voltage_pu))
        
        ax2.hist(voltages, bins=20, color='skyblue', edgecolor='black')
        ax2.axvline(x=1.0, color='red', linestyle='--', label='Nominal')
        ax2.set_xlabel('Voltage (pu)')
        ax2.set_ylabel('Count')
        ax2.set_title('Voltage Distribution')
        ax2.legend()
        
        # 3. Coupling strength distribution
        ax3 = fig.add_subplot(gs[1, :])
        coupling_strengths = []
        
        # Collect all coupling values
        for coupling in self.graph.edge_couplings.values():
            matrix = coupling.matrix
            for i in range(3):
                for j in range(3):
                    if i != j:
                        coupling_strengths.append(abs(matrix[i, j]))
        
        if coupling_strengths:
            ax3.hist(coupling_strengths, bins=30, color='orange', edgecolor='black', alpha=0.7)
            ax3.set_xlabel('Coupling Strength')
            ax3.set_ylabel('Count')
            ax3.set_title('Distribution of Inter-Phase Coupling Values')
            ax3.set_yscale('log')  # Log scale for better visualization
        
        # 4. Power balance
        ax4 = fig.add_subplot(gs[2, :])
        total_gen = 0
        total_load = 0
        
        for node_id, phase_nodes in self.graph.nodes.items():
            for phase in [PhaseType.A, PhaseType.B, PhaseType.C]:
                node = phase_nodes[phase]
                if node.node_type == 'generator':
                    gen_obj = node.properties.get('node_object')
                    if gen_obj:
                        total_gen += gen_obj.P_actual_MW
                elif node.node_type == 'load':
                    load_obj = node.properties.get('load_object')
                    if load_obj:
                        total_load += load_obj.P_MW
        
        losses = total_gen - total_load
        
        categories = ['Generation', 'Load', 'Losses']
        values = [total_gen, total_load, losses]
        colors = ['#26de81', '#fc5c65', '#fed330']
        
        bars = ax4.bar(categories, values, color=colors, edgecolor='black', linewidth=1.5)
        ax4.set_title('Power Balance')
        ax4.set_ylabel('Power (MW)')
        ax4.grid(True, axis='y', alpha=0.3)
        
        # Add value labels on bars
        for bar, value in zip(bars, values):
            height = bar.get_height()
            ax4.text(bar.get_x() + bar.get_width()/2., height,
                    f'{value:.1f} MW',
                    ha='center', va='bottom', fontweight='bold')
        
        plt.suptitle('Power System Statistics', fontsize=16, fontweight='bold')
        return fig
    
    def save_all_plots(self, output_dir: str = './plots'):
        """Save all visualization plots."""
        import os
        os.makedirs(output_dir, exist_ok=True)
        
        # 3D coupled graph
        fig, _ = self.plot_3d_coupled_graph(show_coupling=True)
        fig.savefig(f'{output_dir}/3d_coupled_graph.png', dpi=300, bbox_inches='tight')
        plt.close(fig)
        
        # Spectral 3D graph
        fig, _ = self.plot_spectral_3d_graph(coupling_color_nodes=True, coupling_color_edges=True)
        fig.savefig(f'{output_dir}/spectral_3d_graph.png', dpi=300, bbox_inches='tight')
        plt.close(fig)
        
        # Single phase plots
        for phase in PhaseType:
            fig, _ = self.plot_single_phase(phase)
            fig.savefig(f'{output_dir}/phase_{phase.value}_graph.png', dpi=300, bbox_inches='tight')
            plt.close(fig)
        
        # Three phase overview
        fig, _ = self.plot_three_phase_overview()
        fig.savefig(f'{output_dir}/three_phase_overview.png', dpi=300, bbox_inches='tight')
        plt.close(fig)
        
        # Coupling strength
        fig, _ = self.plot_coupling_strength()
        fig.savefig(f'{output_dir}/coupling_strength.png', dpi=300, bbox_inches='tight')
        plt.close(fig)
        
        # Voltage profiles
        for phase in PhaseType:
            fig, _ = self.plot_voltage_profile(phase)
            fig.savefig(f'{output_dir}/voltage_profile_phase_{phase.value}.png', 
                       dpi=300, bbox_inches='tight')
            plt.close(fig)
        
        # Statistics
        fig = self.plot_system_statistics()
        fig.savefig(f'{output_dir}/system_statistics.png', dpi=300, bbox_inches='tight')
        plt.close(fig)
        
        print(f"✓ All plots saved to {output_dir}/")