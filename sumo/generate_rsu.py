"""
RSU Placement Optimizer for NS2 Mobility Traces - FIXED VERSION
Finds optimal RSU positions to ensure maximum vehicle coverage
"""

import re
import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import KMeans
import math

class NS2MobilityParser:
    def __init__(self, filename):
        self.filename = filename
        self.all_positions = []      # All vehicle positions for clustering
        
    def parse_mobility_file(self):
        """Parse NS2 mobility file and extract vehicle positions over time"""
        print(f"Parsing mobility file: {self.filename}")
        
        # Store temporary node positions
        node_positions = {}  # {node_id: {'x': val, 'y': val}}
        position_set = set()
        
        with open(self.filename, 'r') as file:
            for line in file:
                line = line.strip()
                
                # Parse initial positions: $node_(X) set X_ Y.Y
                pos_match = re.search(r'\$node_\((\d+)\) set ([XY])_ ([\d.]+)', line)
                if pos_match:
                    node_id = int(pos_match.group(1))
                    coord = pos_match.group(2)
                    value = float(pos_match.group(3))
                    
                    if node_id not in node_positions:
                        node_positions[node_id] = {}
                    
                    node_positions[node_id][coord.lower()] = value
                    
                    # If we have both x and y, add to positions
                    if 'x' in node_positions[node_id] and 'y' in node_positions[node_id]:
                        pos = (node_positions[node_id]['x'], node_positions[node_id]['y'])
                        position_set.add(pos)
                
                # Parse movement commands: $ns_ at TIME "$node_(X) setdest X Y SPEED"
                move_match = re.search(r'\$ns_ at ([\d.]+) "\$node_\((\d+)\) setdest ([\d.]+) ([\d.]+) ([\d.]+)"', line)
                if move_match:
                    time = float(move_match.group(1))
                    node_id = int(move_match.group(2))
                    x = float(move_match.group(3))
                    y = float(move_match.group(4))
                    speed = float(move_match.group(5))
                    
                    # Add destination position
                    position_set.add((x, y))
        
        # Convert set to list
        self.all_positions = list(position_set)
        print(f"Found {len(self.all_positions)} unique vehicle positions")
        
        if self.all_positions:
            # Print area bounds for verification
            positions_array = np.array(self.all_positions)
            min_x, max_x = positions_array[:, 0].min(), positions_array[:, 0].max()
            min_y, max_y = positions_array[:, 1].min(), positions_array[:, 1].max()
            print(f"Area bounds: X=[{min_x:.1f}, {max_x:.1f}], Y=[{min_y:.1f}, {max_y:.1f}]")
        
        return len(self.all_positions) > 0

class RSUPlacementOptimizer:
    def __init__(self, vehicle_positions, rsu_range=300):
        self.vehicle_positions = vehicle_positions
        self.rsu_range = rsu_range
        self.best_rsu_positions = []
        
    def find_optimal_rsu_positions(self, num_rsus=20):
        """Find optimal RSU positions using multiple strategies"""
        
        print(f"\n=== RSU PLACEMENT OPTIMIZATION ===")
        print(f"Target RSUs: {num_rsus}")
        print(f"RSU Range: {self.rsu_range}m")
        print(f"Vehicle positions to cover: {len(self.vehicle_positions)}")
        
        strategies = [
            ("K-Means Clustering", self.kmeans_placement),
            ("Grid-Based Placement", self.grid_placement), 
            ("Density-Based Placement", self.density_placement),
            ("Coverage Optimization", self.coverage_optimization)
        ]
        
        best_strategy = None
        best_coverage = 0
        best_positions = []
        
        for strategy_name, strategy_func in strategies:
            print(f"\n--- Testing {strategy_name} ---")
            try:
                positions = strategy_func(num_rsus)
                coverage = self.calculate_coverage(positions)
                
                print(f"{strategy_name}: {coverage:.2f}% coverage")
                
                if coverage > best_coverage:
                    best_coverage = coverage
                    best_positions = positions
                    best_strategy = strategy_name
            except Exception as e:
                print(f"Error in {strategy_name}: {e}")
                continue
        
        self.best_rsu_positions = best_positions
        print(f"\n*** BEST STRATEGY: {best_strategy} ***")
        print(f"*** BEST COVERAGE: {best_coverage:.2f}% ***")
        
        return best_positions, best_coverage
    
    def kmeans_placement(self, num_rsus):
        """Use K-means clustering to find RSU positions"""
        if len(self.vehicle_positions) < num_rsus:
            print(f"Warning: Only {len(self.vehicle_positions)} positions available for {num_rsus} RSUs")
            return [[pos[0], pos[1]] for pos in self.vehicle_positions]
        
        kmeans = KMeans(n_clusters=num_rsus, random_state=42, n_init=10)
        positions_array = np.array(self.vehicle_positions)
        kmeans.fit(positions_array)
        
        return kmeans.cluster_centers_.tolist()
    
    def grid_placement(self, num_rsus):
        """Place RSUs in a grid pattern over the area"""
        positions_array = np.array(self.vehicle_positions)
        min_x, max_x = positions_array[:, 0].min(), positions_array[:, 0].max()
        min_y, max_y = positions_array[:, 1].min(), positions_array[:, 1].max()
        
        # Add padding to ensure coverage at edges
        padding = self.rsu_range * 0.3
        min_x -= padding
        max_x += padding
        min_y -= padding
        max_y += padding
        
        # Calculate grid dimensions
        grid_size = math.ceil(math.sqrt(num_rsus))
        
        x_step = (max_x - min_x) / max(1, grid_size - 1)
        y_step = (max_y - min_y) / max(1, grid_size - 1)
        
        positions = []
        for i in range(grid_size):
            for j in range(grid_size):
                if len(positions) >= num_rsus:
                    break
                x = min_x + i * x_step
                y = min_y + j * y_step
                positions.append([x, y])
            if len(positions) >= num_rsus:
                break
        
        return positions[:num_rsus]
    
    def density_placement(self, num_rsus):
        """Place RSUs in high-density vehicle areas"""
        positions_array = np.array(self.vehicle_positions)
        min_x, max_x = positions_array[:, 0].min(), positions_array[:, 0].max()
        min_y, max_y = positions_array[:, 1].min(), positions_array[:, 1].max()
        
        # Create grid for density calculation
        grid_resolution = min(50, int(math.sqrt(len(self.vehicle_positions))))
        
        try:
            x_bins = np.linspace(min_x, max_x, grid_resolution)
            y_bins = np.linspace(min_y, max_y, grid_resolution)
            
            # Calculate density histogram
            hist, x_edges, y_edges = np.histogram2d(
                positions_array[:, 0], positions_array[:, 1], 
                bins=[x_bins, y_bins]
            )
            
            # Find top density locations
            flat_indices = np.argsort(hist.flatten())[-num_rsus:]
            row_indices, col_indices = np.unravel_index(flat_indices, hist.shape)
            
            positions = []
            for i, j in zip(row_indices, col_indices):
                if i < len(x_edges)-1 and j < len(y_edges)-1:
                    x = (x_edges[i] + x_edges[i+1]) / 2
                    y = (y_edges[j] + y_edges[j+1]) / 2
                    positions.append([x, y])
            
            # If we don't have enough positions, fill with grid placement
            while len(positions) < num_rsus:
                remaining = num_rsus - len(positions)
                grid_positions = self.grid_placement(remaining)
                positions.extend(grid_positions)
                break
            
            return positions[:num_rsus]
            
        except Exception as e:
            print(f"Density placement failed: {e}, falling back to grid")
            return self.grid_placement(num_rsus)
    
    def coverage_optimization(self, num_rsus):
        """Iterative coverage optimization"""
        try:
            # Start with k-means
            initial_positions = self.kmeans_placement(num_rsus)
            best_positions = [pos[:] for pos in initial_positions]  # Deep copy
            best_coverage = self.calculate_coverage(initial_positions)
            
            print(f"Starting coverage optimization from {best_coverage:.2f}%")
            
            # Iterative improvement (limited iterations for performance)
            for iteration in range(5):
                improved = False
                
                for i in range(len(best_positions)):
                    # Try moving each RSU to improve coverage
                    original_pos = best_positions[i][:]
                    
                    # Try positions near uncovered vehicles
                    uncovered = self.find_uncovered_vehicles(best_positions)
                    if uncovered and len(uncovered) > 0:
                        # Try up to 3 uncovered positions
                        test_positions = uncovered[:min(3, len(uncovered))]
                        
                        for vehicle_pos in test_positions:
                            test_rsu_positions = [pos[:] for pos in best_positions]  # Deep copy
                            test_rsu_positions[i] = [vehicle_pos[0], vehicle_pos[1]]
                            
                            coverage = self.calculate_coverage(test_rsu_positions)
                            if coverage > best_coverage:
                                best_coverage = coverage
                                best_positions = test_rsu_positions
                                improved = True
                                print(f"Iteration {iteration}: Improved to {coverage:.2f}%")
                                break
                    
                    if improved:
                        break
                
                if not improved:
                    break
            
            return best_positions
            
        except Exception as e:
            print(f"Coverage optimization failed: {e}, falling back to k-means")
            return self.kmeans_placement(num_rsus)
    
    def calculate_coverage(self, rsu_positions):
        """Calculate percentage of vehicles covered by RSUs"""
        if not rsu_positions or not self.vehicle_positions:
            return 0.0
        
        covered_vehicles = 0
        total_vehicles = len(self.vehicle_positions)
        
        for vehicle_pos in self.vehicle_positions:
            is_covered = False
            for rsu_pos in rsu_positions:
                distance = math.sqrt(
                    (vehicle_pos[0] - rsu_pos[0])**2 + 
                    (vehicle_pos[1] - rsu_pos[1])**2
                )
                if distance <= self.rsu_range:
                    is_covered = True
                    break
            
            if is_covered:
                covered_vehicles += 1
        
        return (covered_vehicles / total_vehicles) * 100 if total_vehicles > 0 else 0
    
    def find_uncovered_vehicles(self, rsu_positions):
        """Find vehicles not covered by current RSU placement"""
        uncovered = []
        
        for vehicle_pos in self.vehicle_positions:
            is_covered = False
            for rsu_pos in rsu_positions:
                distance = math.sqrt(
                    (vehicle_pos[0] - rsu_pos[0])**2 + 
                    (vehicle_pos[1] - rsu_pos[1])**2
                )
                if distance <= self.rsu_range:
                    is_covered = True
                    break
            
            if not is_covered:
                uncovered.append(vehicle_pos)
        
        return uncovered
    
    def generate_cpp_positions(self, positions):
        """Generate C++ code for RSU positions"""
        cpp_code = "// Optimized RSU Positions\n"
        cpp_code += "std::vector<RSUPosition> allRSUs = {\n"
        
        for i, pos in enumerate(positions):
            x, y = pos[0], pos[1]
            cpp_code += f'    {{{i}, Vector({x:.2f}, {y:.2f}, 0.0), "RSU-{i}"}}'
            if i < len(positions) - 1:
                cpp_code += ","
            cpp_code += "\n"
        
        cpp_code += "};\n"
        return cpp_code
    
    def save_simple_visualization(self):
        """Create a simple text-based visualization"""
        if not self.best_rsu_positions or not self.vehicle_positions:
            return
        
        try:
            plt.figure(figsize=(15, 12))
            
            # Plot vehicle positions
            vehicle_array = np.array(self.vehicle_positions)
            plt.scatter(vehicle_array[:, 0], vehicle_array[:, 1], 
                       c='blue', alpha=0.6, s=10, label='Vehicle Positions')
            
            # Plot RSU positions and coverage
            rsu_array = np.array(self.best_rsu_positions)
            plt.scatter(rsu_array[:, 0], rsu_array[:, 1], 
                       c='red', s=200, marker='^', label='RSU Positions')
            
            # Draw coverage circles
            for i, pos in enumerate(self.best_rsu_positions):
                x, y = pos[0], pos[1]
                circle = plt.Circle((x, y), self.rsu_range, 
                                  fill=False, color='red', alpha=0.3, linestyle='--')
                plt.gca().add_patch(circle)
                plt.annotate(f'RSU-{i}', (x, y), xytext=(5, 5), 
                            textcoords='offset points', fontsize=8)
            
            plt.xlabel('X Coordinate (m)')
            plt.ylabel('Y Coordinate (m)')
            plt.title(f'Optimal RSU Placement ({len(self.best_rsu_positions)} RSUs, Range: {self.rsu_range}m)')
            plt.legend()
            plt.grid(True, alpha=0.3)
            plt.axis('equal')
            
            plt.savefig('rsu_placement_optimization.png', dpi=300, bbox_inches='tight')
            print("Visualization saved as 'rsu_placement_optimization.png'")
            
        except Exception as e:
            print(f"Could not create visualization: {e}")

def main():
    # Configuration
    MOBILITY_FILE = "ns2mobility.tcl"  # Changed to current directory
    NUM_RSUS = 20
    RSU_RANGE = 300  # meters
    
    print("=== RSU Placement Optimization Tool ===")
    print(f"Mobility file: {MOBILITY_FILE}")
    print(f"Target RSUs: {NUM_RSUS}")
    print(f"RSU Range: {RSU_RANGE}m")
    
    # Parse mobility file
    parser = NS2MobilityParser(MOBILITY_FILE)
    success = parser.parse_mobility_file()
    
    if not success or not parser.all_positions:
        print("ERROR: No vehicle positions found in mobility file!")
        print("Make sure the file exists and contains valid NS2 mobility commands")
        return
    
    # Optimize RSU placement
    optimizer = RSUPlacementOptimizer(parser.all_positions, RSU_RANGE)
    best_positions, coverage = optimizer.find_optimal_rsu_positions(NUM_RSUS)
    
    if not best_positions:
        print("ERROR: Could not generate RSU positions!")
        return
    
    # Generate results
    print(f"\n=== OPTIMIZATION RESULTS ===")
    print(f"Coverage achieved: {coverage:.2f}%")
    print(f"RSU positions:")
    
    for i, pos in enumerate(best_positions):
        x, y = pos[0], pos[1]
        print(f"  RSU-{i}: ({x:.2f}, {y:.2f})")
    
    # Generate C++ code
    cpp_code = optimizer.generate_cpp_positions(best_positions)
    
    # Save results
    with open("optimized_rsu_positions.cpp", "w") as f:
        f.write(cpp_code)
    
    with open("rsu_optimization_report.txt", "w") as f:
        f.write("RSU Placement Optimization Report\n")
        f.write("=" * 40 + "\n\n")
        f.write(f"Input file: {MOBILITY_FILE}\n")
        f.write(f"Vehicle positions analyzed: {len(parser.all_positions)}\n")
        f.write(f"Target RSUs: {NUM_RSUS}\n")
        f.write(f"RSU Range: {RSU_RANGE}m\n")
        f.write(f"Coverage achieved: {coverage:.2f}%\n\n")
        f.write("Optimized RSU Positions:\n")
        for i, pos in enumerate(best_positions):
            x, y = pos[0], pos[1]
            f.write(f"RSU-{i}: ({x:.2f}, {y:.2f})\n")
        f.write(f"\nC++ Code:\n{cpp_code}")
    
    # Create visualization
    optimizer.save_simple_visualization()
    
    # Analyze coverage
    uncovered = optimizer.find_uncovered_vehicles(best_positions)
    print(f"\n=== COVERAGE ANALYSIS ===")
    print(f"Total vehicles: {len(parser.all_positions)}")
    print(f"Covered vehicles: {len(parser.all_positions) - len(uncovered)}")
    print(f"Uncovered vehicles: {len(uncovered)}")
    
    print(f"\n=== FILES GENERATED ===")
    print("- optimized_rsu_positions.cpp (C++ code)")
    print("- rsu_optimization_report.txt (detailed report)")
    print("- rsu_placement_optimization.png (visualization)")
    
    print(f"\n=== RECOMMENDATION ===")
    if coverage >= 95:
        print("EXCELLENT: Coverage is excellent!")
    elif coverage >= 85:
        print("GOOD: Coverage is good. Consider minor adjustments.")
    elif coverage >= 75:
        print("FAIR: Coverage is acceptable but could be improved.")
    else:
        print("POOR: Coverage is low. Consider more RSUs or different placement.")

if __name__ == "__main__":
    main()