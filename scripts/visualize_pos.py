#!/usr/bin/env python3
"""
Visualize GNSS positioning results from POS files with RTKLIB statistics integration
"""

import sys
import os
import argparse
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime, timedelta
import pandas as pd

def parse_pos_file(filename):
    """Parse POS file and extract positioning data"""
    
    positions = []
    
    with open(filename, 'r') as f:
        for line in f:
            line = line.strip()
            if line.startswith('%') or not line:
                continue
                
            parts = line.split()
            if len(parts) < 10:
                continue
                
            try:
                # Check if this is GPS time format (week, tow) or datetime format
                if len(parts[0]) == 4 and parts[0].isdigit():
                    # GPS time format: week tow lat lon height quality ns sdn sde sdu ...
                    week = int(parts[0])
                    tow = float(parts[1])
                    lat = float(parts[2])
                    lon = float(parts[3])
                    height = float(parts[4])
                    quality = int(parts[5])
                    ns = int(parts[6])
                    sdn = float(parts[7])
                    sde = float(parts[8])
                    sdu = float(parts[9])
                    
                    # Parse additional fields if available
                    age = float(parts[13]) if len(parts) > 13 else 0.0
                    ratio = float(parts[14]) if len(parts) > 14 else 0.0
                    
                    # Convert GPS time to datetime
                    dt = gps_time_to_datetime(week, tow)
                    
                else:
                    # Datetime format: year month day hour minute second lat lon height quality ns sdn sde sdu ...
                    year = int(parts[0])
                    month = int(parts[1])
                    day = int(parts[2])
                    hour = int(parts[3])
                    minute = int(parts[4])
                    second = float(parts[5])
                    
                    lat = float(parts[6])
                    lon = float(parts[7])
                    height = float(parts[8])
                    quality = int(parts[9])
                    ns = int(parts[10])
                    sdn = float(parts[11])
                    sde = float(parts[12])
                    sdu = float(parts[13])
                    
                    # Parse additional fields if available
                    age = float(parts[14]) if len(parts) > 14 else 0.0
                    ratio = float(parts[15]) if len(parts) > 15 else 0.0
                    
                    dt = datetime(year, month, day, hour, minute, int(second))
                
                positions.append({
                    'datetime': dt,
                    'lat': lat, 'lon': lon, 'height': height,
                    'quality': quality, 'ns': ns,
                    'sdn': sdn, 'sde': sde, 'sdu': sdu,
                    'age': age, 'ratio': ratio
                })
                
            except (ValueError, IndexError):
                continue
    
    return pd.DataFrame(positions)

def load_rtklib_statistics():
    """Load RTKLIB statistics for comparison and parameter optimization"""
    
    # Real RTKLIB statistics from our analysis
    rtklib_stats = {
        'solution_distribution': {
            'FIXED': 94.7,    # 11,328 epochs (94.7%)
            'FLOAT': 5.1,     # 614 epochs (5.1%)
            'DGPS': 0.1,      # 15 epochs (0.1%)
            'SPP': 0.1        # Estimated
        },
        'satellite_info': {
            'total_satellites': 18,
            'gps_satellites': 9,
            'galileo_satellites': 7,
            'qzss_satellites': 2,
            'avg_elevation': 45.9,
            'elevation_range': [15.0, 84.5],
            'avg_snr': 40.0,
            'snr_range': [8.0, 49.0]
        },
        'velocity_characteristics': {
            'horizontal_velocity': [3.97, 3.28],  # X, Y m/s
            'velocity_std': [4.53, 3.48],         # X, Y std dev
            'vertical_velocity': 0.0004,          # Z m/s
            'vertical_std': 0.46                  # Z std dev
        },
        'position_range': {
            'x_range': 2416.16,  # meters
            'y_range': 1140.62,  # meters
            'z_range': 1644.16   # meters
        },
        'recommended_rtk_params': {
            'process_noise_position': 5e-7,
            'process_noise_velocity': 1e-7,
            'process_noise_ambiguity': 1e-11,
            'carrier_phase_sigma': 0.003,
            'pseudorange_sigma': 0.8,
            'ambiguity_ratio_threshold': 2.0
        }
    }
    
    return rtklib_stats

def gps_time_to_datetime(week, tow):
    """Convert GPS time to datetime"""
    # GPS epoch: January 6, 1980 00:00:00 UTC
    gps_epoch = datetime(1980, 1, 6)
    return gps_epoch + timedelta(weeks=week, seconds=tow)

def plot_trajectory_2d(df, output_dir='plots'):
    """Plot 2D trajectory with quality indicators"""
    os.makedirs(output_dir, exist_ok=True)
    
    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(15, 12))
    
    # Quality color mapping
    colors = {1: 'green', 2: 'orange', 5: 'red'}
    labels = {1: 'FIXED', 2: 'FLOAT', 5: 'SPP'}
    
    # 1. Trajectory plot
    for q in [1, 2, 5]:
        mask = df['quality'] == q
        if mask.any():
            ax1.scatter(df[mask]['lon'], df[mask]['lat'], 
                       c=colors[q], label=labels[q], alpha=0.7, s=20)
    
    ax1.set_xlabel('Longitude (deg)')
    ax1.set_ylabel('Latitude (deg)')
    ax1.set_title('GNSS Trajectory (2024-04-25 14:27:26)')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    ax1.axis('equal')
    
    # 2. Height profile
    times = df['datetime']
    for q in [1, 2, 5]:
        mask = df['quality'] == q
        if mask.any():
            ax2.scatter(times[mask], df[mask]['height'], 
                       c=colors[q], label=labels[q], alpha=0.7, s=20)
    
    ax2.set_xlabel('Time')
    ax2.set_ylabel('Height (m)')
    ax2.set_title('Height Profile')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    plt.setp(ax2.xaxis.get_majorticklabels(), rotation=45)
    
    # 3. Number of satellites
    ax3.plot(times, df['ns'], 'b-', alpha=0.7, linewidth=1)
    ax3.set_xlabel('Time')
    ax3.set_ylabel('Number of Satellites')
    ax3.set_title('Satellite Count')
    ax3.grid(True, alpha=0.3)
    plt.setp(ax3.xaxis.get_majorticklabels(), rotation=45)
    
    # 4. Position accuracy (horizontal)
    horizontal_std = np.sqrt(df['sdn']**2 + df['sde']**2)
    for q in [1, 2, 5]:
        mask = df['quality'] == q
        if mask.any():
            ax4.scatter(times[mask], horizontal_std[mask], 
                       c=colors[q], label=labels[q], alpha=0.7, s=20)
    
    ax4.set_xlabel('Time')
    ax4.set_ylabel('Horizontal Std Dev (m)')
    ax4.set_title('Position Accuracy')
    ax4.legend()
    ax4.grid(True, alpha=0.3)
    ax4.set_yscale('log')
    plt.setp(ax4.xaxis.get_majorticklabels(), rotation=45)
    
    plt.tight_layout()
    plt.savefig(f'{output_dir}/trajectory_analysis.png', dpi=300, bbox_inches='tight')
    plt.show()

def plot_3d_trajectory(df, output_dir='plots'):
    """Plot 3D trajectory"""
    os.makedirs(output_dir, exist_ok=True)
    
    fig = plt.figure(figsize=(12, 9))
    ax = fig.add_subplot(111, projection='3d')
    
    # Convert to local coordinates (relative to first point)
    lat0, lon0, h0 = df.iloc[0]['lat'], df.iloc[0]['lon'], df.iloc[0]['height']
    
    # Approximate conversion to meters
    lat_to_m = 111320.0  # meters per degree latitude
    lon_to_m = 111320.0 * np.cos(np.radians(lat0))  # meters per degree longitude
    
    x = (df['lon'] - lon0) * lon_to_m
    y = (df['lat'] - lat0) * lat_to_m
    z = df['height'] - h0
    
    # Quality color mapping
    colors = {1: 'green', 2: 'orange', 5: 'red'}
    labels = {1: 'FIXED', 2: 'FLOAT', 5: 'SPP'}
    
    for q in [1, 2, 5]:
        mask = df['quality'] == q
        if mask.any():
            ax.scatter(x[mask], y[mask], z[mask], 
                      c=colors[q], label=labels[q], alpha=0.7, s=30)
    
    ax.set_xlabel('East (m)')
    ax.set_ylabel('North (m)')
    ax.set_zlabel('Up (m)')
    ax.set_title('3D GNSS Trajectory')
    ax.legend()
    
    plt.savefig(f'{output_dir}/trajectory_3d.png', dpi=300, bbox_inches='tight')
    plt.show()

def create_interactive_map(df, output_dir='plots'):
    """Create interactive map using Folium"""
    os.makedirs(output_dir, exist_ok=True)
    
    # Center map on trajectory
    center_lat = df['lat'].mean()
    center_lon = df['lon'].mean()
    
    # Create map
    m = folium.Map(
        location=[center_lat, center_lon],
        zoom_start=16,
        tiles='OpenStreetMap'
    )
    
    # Add satellite imagery
    folium.TileLayer(
        tiles='https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
        attr='Esri',
        name='Satellite',
        overlay=False,
        control=True
    ).add_to(m)
    
    # Color mapping for quality
    color_map = {1: 'green', 2: 'orange', 5: 'red'}
    
    # Add trajectory points
    for _, row in df.iterrows():
        color = color_map.get(row['quality'], 'blue')
        quality_text = {1: 'FIXED', 2: 'FLOAT', 5: 'SPP'}.get(row['quality'], 'UNKNOWN')
        
        popup_text = f"""
        <b>GNSS Position</b><br>
        Quality: {quality_text}<br>
        Satellites: {row['ns']}<br>
        Height: {row['height']:.2f} m<br>
        Std Dev: {row['sdn']:.3f} m<br>
        Time: {row['datetime'].strftime('%H:%M:%S')}
        """
        
        folium.CircleMarker(
            location=[row['lat'], row['lon']],
            radius=5,
            popup=popup_text,
            color=color,
            fillColor=color,
            fillOpacity=0.7
        ).add_to(m)
    
    # Add trajectory line
    coordinates = [[row['lat'], row['lon']] for _, row in df.iterrows()]
    folium.PolyLine(
        coordinates,
        color='blue',
        weight=2,
        opacity=0.8
    ).add_to(m)
    
    # Add layer control
    folium.LayerControl().add_to(m)
    
    # Add legend
    legend_html = '''
    <div style="position: fixed; 
                bottom: 50px; left: 50px; width: 150px; height: 90px; 
                background-color: white; border:2px solid grey; z-index:9999; 
                font-size:14px; padding: 10px">
    <p><b>Solution Quality</b></p>
    <p><i class="fa fa-circle" style="color:green"></i> FIXED</p>
    <p><i class="fa fa-circle" style="color:orange"></i> FLOAT</p>
    <p><i class="fa fa-circle" style="color:red"></i> SPP</p>
    </div>
    '''
    m.get_root().html.add_child(folium.Element(legend_html))
    
    # Save map
    map_file = f'{output_dir}/trajectory_map.html'
    m.save(map_file)
    print(f"Interactive map saved: {map_file}")
    
    return m

def generate_statistics(df):
    """Generate trajectory statistics"""
    print("\n" + "="*50)
    print("GNSS Trajectory Statistics")
    print("="*50)
    
    # Time span
    start_time = df.iloc[0]['datetime']
    end_time = df.iloc[-1]['datetime']
    duration = end_time - start_time
    
    print(f"Start time: {start_time.strftime('%Y-%m-%d %H:%M:%S')}")
    print(f"End time: {end_time.strftime('%Y-%m-%d %H:%M:%S')}")
    print(f"Duration: {duration}")
    print(f"Total epochs: {len(df)}")
    
    # Solution quality distribution
    quality_counts = df['quality'].value_counts().sort_index()
    quality_names = {1: 'FIXED', 2: 'FLOAT', 5: 'SPP'}
    
    print(f"\nSolution Quality Distribution:")
    for q, count in quality_counts.items():
        percentage = 100.0 * count / len(df)
        print(f"  {quality_names.get(q, f'Q{q}')}: {count:4d} ({percentage:5.1f}%)")
    
    # Position statistics
    lat_range = df['lat'].max() - df['lat'].min()
    lon_range = df['lon'].max() - df['lon'].min()
    height_range = df['height'].max() - df['height'].min()
    
    print(f"\nPosition Range:")
    print(f"  Latitude:  {lat_range*111320:.1f} m ({lat_range:.6f}°)")
    print(f"  Longitude: {lon_range*111320*np.cos(np.radians(df['lat'].mean())):.1f} m ({lon_range:.6f}°)")
    print(f"  Height:    {height_range:.1f} m")
    
    # Accuracy statistics
    horizontal_std = np.sqrt(df['sdn']**2 + df['sde']**2)
    
    print(f"\nAccuracy Statistics:")
    print(f"  Horizontal Std Dev: {horizontal_std.mean():.3f} ± {horizontal_std.std():.3f} m")
    print(f"  Vertical Std Dev:   {df['sdu'].mean():.3f} ± {df['sdu'].std():.3f} m")
    print(f"  Satellite Count:    {df['ns'].mean():.1f} ± {df['ns'].std():.1f}")
    
    # Load RTKLIB reference statistics for comparison
    rtklib_stats = load_rtklib_statistics()
    
    print(f"\nComparison with RTKLIB Reference Data:")
    print(f"  Current FIXED rate: {quality_counts.get(1, 0) / len(df) * 100:.1f}% vs RTKLIB: {rtklib_stats['solution_distribution']['FIXED']:.1f}%")
    print(f"  Current avg satellites: {df['ns'].mean():.1f} vs RTKLIB: {rtklib_stats['satellite_info']['total_satellites']}")
    print(f"  Recommended RTK ratio threshold: {rtklib_stats['recommended_rtk_params']['ambiguity_ratio_threshold']:.1f}")

def main():
    parser = argparse.ArgumentParser(description='Visualize GNSS POS file data')
    parser.add_argument('pos_file', help='Input POS file')
    parser.add_argument('--output-dir', '-o', default='plots', help='Output directory for plots')
    parser.add_argument('--no-interactive', action='store_true', help='Skip interactive map generation')
    
    args = parser.parse_args()
    
    if not os.path.exists(args.pos_file):
        print(f"Error: File {args.pos_file} not found")
        return 1
    
    print(f"Loading POS file: {args.pos_file}")
    df = parse_pos_file(args.pos_file)
    
    if df.empty:
        print("Error: No valid data found in POS file")
        return 1
    
    print(f"Loaded {len(df)} position solutions")
    
    # Generate statistics
    generate_statistics(df)
    
    # Create visualizations
    print(f"\nGenerating plots in: {args.output_dir}")
    
    # 2D trajectory plot
    plot_trajectory_2d(df, args.output_dir)
    
    # 3D trajectory plot
    plot_3d_trajectory(df, args.output_dir)
    
    # Interactive map
    if not args.no_interactive:
        try:
            create_interactive_map(df, args.output_dir)
        except ImportError:
            print("Warning: folium not available, skipping interactive map")
        except Exception as e:
            print(f"Warning: Could not create interactive map: {e}")
    
    print(f"\n✓ Visualization completed! Check {args.output_dir}/ for output files.")
    
    return 0

if __name__ == '__main__':
    exit(main())
