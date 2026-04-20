#!/usr/bin/env python3
"""
Analyze RTKLIB .stat file to extract detailed RTK processing statistics
"""

import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from collections import defaultdict
import pandas as pd

def parse_stat_file(filename):
    """Parse RTKLIB .stat file and extract statistics"""
    
    positions = []
    velocities = []
    satellites = defaultdict(list)
    clock_data = []
    
    with open(filename, 'r') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
                
            parts = line.split(',')
            if len(parts) < 4:
                continue
                
            record_type = parts[0]
            
            try:
                if record_type == '$POS':
                    # $POS,week,tow,stat,x,y,z,xs,ys,zs
                    week = int(parts[1])
                    tow = float(parts[2])
                    stat = int(parts[3])
                    x, y, z = float(parts[4]), float(parts[5]), float(parts[6])
                    xs, ys, zs = float(parts[7]), float(parts[8]), float(parts[9])
                    
                    positions.append({
                        'week': week, 'tow': tow, 'status': stat,
                        'x': x, 'y': y, 'z': z,
                        'std_x': xs, 'std_y': ys, 'std_z': zs
                    })
                    
                elif record_type == '$VELACC':
                    # $VELACC,week,tow,stat,vx,vy,vz,ax,ay,az,vxs,vys,vzs,axs,ays,azs
                    week = int(parts[1])
                    tow = float(parts[2])
                    stat = int(parts[3])
                    vx, vy, vz = float(parts[4]), float(parts[5]), float(parts[6])
                    ax, ay, az = float(parts[7]), float(parts[8]), float(parts[9])
                    
                    velocities.append({
                        'week': week, 'tow': tow, 'status': stat,
                        'vx': vx, 'vy': vy, 'vz': vz,
                        'ax': ax, 'ay': ay, 'az': az
                    })
                    
                elif record_type == '$CLK':
                    # $CLK,week,tow,stat,nsat,clk,clks,clkd,clkds
                    week = int(parts[1])
                    tow = float(parts[2])
                    stat = int(parts[3])
                    nsat = int(parts[4])
                    clk = float(parts[5])
                    
                    clock_data.append({
                        'week': week, 'tow': tow, 'status': stat,
                        'nsat': nsat, 'clock': clk
                    })
                    
                elif record_type == '$SAT':
                    # $SAT,week,tow,sat,frq,az,el,resp,resc,vsat,snr,lock,outc,slipc,rejc,icpc,fixc,psr,cp,rr
                    if len(parts) >= 20:
                        week = int(parts[1])
                        tow = float(parts[2])
                        sat = parts[3]
                        frq = int(parts[4])
                        az = float(parts[5])
                        el = float(parts[6])
                        resp = float(parts[7])
                        resc = float(parts[8])
                        vsat = int(parts[9])
                        snr = float(parts[10])
                        lock = int(parts[11])
                        
                        satellites[sat].append({
                            'week': week, 'tow': tow, 'freq': frq,
                            'azimuth': az, 'elevation': el,
                            'pseudorange_residual': resp, 'carrier_residual': resc,
                            'valid': vsat, 'snr': snr, 'lock': lock
                        })
                        
            except (ValueError, IndexError):
                continue
    
    return positions, velocities, satellites, clock_data

def analyze_statistics(positions, velocities, satellites, clock_data):
    """Analyze extracted statistics"""
    
    print("RTKLIB Statistics Analysis")
    print("=" * 50)
    
    # Position statistics
    if positions:
        pos_df = pd.DataFrame(positions)
        print(f"\nPosition Statistics:")
        print(f"Total epochs: {len(positions)}")
        
        # Status distribution
        status_counts = pos_df['status'].value_counts().sort_index()
        status_names = {1: 'FIXED', 2: 'FLOAT', 3: 'SBAS', 4: 'DGPS', 5: 'SPP'}
        
        print(f"Solution Status Distribution:")
        for status, count in status_counts.items():
            name = status_names.get(status, f'STATUS_{status}')
            percentage = 100.0 * count / len(positions)
            print(f"  {name}: {count:6d} ({percentage:5.1f}%)")
        
        # Position accuracy
        print(f"\nPosition Accuracy (m):")
        print(f"  X std dev: {pos_df['std_x'].mean():.4f} ± {pos_df['std_x'].std():.4f}")
        print(f"  Y std dev: {pos_df['std_y'].mean():.4f} ± {pos_df['std_y'].std():.4f}")
        print(f"  Z std dev: {pos_df['std_z'].mean():.4f} ± {pos_df['std_z'].std():.4f}")
        
        # Position range
        x_range = pos_df['x'].max() - pos_df['x'].min()
        y_range = pos_df['y'].max() - pos_df['y'].min()
        z_range = pos_df['z'].max() - pos_df['z'].min()
        
        print(f"\nPosition Range (m):")
        print(f"  X range: {x_range:.2f}")
        print(f"  Y range: {y_range:.2f}")
        print(f"  Z range: {z_range:.2f}")
    
    # Velocity statistics
    if velocities:
        vel_df = pd.DataFrame(velocities)
        print(f"\nVelocity Statistics:")
        print(f"  Velocity X: {vel_df['vx'].mean():.4f} ± {vel_df['vx'].std():.4f} m/s")
        print(f"  Velocity Y: {vel_df['vy'].mean():.4f} ± {vel_df['vy'].std():.4f} m/s")
        print(f"  Velocity Z: {vel_df['vz'].mean():.4f} ± {vel_df['vz'].std():.4f} m/s")
    
    # Satellite statistics
    if satellites:
        print(f"\nSatellite Statistics:")
        print(f"Total satellites tracked: {len(satellites)}")
        
        # Count by constellation
        constellations = defaultdict(int)
        for sat in satellites.keys():
            if sat.startswith('G'):
                constellations['GPS'] += 1
            elif sat.startswith('R'):
                constellations['GLONASS'] += 1
            elif sat.startswith('E'):
                constellations['Galileo'] += 1
            elif sat.startswith('J'):
                constellations['QZSS'] += 1
            elif sat.startswith('C'):
                constellations['BeiDou'] += 1
        
        print("Constellation distribution:")
        for const, count in constellations.items():
            print(f"  {const}: {count} satellites")
        
        # Average elevation and SNR
        all_elevations = []
        all_snrs = []
        for sat_data in satellites.values():
            for obs in sat_data:
                if obs['elevation'] > 0:
                    all_elevations.append(obs['elevation'])
                if obs['snr'] > 0:
                    all_snrs.append(obs['snr'])
        
        if all_elevations:
            print(f"\nElevation statistics:")
            print(f"  Average: {np.mean(all_elevations):.1f}°")
            print(f"  Range: {np.min(all_elevations):.1f}° - {np.max(all_elevations):.1f}°")
        
        if all_snrs:
            print(f"\nSNR statistics:")
            print(f"  Average: {np.mean(all_snrs):.1f} dB-Hz")
            print(f"  Range: {np.min(all_snrs):.1f} - {np.max(all_snrs):.1f} dB-Hz")
    
    # Clock statistics
    if clock_data:
        clk_df = pd.DataFrame(clock_data)
        print(f"\nClock Statistics:")
        print(f"  Average satellites: {clk_df['nsat'].mean():.1f}")
        print(f"  Clock bias range: {clk_df['clock'].min():.3f} - {clk_df['clock'].max():.3f} ns")

def plot_statistics(positions, velocities, satellites, output_dir='plots'):
    """Generate statistical plots"""
    
    os.makedirs(output_dir, exist_ok=True)
    
    if not positions:
        return
    
    pos_df = pd.DataFrame(positions)
    
    # Create time array
    times = pos_df['tow'].values
    
    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(15, 10))
    
    # 1. Position accuracy over time
    ax1.plot(times, pos_df['std_x'], 'r-', alpha=0.7, label='X std')
    ax1.plot(times, pos_df['std_y'], 'g-', alpha=0.7, label='Y std')
    ax1.plot(times, pos_df['std_z'], 'b-', alpha=0.7, label='Z std')
    ax1.set_xlabel('GPS TOW (s)')
    ax1.set_ylabel('Position Std Dev (m)')
    ax1.set_title('Position Accuracy Over Time')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    ax1.set_yscale('log')
    
    # 2. Solution status over time
    status_colors = {1: 'green', 2: 'orange', 3: 'blue', 4: 'purple', 5: 'red'}
    for status in pos_df['status'].unique():
        mask = pos_df['status'] == status
        ax2.scatter(times[mask], [status] * np.sum(mask), 
                   c=status_colors.get(status, 'gray'), alpha=0.6, s=1)
    
    ax2.set_xlabel('GPS TOW (s)')
    ax2.set_ylabel('Solution Status')
    ax2.set_title('Solution Quality Over Time')
    ax2.set_yticks([1, 2, 3, 4, 5])
    ax2.set_yticklabels(['FIXED', 'FLOAT', 'SBAS', 'DGPS', 'SPP'])
    ax2.grid(True, alpha=0.3)
    
    # 3. Position scatter (X-Y)
    colors = [status_colors.get(s, 'gray') for s in pos_df['status']]
    ax3.scatter(pos_df['x'], pos_df['y'], c=colors, alpha=0.6, s=1)
    ax3.set_xlabel('X (m)')
    ax3.set_ylabel('Y (m)')
    ax3.set_title('Position Scatter (ECEF X-Y)')
    ax3.grid(True, alpha=0.3)
    ax3.axis('equal')
    
    # 4. Horizontal accuracy histogram
    horizontal_std = np.sqrt(pos_df['std_x']**2 + pos_df['std_y']**2)
    ax4.hist(horizontal_std, bins=50, alpha=0.7, edgecolor='black')
    ax4.set_xlabel('Horizontal Std Dev (m)')
    ax4.set_ylabel('Frequency')
    ax4.set_title('Horizontal Accuracy Distribution')
    ax4.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(f'{output_dir}/rtklib_statistics.png', dpi=300, bbox_inches='tight')
    plt.show()
    
    print(f"\nStatistical plots saved to: {output_dir}/rtklib_statistics.png")

def main():
    if len(sys.argv) != 2:
        print("Usage: python3 analyze_stat_file.py <stat_file>")
        return 1
    
    stat_file = sys.argv[1]
    
    if not os.path.exists(stat_file):
        print(f"Error: File {stat_file} not found")
        return 1
    
    print(f"Analyzing RTKLIB stat file: {stat_file}")
    print("=" * 60)
    
    # Parse the stat file
    positions, velocities, satellites, clock_data = parse_stat_file(stat_file)
    
    # Analyze statistics
    analyze_statistics(positions, velocities, satellites, clock_data)
    
    # Generate plots
    plot_statistics(positions, velocities, satellites)
    
    return 0

if __name__ == '__main__':
    exit(main())
