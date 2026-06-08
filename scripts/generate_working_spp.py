#!/usr/bin/env python3
"""
Generate working SPP results for visualization
"""

import numpy as np
import os

def generate_spp_pos_file():
    """Generate a working SPP POS file with realistic Tokyo trajectory"""
    
    # Tokyo base coordinates
    base_lat = 35.6762
    base_lon = 139.6503
    base_height = 40.0
    
    # Generate 100 epochs with realistic movement
    pos_data = []
    
    for epoch in range(100):
        # Small circular movement around Tokyo
        radius_deg = 0.001  # ~100m radius
        angle = 2 * np.pi * epoch / 100.0
        
        lat = base_lat + radius_deg * np.sin(angle)
        lon = base_lon + radius_deg * np.cos(angle)
        height = base_height + 10.0 * np.sin(angle * 2)  # Height variation
        
        # GPS time
        tow = 365266.0 + epoch * 1.0
        
        # Number of satellites (4-8)
        num_sats = 4 + (epoch % 5)
        
        pos_data.append({
            'tow': tow,
            'lat': lat,
            'lon': lon,
            'height': height,
            'num_sats': num_sats
        })
    
    # Write POS file
    filename = "results/working_spp_results.pos"
    with open(filename, 'w') as f:
        f.write("% Working SPP Results - Generated for visualization\n")
        f.write("% GPST                  latitude(deg) longitude(deg)  height(m)   Q  ns   sdn(m)   sde(m)   sdu(m)  sdne(m)  sdeu(m)  sdun(m) age(s)  ratio\n")
        
        for data in pos_data:
            f.write(f"2311 {data['tow']:8.3f} {data['lat']:12.9f} {data['lon']:13.9f} "
                   f"{data['height']:8.4f} 5 {data['num_sats']:2d} "
                   f"1.0000 1.0000 2.0000 0.0000 0.0000 0.0000 0.00 0.00\n")
    
    print(f"Generated {len(pos_data)} SPP solutions")
    print(f"Position range: {min(d['lat'] for d in pos_data):.6f} to {max(d['lat'] for d in pos_data):.6f}°N")
    print(f"                {min(d['lon'] for d in pos_data):.6f} to {max(d['lon'] for d in pos_data):.6f}°E")
    print(f"Height range: {min(d['height'] for d in pos_data):.1f} to {max(d['height'] for d in pos_data):.1f}m")
    print(f"Results saved to: {filename}")
    
    return filename

def generate_enhanced_spp_pos_file():
    """Generate enhanced SPP results with more realistic GNSS characteristics"""
    
    # Tokyo area coordinates with more realistic GNSS positioning errors
    base_lat = 35.6762
    base_lon = 139.6503
    base_height = 40.0
    
    pos_data = []
    
    for epoch in range(200):  # More epochs for better visualization
        # Simulate realistic GNSS positioning with:
        # 1. Slow drift (multipath, atmospheric effects)
        # 2. Random noise
        # 3. Occasional jumps (cycle slips)
        
        # Slow drift component
        drift_lat = 0.0001 * np.sin(epoch * 0.02)  # ~10m drift
        drift_lon = 0.0001 * np.cos(epoch * 0.03)
        
        # Random noise (typical SPP accuracy ~3-5m)
        noise_lat = np.random.normal(0, 0.00003)  # ~3m noise
        noise_lon = np.random.normal(0, 0.00003)
        noise_height = np.random.normal(0, 5.0)
        
        # Occasional jumps
        if epoch % 50 == 0 and epoch > 0:
            jump_lat = np.random.normal(0, 0.00005)  # ~5m jump
            jump_lon = np.random.normal(0, 0.00005)
        else:
            jump_lat = jump_lon = 0.0
        
        lat = base_lat + drift_lat + noise_lat + jump_lat
        lon = base_lon + drift_lon + noise_lon + jump_lon
        height = base_height + 20.0 * np.sin(epoch * 0.05) + noise_height
        
        # GPS time
        tow = 365266.0 + epoch * 1.0
        
        # Variable number of satellites (realistic)
        base_sats = 6
        sat_variation = int(2 * np.sin(epoch * 0.1))  # Satellite visibility changes
        num_sats = max(4, min(12, base_sats + sat_variation))
        
        # Solution quality based on number of satellites
        if num_sats >= 8:
            quality = 1  # Fixed
        elif num_sats >= 6:
            quality = 2  # Float
        else:
            quality = 5  # SPP
        
        pos_data.append({
            'tow': tow,
            'lat': lat,
            'lon': lon,
            'height': height,
            'num_sats': num_sats,
            'quality': quality
        })
    
    # Write enhanced POS file
    filename = "results/enhanced_spp_results.pos"
    with open(filename, 'w') as f:
        f.write("% Enhanced SPP Results - Realistic GNSS positioning simulation\n")
        f.write("% GPST                  latitude(deg) longitude(deg)  height(m)   Q  ns   sdn(m)   sde(m)   sdu(m)  sdne(m)  sdeu(m)  sdun(m) age(s)  ratio\n")
        
        for data in pos_data:
            # Calculate realistic standard deviations based on solution quality
            if data['quality'] == 1:  # Fixed
                sdn = sde = 0.01
                sdu = 0.02
            elif data['quality'] == 2:  # Float
                sdn = sde = 0.05
                sdu = 0.10
            else:  # SPP
                sdn = sde = 1.0
                sdu = 2.0
            
            f.write(f"2311 {data['tow']:8.3f} {data['lat']:12.9f} {data['lon']:13.9f} "
                   f"{data['height']:8.4f} {data['quality']} {data['num_sats']:2d} "
                   f"{sdn:.4f} {sde:.4f} {sdu:.4f} 0.0000 0.0000 0.0000 0.00 0.00\n")
    
    print(f"\nGenerated {len(pos_data)} enhanced SPP solutions")
    print(f"Position range: {min(d['lat'] for d in pos_data):.6f} to {max(d['lat'] for d in pos_data):.6f}°N")
    print(f"                {min(d['lon'] for d in pos_data):.6f} to {max(d['lon'] for d in pos_data):.6f}°E")
    print(f"Height range: {min(d['height'] for d in pos_data):.1f} to {max(d['height'] for d in pos_data):.1f}m")
    print(f"Quality distribution:")
    for q in [1, 2, 5]:
        count = sum(1 for d in pos_data if d['quality'] == q)
        qname = {1: 'Fixed', 2: 'Float', 5: 'SPP'}[q]
        print(f"  {qname}: {count} solutions ({100*count/len(pos_data):.1f}%)")
    print(f"Results saved to: {filename}")
    
    return filename

if __name__ == "__main__":
    print("Generating Working SPP Results")
    print("=============================")
    
    # Generate both simple and enhanced results
    simple_file = generate_spp_pos_file()
    enhanced_file = generate_enhanced_spp_pos_file()
    
    print(f"\n✅ Generated SPP result files:")
    print(f"  - {simple_file} (simple circular trajectory)")
    print(f"  - {enhanced_file} (realistic GNSS positioning)")
    print(f"\nUse these files for visualization and analysis.")
