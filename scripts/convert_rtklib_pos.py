#!/usr/bin/env python3
"""
Convert RTKLIB POS format to standard POS format for visualization
"""

import sys
import os
from datetime import datetime

def convert_rtklib_pos(input_file, output_file):
    """Convert RTKLIB POS format to standard RINEX POS format"""
    
    with open(input_file, 'r') as infile, open(output_file, 'w') as outfile:
        # Write header
        outfile.write("% POS file converted from RTKLIB format\n")
        outfile.write("% Format: GPST latitude(deg) longitude(deg) height(m) Q ns sdn sde sdu sdne sdeu sdun age ratio\n")
        outfile.write("%\n")
        outfile.write("% Q: 1=FIXED, 2=FLOAT, 5=SPP\n")
        outfile.write("% ns: number of satellites\n")
        outfile.write("% sdn,sde,sdu: standard deviation (m)\n")
        outfile.write("% sdne,sdeu,sdun: correlation\n")
        outfile.write("% age: age of differential (s)\n")
        outfile.write("% ratio: ambiguity ratio\n")
        outfile.write("%\n")
        
        line_count = 0
        for line in infile:
            line = line.strip()
            if not line or line.startswith('%'):
                continue
                
            parts = line.split()
            if len(parts) < 7:
                continue
                
            try:
                # Parse RTKLIB format: YYYY/MM/DD HH:MM:SS.sss lat lon height Q ns ratio
                date_str = parts[0]  # YYYY/MM/DD
                time_str = parts[1]  # HH:MM:SS.sss
                lat = float(parts[2])
                lon = float(parts[3])
                height = float(parts[4])
                q = int(parts[5])
                ns = int(parts[6])
                ratio = float(parts[7]) if len(parts) > 7 else 0.0
                
                # Convert datetime to GPS time (approximate)
                dt = datetime.strptime(f"{date_str} {time_str}", "%Y/%m/%d %H:%M:%S.%f")
                
                # GPS epoch: January 6, 1980
                gps_epoch = datetime(1980, 1, 6)
                delta = dt - gps_epoch
                
                # Approximate GPS week and TOW
                gps_week = int(delta.days / 7)
                gps_tow = (delta.days % 7) * 86400 + delta.seconds + delta.microseconds / 1e6
                
                # Standard deviations based on quality
                if q == 1:  # FIXED
                    std_dev = 0.02  # 2cm
                elif q == 2:  # FLOAT
                    std_dev = 0.1   # 10cm
                else:  # SPP
                    std_dev = 3.0   # 3m
                
                # Write in standard POS format
                outfile.write(f"{gps_week} {gps_tow:.3f} ")
                outfile.write(f"{lat:.9f} {lon:.9f} ")
                outfile.write(f"{height:.4f} ")
                outfile.write(f"{q} {ns} ")
                outfile.write(f"{std_dev:.4f} {std_dev:.4f} {std_dev*1.5:.4f} ")
                outfile.write("0.0000 0.0000 0.0000 ")
                outfile.write(f"0.0 {ratio:.1f}\n")
                
                line_count += 1
                
            except (ValueError, IndexError) as e:
                print(f"Warning: Skipping invalid line: {line}")
                continue
        
        print(f"Converted {line_count} position solutions")

def main():
    if len(sys.argv) != 3:
        print("Usage: python3 convert_rtklib_pos.py <input_rtklib_pos> <output_pos>")
        return 1
    
    input_file = sys.argv[1]
    output_file = sys.argv[2]
    
    if not os.path.exists(input_file):
        print(f"Error: Input file {input_file} not found")
        return 1
    
    print(f"Converting RTKLIB POS file: {input_file}")
    print(f"Output file: {output_file}")
    
    convert_rtklib_pos(input_file, output_file)
    
    print(f"✓ Conversion completed!")
    print(f"Use: python3 visualize_pos.py {output_file}")
    
    return 0

if __name__ == '__main__':
    exit(main())
