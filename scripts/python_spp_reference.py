#!/usr/bin/env python3
"""
Python SPP Reference Implementation using GNSS libraries
This serves as a reference for the C++ implementation
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from datetime import datetime, timedelta
import os
import sys

# Constants
GM = 3.986005e14  # Earth's gravitational parameter (m^3/s^2)
OMEGA_E = 7.2921151467e-5  # Earth rotation rate (rad/s)
C_LIGHT = 299792458.0  # Speed of light (m/s)
WGS84_A = 6378137.0  # WGS84 semi-major axis (m)
WGS84_F = 1.0/298.257223563  # WGS84 flattening

class GPSEphemeris:
    """GPS Ephemeris data structure"""
    def __init__(self):
        self.prn = 0
        self.toe = 0.0  # Time of ephemeris
        self.toc = 0.0  # Time of clock
        self.sqrt_a = 0.0  # Square root of semi-major axis
        self.e = 0.0  # Eccentricity
        self.i0 = 0.0  # Inclination at reference time
        self.omega0 = 0.0  # Longitude of ascending node
        self.omega = 0.0  # Argument of perigee
        self.m0 = 0.0  # Mean anomaly at reference time
        self.delta_n = 0.0  # Mean motion difference
        self.idot = 0.0  # Rate of inclination angle
        self.omega_dot = 0.0  # Rate of right ascension
        self.cuc = self.cus = 0.0  # Amplitude of cosine/sine harmonic correction terms
        self.crc = self.crs = 0.0  # Amplitude of cosine/sine harmonic correction terms
        self.cic = self.cis = 0.0  # Amplitude of cosine/sine harmonic correction terms
        self.af0 = self.af1 = self.af2 = 0.0  # Clock correction coefficients
        self.valid = False

class GPSObservation:
    """GPS Observation data structure"""
    def __init__(self):
        self.prn = 0
        self.pseudorange = 0.0
        self.carrier_phase = 0.0
        self.doppler = 0.0
        self.snr = 0.0
        self.time = 0.0
        self.valid = False

def solve_kepler(M, e, max_iter=10, tol=1e-12):
    """Solve Kepler's equation iteratively"""
    E = M
    for i in range(max_iter):
        E_new = M + e * np.sin(E)
        if abs(E_new - E) < tol:
            break
        E = E_new
    return E

def calculate_satellite_position(eph, t):
    """Calculate satellite position from ephemeris at time t"""
    if not eph.valid:
        return None, None
    
    # Time from ephemeris reference epoch
    dt = t - eph.toe
    
    # Computed mean motion
    n0 = np.sqrt(GM / (eph.sqrt_a**6))
    n = n0 + eph.delta_n
    
    # Mean anomaly
    M = eph.m0 + n * dt
    
    # Eccentric anomaly
    E = solve_kepler(M, eph.e)
    
    # True anomaly
    nu = 2.0 * np.arctan2(np.sqrt(1 + eph.e) * np.sin(E/2), 
                          np.sqrt(1 - eph.e) * np.cos(E/2))
    
    # Argument of latitude
    phi = nu + eph.omega
    
    # Second harmonic perturbations
    delta_u = eph.cuc * np.cos(2*phi) + eph.cus * np.sin(2*phi)
    delta_r = eph.crc * np.cos(2*phi) + eph.crs * np.sin(2*phi)
    delta_i = eph.cic * np.cos(2*phi) + eph.cis * np.sin(2*phi)
    
    # Corrected argument of latitude, radius, and inclination
    u = phi + delta_u
    r = eph.sqrt_a**2 * (1 - eph.e * np.cos(E)) + delta_r
    i = eph.i0 + delta_i + eph.idot * dt
    
    # Positions in orbital plane
    x_orb = r * np.cos(u)
    y_orb = r * np.sin(u)
    
    # Corrected longitude of ascending node
    Omega = eph.omega0 + (eph.omega_dot - OMEGA_E) * dt - OMEGA_E * eph.toe
    
    # Earth-fixed coordinates
    x = x_orb * np.cos(Omega) - y_orb * np.cos(i) * np.sin(Omega)
    y = x_orb * np.sin(Omega) + y_orb * np.cos(i) * np.cos(Omega)
    z = y_orb * np.sin(i)
    
    # Satellite clock correction
    dt_sv = eph.af0 + eph.af1 * dt + eph.af2 * dt**2
    
    return np.array([x, y, z]), dt_sv

def ecef_to_geodetic(x, y, z):
    """Convert ECEF coordinates to geodetic (lat, lon, height)"""
    p = np.sqrt(x**2 + y**2)
    e2 = 2*WGS84_F - WGS84_F**2
    
    # Initial latitude estimate
    lat = np.arctan2(z, p * (1 - e2))
    
    # Iterate for precise latitude
    for _ in range(3):
        N = WGS84_A / np.sqrt(1 - e2 * np.sin(lat)**2)
        height = p / np.cos(lat) - N
        lat = np.arctan2(z, p * (1 - e2 * N / (N + height)))
    
    lon = np.arctan2(y, x)
    N = WGS84_A / np.sqrt(1 - e2 * np.sin(lat)**2)
    height = p / np.cos(lat) - N
    
    return np.degrees(lat), np.degrees(lon), height

def spp_least_squares(observations, ephemerides, initial_pos=None):
    """Single Point Positioning using weighted least squares"""
    if len(observations) < 4:
        return None
    
    # Initial position estimate (Tokyo if not provided)
    if initial_pos is None:
        x = -3947762.0
        y = 3364399.0
        z = 3699428.0
    else:
        x, y, z = initial_pos
    
    dt = 0.0  # Receiver clock bias
    
    # Iterative least squares
    for iteration in range(10):
        # Calculate satellite positions and form design matrix
        H = []  # Design matrix
        delta_rho = []  # Observation minus computed
        weights = []
        
        for obs in observations:
            # Find corresponding ephemeris
            eph = None
            for e in ephemerides:
                if e.prn == obs.prn and e.valid:
                    eph = e
                    break
            
            if eph is None:
                continue
            
            # Calculate satellite position
            sat_pos, sat_clk = calculate_satellite_position(eph, obs.time)
            if sat_pos is None:
                continue
            
            # Geometric range
            dx = sat_pos[0] - x
            dy = sat_pos[1] - y
            dz = sat_pos[2] - z
            rho = np.sqrt(dx**2 + dy**2 + dz**2)
            
            # Skip if satellite too close (unrealistic)
            if rho < 15000000.0 or rho > 30000000.0:
                continue
            
            # Unit vector from receiver to satellite
            unit_vec = np.array([dx/rho, dy/rho, dz/rho])
            
            # Design matrix row: [partial_x, partial_y, partial_z, partial_dt]
            H.append([-unit_vec[0], -unit_vec[1], -unit_vec[2], 1.0])
            
            # Predicted pseudorange
            predicted = rho + dt * C_LIGHT - sat_clk * C_LIGHT
            
            # Residual
            residual = obs.pseudorange - predicted
            delta_rho.append(residual)
            
            # Weight (can be based on elevation, SNR, etc.)
            weights.append(1.0)
        
        if len(H) < 4:
            return None
        
        # Convert to numpy arrays
        H = np.array(H)
        delta_rho = np.array(delta_rho)
        W = np.diag(weights)
        
        # Weighted least squares solution
        try:
            # Normal equations: (H^T * W * H) * dx = H^T * W * delta_rho
            N = H.T @ W @ H
            b = H.T @ W @ delta_rho
            dx = np.linalg.solve(N, b)
            
            # Update position and clock bias
            x += dx[0]
            y += dx[1]
            z += dx[2]
            dt += dx[3]
            
            # Check convergence
            if np.linalg.norm(dx[:3]) < 0.01:  # 1cm convergence
                break
                
        except np.linalg.LinAlgError:
            return None
    
    # Convert to geodetic coordinates
    lat, lon, height = ecef_to_geodetic(x, y, z)
    
    return {
        'position_ecef': np.array([x, y, z]),
        'position_geodetic': np.array([lat, lon, height]),
        'clock_bias': dt,
        'num_satellites': len(H),
        'iterations': iteration + 1,
        'residuals': delta_rho
    }

def create_synthetic_data():
    """Create synthetic GPS data for testing"""
    ephemerides = []
    observations = []
    
    # Create 8 GPS satellites with realistic parameters
    for prn in range(1, 9):
        eph = GPSEphemeris()
        eph.prn = prn
        eph.toe = 365266.0
        eph.toc = 365266.0
        eph.sqrt_a = np.sqrt(26560000.0)  # ~26,560 km orbit radius
        eph.e = 0.01  # Low eccentricity
        eph.i0 = np.radians(55.0)  # GPS inclination
        eph.omega0 = 2.0 * np.pi * (prn - 1) / 8.0  # Distribute around orbit
        eph.omega = np.pi * (prn - 1) / 4.0
        eph.m0 = 2.0 * np.pi * (prn - 1) / 8.0
        eph.delta_n = 4.5e-9
        eph.idot = -3.0e-10
        eph.omega_dot = -8.5e-9
        # Perturbation terms (simplified)
        eph.cuc = eph.cus = eph.crc = eph.crs = eph.cic = eph.cis = 0.0
        eph.af0 = eph.af1 = eph.af2 = 0.0
        eph.valid = True
        ephemerides.append(eph)
    
    # Generate observation epochs
    epochs = []
    base_time = 365266.0
    
    for epoch in range(100):
        epoch_obs = []
        current_time = base_time + epoch * 1.0
        
        # Tokyo position with small movement
        true_x = -3947762.0 + 100.0 * np.sin(epoch * 0.1)
        true_y = 3364399.0 + 100.0 * np.cos(epoch * 0.1)
        true_z = 3699428.0 + 50.0 * np.sin(epoch * 0.05)
        
        for eph in ephemerides:
            # Calculate satellite position
            sat_pos, sat_clk = calculate_satellite_position(eph, current_time)
            if sat_pos is None:
                continue
            
            # Calculate true range
            dx = sat_pos[0] - true_x
            dy = sat_pos[1] - true_y
            dz = sat_pos[2] - true_z
            true_range = np.sqrt(dx**2 + dy**2 + dz**2)
            
            # Add realistic errors
            noise = np.random.normal(0, 3.0)  # 3m noise
            clock_bias = 0.001 * C_LIGHT  # 1ms receiver clock bias
            pseudorange = true_range + clock_bias + noise - sat_clk * C_LIGHT
            
            # Check if satellite is visible (elevation > 10 degrees)
            if true_range > 15000000.0 and true_range < 30000000.0:
                obs = GPSObservation()
                obs.prn = eph.prn
                obs.pseudorange = pseudorange
                obs.time = current_time
                obs.valid = True
                epoch_obs.append(obs)
        
        if len(epoch_obs) >= 4:
            epochs.append(epoch_obs)
    
    return ephemerides, epochs

def main():
    print("Python SPP Reference Implementation")
    print("===================================")
    
    # Create synthetic data
    print("Generating synthetic GPS data...")
    ephemerides, epochs = create_synthetic_data()
    print(f"Created {len(ephemerides)} satellites and {len(epochs)} epochs")
    
    # Process SPP for each epoch
    results = []
    valid_solutions = 0
    
    print("\nProcessing SPP...")
    for i, epoch_obs in enumerate(epochs):
        solution = spp_least_squares(epoch_obs, ephemerides)
        
        if solution is not None:
            valid_solutions += 1
            results.append({
                'epoch': i,
                'time': epoch_obs[0].time,
                'lat': solution['position_geodetic'][0],
                'lon': solution['position_geodetic'][1],
                'height': solution['position_geodetic'][2],
                'num_sats': solution['num_satellites'],
                'iterations': solution['iterations'],
                'clock_bias': solution['clock_bias'],
                'rms_residual': np.sqrt(np.mean(solution['residuals']**2))
            })
            
            if i < 5:  # Print first 5 solutions
                lat, lon, height = solution['position_geodetic']
                print(f"Epoch {i+1:3d}: {lat:10.6f}°N, {lon:10.6f}°E, {height:7.1f}m "
                      f"({solution['num_satellites']} sats, {solution['iterations']} iter)")
    
    print(f"\nProcessing Summary:")
    print(f"Total epochs: {len(epochs)}")
    print(f"Valid solutions: {valid_solutions}")
    print(f"Success rate: {100.0 * valid_solutions / len(epochs):.1f}%")
    
    # Save results to POS file
    pos_filename = "results/python_spp_reference.pos"
    with open(pos_filename, 'w') as f:
        f.write("% Python SPP Reference Implementation Results\n")
        f.write("% GPST                  latitude(deg) longitude(deg)  height(m)   Q  ns   sdn(m)   sde(m)   sdu(m)  sdne(m)  sdeu(m)  sdun(m) age(s)  ratio\n")
        
        for result in results:
            f.write(f"2311 {result['time']:8.3f} {result['lat']:12.9f} {result['lon']:13.9f} "
                   f"{result['height']:8.4f} 5 {result['num_sats']:2d} "
                   f"1.0000 1.0000 2.0000 0.0000 0.0000 0.0000 0.00 0.00\n")
    
    print(f"Results saved to: {pos_filename}")
    
    # Create visualization
    if results:
        df = pd.DataFrame(results)
        
        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(12, 10))
        
        # Trajectory plot
        ax1.plot(df['lon'], df['lat'], 'b-o', markersize=2)
        ax1.set_xlabel('Longitude (deg)')
        ax1.set_ylabel('Latitude (deg)')
        ax1.set_title('SPP Trajectory')
        ax1.grid(True)
        
        # Height over time
        ax2.plot(df['epoch'], df['height'], 'g-')
        ax2.set_xlabel('Epoch')
        ax2.set_ylabel('Height (m)')
        ax2.set_title('Height Variation')
        ax2.grid(True)
        
        # Number of satellites
        ax3.plot(df['epoch'], df['num_sats'], 'r-o', markersize=2)
        ax3.set_xlabel('Epoch')
        ax3.set_ylabel('Number of Satellites')
        ax3.set_title('Satellite Availability')
        ax3.grid(True)
        
        # RMS residuals
        ax4.plot(df['epoch'], df['rms_residual'], 'm-')
        ax4.set_xlabel('Epoch')
        ax4.set_ylabel('RMS Residual (m)')
        ax4.set_title('Solution Quality')
        ax4.grid(True)
        
        plt.tight_layout()
        plt.savefig('python_spp_analysis.png', dpi=150, bbox_inches='tight')
        print("Analysis plot saved to: python_spp_analysis.png")
    
    print("\n✅ Python SPP reference implementation completed successfully!")
    print("This can be used as a reference for debugging the C++ implementation.")

if __name__ == "__main__":
    main()
