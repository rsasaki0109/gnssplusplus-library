#!/usr/bin/env python3
"""
Convert RTKLIB .pos file to KML format
"""

def read_rtklib_pos(filename):
    """Read RTKLIB .pos file and extract coordinates"""
    coords = []

    with open(filename, 'r') as f:
        for line in f:
            # Skip comment lines
            if line.startswith('%') or line.strip() == '':
                continue

            parts = line.split()
            if len(parts) >= 15:
                # Extract: week, tow, lat, lon, height, Q
                lat = float(parts[2])
                lon = float(parts[3])
                height = float(parts[4])
                Q = int(parts[5])

                coords.append({
                    'lat': lat,
                    'lon': lon,
                    'height': height,
                    'Q': Q
                })

    return coords

def create_kml(coords, output_file, title="RTK Solution"):
    """Create KML file from coordinates"""

    kml_header = f'''<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2">
  <Document>
    <name>{title}</name>
    <description>RTK positioning results</description>

    <!-- Style for FIXED solutions -->
    <Style id="fixedPoint">
      <IconStyle>
        <color>ff00ff00</color>
        <scale>0.5</scale>
        <Icon>
          <href>http://maps.google.com/mapfiles/kml/shapes/placemark_circle.png</href>
        </Icon>
      </IconStyle>
    </Style>

    <!-- Style for FLOAT solutions -->
    <Style id="floatPoint">
      <IconStyle>
        <color>ff00ffff</color>
        <scale>0.5</scale>
        <Icon>
          <href>http://maps.google.com/mapfiles/kml/shapes/placemark_circle.png</href>
        </Icon>
      </IconStyle>
    </Style>

    <!-- Style for path -->
    <Style id="pathStyle">
      <LineStyle>
        <color>ff0000ff</color>
        <width>2</width>
      </LineStyle>
    </Style>

    <!-- Placemark for trajectory -->
    <Placemark>
      <name>Trajectory</name>
      <styleUrl>#pathStyle</styleUrl>
      <LineString>
        <tessellate>1</tessellate>
        <altitudeMode>absolute</altitudeMode>
        <coordinates>
'''

    kml_footer = '''        </coordinates>
      </LineString>
    </Placemark>
  </Document>
</kml>
'''

    with open(output_file, 'w') as f:
        f.write(kml_header)

        # Write all coordinates for the path
        for coord in coords:
            f.write(f"          {coord['lon']:.8f},{coord['lat']:.8f},{coord['height']:.4f}\n")

        f.write(kml_footer)

    print(f"KML file created: {output_file}")
    print(f"  Total points: {len(coords)}")
    fixed_count = sum(1 for c in coords if c['Q'] == 1)
    float_count = sum(1 for c in coords if c['Q'] == 2)
    print(f"  FIXED solutions: {fixed_count}")
    print(f"  FLOAT solutions: {float_count}")

if __name__ == '__main__':
    # Convert RTKLIB result to KML
    coords = read_rtklib_pos('rtklib_rtk_result.pos')
    create_kml(coords, 'rtklib_rtk_result.kml', title="RTKLIB RTK Solution")
