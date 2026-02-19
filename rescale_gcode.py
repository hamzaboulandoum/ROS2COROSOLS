#!/usr/bin/env python3
"""
Rescale G-code file to have max height of 80mm and center it.
"""

def parse_coordinate(line, prev_x, prev_y):
    """Parse a G-code line and extract X, Y coordinates."""
    x, y = prev_x, prev_y
    parts = line.strip().split()
    
    for part in parts:
        if part.startswith('X'):
            try:
                x = float(part[1:])
            except ValueError:
                pass
        elif part.startswith('Y'):
            try:
                y = float(part[1:])
            except ValueError:
                pass
    
    return x, y

def rescale_gcode(input_file, output_file, target_height_mm=80):
    """
    Rescale G-code to target height and center it.
    
    Args:
        input_file: Path to input G-code file
        output_file: Path to output G-code file
        target_height_mm: Target maximum height in mm (default: 80)
    """
    # Read the file
    with open(input_file, 'r') as f:
        lines = f.readlines()
    
    # First pass: find min/max coordinates
    min_x, max_x = float('inf'), float('-inf')
    min_y, max_y = float('inf'), float('-inf')
    
    current_x, current_y = 0.0, 0.0
    
    for line in lines:
        line = line.strip()
        if line.startswith('X') or line.startswith('G01'):
            current_x, current_y = parse_coordinate(line, current_x, current_y)
            min_x = min(min_x, current_x)
            max_x = max(max_x, current_x)
            min_y = min(min_y, current_y)
            max_y = max(max_y, current_y)
    
    # Calculate current dimensions (in meters, convert to mm)
    width = (max_x - min_x) * 1000  # Convert to mm
    height = (max_y - min_y) * 1000  # Convert to mm
    
    print(f"Original dimensions:")
    print(f"  Width: {width:.2f} mm")
    print(f"  Height: {height:.2f} mm")
    print(f"  X range: [{min_x*1000:.2f}, {max_x*1000:.2f}] mm")
    print(f"  Y range: [{min_y*1000:.2f}, {max_y*1000:.2f}] mm")
    
    # Calculate scaling factor based on height
    scale_factor = target_height_mm / height
    
    print(f"\nScale factor: {scale_factor:.4f}")
    
    # Calculate new dimensions
    new_width = width * scale_factor
    new_height = height * scale_factor
    
    print(f"\nNew dimensions:")
    print(f"  Width: {new_width:.2f} mm")
    print(f"  Height: {new_height:.2f} mm")
    
    # Calculate center offset (to center at 0,0)
    center_x = (min_x + max_x) / 2
    center_y = (min_y + max_y) / 2
    
    print(f"\nCentering offsets:")
    print(f"  X offset: {center_x*1000:.2f} mm")
    print(f"  Y offset: {center_y*1000:.2f} mm")
    
    # Second pass: rescale and center coordinates
    output_lines = []
    current_x, current_y = 0.0, 0.0
    
    for line in lines:
        original_line = line.rstrip()
        
        # Skip empty lines and comments
        if not original_line or original_line.startswith('//'):
            output_lines.append(original_line)
            continue
        
        # Process coordinate lines
        if original_line.startswith('X') or original_line.startswith('G01'):
            prev_x, prev_y = current_x, current_y
            current_x, current_y = parse_coordinate(original_line, current_x, current_y)
            
            # Apply scaling and centering
            new_x = (current_x - center_x) * scale_factor
            new_y = (current_y - center_y) * scale_factor
            
            # Reconstruct the line
            parts = original_line.split()
            new_parts = []
            
            has_x = False
            has_y = False
            
            for part in parts:
                if part.startswith('X'):
                    new_parts.append(f"X{new_x:.4f}")
                    has_x = True
                elif part.startswith('Y'):
                    new_parts.append(f"Y{new_y:.4f}")
                    has_y = True
                elif part.startswith('G'):
                    new_parts.append(part)
            
            # If only X or only Y was specified, keep the same format
            if not new_parts:
                if 'X' in original_line:
                    output_lines.append(f"X{new_x:.4f}")
                elif 'Y' in original_line:
                    output_lines.append(f"Y{new_y:.4f}")
            else:
                output_lines.append(' '.join(new_parts))
        else:
            # Keep other lines unchanged
            output_lines.append(original_line)
    
    # Write output file
    with open(output_file, 'w') as f:
        for line in output_lines:
            f.write(line + '\n')
    
    print(f"\nRescaled G-code written to: {output_file}")
    print(f"Final range:")
    print(f"  X: [{-new_width/2:.2f}, {new_width/2:.2f}] mm")
    print(f"  Y: [{-new_height/2:.2f}, {new_height/2:.2f}] mm")

if __name__ == "__main__":
    input_file = "src/corosols/corosols/gcode/C.txt"
    output_file = "src/corosols/corosols/gcode/C_rescaled.txt"
    
    rescale_gcode(input_file, output_file, target_height_mm=80)
