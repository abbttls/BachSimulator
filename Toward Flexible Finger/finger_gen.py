import xml.etree.ElementTree as ET
from xml.dom import minidom

# Function to pretty-print the XML
def prettify(element):
    rough_string = ET.tostring(element, 'utf-8')
    reparsed = minidom.parseString(rough_string)
    return reparsed.toprettyxml(indent="  ")

# Root element <mujoco>
mujoco = ET.Element('mujoco')
# World body
worldbody = ET.SubElement(mujoco, 'worldbody')
    
block_idx = 2

# ADJUST THESE
stiffness = "50000"
damping = "50000"
gap = 2
blocks_per_fin = 2
blocksize = 12/blocks_per_fin
kp = 99999990 # postion control
kv = 9999999 # velocity control


# integrator
ET.SubElement(mujoco, 'option', {
    'integrator': 'implicitfast'
})

# Add a ball that will fall on top of the structure
ball = ET.SubElement(worldbody, 'body', {
    'name': 'falling_ball',
    'pos': '70 90 150',  # Position above the structure (adjust the height as needed)
    'gravcomp': '1'
})

# Add geometry for the ball (sphere)
ET.SubElement(ball, 'geom', {
    'type': 'sphere',
    'size': '30',  # Radius of the ball
    'rgba': '0.2 0.2 0.8 1',  # Color: blueish
    'density': '10'  # Density of the ball for realistic physics
})

# slider joints in each cartesian direction for the ball
ET.SubElement(ball, 'joint', {
    'type': 'slide', 
    'axis': '1 0 0',  # Movement along the X-axis
    'name': 'ball_slide_x'
})
ET.SubElement(ball, 'joint', {
    'type': 'slide', 
    'axis': '0 1 0',  # Movement along the Y-axis
    'name': 'ball_slide_y'
})
ET.SubElement(ball, 'joint', {
    'type': 'slide', 
    'axis': '0 0 1',  # Movement along the Z-axis
    'name': 'ball_slide_z'
})

# Site attached to the ball, which will serve as a control point for the motor
site = ET.SubElement(ball, 'site', {
    'name': 'ball_site',
    'pos': '0 0 0',  # Position the site at the center of the ball
    'size': '1',  # Size of the site marker (for visualization)
    'rgba': '1 0 0 1'  # Color for visibility
})

# Start with an initial block
# previous_body = worldbody

base_box = ET.SubElement(worldbody, 'body', {
    'name': 'base_box',
    'pos': '60 4.77 60.877'  # Position of the base box
})

# Add geometry for the base box
ET.SubElement(base_box, 'geom', {
    'type': 'box',
    'size': '60 4.77 60.877',  # Size of the base box
    'rgba': '0.6 0.6 0.6 1',  # Color: grey
    'density': '1'
})

# Create the first smaller box attached to the top of the base box via revolute joint
small_box_1 = ET.SubElement(base_box, 'body', {
    'name': 'small_box_1',
    'pos': '0 5.6 60',  # Position relative to the base box
    'euler' : '-15.642 0 0'
})

# Add geometry for the first small box
ET.SubElement(small_box_1, 'geom', {
    'type': 'box',
    'size': '60 0.6 1',  # Size of the small box
    'rgba': '0.8 0.3 0.3 1'  # Color: reddish
})

# Add a revolute joint connecting the base box and the first small box
ET.SubElement(small_box_1, 'joint', {
    'type': 'hinge',
    'name': 'joint_0',
    'axis': '1 0 0',  # Revolving around the Y-axis
    'pos': '0 0 0',  # Position of the joint in the base box
    'range': '-90 90',  # Range of motion
    'stiffness' : stiffness,
    'damping' : damping,
    'springref' : '0'
})

# Create the second smaller box attached to the top of the base box via another revolute joint
small_box_2 = ET.SubElement(base_box, 'body', {
    'name': 'small_box_2',
    'pos': '0 5.6 -60',  # Position relative to the base box (shifted in Z axis)
    'euler' : '15.642 0 0'
})

# Add geometry for the second small box
ET.SubElement(small_box_2, 'geom', {
    'type': 'box',
    'size': '60 0.6 1',  # Size of the small box
    'rgba': '0.3 0.8 0.3 1'  # Color: greenish
})

# Add a revolute joint connecting the base box and the second small box
ET.SubElement(small_box_2, 'joint', {
    'type': 'hinge',
    'name': 'joint_1',
    'axis': '1 0 0',  # Revolving around the Y-axis
    'pos': '0 0 0',  # Position of the joint in the base box
    'range': '-90 90',  # Range of motion
    'stiffness' : stiffness,
    'damping' : damping,
    'springref' : '0'
})

# Add blocks to chains on both sides of the finger
def addn_blocks(num_blocks, previous_body, bsize=6, gap=2):
    global block_idx
    for i in range(num_blocks):
        # Create a new body (block)
        body = ET.SubElement(previous_body, 'body', {
            'name': f'block_{block_idx}',
            'pos': f'0 {2*bsize + gap} 0'
        })

        # Add geometry to represent the block (e.g., a box)
        geom = ET.SubElement(body, 'geom', {
            'type': 'box',
            'size': f'60 {bsize} 1',  # Size of the block
            'rgba': '0.8 0.2 0.2 1',
            'density': "1000"
        })
        
        site = ET.SubElement(body, 'site', {
            'name': f'site{block_idx}',
            'pos': '0 0 0',  # Position the site at the center of the block
            'size': '3',  # Size of the site marker (for visualization)
            'rgba': '1 0 0 1'  # Color for visibility
        })

        # Add a revolute joint connecting to the previous block
        joint = ET.SubElement(body, 'joint', {
            'type': 'hinge',
            'name': f'joint_{block_idx}',
            'axis': '1 0 0',  # Revolving around the Y-axis
            'range': '-180 180',  # Limit of the joint movement
            'pos': '0 0 0',  # Position of the joint in the current body
            'stiffness': stiffness,
            'damping': damping,
            'springref': '0',
            'actuatorfrclimited': 'false'
        })

        # Set the current body as the previous body for the next loop iteration
        previous_body = body
        block_idx += 1

    return previous_body

def fin_connect(left, right, dist, bsize=6, gap=2):
    global block_idx
    
    # left half
    lbox = ET.SubElement(left, 'body', {
        'name': f'lbox_{block_idx}',
        'pos': f'0 {bsize/2} -{dist + 6}',  # Position relative to the base box
        'euler' : '15.642 0 0'
    })

    # Add geometry for the left half
    ET.SubElement(lbox, 'geom', {
       'type': 'box',
       'size': f'60 1 {dist}',
       'rgba': '0.8 0.3 0.3 1'
    })

    # Add a revolute joint connecting the base box
    ET.SubElement(lbox, 'joint', {
        'type': 'hinge',
        'name': f'joint_{block_idx}',
        'axis': '1 0 0',
        'pos': '0 0 0',  # Position of the joint in the base box
        'range': '-180 180',  # Range of motion
        'stiffness' : stiffness,
        'damping' : damping,
        'springref' : '0'
    })
    
    block_idx += 1
    
    # right half
    rbox = ET.SubElement(right, 'body', {
        'name': f'rbox_{block_idx}',
        'pos': f'0 {bsize/2} {dist + 6}',  # Position relative to the base box (shifted in Z axis)
        'euler' : '-15.642 0 0'
    })

    # Add geometry for the rbox
    ET.SubElement(rbox, 'geom', {
        'type': 'box',
        'size': f'60 1 {dist}',  # Size of the box
        'rgba': '0.3 0.8 0.3 1'  # Color: greenish
    })

    # Add a revolute joint connecting the base box and the right box
    ET.SubElement(rbox, 'joint', {
        'type': 'hinge',
        'name': f'joint_{block_idx}',
        'axis': '1 0 0',
        'pos': '0 0 0',  # Position of the joint in the base box
        'range': '-180 180',  # Range of motion
        'stiffness' : stiffness,
        'damping' : damping,
        'springref' : '0'
    })
    block_idx += 1
    
    equality = ET.SubElement(mujoco, 'equality')
    ET.SubElement(equality, 'weld', {
        'body1': lbox.attrib['name'],  # Left box name
        'body2': rbox.attrib['name'],  # Right box name
        'active': 'true',  # Activates the constraint
        'solref': '0.02 1',  # Solver reference values for stability
        'solimp': '0.9 0.95 0.001'  # Solver impedance values for damping and response
    })


prevl = addn_blocks(blocks_per_fin, small_box_1, blocksize, gap)
prevr = addn_blocks(blocks_per_fin, small_box_2, blocksize, gap)
fin_connect(prevl, prevr, 21.7, blocksize, gap)
prevl = addn_blocks(blocks_per_fin, prevl, blocksize, gap)
prevr = addn_blocks(blocks_per_fin, prevr, blocksize, gap)
fin_connect(prevl, prevr, 17.7, blocksize, gap)
prevl = addn_blocks(blocks_per_fin, prevl, blocksize, gap)
prevr = addn_blocks(blocks_per_fin, prevr, blocksize, gap)
fin_connect(prevl, prevr, 14, blocksize, gap)
prevl = addn_blocks(blocks_per_fin, prevl, blocksize, gap)
prevr = addn_blocks(blocks_per_fin, prevr, blocksize, gap)
fin_connect(prevl, prevr, 10, blocksize, gap)
prevl = addn_blocks(blocks_per_fin, prevl, blocksize, gap)
prevr = addn_blocks(blocks_per_fin, prevr, blocksize, gap)
fin_connect(prevl, prevr, 6.3, blocksize, gap)
prevl = addn_blocks(blocks_per_fin, prevl, blocksize, gap)
prevr = addn_blocks(blocks_per_fin, prevr, blocksize, gap)
fin_connect(prevl, prevr, 2.5, blocksize, gap)

actuator = ET.SubElement(mujoco, 'actuator')
for i in range(block_idx):
    ET.SubElement(actuator, 'position', {
        'joint': f'joint_{i}',
        'ctrlrange': '-1 1',
        'gear': '100',
        'kp': f'{kp}',  # High kp for stiff control
        'kv': f'{kv}'
    })

ET.SubElement(actuator, 'position', {
    'joint': 'ball_slide_x',
    'ctrllimited': 'true',
    'ctrlrange': '-70000 70000',
    'kp': '10000',
    'dampratio': '1',
    'gear': '100',
    'name': 'ball_motor_x'
})
ET.SubElement(actuator, 'position', {
    'joint': 'ball_slide_y',
    'ctrllimited': 'true',
    'ctrlrange': '-70000 70000',
    'kp': '10000',
    'dampratio': '1',
    'gear': '100',
    'name': 'ball_motor_y'
})
ET.SubElement(actuator, 'position', {
    'joint': 'ball_slide_z',
    'ctrllimited': 'true',
    'ctrlrange': '-70000 70000',
    'kp': '10000',
    'dampratio': '1',
    'gear': '100',
    'name': 'ball_motor_z'
})

# Pretty-print the generated MJCF XML
mjcf_string = prettify(mujoco)
print(mjcf_string)

# save to an XML file
with open('mjcf_chain_new.xml', 'w') as f:
    f.write(mjcf_string)