"""Batched trunk render meshes; each original analytical collision stays in the SDF."""
import math
import xml.etree.ElementTree as ET


def batch_trunk_visuals(world, directory, chunk_size=256):
    directory.mkdir(parents=True,exist_ok=True)
    link=world.find("model[@name='observed_obstacle_field']/link")
    if link is None:
        raise ValueError('dense forest compound link missing')
    visuals=list(link.findall('visual'))
    sides=16
    for chunk,start in enumerate(range(0,len(visuals),chunk_size)):
        path=directory/f'trunks-{chunk:04d}.obj'
        with path.open('w') as output:
            offset=0
            for visual in visuals[start:start+chunk_size]:
                xyzrpy=list(map(float,visual.findtext('pose').split()))
                if any(abs(v)>1e-9 for v in xyzrpy[3:]):
                    raise ValueError('batched trunk visuals require upright cylinders')
                x,y,z=xyzrpy[:3]
                radius=float(visual.findtext('geometry/cylinder/radius'))
                length=float(visual.findtext('geometry/cylinder/length'))
                for zz in (z-length/2,z+length/2):
                    for i in range(sides):
                        angle=2*math.pi*i/sides
                        output.write(f'v {x+radius*math.cos(angle):.6f} {y+radius*math.sin(angle):.6f} {zz:.6f}\n')
                for i in range(sides):
                    a=offset+i+1;b=offset+(i+1)%sides+1
                    output.write(f'f {a} {b} {b+sides} {a+sides}\n')
                output.write('f '+' '.join(str(offset+i+1) for i in reversed(range(sides)))+'\n')
                output.write('f '+' '.join(str(offset+sides+i+1) for i in range(sides))+'\n')
                offset+=2*sides
        visual=ET.SubElement(link,'visual',name=f'trunk_batch_{chunk}')
        mesh=ET.SubElement(ET.SubElement(visual,'geometry'),'mesh')
        ET.SubElement(mesh,'uri').text=str(path)
        material=ET.SubElement(visual,'material')
        ET.SubElement(material,'ambient').text='.28 .16 .07 1'
        ET.SubElement(material,'diffuse').text='.45 .26 .12 1'
    for visual in visuals:
        link.remove(visual)
    return len(visuals)
