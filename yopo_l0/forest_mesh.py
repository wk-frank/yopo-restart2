"""Static 32-sided trunk mesh batches for rendering and physical collision."""
import math
import xml.etree.ElementTree as ET


def batch_trunk_visuals(world, directory, chunk_size=256):
    directory.mkdir(parents=True,exist_ok=True)
    link=world.find("model[@name='observed_obstacle_field']/link")
    if link is None:
        raise ValueError('dense forest compound link missing')
    visuals=list(link.findall('visual'))
    sides=32
    manifest=[]
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
                manifest.append({'center':[x,y,z],'radius':radius,'length':length,'mesh':path.name})
                for zz in (z-length/2,z+length/2):
                    for i in range(sides):
                        angle=2*math.pi*i/sides
                        output.write(f'v {x+radius*math.cos(angle):.6f} {y+radius*math.sin(angle):.6f} {zz:.6f}\n')
                        output.write(f'vn {math.cos(angle):.8f} {math.sin(angle):.8f} 0\n')
                for i in range(sides):
                    a=offset+i+1;b=offset+(i+1)%sides+1
                    output.write(f'f {a}//{a} {b}//{b} {b+sides}//{b+sides} {a+sides}//{a+sides}\n')
                output.write('f '+' '.join(f'{offset+i+1}//{offset+i+1}' for i in reversed(range(sides)))+'\n')
                output.write('f '+' '.join(f'{offset+sides+i+1}//{offset+sides+i+1}' for i in range(sides))+'\n')
                offset+=2*sides
        visual=ET.SubElement(link,'visual',name=f'trunk_batch_{chunk}')
        mesh=ET.SubElement(ET.SubElement(visual,'geometry'),'mesh')
        ET.SubElement(mesh,'uri').text=str(path)
        material=ET.SubElement(visual,'material')
        ET.SubElement(material,'ambient').text='.28 .16 .07 1'
        ET.SubElement(material,'diffuse').text='.45 .26 .12 1'
        collision=ET.SubElement(link,'collision',name=f'trunk_mesh_{chunk}')
        mesh=ET.SubElement(ET.SubElement(collision,'geometry'),'mesh')
        ET.SubElement(mesh,'uri').text=str(path)
    for visual in visuals:
        link.remove(visual)
    for collision in list(link.findall('collision')):
        if collision.find('geometry/cylinder') is not None:
            link.remove(collision)
    import json
    (directory/'manifest.json').write_text(json.dumps({'sides':sides,'trunks':manifest},separators=(',',':')))
    return len(visuals)
