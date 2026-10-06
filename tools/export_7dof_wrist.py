#!/usr/bin/env python3
"""Export CAD-owned wrist meshes and mass measurements; xacro owns the URDF.

PETG 1270 kg/m³, 15% infill, provisional 1.6 mm shell. The shell-volume
estimate A*t is clamped to the solid CAD volume. COM/tensor retain the uniform
CAD distribution, scaled to the estimated printed mass. Servos are 56 g.
"""
import hashlib
import json
from pathlib import Path
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TDocStd import TDocStd_Document
from OCP.TCollection import TCollection_ExtendedString
from OCP.XCAFDoc import XCAFDoc_DocumentTool
from OCP.TDF import TDF_LabelSequence, TDF_Label
from OCP.TDataStd import TDataStd_Name
from OCP.TopLoc import TopLoc_Location
from OCP.gp import gp_Trsf
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_FACE
from OCP.TopoDS import TopoDS
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Cylinder
from OCP.BRepBndLib import BRepBndLib
from OCP.Bnd import Bnd_Box
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.StlAPI import StlAPI_Writer

ROOT = Path(__file__).resolve().parents[1]
PKG = ROOT / 'src/mod101_description'
CAD = ROOT / 'mod101_7DOF.step'


def read_parts():
    r = STEPCAFControl_Reader(); r.SetNameMode(True)
    assert int(r.ReadFile(str(CAD))) == 1
    doc = TDocStd_Document(TCollection_ExtendedString('7DOF'))
    assert r.Transfer(doc)
    tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    roots = TDF_LabelSequence(); tool.GetFreeShapes(roots)
    parts = {}; empty = []
    def visit(label, loc):
        target = label
        if tool.IsReference_s(label):
            target = TDF_Label(); tool.GetReferredShape_s(label, target)
            loc = loc.Multiplied(tool.GetLocation_s(label))
        a = TDataStd_Name()
        name = a.Get().ToExtString() if target.FindAttribute(TDataStd_Name.GetID_s(), a) else '?'
        children = TDF_LabelSequence()
        if tool.GetComponents_s(target, children):
            for i in range(1, children.Length()+1): visit(children.Value(i), loc)
        else:
            shape = tool.GetShape_s(target).Moved(loc)
            box = Bnd_Box(); BRepBndLib.AddOptimal_s(shape, box, False, False)
            if box.IsVoid(): empty.append(name)
            else: parts[name] = shape
    for i in range(1, roots.Length()+1): visit(roots.Value(i), TopLoc_Location())
    return parts, empty


def bounds(shape):
    b = Bnd_Box(); BRepBndLib.AddOptimal_s(shape, b, False, False)
    return list(b.Get())


def main():
    parts, empty = read_parts()
    rail = bounds(parts['elbow_extrusion'])
    anchor = [(rail[0]+rail[3])/2, rail[1], (rail[2]+rail[5])/2]
    # Output-horn datums: shaft cylinder axes, and outer horn-face planes.
    def output(part, axis, bound):
        explorer = TopExp_Explorer(parts[part], TopAbs_FACE)
        while explorer.More():
            surface = BRepAdaptor_Surface(TopoDS.Face_s(explorer.Current()))
            if surface.GetType() == GeomAbs_Cylinder and abs(surface.Cylinder().Radius()-9.6) < 1e-5:
                cylinder = surface.Cylinder()
                if abs(cylinder.Axis().Direction().Coord()[axis]) > .999:
                    point = list(cylinder.Location().Coord())
                    point[axis] = bounds(parts[part])[bound]
                    return point
            explorer.Next()
        raise ValueError('Missing servo shaft cylinder: '+part)
    p4 = output('servo_4_wrist_yaw', 2, 5)
    p5 = output('servo_5_wrist_pitch', 0, 0)
    p6 = output('servo_6_wrist_roll', 1, 1)
    items = [
        ('wrist_servo_adapter_1','wrist_yaw_mount_b','wrist_yaw_mount_7dof',anchor,False),
        ('servo_wrist_yaw_1','servo_4_wrist_yaw','servo_wrist_yaw_7dof',p4,True),
        ('wrist_yaw_link_1','elbow_link (1)','wrist_yaw_link_7dof',p4,False),
        ('servo_wrist_tilt_1','servo_5_wrist_pitch','servo_wrist_pitch_7dof',p5,True),
        ('wrist_roll_adapter_1','wrist_pitch_link','wrist_pitch_link_7dof',p5,False),
        ('servo_wrist_rotation_1','servo_6_wrist_roll','servo_wrist_roll_7dof',p6,True),
    ]
    result = {'step_sha256': hashlib.sha256(CAD.read_bytes()).hexdigest(),
              'empty_components_omitted': empty,
              'print_profile': {'material':'PETG','density_kg_m3':1270,'infill':0.15,'shell_mm':1.6},
              'cad_anchor_mm':anchor, 'cad_pivots_mm':{'yaw':p4,'pitch':p5,'roll':p6},'links':{}}
    def relative(p, q): return [-(p[1]-q[1])/1000,(p[0]-q[0])/1000,(p[2]-q[2])/1000]
    result['joint_offsets_m'] = {'yaw':relative(p4,anchor),'pitch':relative(p5,p4),'roll':relative(p6,p5)}
    for link, part, stem, pivot, servo in items:
        transform = gp_Trsf()
        # Canonical +X points along CAD -Y; +Y along CAD +X; +Z unchanged.
        transform.SetValues(0,-1,0,pivot[1], 1,0,0,-pivot[0], 0,0,1,-pivot[2])
        shape = parts[part].Moved(TopLoc_Location(transform))
        v = GProp_GProps(); BRepGProp.VolumeProperties_s(shape,v)
        a = GProp_GProps(); BRepGProp.SurfaceProperties_s(shape,a)
        volume = v.Mass(); area = a.Mass()
        mass = .056 if servo else 1270e-9*(min(volume,area*1.6)+.15*max(0,volume-area*1.6))
        scale = mass/volume*1e-6
        I = v.MatrixOfInertia()
        inode = {key:I.Value(i,j)*scale for key,i,j in [('ixx',1,1),('iyy',2,2),('izz',3,3),('ixy',1,2),('ixz',1,3),('iyz',2,3)]}
        result['links'][link] = {'part':part,'mesh':stem+'.stl','mass_kg':mass,
            'com_m':[x/1000 for x in v.CentreOfMass().Coord()], 'inertia':inode,
            'bounds_mm':bounds(shape),'volume_mm3':volume,'surface_mm2':area,'servo':servo}
        BRepMesh_IncrementalMesh(shape,.15,False,.25,True)
        w = StlAPI_Writer(); w.ASCIIMode = False
        assert w.Write(shape,str(PKG/'meshes'/(stem+'.stl')))
    (PKG/'config/7dof_measurements.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result['joint_offsets_m'],indent=2))
    for name,item in result['links'].items(): print(name, item['mass_kg'])

if __name__ == '__main__': main()
