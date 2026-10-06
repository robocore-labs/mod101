#!/usr/bin/env python3
"""Export camera_tool STEP meshes and measurements; authored xacro owns frames.

Known camera weights are supplied by the user. PETG holder: 15% infill,
1.6mm uniform shell, 1270kg/m³. COM/tensors use uniform CAD distributions.
"""
import hashlib,json,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).parent))
import export_7dof_wrist as cad
from OCP.gp import gp_Trsf
from OCP.TopLoc import TopLoc_Location
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.StlAPI import StlAPI_Writer

ROOT=Path(__file__).resolve().parents[1]
PKG=ROOT/'src/mod101_tool_camera'
CAD=ROOT/'camera_tool.step'

def main():
    cad.CAD=CAD
    parts,empty=cad.read_parts()
    (PKG/'meshes').mkdir(parents=True,exist_ok=True)
    (PKG/'config').mkdir(exist_ok=True)
    result={'source_step':CAD.name,'source_sha256':hashlib.sha256(CAD.read_bytes()).hexdigest(),
            'cad_to_tool_rotation':[[0,0,1],[-1,0,0],[0,-1,0]],'cad_anchor_mm':[0,0,0],
            'print_profile':{'material':'PETG','density_kg_m3':1270,'infill':.15,'shell_mm':1.6},'parts':{}}
    for label,stem,mass in [('camera_holder','holder',None),('camera_luxonis','luxonis',.061),('camera_realsense','realsense',.072),('GoPro HERO 10 (1)','gopro',.153)]:
        tr=gp_Trsf();tr.SetValues(0,0,1,0,-1,0,0,0,0,-1,0,0)
        shape=parts[label].Moved(TopLoc_Location(tr))
        v=GProp_GProps();BRepGProp.VolumeProperties_s(shape,v)
        a=GProp_GProps();BRepGProp.SurfaceProperties_s(shape,a)
        volume,area=v.Mass(),a.Mass()
        if mass is None:
            shell=min(volume,area*1.6);mass=1270e-9*(shell+.15*(volume-shell))
        tensor=v.MatrixOfInertia();scale=mass/volume*1e-6
        inertia={key:tensor.Value(i,j)*scale for key,i,j in [('ixx',1,1),('iyy',2,2),('izz',3,3),('ixy',1,2),('ixz',1,3),('iyz',2,3)]}
        result['parts'][stem]={'source_label':label,'mass_kg':mass,'com_m':[c/1000 for c in v.CentreOfMass().Coord()],
                              'inertia':inertia,'bounds_mm':cad.bounds(shape),'volume_mm3':volume,'surface_mm2':area}
        BRepMesh_IncrementalMesh(shape,.15,False,.25,True)
        writer=StlAPI_Writer();writer.ASCIIMode=False
        assert writer.Write(shape,str(PKG/'meshes'/f'{stem}.stl'))
        print(stem,'mass',mass,'COM',result['parts'][stem]['com_m'],flush=True)
    (PKG/'config/measurements.json').write_text(json.dumps(result,indent=2)+'\n')
if __name__=='__main__':main()
