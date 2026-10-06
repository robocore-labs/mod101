#!/usr/bin/env python3
"""Export upstream PGGripper CAD meshes/mass measurements; URDF is authored separately."""
import sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
import export_7dof_wrist as c
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TDocStd import TDocStd_Document
from OCP.TCollection import TCollection_ExtendedString
from OCP.XCAFDoc import XCAFDoc_DocumentTool
from OCP.TDF import TDF_LabelSequence,TDF_Label
from OCP.TDataStd import TDataStd_Name
from OCP.TopLoc import TopLoc_Location
r=STEPCAFControl_Reader();r.SetNameMode(True);r.ReadFile(str(ROOT/'src/mod101_tool_pggripper/config/Gripper.stp'));doc=TDocStd_Document(TCollection_ExtendedString('pg'));r.Transfer(doc);t=XCAFDoc_DocumentTool.ShapeTool_s(doc.Main());roots=TDF_LabelSequence();t.GetFreeShapes(roots);parts=[]
def visit(l,loc):
 target=l
 if t.IsReference_s(l):
  target=TDF_Label();t.GetReferredShape_s(l,target);loc=loc.Multiplied(t.GetLocation_s(l))
 a=TDataStd_Name();name=a.Get().ToExtString() if target.FindAttribute(TDataStd_Name.GetID_s(),a) else '?'
 children=TDF_LabelSequence()
 if t.GetComponents_s(target,children):
  for i in range(1,children.Length()+1):visit(children.Value(i),loc)
 else:
  s=t.GetShape_s(target).Moved(loc);parts.append((name,s))
for i in range(1,roots.Length()+1):visit(roots.Value(i),TopLoc_Location())

import json,math,hashlib
from OCP.gp import gp_Trsf
from OCP.BRep import BRep_Builder
from OCP.TopoDS import TopoDS_Compound
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.StlAPI import StlAPI_Writer
PKG=ROOT/'src/mod101_tool_pggripper'
anchor=[24.780656572828,-2.539420714208,62.526906611007]
radius=18*4.15/(2*math.pi)/1000
q_export=.025/radius
result={'source_url':'https://github.com/norma-core/norma-core/tree/main/hardware/pgripper',
 'upstream_commit':'b935fc4b887e576538921b15630a30db22759ec8',
 'source_sha256':hashlib.sha256((PKG/'config/Gripper.stp').read_bytes()).hexdigest(),
 'anchor_mm':anchor,'cad_to_tool_rotation':[[0,0,-1],[0,1,0],[1,0,0]],
 'rack_pitch_mm':4.15,'pinion_teeth':18,'pitch_radius_m':radius,'export_angle_rad':q_export,
 'max_angle_rad':.027/radius,'print_profile':{'material':'PETG','density_kg_m3':1270,'infill':.15,'shell_mm':1.6},'parts':{}}
# Jaw reference shifted inwards 25mm: measured rib faces then touch at zero.
# Export corresponds to a 50mm gap; another 2mm per jaw reaches rated 54mm.
for stem,indices,pivot,shift in [('body',[0,4,5,7,9,*range(10,22)],anchor,0),
 ('servo',[30],anchor,0),('drive',[1,31,22,23,24,25,32],[anchor[0],anchor[1],14.326906611007],0),
 ('left_jaw',[3],anchor,-25),('right_jaw',[2],anchor,25)]:
 compound=TopoDS_Compound();builder=BRep_Builder();builder.MakeCompound(compound);total=GProp_GProps()
 for i in indices:
  name,shape=parts[i]
  tr=gp_Trsf();tr.SetValues(0,0,-1,pivot[2],0,1,0,-pivot[1]+shift,1,0,0,-pivot[0])
  shape=shape.Moved(TopLoc_Location(tr));builder.Add(compound,shape)
  v=GProp_GProps();a=GProp_GProps();BRepGProp.VolumeProperties_s(shape,v);BRepGProp.SurfaceProperties_s(shape,a)
  vol=v.Mass()
  if i==30:mass=.056
  elif 10<=i<=29 or i==32:mass=7850e-9*vol
  else:mass=1270e-9*(min(vol,a.Mass()*1.6)+.15*max(0,vol-a.Mass()*1.6))
  total.Add(v,mass/vol)
 BRepMesh_IncrementalMesh(compound,.12,False,.25,True);w=StlAPI_Writer();w.ASCIIMode=False;assert w.Write(compound,str(PKG/'meshes'/f'{stem}.stl'))
 I=total.MatrixOfInertia()
 result['parts'][stem]={'mass_kg':total.Mass(),'com_m':[x/1000 for x in total.CentreOfMass().Coord()],
  'inertia':{k:I.Value(i,j)*1e-6 for k,i,j in [('ixx',1,1),('iyy',2,2),('izz',3,3),('ixy',1,2),('ixz',1,3),('iyz',2,3)]},'bounds_mm':c.bounds(compound),'source_indices':indices}
 print(stem,result['parts'][stem]['mass_kg'])
(PKG/'config/measurements.json').write_text(json.dumps(result,indent=2)+'\n')
