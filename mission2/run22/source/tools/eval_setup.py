"""Explicit startup-only evaluation scene/spawn adaptations; never policy input."""
from __future__ import annotations
import copy
import hashlib
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from eval_common import local_path, workspace_builder, write_json


def resolve_setup(raw, scenario):
    if raw is None:return None
    allowed={'schema','name','spawn_xyz','spawn_yaw_rad','scene_variant','fixed_straight_road_length_m','mission2_layout'}
    if not isinstance(raw,dict) or set(raw)-allowed or raw.get('schema')!=1:
        raise ValueError('invalid evaluation_setup schema/fields')
    result=copy.deepcopy(raw)
    if not isinstance(result.get('name'),str) or not result['name']:
        raise ValueError('evaluation_setup needs an explicit name')
    variant=result.get('scene_variant','baseline')
    if variant not in ('baseline','ring1_isolated','ring_sequence') or variant in ('ring1_isolated','ring_sequence') and scenario!='mission2':
        raise ValueError('ring evaluation variants require mission2')
    result['scene_variant']=variant
    if 'mission2_layout' in result:
        if scenario!='mission2' or variant!='baseline' or result['mission2_layout'] not in ('mirror_y','reshuffled_722026','reshuffled_722026_mirror','reshuffled_822026'):
            raise ValueError('layout variants require full static mission2')
    if 'spawn_xyz' in result:
        xyz=result['spawn_xyz']
        if not isinstance(xyz,list) or len(xyz)!=3 or any(isinstance(x,bool) or not isinstance(x,(int,float)) or not math.isfinite(x) for x in xyz):
            raise ValueError('spawn_xyz must be three finite coordinates')
        if not (-50<=xyz[0]<=50 and -10<=xyz[1]<=10 and .02<=xyz[2]<=3):
            raise ValueError('evaluation spawn outside arena bounds')
    if 'spawn_yaw_rad' in result:
        yaw=result['spawn_yaw_rad']
        if isinstance(yaw,bool) or not isinstance(yaw,(int,float)) or not math.isfinite(yaw) or abs(yaw)>2*math.pi:
            raise ValueError('invalid evaluation spawn yaw')
    if 'fixed_straight_road_length_m' in result:
        length=result['fixed_straight_road_length_m']
        if scenario!='mission3_straight' or isinstance(length,bool) or not isinstance(length,(int,float)) or not math.isfinite(length) or not 2<=length<=100:
            raise ValueError('fixed road length requires mission3_straight and length 2..100')
    return result


def initialize_scene(model,data,config,setup,initializer):
    adapted=copy.deepcopy(config)
    if setup:
        for key in ('spawn_xyz','spawn_yaw_rad'):
            if key in setup:adapted[key]=setup[key]
    # The baseline initializer performs this one initial free-joint pose assignment.
    # There are no runtime pose writes in the evaluation wrapper after initialization.
    return initializer(model,data,adapted)


def build_scene(config,directory,setup,builder=None):
    directory=local_path(directory)
    scene=(builder or workspace_builder())(config,directory)
    if not setup:return scene
    original=hashlib.sha256(scene.read_bytes()).hexdigest()
    root=ET.parse(scene).getroot()
    removed=[]
    world=root.find('worldbody')
    if setup['scene_variant']=='ring1_isolated':
        for element in list(world):
            name=element.get('name','')
            if name.startswith(('ring2_','ring3_','gate23_')) or name in ('block1','block2','s_wall1','s_wall2'):
                removed.append(name)
                world.remove(element)
    if setup['scene_variant']=='ring_sequence':
        # Explicit ring-only sequence: keep the actual floor, all stage fences,
        # all three original rings and lighting. Entry/exit gates and other
        # mission obstacles belong to separate passage qualification.
        for element in list(world):
            name=element.get('name','')
            keep=(element.tag=='light' or name=='arena_floor' or name.startswith(
                ('fence_','ring1_','ring2_','ring3_')))
            if not keep:
                removed.append(name)
                world.remove(element)
    if 'fixed_straight_road_length_m' in setup:
        length=setup['fixed_straight_road_length_m']
        road=world.find("geom[@name='straight_road']")
        if road is None:raise ValueError('straight road missing from baseline builder output')
        road.set('pos',f'{length/2-1:.17g} 0 -0.008')
        road.set('size',f'{length/2:.17g} 1 .01')
    layout_changes=[]
    if 'mission2_layout' in setup:
        from eval_layout import apply_layout
        layout_changes=apply_layout(root,setup['mission2_layout'])
    ET.indent(root)
    ET.ElementTree(root).write(scene,encoding='utf-8',xml_declaration=True)
    if layout_changes:
        import json
        manifest_path=directory/'arena_manifest.json';manifest=json.loads(manifest_path.read_text())
        manifest['original_scene_sha256']=original
        manifest['scene_sha256']=hashlib.sha256(scene.read_bytes()).hexdigest()
        manifest['evaluation_layout']=setup['mission2_layout'];manifest['layout_changes']=layout_changes
        for key in ('spawn_xyz','spawn_yaw_rad'):
            if key in setup:manifest['config'][key]=setup[key]
        write_json(manifest_path,manifest)
    write_json(directory/'evaluation_variant.json',dict(evaluation_only=True,setup=setup,
        original_derived_scene_sha256=original,scene_sha256=hashlib.sha256(scene.read_bytes()).hexdigest(),
        removed_geoms=removed,retained_environment_names=[e.get('name') for e in world],policy_may_read=False,
        changes='startup spawn / declared scene selection only; drone/firmware/camera/physics unchanged'))
    return scene


def install_setup(raw_config,output):
    setup=resolve_setup(raw_config.get('evaluation_setup'),raw_config.get('arena',{}).get('scenario'))
    if not setup:return None
    from arena import model_builder,rover_adapter
    builder=model_builder.build
    initializer=rover_adapter.initialize
    model_builder.build=lambda config,directory:build_scene(config,directory,setup,builder)
    rover_adapter.initialize=lambda model,data,config:initialize_scene(model,data,config,setup,initializer)
    write_json(Path(output)/'evaluation_setup.json',dict(evaluation_only=True,policy_may_read=False,
        setup=setup,application='one startup initialization, before agents/sockets; never runtime pose tracking'))
    return setup
