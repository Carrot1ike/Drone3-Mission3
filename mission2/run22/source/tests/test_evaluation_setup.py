"""Evaluation setup schema and scene-selection invariants; offline only."""
import copy
import json
import math
from pathlib import Path
import sys
import uuid
import xml.etree.ElementTree as ET

import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from eval_common import LOCAL,DEFAULT_BASELINE
from eval_configs import configurations
from eval_setup import resolve_setup,build_scene,initialize_scene


def test_generated_arena_schema_has_no_spawn_and_speed_variants_only_change_motion():
    configs=configurations(DEFAULT_BASELINE)
    for raw in configs.values():
        assert set(raw['arena'])<={'scenario','rover'}
        assert 'spawn_xyz' not in raw['arena']
        assert raw['evaluation_setup']['schema']==1
    rover=[configs[n] for n in ('mission3_stationary','mission3_straight_10','mission3_straight_15')]
    assert [r['arena']['rover']['speed_mps'] for r in rover]==[0.,.1,.15]
    for raw in rover:
        assert raw['evaluation_setup']['fixed_straight_road_length_m']==29.65
        assert raw['flowdeck'] and raw['state_link']['frame_state_send']
    ring=configs['mission2_ring1_isolated']['evaluation_setup']
    assert ring['spawn_xyz']==[5.55,0.,.1] and ring['spawn_yaw_rad']==math.pi


@pytest.mark.parametrize('raw,scenario',[
    ({'schema':1,'name':'x','hidden_truth_path':'x'},'mission2'),
    ({'schema':1,'name':'x','spawn_xyz':[0,0,float('nan')]},'mission2'),
    ({'schema':1,'name':'x','scene_variant':'ring1_isolated'},'mission3_straight'),
    ({'schema':1,'name':'x','fixed_straight_road_length_m':30},'mission2')])
def test_setup_rejects_unsupported_or_nonfinite_changes(raw,scenario):
    with pytest.raises(ValueError):resolve_setup(raw,scenario)


def test_initializer_only_adapts_one_startup_config_without_mutating_original():
    config=dict(scenario='mission2',spawn_xyz=[7,-1.4,.1],spawn_yaw_rad=math.pi)
    saved=copy.deepcopy(config)
    setup=resolve_setup(dict(schema=1,name='x',spawn_xyz=[5.55,0,.1]),'mission2')
    calls=[]
    def init(model,data,cfg):calls.append(cfg);return 'adapter'
    assert initialize_scene(None,None,config,setup,init)=='adapter'
    assert config==saved and calls[0]['spawn_xyz']==[5.55,0,.1] and len(calls)==1


def test_variant_removes_declared_downstream_obstacles_preserves_first_ring_and_physics():
    directory=LOCAL/'test-output'/('variant_'+uuid.uuid4().hex)
    directory.mkdir(parents=True)
    def builder(config,out):
        path=out/'scene.xml'
        path.write_text('<mujoco><option timestep=".001" gravity="0 0 -9.81"/><worldbody>'+''.join(
            f'<geom name="{name}"/>' for name in ('ring1_0','ring2_0','ring3_0','block1','s_wall1','gate23_post0','gate12_post0','floor'))+'</worldbody></mujoco>')
        return path
    setup=resolve_setup(dict(schema=1,name='ring',scene_variant='ring1_isolated'),'mission2')
    scene=build_scene({'scenario':'mission2'},directory,setup,builder)
    root=ET.parse(scene).getroot()
    assert {g.get('name') for g in root.find('worldbody')}=={'ring1_0','gate12_post0','floor'}
    assert root.find('option').attrib==dict(timestep='.001',gravity='0 0 -9.81')
    manifest=json.loads((directory/'evaluation_variant.json').read_text())
    assert manifest['evaluation_only'] and not manifest['policy_may_read']


def test_actual_offline_config_manifest_has_same_rover_geometry_and_clear_ring_approach():
    path=LOCAL/'configs/trial_manifest.json'
    if not path.exists():pytest.skip('run offline eval_configs first')
    manifest=json.loads(path.read_text())
    rover=[t for t in manifest['trials'] if t['name'].startswith('mission3')]
    assert len({t['scene_sha256'] for t in rover})==1
    ring=next(t for t in manifest['trials'] if t['name']=='mission2_ring1_isolated')
    assert len(ring['model_metadata']['rings'])==1
    assert len(ring['offline_clearance_checks'])>=50
    assert not any(t['drone_contacts'] for t in ring['offline_clearance_checks'])
    assert ring['initial_drone_position_world_m'][0]-ring['model_metadata']['rings'][0]['center_world_m'][0]==pytest.approx(1.75)


def test_next_ring_suite_is_full_offset_yaw_factorial_and_has_no_policy_layout_data():
    configs=configurations(DEFAULT_BASELINE,'m2-next')
    assert len(configs)==5
    offsets=[raw for raw in configs.values() if raw['evaluation_setup']['scene_variant']=='ring1_isolated']
    pairs={(raw['evaluation_setup']['spawn_xyz'][1],round(math.degrees(raw['evaluation_setup']['spawn_yaw_rad']))) for raw in offsets}
    assert pairs=={(.2,170),(.2,190),(-.2,170),(-.2,190)}
    for raw in configs.values():
        assert set(raw['arena'])=={'scenario'} and raw['arena']['scenario']=='mission2'
        assert raw['evaluation_setup']['spawn_xyz'][0]==5.55
        assert raw['evaluation_setup']['spawn_xyz'][2]==.1
        assert not any(key in raw for key in ('host_positions','ring_count','waypoints','expected_count'))
    sequence=configs['mission2_ring_sequence']['evaluation_setup']
    assert sequence['scene_variant']=='ring_sequence' and sequence['spawn_xyz']==[5.55,0.,.1]
    assert resolve_setup(sequence,'mission2')==sequence
    with pytest.raises(ValueError):resolve_setup(sequence,'mission3_straight')


def test_ring_sequence_retains_all_rings_fences_floor_and_removes_other_obstacles():
    directory=LOCAL/'test-output'/('sequence_'+uuid.uuid4().hex)
    directory.mkdir(parents=True)
    names=('ring1_0','ring2_0','ring3_0','fence_north','arena_floor',
           'gate12_post0','gate23_post0','block1','building1','s_wall1')
    def builder(config,out):
        path=out/'scene.xml'
        path.write_text('<mujoco><option timestep=".001" gravity="0 0 -9.81"/><worldbody><light name="light"/>'+''.join(
            f'<geom name="{name}" size=".05"/>' for name in names)+'</worldbody></mujoco>')
        return path
    setup=resolve_setup(dict(schema=1,name='sequence',scene_variant='ring_sequence'),'mission2')
    root=ET.parse(build_scene({'scenario':'mission2'},directory,setup,builder)).getroot()
    assert {e.get('name') for e in root.find('worldbody')}=={'ring1_0','ring2_0','ring3_0','fence_north','arena_floor','light'}
    assert all(g.get('size')=='.05' for g in root.find('worldbody').findall('geom'))
    assert root.find('option').attrib==dict(timestep='.001',gravity='0 0 -9.81')
    manifest=json.loads((directory/'evaluation_variant.json').read_text())
    assert set(manifest['removed_geoms'])=={'gate12_post0','gate23_post0','block1','building1','s_wall1'}
    assert manifest['evaluation_only'] and not manifest['policy_may_read']


def test_actual_next_suite_geometry_preserved_and_corrected_corridors_clear():
    path=LOCAL/'configs/trial_manifest_m2_next.json'
    if not path.exists():pytest.skip('run eval_configs --suite m2-next first')
    import hashlib
    from eval_configs import environment_signature
    manifest=json.loads(path.read_text())
    centered=LOCAL/'configs/scenes/mission2_ring1_isolated/scene.xml'
    centered_digest=hashlib.sha256(centered.read_bytes()).hexdigest()
    isolated=[t for t in manifest['trials'] if t['evaluation_setup']['scene_variant']=='ring1_isolated']
    assert len(isolated)==4 and {t['scene_sha256'] for t in isolated}=={centered_digest}
    assert len({json.dumps(t['physical_invariants'],sort_keys=True) for t in manifest['trials']})==1
    sequence=next(t for t in manifest['trials'] if t['evaluation_setup']['scene_variant']=='ring_sequence')
    assert [r['name'] for r in sequence['model_metadata']['rings']]==['ring1','ring2','ring3']
    assert sequence['fence_geometry_sha256']==environment_signature(centered,('fence_',))
    # Ring1's attributes match the reviewed centered scene exactly.
    assert environment_signature(Path(sequence['scene']),('ring1_',))==environment_signature(centered,('ring1_',))
    original=Path('/home/razer/Desktop/Drone3/virtual_aideck/runtime/arena/vendor/sim_final/world.xml')
    # The actual capsule geometry attributes that specify shape/pose are retained.
    expected={g.get('name'):g.attrib for g in ET.parse(original).getroot().find('worldbody') if g.get('name','').startswith(('ring1_','ring2_','ring3_'))}
    actual={g.get('name'):g.attrib for g in ET.parse(sequence['scene']).getroot().find('worldbody') if g.get('name','').startswith(('ring1_','ring2_','ring3_'))}
    assert len(expected)==len(actual)==96
    for name,attributes in expected.items():
        assert all(actual[name][key]==value for key,value in attributes.items())
    for trial in manifest['trials']:
        assert not trial['policy_position_prior_provided'] and not trial['policy_ring_count_prior_provided']
        assert trial['camera']=='cf0_fpv_cam' and not trial['native_executed']
        assert not any(probe['drone_contacts'] for probe in trial['offline_clearance_checks'])
        assert '--controller-config' in trial['command_argv']
        assert '--ring-count' not in trial['command_argv'] and '--spawn' not in trial['command_argv']
    assert len(sequence['offline_clearance_checks'])==196
    assert len(sequence['previews'])==4
