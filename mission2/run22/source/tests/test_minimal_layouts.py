"""Regression of independently scored static layouts, using retained native geometry."""
import copy
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from eval_layout import apply_layout
from eval_mission2_course import course_from_scene


def native():
    run=Path(__file__).resolve().parents[1]/'.local/evaluation/fresh_mission2/run11'
    if not (run/'runtime/runtime_state/state.json').exists():pytest.skip('retained native reference unavailable')
    runtime=Path(json.loads((run/'runtime/runtime_state/state.json').read_text())['log_dir'])
    return runtime/'arena/scene.xml',json.loads((runtime/'evaluation/model_metadata.json').read_text())


@pytest.mark.parametrize('layout',['mirror_y','reshuffled_722026','reshuffled_722026_mirror','reshuffled_822026'])
def test_layout_keeps_all_obstacles_and_scores_actual_openings(tmp_path,layout):
    path,metadata=native();old=ET.parse(path).getroot();root=copy.deepcopy(old)
    original=course_from_scene(path,metadata)
    apply_layout(root,layout)
    geoms={g.get('name'):g for g in root.findall('./worldbody/geom')}
    assert set(geoms)=={g.get('name') for g in old.findall('./worldbody/geom')}
    assert [ET.tostring(x) for x in root if x.tag!='worldbody']==[ET.tostring(x) for x in old if x.tag!='worldbody']
    for ring in metadata['rings']:
        endpoints=np.array([list(map(float,g.get('fromto').split())) for name,g in geoms.items() if name.startswith(ring['name']+'_')]).reshape(-1,3)
        ring['center_world_m']=endpoints.mean(axis=0).tolist()
    changed=tmp_path/'scene.xml';ET.ElementTree(root).write(changed)
    sections=course_from_scene(changed,metadata,infer_wall_opening=True)
    assert len(sections)==8
    for a,b in zip(original,sections):
        if a['kind']=='ring':assert a['aperture_radius_m']==pytest.approx(b['aperture_radius_m'])
        if layout=='mirror_y' and a['kind']=='rectangle':
            assert b['gap_y_world_m']==pytest.approx([-a['gap_y_world_m'][1],-a['gap_y_world_m'][0]])
    if layout=='reshuffled_722026':
        assert all(s['gap_y_world_m'][1]<0 for s in sections[5:7])
        assert np.diff(sections[-1]['gap_y_world_m'])[0]==pytest.approx(1.8)
        assert any(a['center_world_m']!=b['center_world_m'] for a,b in zip(original[1:4],sections[1:4]))
    if layout in ('reshuffled_722026_mirror','reshuffled_822026'):
        assert np.diff(sections[-1]['gap_y_world_m'])[0]==pytest.approx(1.8)
        assert any(a['center_world_m']!=b['center_world_m'] for a,b in zip(original[1:4],sections[1:4]))
        assert sections[5]['gap_y_world_m'][0]>0
        if layout=='reshuffled_722026_mirror':assert sections[6]['gap_y_world_m'][0]>0
        else:assert sections[6]['gap_y_world_m'][1]<0

