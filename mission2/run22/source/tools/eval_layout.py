"""Startup-only static Mission2 layout variants. Never imported by flight policy."""
import copy
import numpy as np


def transform_geom(geom,point):
    if geom.get('fromto'):
        v=list(map(float,geom.get('fromto').split()));a=point(v[:3]);b=point(v[3:])
        geom.set('fromto',' '.join(format(x,'.12g') for x in [*a,*b]))
    elif geom.get('pos'):
        old=np.array(list(map(float,geom.get('pos').split())));new=np.array(point(old.copy()))
        if geom.get('type')=='box' and geom.get('size'):
            size=np.array(list(map(float,geom.get('size').split())))
            lo=np.array(point(old-size));hi=np.array(point(old+size));new=(lo+hi)/2
            geom.set('size',' '.join(format(abs(x),'.12g') for x in (hi-lo)/2))
        geom.set('pos',' '.join(format(x,'.12g') for x in new))


def apply_layout(root,name):
    if name not in ('mirror_y','reshuffled_722026','reshuffled_722026_mirror','reshuffled_822026'):raise ValueError('unknown layout')
    world=root.find('worldbody');changes=[]
    if name=='reshuffled_722026_mirror':
        changes=apply_layout(root,'reshuffled_722026')
        changes.extend(apply_layout(root,'mirror_y'))
        return changes
    if name=='mirror_y':
        for geom in world.findall('geom'):
            transform_geom(geom,lambda p:[p[0],-p[1],p[2]])
        changes.append('Reflect all static course geometry across y=0; spawn reflected separately.')
    else:
        alternate=name=='reshuffled_822026'
        rng=np.random.default_rng(822026 if alternate else 722026)
        for ring in ('ring1','ring2','ring3'):
            delta=np.array([rng.uniform(-.3,.3),rng.uniform(-.45,.45),rng.uniform(-.15,.15)])
            for geom in world.findall('geom'):
                if geom.get('name','').startswith(ring+'_'):
                    transform_geom(geom,lambda p,delta=delta:np.array(p)+delta)
            changes.append(dict(object=ring,translation=delta.tolist()))
        objects=([('block1',[-6.8,-1.15,1.2],[.30,.40,1.2]),
                  ('block2',[-6.8,1.1,1.4],[.35,.40,1.4]),
                  ('s_wall1',[-10.8,-1.,1.25],[.10,1.5,1.25]),
                  ('s_wall2',[-14.3,1.1,1.4],[.12,1.4,1.4])] if alternate else
                 [('block1',[-6.7,-.9,1.1],[.225,.45,1.1]),
                              ('block2',[-6.7,1.225,1.5],[.45,.275,1.5]),
                              ('s_wall1',[-10.6,1.1,1.1],[.08,1.4,1.1]),
                              ('s_wall2',[-14.6,.9,1.45],[.12,1.6,1.45])])
        for name,pos,size in objects:
            geom=world.find(f"geom[@name='{name}']");geom.set('pos',' '.join(map(str,pos)));geom.set('size',' '.join(map(str,size)))
            changes.append(dict(object=name,pos=pos,size=size))
        # Map the gate's endpoints piecewise so side meshes and collisions remain
        # attached to the real arena boundaries; retain every collidable element.
        def gate_point(p):
            target=[-2.5,-1.3,.5,2.5] if alternate else [-2.5,-.6,1.2,2.5]
            y=float(np.interp(p[1],[-2.5,-2.15,-.65,2.5],target))
            return [p[0],y,p[2]]
        for geom in world.findall('geom'):
            if geom.get('name','').startswith('gate23_'):transform_geom(geom,gate_point)
        changes.append('Independent ring offsets, resized blocks, alternating wall openings, translated 1.8m exit.' if alternate else 'Exit opening translated/widened; both walls now have the opening on the same side.')
    return changes
