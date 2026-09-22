"""Experimental immutable-scene base screens. Never replace final 3D checks.

Not imported by production planning. A screen overlap always falls back to the
exact base checker, so a conservative enclosure cannot reject a valid pose.
"""
from time import perf_counter
import numpy as np
from motion_toolbox.geometry import Plane, as_plane


def _clip_z(polygon, z, above):
    output=[]
    for a,b in zip(polygon,np.roll(polygon,-1,axis=0)):
        ia,ib=(a[2]>=z,b[2]>=z) if above else (a[2]<=z,b[2]<=z)
        if ia:
            output.append(a)
        if ia != ib:
            output.append(a+(b-a)*((z-a[2])/(b[2]-a[2])))
    return np.asarray(output).reshape(-1,3)


class SlabProjection:
    def __init__(self, vertices, faces, low=0., high=1.):
        vertices=np.asarray(vertices,dtype=float)
        self.bounds=(vertices.min(axis=0),vertices.max(axis=0))
        triangles=[]
        contours=[]
        contour_components=[]
        parents=list(range(len(vertices)))
        def component(i):
            while parents[i]!=i:
                parents[i]=parents[parents[i]]
                i=parents[i]
            return i
        for face in faces:
            for i in face[1:]:
                parents[component(i)]=component(face[0])
        middle=(low+high)/2
        for face in faces:
            for j in range(1,len(face)-1):
                tri=vertices[[face[0],face[j],face[j+1]]]
                polygon=_clip_z(tri,low,True)
                if len(polygon):
                    polygon=_clip_z(polygon,high,False)
                if len(polygon)>=3:
                    for k in range(1,len(polygon)-1):
                        triangles.append(polygon[[0,k,k+1],:2])
                # A volume spanning the entire slab can project only its side
                # boundaries. A mid-slab section also detects full containment.
                intersections=[]
                for a,b in zip(tri,np.roll(tri,-1,axis=0)):
                    if (a[2]>middle)!=(b[2]>middle):
                        intersections.append((a+(b-a)*((middle-a[2])/(b[2]-a[2])))[:2])
                if len(intersections)==2:
                    contours.append(intersections)
                    contour_components.append(component(face[0]))
        self.triangles=np.asarray(triangles).reshape(-1,3,2)
        self.contours=np.asarray(contours).reshape(-1,2,2)
        self.contour_components=np.asarray(contour_components)
        if len(self.triangles):
            self.low=self.triangles.min(axis=1)
            self.high=self.triangles.max(axis=1)

    def overlaps(self, center, axes, half):
        """Rectangle/triangle SAT, including degenerate vertical projections."""
        center,axes,half=map(np.asarray,(center,axes,half))
        world_half=abs(axes)@half
        if len(self.triangles):
            nearby=np.all(self.low<=center+world_half,axis=1)&np.all(self.high>=center-world_half,axis=1)
            local=(self.triangles[nearby]-center)@axes
            # Chunked SAT bounds working storage for detailed meshes.
            for start in range(0,len(local),256):
                tri=local[start:start+256]
                possible=np.all(tri.min(axis=1)<=half,axis=1)&np.all(tri.max(axis=1)>=-half,axis=1)
                tri=tri[possible]
                if not len(tri):
                    continue
                edges=np.roll(tri,-1,axis=1)-tri
                normals=np.stack((-edges[:,:,1],edges[:,:,0]),axis=2)
                projection=np.einsum('nvi,nai->nav',tri,normals)
                radius=np.sum(abs(normals)*half,axis=2)
                separated=np.any((projection.min(axis=2)>radius)|(projection.max(axis=2)<-radius),axis=1)
                if np.any(~separated):
                    return True
        for group in np.unique(self.contour_components):
            contour=self.contours[self.contour_components==group]
            a,b=contour[:,0],contour[:,1]
            cross=(a[:,1]>center[1])!=(b[:,1]>center[1])
            a,b=a[cross],b[cross]
            xs=a[:,0]+(center[1]-a[:,1])*(b[:,0]-a[:,0])/(b[:,1]-a[:,1])
            if np.count_nonzero(xs>center[0])%2:
                return True
        return False


class StaticBaseScreen:
    """Benchmark adapter for a frozen world, fixed joints, and ground base poses.

    Installation is explicit in the replay/benchmark harness. Environment and
    fixed-joint changes require a new adapter; the production world stays live.
    """
    def __init__(self, world, meshes, method):
        if method not in ('rectangle','box'):
            raise ValueError('Unknown experimental screen')
        started=perf_counter()
        self.world,self.method=world,method
        self.exact=world.is_base_valid
        previous=world._base
        world.set_base(Plane.world_xy())
        bounds=[world.p.getAABB(world.robot,i) for i in world.static_links if i in world.collision_links]
        world.set_base(previous)
        if not bounds:
            raise ValueError('No static collision geometry')
        self.low=np.min([b[0] for b in bounds],axis=0)
        self.high=np.max([b[1] for b in bounds],axis=0)
        self.center=(self.low+self.high)/2
        self.half=(self.high-self.low)/2
        self.obstacles=[world.p.getAABB(body) for body,_ in world.environment]
        self.projections=[SlabProjection(m['vertices'],m['faces']) for m in meshes] if method=='rectangle' else []
        self.supported=len(meshes)==len(world.environment)
        self.box=None
        if method=='box':
            shape=world.p.createCollisionShape(world.p.GEOM_BOX,halfExtents=self.half.tolist())
            self.box=world._body(shape)
        self.stats=dict(calls=0,clear=0,fallback=0,screen_seconds=0.,exact_seconds=0.,setup_seconds=perf_counter()-started)

    def possible(self, base, clearance):
        if not self.supported or not np.allclose(base.zaxis,[0,0,1]) or abs(base.origin[2])>1e-10:
            return True
        rotation=base.matrix[:3,:3]
        center=base.origin+rotation@self.center
        extent=abs(rotation)@self.half+clearance+1e-9
        nearby=[i for i,b in enumerate(self.obstacles)
                if np.all(np.asarray(b[0])<=center+extent) and np.all(np.asarray(b[1])>=center-extent)]
        if not nearby:
            return False
        if self.method=='box':
            self.world.p.resetBasePositionAndOrientation(self.box,center.tolist(),self.world._pose(base)[1])
            return any(self.world.p.getClosestPoints(self.box,self.world.environment[i][0],clearance+1e-8) for i in nearby)
        # The 0..1 m projection cannot certify higher/lower parts of the base.
        # Keep exact checks wherever those heights can overlap an obstacle.
        if clearance != 0:
            return True
        for i in nearby:
            low,high=self.obstacles[i]
            if ((self.high[2]>1 and high[2]>1) or (self.low[2]<0 and low[2]<0)):
                return True
            if self.projections[i].overlaps(center[:2],rotation[:2,:2],self.half[:2]+1e-8):
                return True
        return False

    def is_base_valid(self, base, *, clearance=0.):
        if not np.isfinite(clearance) or clearance<0:
            raise ValueError('Clearance must be nonnegative')
        base=as_plane(base)
        start=perf_counter()
        possible=self.possible(base,clearance)
        self.stats['screen_seconds']+=perf_counter()-start
        self.stats['calls']+=1
        if possible:
            self.stats['fallback']+=1
            start=perf_counter()
            result=self.exact(base,clearance=clearance)
            self.stats['exact_seconds']+=perf_counter()-start
            return result
        self.stats['clear']+=1
        self.world.last_failure=None
        # Preserve the public method's base assignment side effect.
        self.world.set_base(base)
        return True

    def close(self):
        if self.box is not None:
            self.world.p.removeBody(self.box)
            self.box=None
