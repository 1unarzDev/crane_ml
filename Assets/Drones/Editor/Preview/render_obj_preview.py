#!/usr/bin/env python3
"""Software z-buffer preview of Docs/DroneVisuals/model/aos7quad.obj (no GPU needed).
Not HDRP: approximate Lambert + fake studio environment, used because Unity runs with -nographics.
usage: python3 -I render_obj_preview.py OBJ OUTDIR"""
import sys, os, numpy as np
from PIL import Image

OVR={'Carbon':(0.045,0.047,0.052),'PCB_Green':(0.03,0.26,0.09),'LiPo_Label':(0.8,0.82,0.84),'Prop_BlurDisc':(0.3,0.3,0.3)}
def load(obj):
    V=[];N=[];F=[];M=[];cur='';mats=[]
    mtl={}
    base=os.path.dirname(obj)
    with open(obj[:-4]+'.mtl') as f:
        name=None
        for l in f:
            p=l.split()
            if not p: continue
            if p[0]=='newmtl': name=p[1]
            elif p[0]=='Kd': mtl[name]=tuple(map(float,p[1:4]))
    names=sorted(mtl); idx={n:i for i,n in enumerate(names)}
    with open(obj) as f:
        for l in f:
            if l.startswith('v '): V.append(list(map(float,l.split()[1:4])))
            elif l.startswith('vn '): N.append(list(map(float,l.split()[1:4])))
            elif l.startswith('usemtl'): cur=idx[l.split()[1]]
            elif l.startswith('f '):
                F.append([int(t.split('/')[0])-1 for t in l.split()[1:4]]); M.append(cur)
    return np.array(V,np.float32),np.array(N,np.float32),np.array(F),np.array(M),names,np.array([OVR.get(n,mtl[n]) for n in names],np.float32)

def env(d):
    # fake studio: gradient dome + two softboxes
    y=d[...,1:2]
    sky=np.array([0.55,0.62,0.75]);gr=np.array([0.10,0.10,0.11])
    e=np.where(y>0,gr+(sky-gr)*np.clip(y,0,1)**0.6,gr*(1+y*0.5))
    k=np.array([-0.5,0.7,-0.5]);k/=np.linalg.norm(k)
    e=e+3.0*np.clip((d@k)[...,None],0,1)**24*np.array([1,0.97,0.92])
    k2=np.array([0.7,0.3,0.6]);k2/=np.linalg.norm(k2)
    return e+1.2*np.clip((d@k2)[...,None],0,1)**16*np.array([0.8,0.9,1.0])

def render(data,eye,target,fov,W,H,ss=2,name_flags=None):
    V,N,F,M,names,Kd=data
    W2,H2=W*ss,H*ss
    eye=np.array(eye,np.float32);t=np.array(target,np.float32)
    f=t-eye;f/=np.linalg.norm(f);r=np.cross([0,1,0],f);r/=np.linalg.norm(r);u=np.cross(f,r)
    P=V-eye;x=P@r;y=P@u;z=P@f
    s=(0.5*H2)/np.tan(np.radians(fov)/2)
    sx=W2/2+s*x/z;sy=H2/2-s*y/z
    zb=np.full((H2,W2),1e9,np.float32);img=np.zeros((H2,W2,3),np.float32)
    bg=np.linspace(0.30,0.12,H2)[:,None,None]*np.ones((1,W2,3),np.float32)*np.array([1,1.02,1.08],np.float32)
    img[:]=bg
    metal=np.array([any(k in n for k in('Aluminum','Steel','Gold','Copper','Anodized')) for n in names])
    glossy=np.array([any(k in n for k in('Carbon','Glass','LiPo','Plastic','PCB','Prop','LED'))for n in names])
    emis=np.array([n.startswith('LED') for n in names])
    alpha=np.array([0.5 if 'BlurDisc' in n else (0.3 if 'BladeFade' in n else (0.4 if 'Smoked' in n else 1.0)) for n in names])
    for ps in (0,1):
     for i in range(len(F)):
        if (alpha[M[i]]<1.0)!=(ps==1): continue
        a,b,c=F[i]
        if z[a]<=0.01 or z[b]<=0.01 or z[c]<=0.01: continue
        xs=np.array([sx[a],sx[b],sx[c]]);ys=np.array([sy[a],sy[b],sy[c]])
        x0=int(max(0,np.floor(xs.min())));x1=int(min(W2-1,np.ceil(xs.max())))
        y0=int(max(0,np.floor(ys.min())));y1=int(min(H2-1,np.ceil(ys.max())))
        if x1<x0 or y1<y0: continue
        d=(ys[1]-ys[2])*(xs[0]-xs[2])+(xs[2]-xs[1])*(ys[0]-ys[2])
        if abs(d)<1e-9: continue
        gx,gy=np.meshgrid(np.arange(x0,x1+1)+0.5,np.arange(y0,y1+1)+0.5)
        l0=((ys[1]-ys[2])*(gx-xs[2])+(xs[2]-xs[1])*(gy-ys[2]))/d
        l1=((ys[2]-ys[0])*(gx-xs[2])+(xs[0]-xs[2])*(gy-ys[2]))/d
        l2=1-l0-l1
        m=(l0>=-1e-4)&(l1>=-1e-4)&(l2>=-1e-4)
        if not m.any(): continue
        zz=1.0/(l0/z[a]+l1/z[b]+l2/z[c])
        sub=zb[y0:y1+1,x0:x1+1]
        m&=zz<sub
        if not m.any(): continue
        nn=(l0[...,None]*N[a]+l1[...,None]*N[b]+l2[...,None]*N[c])
        nn/=np.linalg.norm(nn,axis=-1,keepdims=True)+1e-9
        pos=eye+0  # view dir approx per tri
        mi=M[i];alb=Kd[mi]
        vd=-((V[a]-eye)/np.linalg.norm(V[a]-eye))
        L=np.array([-0.5,0.7,-0.5],np.float32);L/=np.linalg.norm(L)
        dif=np.clip(nn@L,0,1)[...,None]
        amb=env(nn)*0.55
        if emis[mi]: col=alb*3.0
        elif metal[mi]:
            rv=2*(nn@vd)[...,None]*nn-vd
            col=alb*env(rv)*(0.9)
        else:
            col=alb*(amb+dif*1.1)
            rv=2*(nn@vd)[...,None]*nn-vd
            fres=(1-np.clip(nn@vd,0,1))[...,None]**5
            sp=0.05+0.5*fres if glossy[mi] else 0.03
            col=col+env(rv)*sp
        col=np.broadcast_to(col,m.shape+(3,))
        reg=img[y0:y1+1,x0:x1+1]
        if alpha[mi]<1.0: reg[m]=reg[m]*(1-alpha[mi])+col[m]*alpha[mi]
        else:
            sub[m]=zz[m];zb[y0:y1+1,x0:x1+1]=sub;reg[m]=col[m]
    out=img.reshape(H,ss,W,ss,3).mean((1,3))
    out=np.clip(out/(1+out*0.25),0,1)**(1/2.2)
    return Image.fromarray((out*255).astype(np.uint8))

if __name__=='__main__':
    obj,outd=sys.argv[1],sys.argv[2];os.makedirs(outd,exist_ok=True)
    data=load(obj)
    only=sys.argv[3:]
    tag=os.path.basename(obj)[len('aos7quad'):-4]
    views={
     'iso_front_left':((-0.42,0.28,0.55),(0,0.0,0.0),32),
     'iso_rear_right':((0.45,0.30,-0.55),(0,0.0,0.0),32),
     'top':((0.0,0.75,0.0001),(0,0,0),26),
     'front':((0.0,0.03,0.85),(0,0.01,0),24),
     'side':((0.85,0.03,0.0),(0,0.01,0),24),
     'detail_sensors':((-0.17,0.14,0.26),(0,0.04,0.04),30),
     'detail_motor':((0.20,0.07,0.22),(0.092,0.0,0.127),30),
    }
    for k,(e,t,fv) in views.items():
        if only and k not in only: continue
        render(data,e,t,fv,1100,780).save(os.path.join(outd,k+tag+'.png'));print('wrote',k,flush=True)
