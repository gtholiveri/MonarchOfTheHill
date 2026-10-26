import numpy as np, matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Arc, FancyArrowPatch
plt.rcParams.update({'font.family':'Liberation Serif','font.size':11})
th=np.arcsin(33/60); c,s=np.cos(th),np.sin(th)
L=60; run=L*c; H=33; M=12
A=(0,0); B=(run,H); C=(run+M,H); D=(2*run+M,0)
fig,ax=plt.subplots(figsize=(6.5,2.7),dpi=200)
ax.add_patch(Polygon([A,B,C,D],closed=True,fc='#f1e3b8',ec='#333',lw=1.2))
ax.plot([-8,D[0]+8],[0,0],color='#333',lw=1)
def dim(p,q,txt,off,fs=11,toff=3.2,rot=None):
    p=np.array(p,float);q=np.array(q,float);d=q-p;n=np.array([-d[1],d[0]])/np.linalg.norm(d)
    P,Q=p+n*off,q+n*off
    for a in (p,q): ax.plot([a[0]+n[0]*0.8*np.sign(off),P[0]+n[0]*1.2*np.sign(off) if a is p else Q[0]+n[0]*1.2*np.sign(off)],[a[1]+n[1]*0.8*np.sign(off),(P if a is p else Q)[1]+n[1]*1.2*np.sign(off)],color='#555',lw=0.6)
    ax.add_patch(FancyArrowPatch(P,Q,arrowstyle='<|-|>',mutation_scale=7,color='#222',lw=0.7,shrinkA=0,shrinkB=0))
    m=(P+Q)/2+n*toff*np.sign(off)
    r=np.degrees(np.arctan2(d[1],d[0])) if rot is None else rot
    if r>90: r-=180
    if r<-90: r+=180
    ax.text(*m,txt,ha='center',va='center',rotation=r,fontsize=fs,rotation_mode='anchor')
dim(A,B,'5 ft ± 2 in',15)
dim(B,C,'1 ft',6)
dim(D,C,'5 ft ± 2 in',-15)
# height
x=D[0]+12
ax.plot([C[0]+1,x+1.5],[H,H],color='#555',lw=0.6,ls=(0,(2,2)))
ax.add_patch(FancyArrowPatch((x,0),(x,H),arrowstyle='<|-|>',mutation_scale=7,color='#222',lw=0.7,shrinkA=0,shrinkB=0))
ax.text(x+2,H/2,'33 in',ha='left',va='center')
# angle
ax.add_patch(Arc(A,26,26,theta1=0,theta2=np.degrees(th),color='#222',lw=0.7))
ax.text(15,4.5,'33° ± 1°',ha='left',va='center')
# robots
def robot(foot,dirn):
    u=np.array([dirn*c,s]);n=np.array([-s*dirn,c]) if dirn>0 else np.array([s,c])
    p0=np.array(foot)+u*3; w,h=10,6
    pts=[p0,p0+u*w,p0+u*w+n*h,p0+n*h]
    ax.add_patch(Polygon(pts,closed=True,fc='#9aa7b4',ec='#333',lw=0.8))
    return p0,p0+u*w,n
p0,p1,n=robot(A,1)
robot(D,-1)
# 12 in dim under left robot, offset below slope
P,Q=p0+n*9,p1+n*9
ax.add_patch(FancyArrowPatch(P,Q,arrowstyle='<|-|>',mutation_scale=6,color='#222',lw=0.6,shrinkA=0,shrinkB=0))
for a,b in ((p0,P),(p1,Q)): ax.plot([a[0]+n[0]*7,b[0]+n[0]*1],[a[1]+n[1]*7,b[1]+n[1]*1],color='#555',lw=0.5)
m=(P+Q)/2+n*2.4
ax.text(*m,'≤ 12 in',fontsize=8.5,ha='center',va='center',rotation=np.degrees(th))
# mesa center
mc=(run+M/2,H)
ax.plot(*mc,'o',color='#c0392b',ms=6,zorder=5)
ax.annotate('mesa center\n(scoring point)',xy=mc,xytext=(mc[0],H-13),fontsize=10,ha='center',va='center',
            arrowprops=dict(arrowstyle='-',color='#555',lw=0.6,shrinkA=2,shrinkB=4))
ax.set_aspect('equal');ax.set_xlim(-14,D[0]+22);ax.set_ylim(-3,H+16);ax.axis('off')
fig.savefig('side.png',bbox_inches='tight',pad_inches=0.03,dpi=200)
